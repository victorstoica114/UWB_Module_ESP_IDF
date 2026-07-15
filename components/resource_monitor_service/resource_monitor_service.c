#include "resource_monitor_service.h"

#include <stdio.h>
#include <string.h>

#include "driver/temperature_sensor.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/freertos_debug.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "soc/soc_caps.h"

static const char *TAG = "resource_monitor";

enum {
    RESOURCE_MONITOR_TASK_STACK_WORDS = 3072,
    RESOURCE_MONITOR_TASK_PRIORITY = 1,
    RESOURCE_MONITOR_SAMPLE_MS = 1000,
    RESOURCE_MONITOR_MAX_TRACKED_TASKS = 128,
};

#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
#define RESOURCE_MONITOR_TASK_CORE 0
#else
#define RESOURCE_MONITOR_TASK_CORE 0
#endif

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static resource_monitor_snapshot_t s_snapshot;
static int64_t s_last_update_us;
static bool s_started;
static temperature_sensor_handle_t s_temp_sensor;
static bool s_flash_stats_cached;
static bool s_flash_stats_valid;
static size_t s_flash_total_bytes;
static size_t s_flash_reserved_bytes;
static size_t s_flash_free_bytes;
static uint32_t s_flash_partition_count;
static esp_err_t s_flash_error = ESP_ERR_INVALID_STATE;

typedef struct {
    TaskHandle_t handle;
    char name[RESOURCE_MONITOR_TASK_NAME_LEN];
    int32_t core_id;
    uint64_t runtime_counter;
} task_runtime_record_t;

static task_runtime_record_t *s_current_tasks;
static task_runtime_record_t *s_previous_tasks;
static UBaseType_t s_task_sample_capacity;
static UBaseType_t s_previous_task_count;
static int64_t s_previous_task_time_us;
static bool s_previous_tasks_valid;

static uint32_t age_ms_from_time_us(int64_t timestamp_us)
{
    if (timestamp_us <= 0) {
        return UINT32_MAX;
    }
    const int64_t age_us = esp_timer_get_time() - timestamp_us;
    if (age_us <= 0) {
        return 0;
    }
    const int64_t age_ms = age_us / 1000;
    return age_ms > UINT32_MAX ? UINT32_MAX : (uint32_t)age_ms;
}

static void update_heap_stats(resource_monitor_snapshot_t *snapshot)
{
    snapshot->heap_free_bytes = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    snapshot->heap_total_bytes = heap_caps_get_total_size(MALLOC_CAP_8BIT);
    snapshot->heap_min_free_bytes =
        heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
    snapshot->heap_largest_free_block_bytes =
        heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);

    snapshot->internal_free_bytes =
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    snapshot->internal_total_bytes =
        heap_caps_get_total_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    snapshot->internal_min_free_bytes =
        heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    snapshot->internal_largest_free_block_bytes =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    snapshot->psram_free_bytes =
        heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    snapshot->psram_total_bytes =
        heap_caps_get_total_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    snapshot->psram_min_free_bytes =
        heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    snapshot->psram_largest_free_block_bytes =
        heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

static void update_flash_stats(resource_monitor_snapshot_t *snapshot)
{
    if (!s_flash_stats_cached) {
        uint32_t flash_size = 0;
        s_flash_error = esp_flash_get_size(NULL, &flash_size);
        if (s_flash_error == ESP_OK && flash_size > 0U) {
            size_t reserved = 0;
            uint32_t partition_count = 0;

            for (esp_partition_iterator_t it =
                     esp_partition_find(ESP_PARTITION_TYPE_ANY,
                                        ESP_PARTITION_SUBTYPE_ANY, NULL);
                 it != NULL; it = esp_partition_next(it)) {
                const esp_partition_t *partition = esp_partition_get(it);
                if (partition != NULL) {
                    reserved += partition->size;
                    ++partition_count;
                }
            }

            s_flash_total_bytes = (size_t)flash_size;
            s_flash_reserved_bytes = reserved;
            s_flash_free_bytes =
                reserved < s_flash_total_bytes ? s_flash_total_bytes - reserved
                                               : 0;
            s_flash_partition_count = partition_count;
            s_flash_stats_valid = true;
        } else {
            s_flash_total_bytes = 0;
            s_flash_reserved_bytes = 0;
            s_flash_free_bytes = 0;
            s_flash_partition_count = 0;
            s_flash_stats_valid = false;
        }
        s_flash_stats_cached = true;
    }

    snapshot->flash_total_bytes = s_flash_total_bytes;
    snapshot->flash_reserved_bytes = s_flash_reserved_bytes;
    snapshot->flash_free_bytes = s_flash_free_bytes;
    snapshot->flash_partition_count = s_flash_partition_count;
    snapshot->flash_valid = s_flash_stats_valid;
    snapshot->flash_error = s_flash_error;
}

static void update_temperature(resource_monitor_snapshot_t *snapshot)
{
    snapshot->temperature_valid = false;
    snapshot->temperature_c = 0.0f;
    snapshot->temperature_error = ESP_ERR_NOT_SUPPORTED;

#if SOC_TEMP_SENSOR_SUPPORTED
    if (s_temp_sensor == NULL) {
        return;
    }
    float temp_c = 0.0f;
    const esp_err_t err = temperature_sensor_get_celsius(s_temp_sensor, &temp_c);
    snapshot->temperature_error = err;
    if (err == ESP_OK) {
        snapshot->temperature_valid = true;
        snapshot->temperature_c = temp_c;
    }
#endif
}

static void update_cpu_load(resource_monitor_snapshot_t *snapshot)
{
    static bool have_previous;
    static int64_t previous_time_us;
    static uint64_t previous_idle[2];

    snapshot->cpu_load_valid = false;
    snapshot->core0_load_percent = 0.0f;
    snapshot->core1_load_percent = 0.0f;

#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
    const int64_t now_us = esp_timer_get_time();
    const uint64_t idle0 = (uint64_t)ulTaskGetIdleRunTimeCounterForCore(0);
    const uint64_t idle1 =
#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
        (uint64_t)ulTaskGetIdleRunTimeCounterForCore(1);
#else
        0;
#endif

    if (have_previous && now_us > previous_time_us) {
        const uint64_t elapsed_us = (uint64_t)(now_us - previous_time_us);
        const uint64_t idle_delta0 = idle0 - previous_idle[0];
        const uint64_t idle_delta1 = idle1 - previous_idle[1];
        const float idle0_pct =
            elapsed_us > 0 ? (100.0f * (float)idle_delta0) / (float)elapsed_us
                           : 0.0f;
        const float idle1_pct =
            elapsed_us > 0 ? (100.0f * (float)idle_delta1) / (float)elapsed_us
                           : 0.0f;
        snapshot->core0_load_percent =
            idle0_pct >= 100.0f ? 0.0f : 100.0f - idle0_pct;
#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
        snapshot->core1_load_percent =
            idle1_pct >= 100.0f ? 0.0f : 100.0f - idle1_pct;
#else
        snapshot->core1_load_percent = 0.0f;
#endif
        if (snapshot->core0_load_percent < 0.0f) {
            snapshot->core0_load_percent = 0.0f;
        }
        if (snapshot->core1_load_percent < 0.0f) {
            snapshot->core1_load_percent = 0.0f;
        }
        snapshot->cpu_load_valid = true;
    }

    previous_time_us = now_us;
    previous_idle[0] = idle0;
    previous_idle[1] = idle1;
    have_previous = true;
#endif
}

static void copy_task_name(char *dest, size_t dest_size, const char *src)
{
    if (dest == NULL || dest_size == 0) {
        return;
    }
    if (src == NULL) {
        src = "?";
    }
    size_t i = 0;
    for (; i + 1 < dest_size && src[i] != '\0'; ++i) {
        dest[i] = src[i];
    }
    dest[i] = '\0';
}

static esp_err_t init_task_sampling_buffers(void)
{
    if (s_current_tasks != NULL && s_previous_tasks != NULL) {
        return ESP_OK;
    }

    const size_t bytes =
        RESOURCE_MONITOR_MAX_TRACKED_TASKS * sizeof(task_runtime_record_t);
    s_current_tasks = heap_caps_calloc(
        RESOURCE_MONITOR_MAX_TRACKED_TASKS, sizeof(task_runtime_record_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_previous_tasks = heap_caps_calloc(
        RESOURCE_MONITOR_MAX_TRACKED_TASKS, sizeof(task_runtime_record_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_current_tasks == NULL || s_previous_tasks == NULL) {
        if (s_current_tasks != NULL) {
            heap_caps_free(s_current_tasks);
            s_current_tasks = NULL;
        }
        if (s_previous_tasks != NULL) {
            heap_caps_free(s_previous_tasks);
            s_previous_tasks = NULL;
        }
        s_task_sample_capacity = 0;
        ESP_LOGW(TAG, "Task load buffers unavailable in PSRAM (%u bytes)",
                 (unsigned)(bytes * 2U));
        return ESP_ERR_NO_MEM;
    }

    s_task_sample_capacity = RESOURCE_MONITOR_MAX_TRACKED_TASKS;
    ESP_LOGI(TAG, "Task load buffers allocated in PSRAM: %u bytes",
             (unsigned)(bytes * 2U));
    return ESP_OK;
}

static const task_runtime_record_t *find_previous_task(TaskHandle_t handle)
{
    for (UBaseType_t i = 0; i < s_previous_task_count; ++i) {
        if (s_previous_tasks[i].handle == handle) {
            return &s_previous_tasks[i];
        }
    }
    return NULL;
}

static void store_previous_tasks(const task_runtime_record_t *tasks, UBaseType_t count,
                                 int64_t timestamp_us)
{
    const UBaseType_t limit =
        count > RESOURCE_MONITOR_MAX_TRACKED_TASKS
            ? RESOURCE_MONITOR_MAX_TRACKED_TASKS
            : count;
    for (UBaseType_t i = 0; i < limit; ++i) {
        s_previous_tasks[i] = tasks[i];
    }
    s_previous_task_count = limit;
    s_previous_task_time_us = timestamp_us;
    s_previous_tasks_valid = true;
}

static bool is_idle_task_name(const char *name)
{
    return name != NULL &&
           (strncmp(name, "IDLE", 4) == 0 || strncmp(name, "idle", 4) == 0);
}

static void insert_top_task(resource_monitor_snapshot_t *snapshot,
                            const task_runtime_record_t *task,
                            uint64_t delta_us,
                            float load_percent)
{
    if (delta_us == 0 || is_idle_task_name(task->name)) {
        return;
    }

    size_t insert_at = 0;
    while (insert_at < snapshot->top_task_count &&
           delta_us <= snapshot->top_tasks[insert_at].runtime_delta_us) {
        ++insert_at;
    }
    if (insert_at >= RESOURCE_MONITOR_TOP_TASK_COUNT) {
        return;
    }

    if (snapshot->top_task_count < RESOURCE_MONITOR_TOP_TASK_COUNT) {
        ++snapshot->top_task_count;
    }
    for (size_t i = snapshot->top_task_count - 1; i > insert_at; --i) {
        snapshot->top_tasks[i] = snapshot->top_tasks[i - 1];
    }

    resource_monitor_task_load_t *entry = &snapshot->top_tasks[insert_at];
    copy_task_name(entry->name, sizeof(entry->name), task->name);
    entry->runtime_delta_us = delta_us;
    entry->load_percent =
        load_percent < 0.0f ? 0.0f
                            : (load_percent > 100.0f ? 100.0f : load_percent);
    entry->core_id = task->core_id;
}

static UBaseType_t collect_task_samples(task_runtime_record_t *tasks,
                                        UBaseType_t capacity)
{
    UBaseType_t count = 0;

#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS && CONFIG_FREERTOS_USE_TRACE_FACILITY
    TaskIterator_t iterator = {0};

    vTaskSuspendAll();
    while (xTaskGetNext(&iterator) != -1) {
        const TaskHandle_t handle = iterator.pxTaskHandle;
        if (handle == NULL) {
            continue;
        }
        if (count >= capacity) {
            break;
        }

        task_runtime_record_t *sample = &tasks[count++];
        TaskStatus_t status = {0};
        vTaskGetInfo(handle, &status, pdFALSE, eReady);
        sample->handle = handle;
        sample->runtime_counter = (uint64_t)status.ulRunTimeCounter;
#if configTASKLIST_INCLUDE_COREID == 1
        sample->core_id = status.xCoreID;
#else
        sample->core_id = -1;
#endif
        copy_task_name(sample->name, sizeof(sample->name), status.pcTaskName);
    }
    (void)xTaskResumeAll();
#else
    (void)tasks;
    (void)capacity;
#endif

    return count;
}

static void update_task_load(resource_monitor_snapshot_t *snapshot)
{
    snapshot->task_load_valid = false;
    snapshot->task_list_overflow = false;
    snapshot->top_task_count = 0;
    memset(snapshot->top_tasks, 0, sizeof(snapshot->top_tasks));

#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS && CONFIG_FREERTOS_USE_TRACE_FACILITY
    if (s_current_tasks == NULL || s_previous_tasks == NULL ||
        s_task_sample_capacity == 0) {
        snapshot->task_list_overflow = true;
        return;
    }

    const UBaseType_t count = collect_task_samples(
        s_current_tasks, s_task_sample_capacity);
    const int64_t now_us = esp_timer_get_time();
    if (count == 0) {
        return;
    }

    if (s_previous_tasks_valid && now_us > s_previous_task_time_us) {
        const uint64_t elapsed_us =
            (uint64_t)(now_us - s_previous_task_time_us);
        for (UBaseType_t i = 0; i < count; ++i) {
            const task_runtime_record_t *previous =
                find_previous_task(s_current_tasks[i].handle);
            if (previous == NULL) {
                continue;
            }
            const uint64_t current =
                s_current_tasks[i].runtime_counter;
            if (current < previous->runtime_counter) {
                continue;
            }
            const uint64_t delta_us = current - previous->runtime_counter;
            const float load_percent =
                elapsed_us > 0
                    ? (100.0f * (float)delta_us) / (float)elapsed_us
                    : 0.0f;
            insert_top_task(snapshot, &s_current_tasks[i], delta_us,
                            load_percent);
        }
        snapshot->task_load_valid = true;
    }

    store_previous_tasks(s_current_tasks, count, now_us);
#endif
}

static esp_err_t init_temperature_sensor(void)
{
#if SOC_TEMP_SENSOR_SUPPORTED
    temperature_sensor_config_t temp_config =
        TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    esp_err_t err = temperature_sensor_install(&temp_config, &s_temp_sensor);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Temperature sensor install failed: %s",
                 esp_err_to_name(err));
        s_temp_sensor = NULL;
        return err;
    }
    err = temperature_sensor_enable(s_temp_sensor);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Temperature sensor enable failed: %s",
                 esp_err_to_name(err));
        (void)temperature_sensor_uninstall(s_temp_sensor);
        s_temp_sensor = NULL;
        return err;
    }
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

static void publish_snapshot(const resource_monitor_snapshot_t *snapshot)
{
    portENTER_CRITICAL(&s_lock);
    s_snapshot = *snapshot;
    s_last_update_us = esp_timer_get_time();
    portEXIT_CRITICAL(&s_lock);
}

static void resource_monitor_task(void *arg)
{
    (void)arg;
    resource_monitor_snapshot_t local = {0};

    while (true) {
        local.running = true;
        local.update_count++;
        update_heap_stats(&local);
        update_flash_stats(&local);
        update_cpu_load(&local);
        update_task_load(&local);
        update_temperature(&local);
        local.last_update_age_ms = 0;
        publish_snapshot(&local);
        vTaskDelay(pdMS_TO_TICKS(RESOURCE_MONITOR_SAMPLE_MS));
    }
}

esp_err_t resource_monitor_service_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    (void)init_temperature_sensor();
    (void)init_task_sampling_buffers();

    /*
     * Keep this stack internal. The task iterator in xTaskGetNext() walks
     * FreeRTOS list internals and validates pointers; with the iterator and
     * call frames on PSRAM-backed stack, M1 reproduced a CPU0 task watchdog
     * lockup inside collect_task_samples().
     */
    const BaseType_t created = xTaskCreatePinnedToCore(
        resource_monitor_task, "resource_mon",
        RESOURCE_MONITOR_TASK_STACK_WORDS, NULL,
        RESOURCE_MONITOR_TASK_PRIORITY, NULL, RESOURCE_MONITOR_TASK_CORE);
    if (created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    ESP_LOGI(TAG, "Resource monitor started: interval=%ums core=%d stack=internal",
             (unsigned)RESOURCE_MONITOR_SAMPLE_MS, RESOURCE_MONITOR_TASK_CORE);
    return ESP_OK;
}

void resource_monitor_service_get_snapshot(resource_monitor_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_lock);
    *snapshot = s_snapshot;
    const int64_t last_update_us = s_last_update_us;
    portEXIT_CRITICAL(&s_lock);
    snapshot->last_update_age_ms = age_ms_from_time_us(last_update_us);
}
