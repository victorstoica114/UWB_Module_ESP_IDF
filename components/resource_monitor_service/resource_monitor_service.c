#include "resource_monitor_service.h"

#include <string.h>

#include "driver/temperature_sensor.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "soc/soc_caps.h"

static const char *TAG = "resource_monitor";

enum {
    RESOURCE_MONITOR_TASK_STACK_WORDS = 3072,
    RESOURCE_MONITOR_TASK_PRIORITY = 1,
    RESOURCE_MONITOR_SAMPLE_MS = 1000,
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
        update_cpu_load(&local);
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

    const BaseType_t created = xTaskCreatePinnedToCore(
        resource_monitor_task, "resource_mon",
        RESOURCE_MONITOR_TASK_STACK_WORDS, NULL,
        RESOURCE_MONITOR_TASK_PRIORITY, NULL, RESOURCE_MONITOR_TASK_CORE);
    if (created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    ESP_LOGI(TAG, "Resource monitor started: interval=%ums core=%d",
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
