#include "bno085_service.h"
#include "bno085_timing.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "app_config.h"
#include "app_runtime_config.h"
#include "board_config.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus_service.h"
#include "wireless_telemetry_service.h"
#include "sdkconfig.h"

static const char *TAG = "bno085_service";

enum {
    BNO085_I2C_PORT = 0,
    /* ESP-IDF task stack sizes are bytes.  Startup/reset and I2C error
     * handling can nest deeply enough to exhaust the former 4 KiB stack. */
    BNO085_TASK_STACK_BYTES = 8192,
    BNO085_TASK_PRIORITY = 5,
    BNO085_SHTP_HEADER_LEN = 4,
    BNO085_MAX_PACKET_LEN = 512,
    BNO085_READ_TIMEOUT_MS = 20,
    BNO085_WRITE_TIMEOUT_MS = 100,
    BNO085_STARTUP_DRAIN_MS = 500,
    BNO085_RESET_SETTLE_MS = 5,
    BNO085_RESET_PULSE_MS = 20,
    BNO085_BOOT_AFTER_RESET_MS = 800,
    BNO085_MAX_PACKETS_PER_WAKE = 16,
    BNO085_HIGH_RATE_INTERVAL_MS = 10,
    BNO085_HIGH_RATE_DRAIN_BUDGET_MS = 8,
    BNO085_STALL_RECOVERY_MS = 5000,
    BNO085_ACCEL_REPORT_LEN = 10,
    BNO085_TIMEBASE_REPORT_LEN = 5,
    BNO085_REBASE_REPORT_LEN = 5,
    BNO085_CHANNEL_CONTROL = 2,
    BNO085_CHANNEL_EXECUTABLE = 1,
    BNO085_CHANNEL_INPUT_REPORTS = 3,
    BNO085_CHANNEL_GYRO_RV = 5,
    BNO085_EXECUTABLE_RESET = 0x01,
    BNO085_REPORT_SET_FEATURE = 0xFD,
    BNO085_REPORT_GET_FEATURE_RESPONSE = 0xFC,
    BNO085_REPORT_TIMEBASE = 0xFB,
    BNO085_REPORT_REBASE = 0xFA,
    BNO085_REPORT_ACCELEROMETER = 0x01,
    BNO085_REPORT_GYRO_RV = 0x2A,
    BNO085_GYRO_RV_REPORT_LEN = 14,
    /* Leave enough SH2 processing budget for the 500 Hz accelerometer.  A
     * 50 Hz orientation update is still inside the dashboard's 40 ms merge
     * window, while 100 Hz reduced measured acceleration output to ~460 Hz. */
    BNO085_GYRO_RV_RATE_HZ = 50,
    BNO085_GYRO_RV_INTERVAL_US = 1000000U / BNO085_GYRO_RV_RATE_HZ,
    BNO085_ACCEL_Q_POINT = 8,
    BNO085_NOTIFY_INT = 1U << 0,
    BNO085_NOTIFY_CONFIG = 1U << 1,
    BNO085_NOTIFY_STOP = 1U << 2,
    BNO085_TELEMETRY_RATE_HZ = 500,
    BNO085_CLOCK_ANCHOR_RATE_HZ = 1,
    /* Zero requests immediate SH2 delivery.  This is the field-validated
     * 500 Hz profile; a 10 ms SH2 batch reduced the observed rate to about
     * 455 Hz even though the requested sample interval remained 2 ms. */
    BNO085_HIGH_RATE_BATCH_INTERVAL_US = 0,
    BNO085_SAMPLE_RING_PSRAM_CAPACITY = 512,
    BNO085_SAMPLE_RING_INTERNAL_CAPACITY = 128,
    BNO085_GYRO_RV_RING_CAPACITY = 256,
    BNO085_COPY_MAX_SAMPLES = 32,
    /* Estimate sensor-clock tolerance over about one second so I2C/SHTP
     * packetization jitter cannot modulate individual sample timestamps. */
    BNO085_TIMESTAMP_TRACK_MIN_REPORTS = 256,
};

#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
#define BNO085_TASK_CORE 0
#else
#define BNO085_TASK_CORE 0
#endif

static bool s_service_started;
static bool s_int_irq_enabled;
static volatile bool s_int_irq_armed;
static TaskHandle_t s_task_handle;
static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_i2c_dev;
static uint8_t s_shtp_sequence[6];
static uint32_t s_report_count;
static uint32_t s_packet_count;
static uint32_t s_input_packet_count;
static uint32_t s_timebase_count;
static uint32_t s_max_reports_per_packet;
static uint32_t s_continuation_packet_count;
static uint32_t s_continuation_transfer_count;
static uint32_t s_continuation_header_error_count;
static uint32_t s_high_rate_poll_count;
static uint32_t s_wait_immediate_count;
static uint32_t s_wait_notify_count;
static uint32_t s_wait_late_active_count;
static uint32_t s_null_header_count;
static uint32_t s_read_error_count;
static uint32_t s_parse_error_count;
static volatile uint32_t s_int_irq_count;
static uint32_t s_int_wait_timeout_count;
static float s_last_x_mps2;
static float s_last_y_mps2;
static float s_last_z_mps2;
static uint8_t s_last_accuracy;
static uint32_t s_last_log_ms;
static uint32_t s_last_read_warning_ms;
static uint32_t s_configured_accel_interval_ms;
static uint32_t s_effective_accel_period_us;
static uint32_t s_last_reconfigure_warning_ms;
static uint32_t s_last_progress_ms;
static uint32_t s_last_progress_report_count;
static uint32_t s_last_stall_recovery_ms;
static size_t s_last_packet_len;
static size_t s_last_input_payload_len;
static int s_i2c_scl_measure_error = ESP_ERR_NOT_SUPPORTED;
static uint32_t s_i2c_scl_edges;
static uint32_t s_i2c_scl_elapsed_us;
static uint32_t s_i2c_scl_measured_hz;
static uint32_t s_i2c_scl_measure_count;
static gptimer_handle_t s_fusion_timer;
static bno085_accel_sample_t *s_sample_ring;
static size_t s_sample_ring_capacity;
static size_t s_sample_ring_count;
static size_t s_sample_ring_write;
static uint32_t s_sample_overwrite_count;
static uint32_t s_telemetry_submit_count;
static uint32_t s_telemetry_drop_count;
static uint32_t s_telemetry_decimated_count;
static uint32_t s_monotonic_repair_count;
static uint64_t s_last_fusion_time_ticks;
static uint64_t s_last_telemetry_ticks;
static uint64_t s_last_clock_anchor_ticks;
static uint32_t s_timestamp_period_ticks;
static uint64_t s_timestamp_anchor_hint_ticks;
static uint32_t s_timestamp_anchor_report_count;
static bool s_batch_timebase_valid;
static uint64_t s_batch_hint_ticks;
static int32_t s_batch_base_delta_100us;
static int32_t s_batch_rebase_delta_100us;
static portMUX_TYPE s_sample_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile uint64_t s_last_hint_ticks;
static volatile uint32_t s_hint_generation;
static uint32_t s_consumed_hint_generation;
static portMUX_TYPE s_hint_lock = portMUX_INITIALIZER_UNLOCKED;
static bno085_gyro_rv_sample_t *s_gyro_rv_ring;
static size_t s_gyro_rv_ring_count;
static size_t s_gyro_rv_ring_write;
static uint32_t s_gyro_rv_report_count;
static uint32_t s_gyro_rv_overwrite_count;

static esp_err_t bno085_hold_in_reset(void);
static bool bno085_drain_startup_packets(void);
static esp_err_t bno085_rebind_i2c_device(void);

static void bno085_notify_task(uint32_t bits)
{
    const TaskHandle_t task = s_task_handle;
    if (task != NULL) {
        (void)xTaskNotify(task, bits, eSetBits);
    }
}

static void bno085_release_i2c_device(void)
{
    if (s_i2c_dev != NULL) {
        (void)i2c_master_bus_rm_device(s_i2c_dev);
        s_i2c_dev = NULL;
    }
}

static void bno085_stop_interrupt(void)
{
    if (BOARD_CONFIG_BNO085_INT_GPIO != BOARD_CONFIG_GPIO_UNUSED) {
        (void)gpio_intr_disable(BOARD_CONFIG_BNO085_INT_GPIO);
        (void)gpio_isr_handler_remove(BOARD_CONFIG_BNO085_INT_GPIO);
    }
    s_int_irq_enabled = false;
    s_int_irq_armed = false;
}

static void bno085_task_finish(bool hold_reset)
{
    i2c_bus_service_set_realtime_period_us(0);
    bno085_stop_interrupt();
    bno085_release_i2c_device();
    if (hold_reset) {
        (void)bno085_hold_in_reset();
    }
    s_task_handle = NULL;
    s_service_started = false;
    vTaskDelete(NULL);
}

static uint16_t read_le_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t read_le_i16(const uint8_t *data)
{
    return (int16_t)read_le_u16(data);
}

static uint32_t read_le_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static float q_to_float(int16_t raw, uint8_t q_point)
{
    return (float)raw / (float)(1U << q_point);
}

static int32_t float_to_milli(float value)
{
    return (int32_t)(value * 1000.0f + (value >= 0.0f ? 0.5f : -0.5f));
}

static esp_err_t bno085_fusion_timer_init(void)
{
    if (s_fusion_timer != NULL) {
        return ESP_OK;
    }

    const gptimer_config_t config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = BNO085_FUSION_TIMER_HZ,
    };
    esp_err_t err = gptimer_new_timer(&config, &s_fusion_timer);
    if (err != ESP_OK) {
        return err;
    }
    err = gptimer_enable(s_fusion_timer);
    if (err == ESP_OK) {
        err = gptimer_start(s_fusion_timer);
    }
    if (err != ESP_OK) {
        (void)gptimer_disable(s_fusion_timer);
        (void)gptimer_del_timer(s_fusion_timer);
        s_fusion_timer = NULL;
        return err;
    }
    ESP_LOGI(TAG, "BNO085 fusion clock started at %u Hz",
             (unsigned)BNO085_FUSION_TIMER_HZ);
    return ESP_OK;
}

uint64_t bno085_service_fusion_time_ticks(void)
{
    uint64_t ticks = 0;
    if (s_fusion_timer != NULL) {
        (void)gptimer_get_raw_count(s_fusion_timer, &ticks);
    }
    return ticks;
}

static esp_err_t bno085_sample_ring_init(void)
{
    if (s_sample_ring != NULL && s_gyro_rv_ring != NULL) {
        return ESP_OK;
    }

    if (s_sample_ring == NULL) {
        s_sample_ring = heap_caps_calloc(BNO085_SAMPLE_RING_PSRAM_CAPACITY,
                                         sizeof(*s_sample_ring),
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_sample_ring != NULL) {
            s_sample_ring_capacity = BNO085_SAMPLE_RING_PSRAM_CAPACITY;
        } else {
            s_sample_ring = heap_caps_calloc(
                BNO085_SAMPLE_RING_INTERNAL_CAPACITY, sizeof(*s_sample_ring),
                MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            s_sample_ring_capacity =
                s_sample_ring != NULL ? BNO085_SAMPLE_RING_INTERNAL_CAPACITY
                                      : 0;
        }
    }
    if (s_sample_ring == NULL) {
        return ESP_ERR_NO_MEM;
    }

    if (s_gyro_rv_ring == NULL) {
        s_gyro_rv_ring = heap_caps_calloc(
            BNO085_GYRO_RV_RING_CAPACITY, sizeof(*s_gyro_rv_ring),
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_gyro_rv_ring == NULL) {
            s_gyro_rv_ring = heap_caps_calloc(
                BNO085_GYRO_RV_RING_CAPACITY, sizeof(*s_gyro_rv_ring),
                MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        }
    }
    if (s_gyro_rv_ring == NULL) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "BNO085 local sample ring ready: %u samples (%s)",
             (unsigned)s_sample_ring_capacity,
             s_sample_ring_capacity == BNO085_SAMPLE_RING_PSRAM_CAPACITY
                 ? "PSRAM"
                 : "internal RAM");
    ESP_LOGI(TAG, "BNO085 GyroRV ring ready: %u samples",
             (unsigned)BNO085_GYRO_RV_RING_CAPACITY);
    return ESP_OK;
}

static void bno085_store_sample(const bno085_accel_sample_t *sample)
{
    if (sample == NULL || s_sample_ring == NULL ||
        s_sample_ring_capacity == 0) {
        return;
    }

    portENTER_CRITICAL(&s_sample_lock);
    s_last_fusion_time_ticks = sample->fusion_time_ticks;
    s_sample_ring[s_sample_ring_write] = *sample;
    s_sample_ring_write = (s_sample_ring_write + 1U) % s_sample_ring_capacity;
    if (s_sample_ring_count < s_sample_ring_capacity) {
        s_sample_ring_count++;
    } else {
        s_sample_overwrite_count++;
    }
    portEXIT_CRITICAL(&s_sample_lock);
}

static void bno085_store_gyro_rv_sample(
    const bno085_gyro_rv_sample_t *sample)
{
    if (sample == NULL || s_gyro_rv_ring == NULL) {
        return;
    }

    portENTER_CRITICAL(&s_sample_lock);
    s_gyro_rv_ring[s_gyro_rv_ring_write] = *sample;
    s_gyro_rv_ring_write =
        (s_gyro_rv_ring_write + 1U) % BNO085_GYRO_RV_RING_CAPACITY;
    if (s_gyro_rv_ring_count < BNO085_GYRO_RV_RING_CAPACITY) {
        s_gyro_rv_ring_count++;
    } else {
        s_gyro_rv_overwrite_count++;
    }
    portEXIT_CRITICAL(&s_sample_lock);
}

static uint32_t ticks_to_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static TickType_t ms_to_ticks_min_1(uint32_t delay_ms)
{
    TickType_t ticks = pdMS_TO_TICKS(delay_ms);
    if (ticks == 0) {
        ticks = 1;
    }
    return ticks;
}

static int bno085_reset_release_level(void)
{
    return BOARD_CONFIG_BNO085_RST_ACTIVE_LEVEL ? 0 : 1;
}

static uint32_t bno085_accel_interval_ms(void)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    if (config != NULL && config->bno085_accel_interval_ms > 0) {
        return config->bno085_accel_interval_ms;
    }
    return APP_BNO085_ACCEL_INTERVAL_MS;
}

static uint32_t bno085_requested_accel_period_us(void)
{
    const uint32_t interval_ms = bno085_accel_interval_ms();
    if (interval_ms > UINT32_MAX / 1000U) {
        return UINT32_MAX;
    }
    return interval_ms * 1000U;
}

static uint32_t bno085_accel_period_us(void)
{
    return s_effective_accel_period_us > 0
               ? s_effective_accel_period_us
               : bno085_requested_accel_period_us();
}

static void bno085_update_i2c_realtime_period(void)
{
    const uint32_t sample_period_us = bno085_accel_period_us();
    /* Immediate SH2 delivery means the shared-bus scheduler must reserve the
     * actual sample cadence.  The same 2 ms reservation was previously
     * validated with BQ25792 and MAX77958 active on all five modules. */
    i2c_bus_service_set_realtime_period_us(sample_period_us);
}

static uint32_t bno085_log_interval_ms(void)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    if (config != NULL && config->bno085_log_interval_ms > 0) {
        return config->bno085_log_interval_ms;
    }
    return APP_BNO085_LOG_INTERVAL_MS;
}

static bool bno085_high_rate_mode(void)
{
    return bno085_accel_interval_ms() <= BNO085_HIGH_RATE_INTERVAL_MS;
}

static uint32_t bno085_int_wait_timeout_ms(void)
{
    uint32_t timeout_ms = APP_BNO085_INT_WAIT_TIMEOUT_MS;
    if (bno085_high_rate_mode()) {
        const uint32_t interval_ms = bno085_accel_interval_ms();
        const uint32_t high_rate_timeout_ms = interval_ms + 2U;
        if (timeout_ms > high_rate_timeout_ms) {
            timeout_ms = high_rate_timeout_ms;
        }
    }
    return timeout_ms;
}

static esp_err_t bno085_hold_in_reset(void)
{
    const gpio_config_t reset_config = {
        .pin_bit_mask = 1ULL << BOARD_CONFIG_BNO085_RST_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&reset_config), TAG,
                        "BNO085 reset GPIO init failed");
    ESP_RETURN_ON_ERROR(
        gpio_set_level(BOARD_CONFIG_BNO085_RST_GPIO,
                       BOARD_CONFIG_BNO085_RST_ACTIVE_LEVEL),
        TAG, "BNO085 reset hold failed");
    ESP_LOGI(TAG, "BNO085 held in reset on GPIO%d active-%s",
             BOARD_CONFIG_BNO085_RST_GPIO,
             BOARD_CONFIG_BNO085_RST_ACTIVE_LEVEL ? "high" : "low");
    return ESP_OK;
}

static bool bno085_int_active(void)
{
    return gpio_get_level(BOARD_CONFIG_BNO085_INT_GPIO) ==
           BOARD_CONFIG_BNO085_INT_ACTIVE_LEVEL;
}

static void IRAM_ATTR bno085_int_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t higher_priority_task_woken = pdFALSE;
    uint64_t hint_ticks = 0;
    if (s_fusion_timer != NULL &&
        gptimer_get_raw_count(s_fusion_timer, &hint_ticks) == ESP_OK) {
        portENTER_CRITICAL_ISR(&s_hint_lock);
        s_last_hint_ticks = hint_ticks;
        s_hint_generation++;
        portEXIT_CRITICAL_ISR(&s_hint_lock);
    }
    s_int_irq_count++;
    /* H_INTN is level-low on I2C and deasserts as soon as the address is seen;
       mask it until the task drains the SHTP packet and waits again. */
    s_int_irq_armed = false;
    (void)gpio_intr_disable(BOARD_CONFIG_BNO085_INT_GPIO);
    if (s_task_handle != NULL) {
        (void)xTaskNotifyFromISR(s_task_handle, BNO085_NOTIFY_INT, eSetBits,
                                 &higher_priority_task_woken);
    }
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t bno085_configure_host_gpios(void)
{
    const gpio_config_t reset_config = {
        .pin_bit_mask = 1ULL << BOARD_CONFIG_BNO085_RST_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&reset_config);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_set_level(BOARD_CONFIG_BNO085_RST_GPIO,
                         bno085_reset_release_level());
    if (err != ESP_OK) {
        return err;
    }

    const gpio_config_t int_config = {
        .pin_bit_mask = 1ULL << BOARD_CONFIG_BNO085_INT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&int_config);
}

static esp_err_t bno085_configure_host_interrupt(void)
{
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "BNO085 INT service unavailable: %s",
                 esp_err_to_name(err));
        return err;
    }

    (void)gpio_intr_disable(BOARD_CONFIG_BNO085_INT_GPIO);
    (void)gpio_isr_handler_remove(BOARD_CONFIG_BNO085_INT_GPIO);

    err = gpio_set_intr_type(BOARD_CONFIG_BNO085_INT_GPIO,
                             BOARD_CONFIG_BNO085_INT_ACTIVE_LEVEL
                                 ? GPIO_INTR_HIGH_LEVEL
                                 : GPIO_INTR_LOW_LEVEL);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_isr_handler_add(BOARD_CONFIG_BNO085_INT_GPIO,
                               bno085_int_isr_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BNO085 INT handler unavailable: %s",
                 esp_err_to_name(err));
        return err;
    }

    s_int_irq_enabled = true;
    s_int_irq_armed = true;
    err = gpio_intr_enable(BOARD_CONFIG_BNO085_INT_GPIO);
    if (err != ESP_OK) {
        s_int_irq_enabled = false;
        s_int_irq_armed = false;
        return err;
    }

    ESP_LOGI(TAG,
             "BNO085 INT enabled on GPIO%d active-%s level fallback_timeout=%u ms",
             BOARD_CONFIG_BNO085_INT_GPIO,
             BOARD_CONFIG_BNO085_INT_ACTIVE_LEVEL ? "high" : "low",
             (unsigned)APP_BNO085_INT_WAIT_TIMEOUT_MS);
    return ESP_OK;
}

static void bno085_arm_interrupt_if_needed(void)
{
    if (!s_int_irq_enabled || s_int_irq_armed) {
        return;
    }

    if (gpio_intr_enable(BOARD_CONFIG_BNO085_INT_GPIO) == ESP_OK) {
        s_int_irq_armed = true;
    }
}

static esp_err_t bno085_hard_reset(void)
{
    esp_err_t err = gpio_set_level(BOARD_CONFIG_BNO085_RST_GPIO,
                                   bno085_reset_release_level());
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(BNO085_RESET_SETTLE_MS));

    err = gpio_set_level(BOARD_CONFIG_BNO085_RST_GPIO,
                         BOARD_CONFIG_BNO085_RST_ACTIVE_LEVEL);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(BNO085_RESET_PULSE_MS));

    err = gpio_set_level(BOARD_CONFIG_BNO085_RST_GPIO,
                         bno085_reset_release_level());
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(BNO085_BOOT_AFTER_RESET_MS));

    memset(s_shtp_sequence, 0, sizeof(s_shtp_sequence));

    ESP_LOGI(TAG,
             "BNO085 hard reset pulse sent on GPIO%d active-%s, INT GPIO%d level=%d",
             BOARD_CONFIG_BNO085_RST_GPIO,
             BOARD_CONFIG_BNO085_RST_ACTIVE_LEVEL ? "high" : "low",
             BOARD_CONFIG_BNO085_INT_GPIO,
             gpio_get_level(BOARD_CONFIG_BNO085_INT_GPIO));
    return ESP_OK;
}

static void bno085_record_scl_measure(
    const i2c_bus_service_scl_measure_result_t *result)
{
    if (result == NULL) {
        return;
    }
    s_i2c_scl_measure_error = result->error;
    s_i2c_scl_edges = result->edges;
    s_i2c_scl_elapsed_us = result->elapsed_us;
    s_i2c_scl_measured_hz = result->measured_hz;
}

static esp_err_t bno085_receive_measured(uint8_t *data, size_t len,
                                         int timeout_ms)
{
    i2c_bus_service_scl_measure_t measure = {0};
    esp_err_t measure_err = ESP_ERR_NOT_SUPPORTED;
    const bool should_measure =
        APP_BNO085_I2C_MEASURE_SCL_ENABLED &&
        s_i2c_scl_measure_count < APP_BNO085_I2C_MEASURE_MAX_SAMPLES;
    if (should_measure) {
        measure_err = i2c_bus_service_scl_measure_start(
            BOARD_CONFIG_BNO085_SCL_GPIO, &measure);
    }

    const int64_t start_us = esp_timer_get_time();
    const esp_err_t err = i2c_master_receive(s_i2c_dev, data, len, timeout_ms);
    const int64_t elapsed_raw = esp_timer_get_time() - start_us;
    const uint32_t elapsed_us =
        elapsed_raw > 0 ? (uint32_t)elapsed_raw : 0U;

    if (should_measure) {
        i2c_bus_service_scl_measure_result_t result = {
            .error = measure_err,
        };
        if (measure.unit != NULL) {
            result.error = i2c_bus_service_scl_measure_stop(
                &measure, elapsed_us, &result);
        }
        bno085_record_scl_measure(&result);
        s_i2c_scl_measure_count++;
    }

    return err;
}

static esp_err_t bno085_read_packet(uint8_t *packet, size_t packet_size,
                                    size_t *packet_len)
{
    uint8_t header[BNO085_SHTP_HEADER_LEN] = {0};
    if (!i2c_bus_service_lock_realtime(pdMS_TO_TICKS(BNO085_READ_TIMEOUT_MS))) {
        return ESP_ERR_TIMEOUT;
    }
    i2c_bus_service_note_realtime_activity();

    esp_err_t err = bno085_receive_measured(
        header, sizeof(header), BNO085_READ_TIMEOUT_MS);
    if (err != ESP_OK) {
        i2c_bus_service_unlock();
        return err;
    }

    const uint16_t raw_len = read_le_u16(header);
    if (raw_len == 0 || raw_len == 0xFFFFU) {
        if (raw_len == 0) {
            s_null_header_count++;
        }
        i2c_bus_service_unlock();
        return ESP_ERR_TIMEOUT;
    }

    const size_t total_len = raw_len & 0x7FFFU;
    if (total_len < BNO085_SHTP_HEADER_LEN) {
        i2c_bus_service_unlock();
        return ESP_ERR_INVALID_SIZE;
    }

    const size_t payload_len = total_len - BNO085_SHTP_HEADER_LEN;

    if (total_len <= packet_size) {
        memcpy(packet, header, sizeof(header));
    }

    if (total_len > packet_size) {
        i2c_bus_service_unlock();
        return ESP_ERR_INVALID_SIZE;
    }

    if ((raw_len & 0x8000U) != 0) {
        s_continuation_packet_count++;
    }

    if (payload_len > 0) {
        uint8_t chunk[BNO085_MAX_PACKET_LEN] = {0};
        err = bno085_receive_measured(
            chunk, payload_len + BNO085_SHTP_HEADER_LEN,
            BNO085_READ_TIMEOUT_MS);
        if (err != ESP_OK) {
            i2c_bus_service_unlock();
            return err;
        }

        s_continuation_transfer_count++;
        const uint16_t continuation_raw_len = read_le_u16(chunk);
        const size_t continuation_len = continuation_raw_len & 0x7FFFU;
        if ((continuation_raw_len & 0x8000U) == 0 ||
            continuation_len != payload_len + BNO085_SHTP_HEADER_LEN ||
            chunk[2] != header[2]) {
            s_continuation_header_error_count++;
        }

        memcpy(&packet[BNO085_SHTP_HEADER_LEN],
               &chunk[BNO085_SHTP_HEADER_LEN], payload_len);
    }

    s_last_packet_len = total_len;
    *packet_len = total_len;
    i2c_bus_service_unlock();
    return ESP_OK;
}

static esp_err_t bno085_send_packet(uint8_t channel, const uint8_t *payload,
                                    size_t payload_len)
{
    if (channel >= sizeof(s_shtp_sequence) ||
        payload_len + BNO085_SHTP_HEADER_LEN > BNO085_MAX_PACKET_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t packet[BNO085_MAX_PACKET_LEN] = {0};
    const size_t total_len = payload_len + BNO085_SHTP_HEADER_LEN;
    packet[0] = (uint8_t)(total_len & 0xFFU);
    packet[1] = (uint8_t)((total_len >> 8) & 0x7FU);
    packet[2] = channel;
    packet[3] = s_shtp_sequence[channel]++;
    memcpy(&packet[BNO085_SHTP_HEADER_LEN], payload, payload_len);

    if (!i2c_bus_service_lock_realtime(pdMS_TO_TICKS(BNO085_WRITE_TIMEOUT_MS))) {
        return ESP_ERR_TIMEOUT;
    }
    const esp_err_t err =
        i2c_master_transmit(s_i2c_dev, packet, total_len,
                            BNO085_WRITE_TIMEOUT_MS);
    i2c_bus_service_unlock();
    return err;
}

static esp_err_t bno085_enable_accelerometer(void)
{
    const uint32_t interval_ms = bno085_accel_interval_ms();
    const uint32_t interval_us = interval_ms * 1000U;
    const uint32_t batch_interval_us =
        interval_ms <= BNO085_HIGH_RATE_INTERVAL_MS
            ? BNO085_HIGH_RATE_BATCH_INTERVAL_US
            : 0U;
    const uint8_t payload[] = {
        BNO085_REPORT_SET_FEATURE,
        BNO085_REPORT_ACCELEROMETER,
        0x00,
        0x00,
        0x00,
        (uint8_t)(interval_us & 0xFFU),
        (uint8_t)((interval_us >> 8) & 0xFFU),
        (uint8_t)((interval_us >> 16) & 0xFFU),
        (uint8_t)((interval_us >> 24) & 0xFFU),
        (uint8_t)(batch_interval_us & 0xFFU),
        (uint8_t)((batch_interval_us >> 8) & 0xFFU),
        (uint8_t)((batch_interval_us >> 16) & 0xFFU),
        (uint8_t)((batch_interval_us >> 24) & 0xFFU),
        0x00,
        0x00,
        0x00,
        0x00,
    };

    const uint32_t previous_effective_period_us =
        s_effective_accel_period_us;
    s_effective_accel_period_us = interval_us;
    const esp_err_t err =
        bno085_send_packet(BNO085_CHANNEL_CONTROL, payload, sizeof(payload));
    if (err == ESP_OK) {
        s_configured_accel_interval_ms = interval_ms;
        bno085_update_i2c_realtime_period();
    } else {
        s_effective_accel_period_us = previous_effective_period_us;
    }
    return err;
}

static esp_err_t bno085_enable_gyro_rv(void)
{
    const uint32_t interval_us = BNO085_GYRO_RV_INTERVAL_US;
    const uint8_t payload[] = {
        BNO085_REPORT_SET_FEATURE,
        BNO085_REPORT_GYRO_RV,
        0x00,
        0x00,
        0x00,
        (uint8_t)(interval_us & 0xFFU),
        (uint8_t)((interval_us >> 8) & 0xFFU),
        (uint8_t)((interval_us >> 16) & 0xFFU),
        (uint8_t)((interval_us >> 24) & 0xFFU),
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
    };
    return bno085_send_packet(BNO085_CHANNEL_CONTROL, payload,
                              sizeof(payload));
}

static void bno085_reconfigure_accelerometer_if_needed(void)
{
    const uint32_t desired_interval_ms = bno085_accel_interval_ms();
    if (desired_interval_ms == s_configured_accel_interval_ms) {
        return;
    }

    const uint32_t previous_interval_ms = s_configured_accel_interval_ms;
    const esp_err_t err = bno085_enable_accelerometer();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "BNO085 accelerometer interval updated: %u ms -> %u ms",
                 (unsigned)previous_interval_ms,
                 (unsigned)s_configured_accel_interval_ms);
        return;
    }

    const uint32_t now_ms = ticks_to_ms();
    if ((uint32_t)(now_ms - s_last_reconfigure_warning_ms) >=
        bno085_log_interval_ms()) {
        s_last_reconfigure_warning_ms = now_ms;
        ESP_LOGW(TAG, "BNO085 accelerometer interval update failed: %s",
                 esp_err_to_name(err));
    }
}

static esp_err_t bno085_soft_reset(void)
{
    const uint8_t payload[] = {BNO085_EXECUTABLE_RESET};
    return bno085_send_packet(BNO085_CHANNEL_EXECUTABLE, payload,
                              sizeof(payload));
}

static void bno085_emit_telemetry(const bno085_accel_sample_t *sample)
{
    /* Forward every report generated by SH2.  The sensor itself enforces its
     * configured/max rate; an additional host-side 500 Hz gate can discard
     * valid reports when the discrete BNO clock is slightly faster than the
     * nominal 2 ms interval. */
    s_last_telemetry_ticks = sample->fusion_time_ticks;
    const uint64_t anchor_period_ticks =
        BNO085_FUSION_TIMER_HZ / BNO085_CLOCK_ANCHOR_RATE_HZ;
    if (s_last_clock_anchor_ticks == 0 ||
        sample->fusion_time_ticks - s_last_clock_anchor_ticks >=
            anchor_period_ticks) {
        const int64_t esp_before_us = esp_timer_get_time();
        uint64_t anchor_ticks = 0;
        const esp_err_t anchor_err =
            gptimer_get_raw_count(s_fusion_timer, &anchor_ticks);
        const int64_t esp_after_us = esp_timer_get_time();
        if (anchor_err == ESP_OK) {
            const uint64_t esp_mid_us =
                (uint64_t)(esp_before_us +
                           (esp_after_us - esp_before_us) / 2);
            (void)wireless_telemetry_service_submit_bno085_clock_anchor(
                anchor_ticks, esp_mid_us);
            s_last_clock_anchor_ticks = anchor_ticks;
        }
    }
    if (wireless_telemetry_service_submit_bno085_accel_compact(
            sample->fusion_time_ticks, sample->sequence,
            sample->x_q8, sample->y_q8, sample->z_q8,
            sample->sensor_delay_100us, sample->accuracy,
            sample->time_flags)) {
        s_telemetry_submit_count++;
    } else {
        s_telemetry_drop_count++;
    }
}

static void bno085_emit_orientation_telemetry(
    const bno085_gyro_rv_sample_t *sample)
{
    if (sample == NULL) {
        return;
    }
    if (wireless_telemetry_service_submit_bno085_orientation(
            sample->fusion_time_ticks, sample->sequence,
            sample->quat_i_q14, sample->quat_j_q14,
            sample->quat_k_q14, sample->quat_real_q14,
            sample->gyro_x_q10, sample->gyro_y_q10,
            sample->gyro_z_q10, sample->time_flags)) {
        s_telemetry_submit_count++;
    } else {
        s_telemetry_drop_count++;
    }
}

static uint64_t bno085_packet_hint_ticks(uint8_t *time_flags,
                                         uint32_t *hint_generation)
{
    uint64_t hint_ticks = 0;
    uint32_t generation = 0;
    portENTER_CRITICAL(&s_hint_lock);
    hint_ticks = s_last_hint_ticks;
    generation = s_hint_generation;
    portEXIT_CRITICAL(&s_hint_lock);

    if (hint_generation != NULL) {
        *hint_generation = 0;
    }

    if (generation != s_consumed_hint_generation && hint_ticks != 0) {
        if (hint_generation != NULL) {
            *hint_generation = generation;
        }
        if (time_flags != NULL) {
            *time_flags = BNO085_ACCEL_TIME_HINT_EXACT;
        }
        return hint_ticks;
    }

    if (time_flags != NULL) {
        *time_flags = BNO085_ACCEL_TIME_HINT_ESTIMATED;
    }
    /* A consumed IRQ timestamp describes an older packet, not the packet we
     * are about to read while draining an asserted H_INTN level. Reusing the
     * last accelerometer timestamp here creates a self-referential clock:
     * monotonic repair advances it once per report and can move acceleration
     * hundreds of milliseconds away from the independent GyroRV channel.
     * The current GPTimer count is the correct causal upper bound for a
     * packet without a fresh IRQ edge and keeps both sensor channels on the
     * same hardware clock. */
    uint64_t current_ticks = 0;
    if (s_fusion_timer != NULL &&
        gptimer_get_raw_count(s_fusion_timer, &current_ticks) == ESP_OK &&
        current_ticks != 0) {
        return current_ticks;
    }
    return bno085_service_fusion_time_ticks();
}

static uint32_t bno085_nominal_period_ticks(void)
{
    return bno085_accel_period_us() *
           (BNO085_FUSION_TIMER_HZ / 1000000U);
}

static uint32_t bno085_timestamp_period_ticks(void)
{
    return s_timestamp_period_ticks != 0 ? s_timestamp_period_ticks
                                         : bno085_nominal_period_ticks();
}

static void bno085_track_timestamp_period(uint64_t packet_hint_ticks,
                                          uint8_t packet_time_flags)
{
    if ((packet_time_flags & BNO085_ACCEL_TIME_HINT_EXACT) == 0 ||
        packet_hint_ticks == 0) {
        return;
    }

    if (s_timestamp_anchor_hint_ticks == 0) {
        s_timestamp_anchor_hint_ticks = packet_hint_ticks;
        s_timestamp_anchor_report_count = s_report_count;
        return;
    }

    const uint32_t report_delta =
        s_report_count - s_timestamp_anchor_report_count;
    if (packet_hint_ticks > s_timestamp_anchor_hint_ticks &&
        report_delta >= BNO085_TIMESTAMP_TRACK_MIN_REPORTS) {
        s_timestamp_period_ticks = bno085_timing_track_period(
            s_timestamp_period_ticks,
            packet_hint_ticks - s_timestamp_anchor_hint_ticks,
            report_delta,
            bno085_nominal_period_ticks());
        s_timestamp_anchor_hint_ticks = packet_hint_ticks;
        s_timestamp_anchor_report_count = s_report_count;
    }
}

static void bno085_consume_packet_hint(uint32_t hint_generation)
{
    if (hint_generation != 0) {
        s_consumed_hint_generation = hint_generation;
    }
}

static void bno085_parse_input_reports(const uint8_t *payload, size_t len,
                                       uint64_t packet_hint_ticks,
                                       uint8_t packet_time_flags)
{
    size_t pos = 0;
    s_batch_timebase_valid = false;

    while (pos < len) {
        const uint8_t report_id = payload[pos];
        if (report_id == BNO085_REPORT_TIMEBASE) {
            if (pos + BNO085_TIMEBASE_REPORT_LEN > len) {
                s_parse_error_count++;
                return;
            }
            s_timebase_count++;
            s_batch_hint_ticks = packet_hint_ticks;
            s_batch_base_delta_100us =
                bno085_timing_read_le_i32(&payload[pos + 1]);
            s_batch_rebase_delta_100us = 0;
            s_batch_timebase_valid =
                s_batch_base_delta_100us != INT32_MAX;
            pos += BNO085_TIMEBASE_REPORT_LEN;
            continue;
        }

        if (report_id == BNO085_REPORT_REBASE) {
            if (pos + BNO085_REBASE_REPORT_LEN > len) {
                s_parse_error_count++;
                return;
            }
            s_batch_rebase_delta_100us =
                bno085_timing_read_le_i32(&payload[pos + 1]);
            pos += BNO085_REBASE_REPORT_LEN;
            continue;
        }

        if (report_id == BNO085_REPORT_ACCELEROMETER) {
            if (pos + BNO085_ACCEL_REPORT_LEN > len) {
                s_parse_error_count++;
                return;
            }

            const uint8_t status = payload[pos + 2] & 0x03U;
            const uint16_t delay_100us =
                bno085_timing_report_delay_100us(&payload[pos]);
            const int16_t raw_x = read_le_i16(&payload[pos + 4]);
            const int16_t raw_y = read_le_i16(&payload[pos + 6]);
            const int16_t raw_z = read_le_i16(&payload[pos + 8]);

            s_last_x_mps2 = q_to_float(raw_x, BNO085_ACCEL_Q_POINT);
            s_last_y_mps2 = q_to_float(raw_y, BNO085_ACCEL_Q_POINT);
            s_last_z_mps2 = q_to_float(raw_z, BNO085_ACCEL_Q_POINT);
            s_last_accuracy = status;
            s_report_count++;

            uint8_t time_flags = packet_time_flags;
            uint64_t sample_ticks = packet_hint_ticks;
            if (s_batch_timebase_valid) {
                sample_ticks = bno085_timing_reconstruct_ticks(
                    s_batch_hint_ticks, s_batch_base_delta_100us,
                    s_batch_rebase_delta_100us, delay_100us);
                time_flags |= BNO085_ACCEL_TIME_SH2_VALID;
            }
            uint64_t previous_sample_ticks = 0;
            portENTER_CRITICAL(&s_sample_lock);
            previous_sample_ticks = s_last_fusion_time_ticks;
            portEXIT_CRITICAL(&s_sample_lock);
            const uint64_t causal_ticks = bno085_timing_causal_monotonic(
                sample_ticks, packet_hint_ticks, previous_sample_ticks,
                bno085_timestamp_period_ticks());
            if (causal_ticks != sample_ticks) {
                sample_ticks = causal_ticks;
                time_flags |= BNO085_ACCEL_TIME_MONOTONIC_REPAIRED;
                s_monotonic_repair_count++;
            }
            const bno085_accel_sample_t sample = {
                .fusion_time_ticks = sample_ticks,
                .sequence = s_report_count,
                .x_milli_mps2 = float_to_milli(s_last_x_mps2),
                .y_milli_mps2 = float_to_milli(s_last_y_mps2),
                .z_milli_mps2 = float_to_milli(s_last_z_mps2),
                .x_q8 = raw_x,
                .y_q8 = raw_y,
                .z_q8 = raw_z,
                .sensor_delay_100us = delay_100us,
                .accuracy = status,
                .time_flags = time_flags,
            };
            bno085_store_sample(&sample);
            bno085_emit_telemetry(&sample);
            pos += BNO085_ACCEL_REPORT_LEN;
            continue;
        }

        pos++;
    }
}

static void bno085_parse_packet(const uint8_t *packet, size_t packet_len,
                                uint64_t packet_hint_ticks,
                                uint8_t packet_time_flags)
{
    if (packet_len < BNO085_SHTP_HEADER_LEN) {
        s_parse_error_count++;
        return;
    }

    s_packet_count++;
    const uint8_t channel = packet[2];
    const uint8_t *payload = &packet[BNO085_SHTP_HEADER_LEN];
    const size_t payload_len = packet_len - BNO085_SHTP_HEADER_LEN;

    if (channel == BNO085_CHANNEL_INPUT_REPORTS) {
        s_input_packet_count++;
        s_last_input_payload_len = payload_len;
        const uint32_t before = s_report_count;
        bno085_parse_input_reports(payload, payload_len, packet_hint_ticks,
                                   packet_time_flags);
        const uint32_t reports_in_packet = s_report_count - before;
        bno085_track_timestamp_period(packet_hint_ticks, packet_time_flags);
        if (reports_in_packet > s_max_reports_per_packet) {
            s_max_reports_per_packet = reports_in_packet;
        }
        return;
    }

    if (channel == BNO085_CHANNEL_GYRO_RV) {
        if (payload_len == 0 ||
            payload_len % BNO085_GYRO_RV_REPORT_LEN != 0) {
            s_parse_error_count++;
            return;
        }
        const size_t report_total =
            payload_len / BNO085_GYRO_RV_REPORT_LEN;
        const uint64_t period_ticks =
            (uint64_t)BNO085_GYRO_RV_INTERVAL_US *
            (BNO085_FUSION_TIMER_HZ / 1000000U);
        uint64_t first_ticks = packet_hint_ticks;
        const uint64_t preceding_ticks = (report_total - 1U) * period_ticks;
        if (first_ticks >= preceding_ticks) {
            first_ticks -= preceding_ticks;
        }
        for (size_t index = 0; index < report_total; ++index) {
            const uint8_t *report =
                &payload[index * BNO085_GYRO_RV_REPORT_LEN];
            const bno085_gyro_rv_sample_t sample = {
                .fusion_time_ticks = first_ticks + index * period_ticks,
                .sequence = ++s_gyro_rv_report_count,
                .quat_i_q14 = read_le_i16(&report[0]),
                .quat_j_q14 = read_le_i16(&report[2]),
                .quat_k_q14 = read_le_i16(&report[4]),
                .quat_real_q14 = read_le_i16(&report[6]),
                .gyro_x_q10 = read_le_i16(&report[8]),
                .gyro_y_q10 = read_le_i16(&report[10]),
                .gyro_z_q10 = read_le_i16(&report[12]),
                .time_flags = packet_time_flags,
            };
            bno085_store_gyro_rv_sample(&sample);
            bno085_emit_orientation_telemetry(&sample);
        }
        return;
    }

    if (channel == BNO085_CHANNEL_CONTROL && payload_len >= 13 &&
        payload[0] == BNO085_REPORT_GET_FEATURE_RESPONSE &&
        payload[1] == BNO085_REPORT_ACCELEROMETER) {
        const uint32_t interval_us = read_le_u32(&payload[5]);
        const uint32_t batch_interval_us = read_le_u32(&payload[9]);
        if (interval_us > 0) {
            s_effective_accel_period_us = interval_us;
            s_timestamp_period_ticks = 0;
            s_timestamp_anchor_hint_ticks = 0;
            s_timestamp_anchor_report_count = 0;
            bno085_update_i2c_realtime_period();
        }
        ESP_LOGI(TAG,
                 "BNO085 accelerometer feature accepted: interval=%u us batch=%u us",
                 (unsigned)interval_us, (unsigned)batch_interval_us);
        return;
    }
    if (channel == BNO085_CHANNEL_CONTROL && payload_len >= 9 &&
        payload[0] == BNO085_REPORT_GET_FEATURE_RESPONSE &&
        payload[1] == BNO085_REPORT_GYRO_RV) {
        ESP_LOGI(TAG, "BNO085 GyroRV feature accepted: interval=%u us",
                 (unsigned)read_le_u32(&payload[5]));
    }
}

static void bno085_log_status(void)
{
    const uint32_t now_ms = ticks_to_ms();
    if ((uint32_t)(now_ms - s_last_log_ms) <
        APP_BNO085_SUMMARY_LOG_INTERVAL_MS) {
        return;
    }
    s_last_log_ms = now_ms;

    ESP_LOGI(TAG,
             "BNO085 summary x=%.2f y=%.2f z=%.2f acc=%u rep=%u gyro=%u req=%ums eff=%uus ts=%u ticks pkt=%u in=%u tb=%u max=%u ring=%u/%u ovw=%u telem=%u/%u dec=%u fix=%u len=%u cont=%u/%u cerr=%u hp=%u null=%u err=%u/%u irq=%u il=%d wt=%u",
             (double)s_last_x_mps2, (double)s_last_y_mps2,
             (double)s_last_z_mps2, (unsigned)s_last_accuracy,
             (unsigned)s_report_count, (unsigned)s_gyro_rv_report_count,
             (unsigned)bno085_accel_interval_ms(),
              (unsigned)bno085_accel_period_us(),
              (unsigned)bno085_timestamp_period_ticks(),
             (unsigned)s_packet_count, (unsigned)s_input_packet_count,
             (unsigned)s_timebase_count, (unsigned)s_max_reports_per_packet,
             (unsigned)s_sample_ring_count,
             (unsigned)s_sample_ring_capacity,
             (unsigned)s_sample_overwrite_count,
             (unsigned)s_telemetry_submit_count,
             (unsigned)s_telemetry_drop_count,
             (unsigned)s_telemetry_decimated_count,
             (unsigned)s_monotonic_repair_count,
             (unsigned)s_last_packet_len, (unsigned)s_continuation_packet_count,
             (unsigned)s_continuation_transfer_count,
             (unsigned)s_continuation_header_error_count,
             (unsigned)s_high_rate_poll_count, (unsigned)s_null_header_count,
             (unsigned)s_read_error_count, (unsigned)s_parse_error_count,
             (unsigned)s_int_irq_count,
             gpio_get_level(BOARD_CONFIG_BNO085_INT_GPIO),
             (unsigned)s_int_wait_timeout_count);
}

static void bno085_log_read_error(esp_err_t err)
{
    if (err == ESP_ERR_TIMEOUT) {
        return;
    }

    s_read_error_count++;
    const uint32_t now_ms = ticks_to_ms();
    if ((uint32_t)(now_ms - s_last_read_warning_ms) <
        APP_BNO085_SUMMARY_LOG_INTERVAL_MS) {
        return;
    }

    s_last_read_warning_ms = now_ms;
    ESP_LOGW(TAG, "BNO085 read issues: last=%s read_errors=%u",
             esp_err_to_name(err), (unsigned)s_read_error_count);
}

static bool bno085_recover_if_stalled(void)
{
    const uint32_t now_ms = ticks_to_ms();
    if (s_report_count != s_last_progress_report_count) {
        s_last_progress_report_count = s_report_count;
        s_last_progress_ms = now_ms;
        return true;
    }

    if (s_last_progress_ms == 0) {
        s_last_progress_ms = now_ms;
        return true;
    }

    if ((uint32_t)(now_ms - s_last_progress_ms) < BNO085_STALL_RECOVERY_MS ||
        (uint32_t)(now_ms - s_last_stall_recovery_ms) <
            BNO085_STALL_RECOVERY_MS) {
        return true;
    }
    s_last_stall_recovery_ms = now_ms;

    ESP_LOGW(TAG, "BNO085 stalled for %u ms at reports=%u; resetting",
             (unsigned)(now_ms - s_last_progress_ms),
             (unsigned)s_report_count);

    esp_err_t err = bno085_soft_reset();
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(50));
        err = bno085_rebind_i2c_device();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "BNO085 stall reset address detection failed: %s",
                     esp_err_to_name(err));
            return false;
        }
        if (!bno085_drain_startup_packets()) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        if (!bno085_drain_startup_packets()) {
            return false;
        }
    } else {
        ESP_LOGW(TAG, "BNO085 stall soft reset failed: %s",
                 esp_err_to_name(err));
    }

    err = bno085_enable_gyro_rv();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BNO085 GyroRV stall recovery enable failed: %s",
                 esp_err_to_name(err));
    }
    err = bno085_enable_accelerometer();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "BNO085 accelerometer re-enabled after stall");
    } else {
        ESP_LOGW(TAG, "BNO085 stall recovery enable failed: %s",
                 esp_err_to_name(err));
    }

    s_last_progress_report_count = s_report_count;
    s_last_progress_ms = ticks_to_ms();
    return true;
}

static esp_err_t bno085_i2c_init(void)
{
    esp_err_t err = i2c_bus_service_get(&s_i2c_bus);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t device_address = APP_BNO085_I2C_ADDRESS;
    err = i2c_master_probe(s_i2c_bus, device_address, 100);
    if (err != ESP_OK) {
        const uint8_t alternate_address =
            device_address == 0x4A ? 0x4B : 0x4A;
        const esp_err_t alternate_err =
            i2c_master_probe(s_i2c_bus, alternate_address, 100);
        if (alternate_err != ESP_OK) {
            ESP_LOGE(TAG,
                     "BNO085 not found at preferred address 0x%02X (%s) or alternate 0x%02X (%s)",
                     device_address, esp_err_to_name(err), alternate_address,
                     esp_err_to_name(alternate_err));
            return err;
        }

        ESP_LOGW(TAG,
                 "BNO085 detected at alternate address 0x%02X (preferred 0x%02X did not respond)",
                 alternate_address, device_address);
        device_address = alternate_address;
    }

    const i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = APP_BNO085_I2C_CLOCK_HZ,
        .scl_wait_us = 20000,
    };
    err = i2c_master_bus_add_device(s_i2c_bus, &dev_config, &s_i2c_dev);
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "BNO085 I2C device ready at 0x%02X", device_address);
    return ESP_OK;
}

static esp_err_t bno085_rebind_i2c_device(void)
{
    bno085_release_i2c_device();
    return bno085_i2c_init();
}

static uint32_t bno085_wait_notify_bits(uint32_t timeout_ms)
{
    uint32_t notification = 0;
    if (xTaskNotifyWait(0, UINT32_MAX, &notification,
                        ms_to_ticks_min_1(timeout_ms)) == pdTRUE) {
        return notification;
    }
    return 0;
}

static uint32_t bno085_take_notify_bits_now(void)
{
    uint32_t notification = 0;
    if (xTaskNotifyWait(0, UINT32_MAX, &notification, 0) == pdTRUE) {
        return notification;
    }
    return 0;
}

static bool bno085_drain_startup_packets(void)
{
    const uint32_t start_ms = ticks_to_ms();
    uint8_t packet[BNO085_MAX_PACKET_LEN] = {0};

    while ((uint32_t)(ticks_to_ms() - start_ms) < BNO085_STARTUP_DRAIN_MS) {
        if ((bno085_take_notify_bits_now() & BNO085_NOTIFY_STOP) != 0) {
            return false;
        }

        if (s_int_irq_enabled && !bno085_int_active()) {
            bno085_arm_interrupt_if_needed();
            const uint32_t notify_bits = bno085_wait_notify_bits(50);
            if ((notify_bits & BNO085_NOTIFY_STOP) != 0) {
                return false;
            }
            if (!bno085_int_active()) {
                continue;
            }
        }

        uint8_t packet_time_flags = 0;
        uint32_t packet_hint_generation = 0;
        const uint64_t packet_hint_ticks =
            bno085_packet_hint_ticks(&packet_time_flags,
                                     &packet_hint_generation);
        size_t packet_len = 0;
        const esp_err_t err = bno085_read_packet(packet, sizeof(packet),
                                                 &packet_len);
        if (err == ESP_OK) {
            bno085_consume_packet_hint(packet_hint_generation);
            bno085_parse_packet(packet, packet_len, packet_hint_ticks,
                                packet_time_flags);
            continue;
        }
        if (s_int_irq_enabled) {
            const uint32_t notify_bits = bno085_wait_notify_bits(50);
            if ((notify_bits & BNO085_NOTIFY_STOP) != 0) {
                return false;
            }
        } else {
            vTaskDelay(ms_to_ticks_min_1(50));
        }
    }
    return true;
}

static bool bno085_wait_for_interrupt_or_timeout(uint32_t *notify_bits)
{
    *notify_bits = bno085_take_notify_bits_now();
    if ((*notify_bits & BNO085_NOTIFY_STOP) != 0) {
        return false;
    }

    if (bno085_int_active()) {
        s_wait_immediate_count++;
        i2c_bus_service_note_realtime_activity();
        return true;
    }

    const uint32_t timeout_ms = bno085_int_wait_timeout_ms();
    if (!s_int_irq_enabled) {
        *notify_bits = bno085_wait_notify_bits(timeout_ms);
        const bool active = bno085_int_active();
        if (active) {
            i2c_bus_service_note_realtime_activity();
        }
        return active;
    }

    bno085_arm_interrupt_if_needed();

    *notify_bits = bno085_wait_notify_bits(timeout_ms);
    if (*notify_bits != 0) {
        if ((*notify_bits & BNO085_NOTIFY_INT) != 0) {
            s_wait_notify_count++;
            i2c_bus_service_note_realtime_activity();
        }
        return bno085_int_active();
    }
    if (bno085_int_active()) {
        s_wait_late_active_count++;
        i2c_bus_service_note_realtime_activity();
        return true;
    }

    s_int_wait_timeout_count++;
    return false;
}

static void bno085_drain_ready_packets(uint8_t *packet, size_t packet_size)
{
    const uint32_t start_ms = ticks_to_ms();
    for (uint32_t i = 0; i < BNO085_MAX_PACKETS_PER_WAKE; ++i) {
        uint8_t packet_time_flags = 0;
        uint32_t packet_hint_generation = 0;
        const uint64_t packet_hint_ticks =
            bno085_packet_hint_ticks(&packet_time_flags,
                                     &packet_hint_generation);
        size_t packet_len = 0;
        const esp_err_t err = bno085_read_packet(packet, packet_size,
                                                 &packet_len);
        if (err == ESP_OK) {
            bno085_consume_packet_hint(packet_hint_generation);
            bno085_parse_packet(packet, packet_len, packet_hint_ticks,
                                packet_time_flags);

            if (bno085_high_rate_mode() &&
                (uint32_t)(ticks_to_ms() - start_ms) <
                    BNO085_HIGH_RATE_DRAIN_BUDGET_MS) {
                s_high_rate_poll_count++;
                continue;
            }

            if (!bno085_int_active()) {
                return;
            }
            continue;
        }

        bno085_log_read_error(err);
        if (bno085_int_active()) {
            vTaskDelay(ms_to_ticks_min_1(5));
        }
        return;
    }
}

static void bno085_task(void *arg)
{
    (void)arg;
    s_task_handle = xTaskGetCurrentTaskHandle();
    const uint32_t accel_interval_ms = bno085_accel_interval_ms();
    const uint32_t log_interval_ms = bno085_log_interval_ms();
    s_effective_accel_period_us = bno085_requested_accel_period_us();
    s_i2c_scl_measure_count = 0;

    esp_err_t err = bno085_fusion_timer_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BNO085 fusion clock init failed: %s",
                 esp_err_to_name(err));
        bno085_task_finish(true);
        return;
    }
    err = bno085_sample_ring_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BNO085 sample ring init failed: %s",
                 esp_err_to_name(err));
        bno085_task_finish(true);
        return;
    }
    portENTER_CRITICAL(&s_sample_lock);
    s_sample_ring_count = 0;
    s_sample_ring_write = 0;
    s_gyro_rv_ring_count = 0;
    s_gyro_rv_ring_write = 0;
    s_last_fusion_time_ticks = 0;
    portEXIT_CRITICAL(&s_sample_lock);
    portENTER_CRITICAL(&s_hint_lock);
    s_consumed_hint_generation = s_hint_generation;
    portEXIT_CRITICAL(&s_hint_lock);
    s_batch_timebase_valid = false;
    s_last_telemetry_ticks = 0;
    s_last_clock_anchor_ticks = 0;
    s_timestamp_period_ticks = 0;
    s_timestamp_anchor_hint_ticks = 0;
    s_timestamp_anchor_report_count = 0;

    ESP_LOGI(TAG,
             "BNO085 accelerometer test enabled: SDA=%d SCL=%d RST=%d INT=%d preferred_addr=0x%02X clock=%u Hz sample=%u ms log=%u ms int_timeout=%u ms core=%d",
             BOARD_CONFIG_BNO085_SDA_GPIO, BOARD_CONFIG_BNO085_SCL_GPIO,
             BOARD_CONFIG_BNO085_RST_GPIO, BOARD_CONFIG_BNO085_INT_GPIO,
             APP_BNO085_I2C_ADDRESS, (unsigned)APP_BNO085_I2C_CLOCK_HZ,
             (unsigned)accel_interval_ms,
             (unsigned)log_interval_ms,
             (unsigned)APP_BNO085_INT_WAIT_TIMEOUT_MS, BNO085_TASK_CORE);

    err = bno085_configure_host_gpios();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BNO085 GPIO init failed: %s", esp_err_to_name(err));
        bno085_task_finish(true);
        return;
    }

    err = bno085_configure_host_interrupt();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BNO085 INT init failed, using timeout fallback: %s",
                 esp_err_to_name(err));
        s_int_irq_enabled = false;
    }

    err = bno085_hard_reset();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BNO085 hard reset failed: %s", esp_err_to_name(err));
        bno085_task_finish(true);
        return;
    }

    err = bno085_i2c_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BNO085 I2C init failed: %s", esp_err_to_name(err));
        bno085_task_finish(true);
        return;
    }

    err = bno085_soft_reset();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "BNO085 soft reset command sent");
        vTaskDelay(pdMS_TO_TICKS(50));
        err = bno085_rebind_i2c_device();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "BNO085 post-reset address detection failed: %s",
                     esp_err_to_name(err));
            bno085_task_finish(true);
            return;
        }
        if (!bno085_drain_startup_packets()) {
            ESP_LOGI(TAG, "BNO085 stop requested during startup");
            bno085_task_finish(true);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        if (!bno085_drain_startup_packets()) {
            ESP_LOGI(TAG, "BNO085 stop requested during startup");
            bno085_task_finish(true);
            return;
        }
    } else {
        ESP_LOGW(TAG, "BNO085 soft reset failed: %s", esp_err_to_name(err));
    }

    err = bno085_enable_gyro_rv();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BNO085 GyroRV enable failed: %s",
                 esp_err_to_name(err));
        bno085_task_finish(true);
        return;
    }
    ESP_LOGI(TAG, "BNO085 GyroRV enable command sent at %u Hz",
             (unsigned)BNO085_GYRO_RV_RATE_HZ);
    err = bno085_enable_accelerometer();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BNO085 accelerometer enable failed: %s",
                 esp_err_to_name(err));
        bno085_task_finish(true);
        return;
    }
    ESP_LOGI(TAG, "BNO085 accelerometer enable command sent last");
    s_last_progress_ms = ticks_to_ms();
    s_last_progress_report_count = s_report_count;

    uint8_t packet[BNO085_MAX_PACKET_LEN] = {0};
    while (true) {
        uint32_t notify_bits = 0;
        const bool packet_ready =
            bno085_wait_for_interrupt_or_timeout(&notify_bits);
        if ((notify_bits & BNO085_NOTIFY_STOP) != 0) {
            break;
        }
        if (packet_ready) {
            bno085_drain_ready_packets(packet, sizeof(packet));
        }

        bno085_reconfigure_accelerometer_if_needed();
        if (!bno085_recover_if_stalled()) {
            break;
        }
        bno085_log_status();
    }

    ESP_LOGI(TAG, "BNO085 stop requested; holding accelerometer in reset");
    bno085_task_finish(true);
}

esp_err_t bno085_service_start(void)
{
    const app_runtime_config_t *runtime_config = app_runtime_config_get();
    if (!runtime_config->bno085_accel_enabled) {
        ESP_LOGI(TAG, "BNO085 accelerometer disabled by runtime config");
        if (s_service_started && s_task_handle != NULL) {
            bno085_notify_task(BNO085_NOTIFY_STOP);
            return ESP_OK;
        }
        return bno085_hold_in_reset();
    }

    if (s_service_started) {
        bno085_notify_task(BNO085_NOTIFY_CONFIG);
        return ESP_OK;
    }

    s_service_started = true;
    const BaseType_t created = xTaskCreatePinnedToCore(bno085_task,
                                                       "bno085",
                                                       BNO085_TASK_STACK_BYTES,
                                                       NULL,
                                                       BNO085_TASK_PRIORITY,
                                                       &s_task_handle,
                                                       BNO085_TASK_CORE);
    if (created != pdPASS) {
        s_service_started = false;
        ESP_LOGE(TAG, "Failed to create BNO085 task");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t bno085_service_apply_runtime_config(void)
{
    return bno085_service_start();
}

size_t bno085_service_copy_accel_samples(uint32_t after_sequence,
                                         bno085_accel_sample_t *samples,
                                         size_t max_samples)
{
    if (samples == NULL || max_samples == 0 || s_sample_ring == NULL ||
        s_sample_ring_capacity == 0) {
        return 0;
    }

    if (max_samples > BNO085_COPY_MAX_SAMPLES) {
        max_samples = BNO085_COPY_MAX_SAMPLES;
    }

    size_t copied = 0;
    portENTER_CRITICAL(&s_sample_lock);
    const size_t oldest =
        (s_sample_ring_write + s_sample_ring_capacity - s_sample_ring_count) %
        s_sample_ring_capacity;
    for (size_t i = 0; i < s_sample_ring_count && copied < max_samples; ++i) {
        const bno085_accel_sample_t *sample =
            &s_sample_ring[(oldest + i) % s_sample_ring_capacity];
        if ((int32_t)(sample->sequence - after_sequence) > 0) {
            samples[copied++] = *sample;
        }
    }
    portEXIT_CRITICAL(&s_sample_lock);
    return copied;
}

size_t bno085_service_copy_gyro_rv_samples(uint32_t after_sequence,
                                           bno085_gyro_rv_sample_t *samples,
                                           size_t max_samples)
{
    if (samples == NULL || max_samples == 0 || s_gyro_rv_ring == NULL) {
        return 0;
    }

    if (max_samples > BNO085_COPY_MAX_SAMPLES) {
        max_samples = BNO085_COPY_MAX_SAMPLES;
    }

    size_t copied = 0;
    portENTER_CRITICAL(&s_sample_lock);
    const size_t oldest =
        (s_gyro_rv_ring_write + BNO085_GYRO_RV_RING_CAPACITY -
         s_gyro_rv_ring_count) %
        BNO085_GYRO_RV_RING_CAPACITY;
    for (size_t i = 0; i < s_gyro_rv_ring_count && copied < max_samples;
         ++i) {
        const bno085_gyro_rv_sample_t *sample =
            &s_gyro_rv_ring[(oldest + i) % BNO085_GYRO_RV_RING_CAPACITY];
        if ((int32_t)(sample->sequence - after_sequence) > 0) {
            samples[copied++] = *sample;
        }
    }
    portEXIT_CRITICAL(&s_sample_lock);
    return copied;
}

void bno085_service_get_snapshot(bno085_service_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    uint32_t sample_ring_capacity = 0;
    uint32_t sample_ring_count = 0;
    uint32_t gyro_rv_ring_count = 0;
    uint64_t last_fusion_time_ticks = 0;
    portENTER_CRITICAL(&s_sample_lock);
    sample_ring_capacity = (uint32_t)s_sample_ring_capacity;
    sample_ring_count = (uint32_t)s_sample_ring_count;
    gyro_rv_ring_count = (uint32_t)s_gyro_rv_ring_count;
    last_fusion_time_ticks = s_last_fusion_time_ticks;
    portEXIT_CRITICAL(&s_sample_lock);

    *snapshot = (bno085_service_snapshot_t){
        .service_started = s_service_started,
        .int_irq_enabled = s_int_irq_enabled,
        .i2c_clock_hz = APP_BNO085_I2C_CLOCK_HZ,
        .accel_interval_ms = bno085_accel_interval_ms(),
        .report_count = s_report_count,
        .packet_count = s_packet_count,
        .input_packet_count = s_input_packet_count,
        .timebase_count = s_timebase_count,
        .max_reports_per_packet = s_max_reports_per_packet,
        .continuation_packet_count = s_continuation_packet_count,
        .continuation_transfer_count = s_continuation_transfer_count,
        .continuation_header_error_count = s_continuation_header_error_count,
        .high_rate_poll_count = s_high_rate_poll_count,
        .wait_immediate_count = s_wait_immediate_count,
        .wait_notify_count = s_wait_notify_count,
        .wait_late_active_count = s_wait_late_active_count,
        .null_header_count = s_null_header_count,
        .read_error_count = s_read_error_count,
        .parse_error_count = s_parse_error_count,
        .int_irq_count = s_int_irq_count,
        .int_wait_timeout_count = s_int_wait_timeout_count,
        .sample_ring_capacity = sample_ring_capacity,
        .sample_ring_count = sample_ring_count,
        .sample_overwrite_count = s_sample_overwrite_count,
        .telemetry_submit_count = s_telemetry_submit_count,
        .telemetry_drop_count = s_telemetry_drop_count,
        .telemetry_decimated_count = s_telemetry_decimated_count,
        .monotonic_repair_count = s_monotonic_repair_count,
        .gyro_rv_report_count = s_gyro_rv_report_count,
        .gyro_rv_ring_capacity = BNO085_GYRO_RV_RING_CAPACITY,
        .gyro_rv_ring_count = gyro_rv_ring_count,
        .gyro_rv_overwrite_count = s_gyro_rv_overwrite_count,
        .gyro_rv_rate_hz = BNO085_GYRO_RV_RATE_HZ,
        .telemetry_rate_hz = BNO085_TELEMETRY_RATE_HZ,
        .fusion_timer_hz = BNO085_FUSION_TIMER_HZ,
        .last_fusion_time_ticks = last_fusion_time_ticks,
        .last_packet_len = s_last_packet_len,
        .last_input_payload_len = s_last_input_payload_len,
        .last_x_mps2 = s_last_x_mps2,
        .last_y_mps2 = s_last_y_mps2,
        .last_z_mps2 = s_last_z_mps2,
        .last_accuracy = s_last_accuracy,
        .i2c_scl_measure_error = s_i2c_scl_measure_error,
        .i2c_scl_edges = s_i2c_scl_edges,
        .i2c_scl_elapsed_us = s_i2c_scl_elapsed_us,
        .i2c_scl_measured_hz = s_i2c_scl_measured_hz,
    };
}
