#include "bno085_service.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "app_config.h"
#include "app_runtime_config.h"
#include "board_config.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wireless_telemetry_service.h"
#include "sdkconfig.h"

static const char *TAG = "bno085_service";

enum {
    BNO085_I2C_PORT = 0,
    BNO085_TASK_STACK_WORDS = 4096,
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
    BNO085_STALL_RECOVERY_MS = 5000,
    BNO085_ACCEL_REPORT_LEN = 10,
    BNO085_TIMEBASE_REPORT_LEN = 5,
    BNO085_CHANNEL_CONTROL = 2,
    BNO085_CHANNEL_EXECUTABLE = 1,
    BNO085_CHANNEL_INPUT_REPORTS = 3,
    BNO085_EXECUTABLE_RESET = 0x01,
    BNO085_REPORT_SET_FEATURE = 0xFD,
    BNO085_REPORT_GET_FEATURE_RESPONSE = 0xFC,
    BNO085_REPORT_TIMEBASE = 0xFB,
    BNO085_REPORT_ACCELEROMETER = 0x01,
    BNO085_ACCEL_Q_POINT = 8,
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
static uint32_t s_last_reconfigure_warning_ms;
static uint32_t s_last_progress_ms;
static uint32_t s_last_progress_report_count;
static uint32_t s_last_stall_recovery_ms;

static void bno085_drain_startup_packets(void);

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

static uint32_t bno085_log_interval_ms(void)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    if (config != NULL && config->bno085_log_interval_ms > 0) {
        return config->bno085_log_interval_ms;
    }
    return APP_BNO085_LOG_INTERVAL_MS;
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
    s_int_irq_count++;
    /* H_INTN is level-low on I2C and deasserts as soon as the address is seen;
       mask it until the task drains the SHTP packet and waits again. */
    s_int_irq_armed = false;
    (void)gpio_intr_disable(BOARD_CONFIG_BNO085_INT_GPIO);
    if (s_task_handle != NULL) {
        vTaskNotifyGiveFromISR(s_task_handle, &higher_priority_task_woken);
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

static esp_err_t bno085_read_packet(uint8_t *packet, size_t packet_size,
                                    size_t *packet_len)
{
    uint8_t header[BNO085_SHTP_HEADER_LEN] = {0};
    esp_err_t err = i2c_master_receive(s_i2c_dev, header, sizeof(header),
                                       BNO085_READ_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }

    const uint16_t raw_len = read_le_u16(header);
    if (raw_len == 0 || raw_len == 0xFFFFU) {
        return ESP_ERR_TIMEOUT;
    }

    const size_t total_len = raw_len & 0x7FFFU;
    if (total_len < BNO085_SHTP_HEADER_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    const size_t payload_len = total_len - BNO085_SHTP_HEADER_LEN;

    if (total_len <= packet_size) {
        memcpy(packet, header, sizeof(header));
    }

    if (total_len > packet_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (payload_len > 0) {
        uint8_t chunk[BNO085_MAX_PACKET_LEN] = {0};
        err = i2c_master_receive(s_i2c_dev, chunk,
                                 payload_len + BNO085_SHTP_HEADER_LEN,
                                 BNO085_READ_TIMEOUT_MS);
        if (err != ESP_OK) {
            return err;
        }

        memcpy(&packet[BNO085_SHTP_HEADER_LEN],
               &chunk[BNO085_SHTP_HEADER_LEN], payload_len);
    }

    *packet_len = total_len;
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

    return i2c_master_transmit(s_i2c_dev, packet, total_len,
                               BNO085_WRITE_TIMEOUT_MS);
}

static esp_err_t bno085_enable_accelerometer(void)
{
    const uint32_t interval_ms = bno085_accel_interval_ms();
    const uint32_t interval_us = interval_ms * 1000U;
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
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
    };

    const esp_err_t err =
        bno085_send_packet(BNO085_CHANNEL_CONTROL, payload, sizeof(payload));
    if (err == ESP_OK) {
        s_configured_accel_interval_ms = interval_ms;
    }
    return err;
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

static void bno085_emit_telemetry(void)
{
    return;

    (void)wireless_telemetry_service_submit_bno085_accel(
        float_to_milli(s_last_x_mps2), float_to_milli(s_last_y_mps2),
        float_to_milli(s_last_z_mps2), s_last_accuracy, s_report_count);
}

static void bno085_parse_input_reports(const uint8_t *payload, size_t len)
{
    size_t pos = 0;

    while (pos < len) {
        const uint8_t report_id = payload[pos];
        if (report_id == BNO085_REPORT_TIMEBASE) {
            if (pos + BNO085_TIMEBASE_REPORT_LEN > len) {
                s_parse_error_count++;
                return;
            }
            pos += BNO085_TIMEBASE_REPORT_LEN;
            continue;
        }

        if (report_id == BNO085_REPORT_ACCELEROMETER) {
            if (pos + BNO085_ACCEL_REPORT_LEN > len) {
                s_parse_error_count++;
                return;
            }

            const uint8_t status = payload[pos + 2] & 0x03U;
            const int16_t raw_x = read_le_i16(&payload[pos + 4]);
            const int16_t raw_y = read_le_i16(&payload[pos + 6]);
            const int16_t raw_z = read_le_i16(&payload[pos + 8]);

            s_last_x_mps2 = q_to_float(raw_x, BNO085_ACCEL_Q_POINT);
            s_last_y_mps2 = q_to_float(raw_y, BNO085_ACCEL_Q_POINT);
            s_last_z_mps2 = q_to_float(raw_z, BNO085_ACCEL_Q_POINT);
            s_last_accuracy = status;
            s_report_count++;
            bno085_emit_telemetry();
            pos += BNO085_ACCEL_REPORT_LEN;
            continue;
        }

        pos++;
    }
}

static void bno085_parse_packet(const uint8_t *packet, size_t packet_len)
{
    if (packet_len < BNO085_SHTP_HEADER_LEN) {
        s_parse_error_count++;
        return;
    }

    const uint8_t channel = packet[2];
    const uint8_t *payload = &packet[BNO085_SHTP_HEADER_LEN];
    const size_t payload_len = packet_len - BNO085_SHTP_HEADER_LEN;

    if (channel == BNO085_CHANNEL_INPUT_REPORTS) {
        bno085_parse_input_reports(payload, payload_len);
        return;
    }

    if (channel == BNO085_CHANNEL_CONTROL && payload_len >= 9 &&
        payload[0] == BNO085_REPORT_GET_FEATURE_RESPONSE &&
        payload[1] == BNO085_REPORT_ACCELEROMETER) {
        const uint32_t interval_us = read_le_u32(&payload[5]);
        ESP_LOGI(TAG, "BNO085 accelerometer feature accepted: interval=%u us",
                 (unsigned)interval_us);
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
             "BNO085 accel summary x=%.2f y=%.2f z=%.2f m/s^2 accuracy=%u reports=%u sample_ms=%u read_errors=%u parse_errors=%u irqs=%u wait_timeouts=%u",
             (double)s_last_x_mps2, (double)s_last_y_mps2,
             (double)s_last_z_mps2, (unsigned)s_last_accuracy,
             (unsigned)s_report_count, (unsigned)bno085_accel_interval_ms(),
             (unsigned)s_read_error_count,
             (unsigned)s_parse_error_count, (unsigned)s_int_irq_count,
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

static void bno085_recover_if_stalled(void)
{
    const uint32_t now_ms = ticks_to_ms();
    if (s_report_count != s_last_progress_report_count) {
        s_last_progress_report_count = s_report_count;
        s_last_progress_ms = now_ms;
        return;
    }

    if (s_last_progress_ms == 0) {
        s_last_progress_ms = now_ms;
        return;
    }

    if ((uint32_t)(now_ms - s_last_progress_ms) < BNO085_STALL_RECOVERY_MS ||
        (uint32_t)(now_ms - s_last_stall_recovery_ms) <
            BNO085_STALL_RECOVERY_MS) {
        return;
    }
    s_last_stall_recovery_ms = now_ms;

    ESP_LOGW(TAG, "BNO085 stalled for %u ms at reports=%u; resetting",
             (unsigned)(now_ms - s_last_progress_ms),
             (unsigned)s_report_count);

    esp_err_t err = bno085_soft_reset();
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(50));
        bno085_drain_startup_packets();
        vTaskDelay(pdMS_TO_TICKS(50));
        bno085_drain_startup_packets();
    } else {
        ESP_LOGW(TAG, "BNO085 stall soft reset failed: %s",
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
}

static esp_err_t bno085_i2c_init(void)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = BNO085_I2C_PORT,
        .sda_io_num = BOARD_CONFIG_BNO085_SDA_GPIO,
        .scl_io_num = BOARD_CONFIG_BNO085_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    esp_err_t err = i2c_new_master_bus(&bus_config, &s_i2c_bus);
    if (err != ESP_OK) {
        return err;
    }

    const i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = APP_BNO085_I2C_ADDRESS,
        .scl_speed_hz = APP_BNO085_I2C_CLOCK_HZ,
        .scl_wait_us = 20000,
    };
    err = i2c_master_bus_add_device(s_i2c_bus, &dev_config, &s_i2c_dev);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_master_probe(s_i2c_bus, APP_BNO085_I2C_ADDRESS, 100);
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

static void bno085_drain_startup_packets(void)
{
    const uint32_t start_ms = ticks_to_ms();
    uint8_t packet[BNO085_MAX_PACKET_LEN] = {0};

    while ((uint32_t)(ticks_to_ms() - start_ms) < BNO085_STARTUP_DRAIN_MS) {
        if (s_int_irq_enabled && !bno085_int_active()) {
            bno085_arm_interrupt_if_needed();
            (void)ulTaskNotifyTake(pdTRUE, ms_to_ticks_min_1(50));
            if (!bno085_int_active()) {
                continue;
            }
        }

        size_t packet_len = 0;
        const esp_err_t err = bno085_read_packet(packet, sizeof(packet),
                                                 &packet_len);
        if (err == ESP_OK) {
            bno085_parse_packet(packet, packet_len);
            continue;
        }
        if (s_int_irq_enabled) {
            (void)ulTaskNotifyTake(pdTRUE, ms_to_ticks_min_1(50));
        } else {
            vTaskDelay(ms_to_ticks_min_1(50));
        }
    }
}

static bool bno085_wait_for_interrupt_or_timeout(void)
{
    if (bno085_int_active()) {
        return true;
    }

    if (!s_int_irq_enabled) {
        vTaskDelay(ms_to_ticks_min_1(APP_BNO085_INT_WAIT_TIMEOUT_MS));
        return bno085_int_active();
    }

    bno085_arm_interrupt_if_needed();

    const uint32_t taken = ulTaskNotifyTake(
        pdTRUE, ms_to_ticks_min_1(APP_BNO085_INT_WAIT_TIMEOUT_MS));
    if (taken > 0) {
        return bno085_int_active();
    }
    if (bno085_int_active()) {
        return true;
    }

    s_int_wait_timeout_count++;
    return false;
}

static void bno085_drain_ready_packets(uint8_t *packet, size_t packet_size)
{
    for (uint32_t i = 0; i < BNO085_MAX_PACKETS_PER_WAKE; ++i) {
        size_t packet_len = 0;
        const esp_err_t err = bno085_read_packet(packet, packet_size,
                                                 &packet_len);
        if (err == ESP_OK) {
            bno085_parse_packet(packet, packet_len);
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

    ESP_LOGI(TAG,
             "BNO085 accelerometer test enabled: SDA=%d SCL=%d RST=%d INT=%d addr=0x%02X clock=%u Hz sample=%u ms log=%u ms int_timeout=%u ms core=%d",
             BOARD_CONFIG_BNO085_SDA_GPIO, BOARD_CONFIG_BNO085_SCL_GPIO,
             BOARD_CONFIG_BNO085_RST_GPIO, BOARD_CONFIG_BNO085_INT_GPIO,
             APP_BNO085_I2C_ADDRESS, (unsigned)APP_BNO085_I2C_CLOCK_HZ,
             (unsigned)accel_interval_ms,
             (unsigned)log_interval_ms,
             (unsigned)APP_BNO085_INT_WAIT_TIMEOUT_MS, BNO085_TASK_CORE);

    esp_err_t err = bno085_configure_host_gpios();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BNO085 GPIO init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
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
        vTaskDelete(NULL);
        return;
    }

    err = bno085_i2c_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BNO085 I2C init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    err = bno085_soft_reset();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "BNO085 soft reset command sent");
        vTaskDelay(pdMS_TO_TICKS(50));
        bno085_drain_startup_packets();
        vTaskDelay(pdMS_TO_TICKS(50));
        bno085_drain_startup_packets();
    } else {
        ESP_LOGW(TAG, "BNO085 soft reset failed: %s", esp_err_to_name(err));
    }

    err = bno085_enable_accelerometer();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BNO085 accelerometer enable failed: %s",
                 esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "BNO085 accelerometer enable command sent");
    s_last_progress_ms = ticks_to_ms();
    s_last_progress_report_count = s_report_count;

    uint8_t packet[BNO085_MAX_PACKET_LEN] = {0};
    while (true) {
        if (bno085_wait_for_interrupt_or_timeout()) {
            bno085_drain_ready_packets(packet, sizeof(packet));
        }

        bno085_reconfigure_accelerometer_if_needed();
        bno085_recover_if_stalled();
        bno085_log_status();
    }
}

esp_err_t bno085_service_start(void)
{
    const app_runtime_config_t *runtime_config = app_runtime_config_get();
    if (!runtime_config->bno085_accel_enabled) {
        ESP_LOGI(TAG, "BNO085 accelerometer disabled by runtime config");
        return bno085_hold_in_reset();
    }

    if (s_service_started) {
        return ESP_OK;
    }

    const BaseType_t created = xTaskCreatePinnedToCore(bno085_task,
                                                       "bno085",
                                                       BNO085_TASK_STACK_WORDS,
                                                       NULL,
                                                       BNO085_TASK_PRIORITY,
                                                       NULL,
                                                       BNO085_TASK_CORE);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create BNO085 task");
        return ESP_ERR_NO_MEM;
    }

    s_service_started = true;
    return ESP_OK;
}
