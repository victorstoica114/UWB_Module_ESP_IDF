#include "charger_service.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "board_config.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "i2c_bus_service.h"
#include "sdkconfig.h"

static const char *TAG = "charger_service";

enum {
    CHARGER_TASK_STACK_WORDS = 3072,
    CHARGER_TASK_PRIORITY = 4,
    CHARGER_I2C_TIMEOUT_MS = 200,
    CHARGER_I2C_LOCK_TIMEOUT_MS = 250,
    CHARGER_LOG_INTERVAL_MS = 5000,
    CHARGER_GPIO_INVALID_LEVEL = -1,
    REG10_CHARGER_CONTROL_1 = 0x10,
    REG14_CHARGER_CONTROL_5 = 0x14,
    REG1B_CHARGER_STATUS_0 = 0x1B,
    REG20_FAULT_STATUS_0 = 0x20,
    REG22_CHARGER_FLAG_0 = 0x22,
    REG26_FAULT_FLAG_0 = 0x26,
    REG2E_ADC_CONTROL = 0x2E,
    REG2F_ADC_DISABLE_0 = 0x2F,
    REG30_ADC_DISABLE_1 = 0x30,
    REG31_IBUS_ADC = 0x31,
    REG33_IBAT_ADC = 0x33,
    REG35_VBUS_ADC = 0x35,
    REG37_VAC1_ADC = 0x37,
    REG39_VAC2_ADC = 0x39,
    REG3B_VBAT_ADC = 0x3B,
    REG3D_VSYS_ADC = 0x3D,
    REG3F_TS_ADC = 0x3F,
    REG41_TDIE_ADC = 0x41,
    REG43_DP_ADC = 0x43,
    REG45_DM_ADC = 0x45,
    REG48_PART_INFO = 0x48,
};

#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
#define CHARGER_TASK_CORE 0
#else
#define CHARGER_TASK_CORE 0
#endif

static bool s_started;
static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_i2c_dev;
static SemaphoreHandle_t s_state_mutex;
static TaskHandle_t s_task_handle;
static uint32_t s_last_update_ms;
static volatile uint32_t s_int_irq_count;
static volatile uint32_t s_last_int_irq_ms;
static charger_service_snapshot_t s_snapshot;

static uint32_t ticks_to_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static uint32_t ticks_to_ms_from_isr(void)
{
    return (uint32_t)(xTaskGetTickCountFromISR() * portTICK_PERIOD_MS);
}

static uint32_t elapsed_since(uint32_t now_ms, uint32_t timestamp_ms)
{
    if (timestamp_ms == 0) {
        return UINT32_MAX;
    }
    return now_ms - timestamp_ms;
}

static bool state_lock(TickType_t timeout)
{
    return s_state_mutex != NULL &&
           xSemaphoreTake(s_state_mutex, timeout) == pdTRUE;
}

static void state_unlock(void)
{
    if (s_state_mutex != NULL) {
        xSemaphoreGive(s_state_mutex);
    }
}

static int read_gpio_level_or_invalid(int gpio_num)
{
    if (gpio_num == BOARD_CONFIG_GPIO_UNUSED) {
        return CHARGER_GPIO_INVALID_LEVEL;
    }
    return gpio_get_level((gpio_num_t)gpio_num);
}

static bool gpio_level_is_active(int level, int active_level)
{
    return level != CHARGER_GPIO_INVALID_LEVEL && level == active_level;
}

static void IRAM_ATTR charger_int_isr_handler(void *arg)
{
    (void)arg;
    s_int_irq_count++;
    s_last_int_irq_ms = ticks_to_ms_from_isr();

    const TaskHandle_t task = s_task_handle;
    if (task != NULL) {
        BaseType_t should_yield = pdFALSE;
        vTaskNotifyGiveFromISR(task, &should_yield);
        if (should_yield == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

static uint16_t read_be_u16(const uint8_t *data, uint8_t offset)
{
    return ((uint16_t)data[offset] << 8) | (uint16_t)data[offset + 1U];
}

static int16_t read_be_i16(const uint8_t *data, uint8_t offset)
{
    return (int16_t)read_be_u16(data, offset);
}

static uint8_t part_number(uint8_t part_info)
{
    return (uint8_t)((part_info >> 3U) & 0x07U);
}

static uint8_t device_revision(uint8_t part_info)
{
    return (uint8_t)(part_info & 0x07U);
}

static esp_err_t charger_gpio_init(void)
{
    if (BOARD_CONFIG_BQ25792_INT_GPIO != BOARD_CONFIG_GPIO_UNUSED) {
        const gpio_config_t int_config = {
            .pin_bit_mask = 1ULL << BOARD_CONFIG_BQ25792_INT_GPIO,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&int_config), TAG,
                            "configure BQ25792 INT GPIO failed");

        esp_err_t err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "BQ25792 INT service unavailable: %s",
                     esp_err_to_name(err));
            return err;
        }

        ESP_RETURN_ON_ERROR(gpio_set_intr_type(
                                BOARD_CONFIG_BQ25792_INT_GPIO,
                                BOARD_CONFIG_BQ25792_INT_ACTIVE_LEVEL
                                    ? GPIO_INTR_POSEDGE
                                    : GPIO_INTR_NEGEDGE),
                            TAG, "set BQ25792 INT edge failed");
        err = gpio_isr_handler_add(BOARD_CONFIG_BQ25792_INT_GPIO,
                                   charger_int_isr_handler, NULL);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "BQ25792 INT handler unavailable: %s",
                     esp_err_to_name(err));
            return err;
        }
        ESP_RETURN_ON_ERROR(gpio_intr_enable(BOARD_CONFIG_BQ25792_INT_GPIO),
                            TAG, "enable BQ25792 INT GPIO failed");
    }

    if (BOARD_CONFIG_BQ25792_PG_GPIO != BOARD_CONFIG_GPIO_UNUSED) {
        const gpio_config_t pg_config = {
            .pin_bit_mask = 1ULL << BOARD_CONFIG_BQ25792_PG_GPIO,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&pg_config), TAG,
                            "configure BQ25792 PG GPIO failed");
    }

    if (BOARD_CONFIG_BQ25792_QON_GPIO != BOARD_CONFIG_GPIO_UNUSED) {
        const gpio_config_t qon_config = {
            .pin_bit_mask = 1ULL << BOARD_CONFIG_BQ25792_QON_GPIO,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&qon_config), TAG,
                            "configure BQ25792 QON GPIO failed");
    }

    return ESP_OK;
}

static esp_err_t charger_i2c_init(void)
{
    esp_err_t err = i2c_bus_service_get(&s_i2c_bus);
    if (err != ESP_OK) {
        return err;
    }

    const i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = APP_BQ25792_I2C_ADDRESS,
        .scl_speed_hz = APP_BQ25792_I2C_CLOCK_HZ,
        .scl_wait_us = 20000,
    };
    err = i2c_master_bus_add_device(s_i2c_bus, &dev_config, &s_i2c_dev);
    if (err != ESP_OK) {
        return err;
    }

    if (!i2c_bus_service_lock(pdMS_TO_TICKS(CHARGER_I2C_LOCK_TIMEOUT_MS))) {
        return ESP_ERR_TIMEOUT;
    }
    err = i2c_master_probe(s_i2c_bus, APP_BQ25792_I2C_ADDRESS,
                           CHARGER_I2C_TIMEOUT_MS);
    i2c_bus_service_unlock();
    return err;
}

static esp_err_t charger_read_bytes(uint8_t start_reg, uint8_t *data,
                                    size_t data_len)
{
    if (data == NULL || data_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!i2c_bus_service_lock(pdMS_TO_TICKS(CHARGER_I2C_LOCK_TIMEOUT_MS))) {
        return ESP_ERR_TIMEOUT;
    }
    const esp_err_t err = i2c_master_transmit_receive(
        s_i2c_dev, &start_reg, sizeof(start_reg), data, data_len,
        CHARGER_I2C_TIMEOUT_MS);
    i2c_bus_service_unlock();
    return err;
}

static esp_err_t charger_read_register_map(uint8_t raw[CHARGER_SERVICE_REGISTER_MAP_SIZE])
{
    size_t offset = 0;
    while (offset < CHARGER_SERVICE_REGISTER_MAP_SIZE) {
        size_t chunk_len = APP_BQ25792_REGISTER_READ_CHUNK_BYTES;
        if (chunk_len == 0 || chunk_len > 16U) {
            chunk_len = 8U;
        }
        const size_t remaining = CHARGER_SERVICE_REGISTER_MAP_SIZE - offset;
        if (chunk_len > remaining) {
            chunk_len = remaining;
        }

        const esp_err_t err =
            charger_read_bytes((uint8_t)offset, &raw[offset], chunk_len);
        if (err != ESP_OK) {
            return err;
        }

        offset += chunk_len;
        if (offset < CHARGER_SERVICE_REGISTER_MAP_SIZE &&
            APP_BQ25792_REGISTER_READ_CHUNK_GAP_MS > 0) {
            vTaskDelay(pdMS_TO_TICKS(APP_BQ25792_REGISTER_READ_CHUNK_GAP_MS));
        }
    }
    return ESP_OK;
}

static void update_snapshot_gpio_fields(charger_service_snapshot_t *snapshot,
                                        uint32_t now_ms)
{
    if (snapshot == NULL) {
        return;
    }

    snapshot->int_gpio_level =
        read_gpio_level_or_invalid(BOARD_CONFIG_BQ25792_INT_GPIO);
    snapshot->int_irq_count = s_int_irq_count;
    snapshot->int_last_irq_age_ms =
        elapsed_since(now_ms, (uint32_t)s_last_int_irq_ms);
    snapshot->pg_gpio_level =
        read_gpio_level_or_invalid(BOARD_CONFIG_BQ25792_PG_GPIO);
    snapshot->pg_asserted = gpio_level_is_active(
        snapshot->pg_gpio_level, BOARD_CONFIG_BQ25792_PG_ACTIVE_LEVEL);
    snapshot->qon_gpio_level =
        read_gpio_level_or_invalid(BOARD_CONFIG_BQ25792_QON_GPIO);
}

static void update_snapshot_from_raw(const uint8_t raw[CHARGER_SERVICE_REGISTER_MAP_SIZE],
                                     esp_err_t read_err)
{
    const uint32_t now_ms = ticks_to_ms();

    if (state_lock(pdMS_TO_TICKS(50))) {
        s_snapshot.monitor_enabled = APP_BQ25792_MONITOR_ENABLED_DEFAULT != 0;
        s_snapshot.config_write_supported = true;
        s_snapshot.config_writes_enabled = false;
        s_snapshot.last_error = read_err;
        update_snapshot_gpio_fields(&s_snapshot, now_ms);
        if (read_err == ESP_OK) {
            memcpy(s_snapshot.raw, raw, CHARGER_SERVICE_REGISTER_MAP_SIZE);
            s_snapshot.present = true;
            s_snapshot.read_ok = true;
            s_snapshot.raw_valid = true;
            s_snapshot.read_count++;
            s_last_update_ms = now_ms;
            s_snapshot.last_update_age_ms = 0;

            s_snapshot.part_info = raw[REG48_PART_INFO];
            s_snapshot.part_number = part_number(s_snapshot.part_info);
            s_snapshot.device_revision = device_revision(s_snapshot.part_info);
            memcpy(s_snapshot.charger_status, &raw[REG1B_CHARGER_STATUS_0],
                   sizeof(s_snapshot.charger_status));
            s_snapshot.pg_stat = (raw[REG1B_CHARGER_STATUS_0] & 0x08U) != 0;
            memcpy(s_snapshot.fault_status, &raw[REG20_FAULT_STATUS_0],
                   sizeof(s_snapshot.fault_status));
            memcpy(s_snapshot.charger_flag, &raw[REG22_CHARGER_FLAG_0],
                   sizeof(s_snapshot.charger_flag));
            memcpy(s_snapshot.fault_flag, &raw[REG26_FAULT_FLAG_0],
                   sizeof(s_snapshot.fault_flag));
            s_snapshot.reg10_charger_control_1 = raw[REG10_CHARGER_CONTROL_1];
            s_snapshot.reg14_charger_control_5 = raw[REG14_CHARGER_CONTROL_5];
            s_snapshot.reg2e_adc_control = raw[REG2E_ADC_CONTROL];
            s_snapshot.reg2f_adc_disable_0 = raw[REG2F_ADC_DISABLE_0];
            s_snapshot.reg30_adc_disable_1 = raw[REG30_ADC_DISABLE_1];
            s_snapshot.adc_enabled = (raw[REG2E_ADC_CONTROL] & 0x80U) != 0;
            s_snapshot.ibus_ma = read_be_i16(raw, REG31_IBUS_ADC);
            s_snapshot.ibat_ma = read_be_i16(raw, REG33_IBAT_ADC);
            s_snapshot.vbus_mv = read_be_u16(raw, REG35_VBUS_ADC);
            s_snapshot.vac1_mv = read_be_u16(raw, REG37_VAC1_ADC);
            s_snapshot.vac2_mv = read_be_u16(raw, REG39_VAC2_ADC);
            s_snapshot.vbat_mv = read_be_u16(raw, REG3B_VBAT_ADC);
            s_snapshot.vsys_mv = read_be_u16(raw, REG3D_VSYS_ADC);
            s_snapshot.ts_percent =
                (double)read_be_u16(raw, REG3F_TS_ADC) * 0.0976563;
            s_snapshot.tdie_c =
                (double)read_be_i16(raw, REG41_TDIE_ADC) * 0.5;
            s_snapshot.dp_mv = read_be_u16(raw, REG43_DP_ADC);
            s_snapshot.dm_mv = read_be_u16(raw, REG45_DM_ADC);
        } else {
            s_snapshot.present = false;
            s_snapshot.read_ok = false;
            s_snapshot.error_count++;
        }
        state_unlock();
    }
}

static void charger_log_summary_if_needed(void)
{
    static uint32_t last_log_ms;
    const uint32_t now_ms = ticks_to_ms();
    if ((uint32_t)(now_ms - last_log_ms) < CHARGER_LOG_INTERVAL_MS) {
        return;
    }
    last_log_ms = now_ms;

    charger_service_snapshot_t snapshot = {0};
    charger_service_get_snapshot(&snapshot);
    if (!snapshot.present) {
        ESP_LOGW(TAG, "BQ25792 not present/read failed: err=%s reads=%lu errors=%lu",
                 esp_err_to_name((esp_err_t)snapshot.last_error),
                 (unsigned long)snapshot.read_count,
                 (unsigned long)snapshot.error_count);
        return;
    }

    ESP_LOGI(TAG,
             "BQ25792 REG48=0x%02X PN=%u rev=%u adc=%s VBAT=%umV VSYS=%umV VBUS=%umV IBUS=%dmA IBAT=%dmA TDIE=%.1fC PG=%d INT=%d irq=%lu QON=%d reads=%lu",
             snapshot.part_info, (unsigned)snapshot.part_number,
             (unsigned)snapshot.device_revision,
             snapshot.adc_enabled ? "on" : "off",
             (unsigned)snapshot.vbat_mv, (unsigned)snapshot.vsys_mv,
             (unsigned)snapshot.vbus_mv, (int)snapshot.ibus_ma,
             (int)snapshot.ibat_ma, snapshot.tdie_c,
             snapshot.pg_gpio_level, snapshot.int_gpio_level,
             (unsigned long)snapshot.int_irq_count,
             snapshot.qon_gpio_level,
             (unsigned long)snapshot.read_count);
}

static void charger_task(void *arg)
{
    (void)arg;
    s_task_handle = xTaskGetCurrentTaskHandle();

    ESP_LOGI(TAG,
             "BQ25792 monitor: SDA=%d SCL=%d addr=0x%02X clock=%u Hz interval=%ums chunk=%u gap=%ums INT=%d PG=%d QON=%d core=%d",
             BOARD_CONFIG_BQ25792_SDA_GPIO, BOARD_CONFIG_BQ25792_SCL_GPIO,
             APP_BQ25792_I2C_ADDRESS, (unsigned)APP_BQ25792_I2C_CLOCK_HZ,
             (unsigned)APP_BQ25792_READ_INTERVAL_MS,
             (unsigned)APP_BQ25792_REGISTER_READ_CHUNK_BYTES,
             (unsigned)APP_BQ25792_REGISTER_READ_CHUNK_GAP_MS,
             BOARD_CONFIG_BQ25792_INT_GPIO, BOARD_CONFIG_BQ25792_PG_GPIO,
             BOARD_CONFIG_BQ25792_QON_GPIO,
             CHARGER_TASK_CORE);

    const esp_err_t gpio_err = charger_gpio_init();
    if (gpio_err != ESP_OK) {
        ESP_LOGW(TAG, "BQ25792 GPIO init failed: %s",
                 esp_err_to_name(gpio_err));
    }

    const esp_err_t init_err = charger_i2c_init();
    if (init_err != ESP_OK) {
        ESP_LOGW(TAG, "BQ25792 I2C init/probe failed: %s",
                 esp_err_to_name(init_err));
        update_snapshot_from_raw((uint8_t[CHARGER_SERVICE_REGISTER_MAP_SIZE]){0},
                                 init_err);
    }

    while (true) {
        uint8_t raw[CHARGER_SERVICE_REGISTER_MAP_SIZE] = {0};
        esp_err_t err = ESP_ERR_INVALID_STATE;
        if (s_i2c_dev != NULL) {
            err = charger_read_register_map(raw);
        }
        update_snapshot_from_raw(raw, err);
        charger_log_summary_if_needed();
        (void)ulTaskNotifyTake(pdTRUE,
                               pdMS_TO_TICKS(APP_BQ25792_READ_INTERVAL_MS));
    }
}

esp_err_t charger_service_start(void)
{
    if (!APP_BQ25792_MONITOR_ENABLED_DEFAULT) {
        ESP_LOGI(TAG, "BQ25792 monitor disabled by config");
        return ESP_OK;
    }

    if (s_started) {
        return ESP_OK;
    }

    s_state_mutex = xSemaphoreCreateMutex();
    if (s_state_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    if (state_lock(pdMS_TO_TICKS(50))) {
        memset(&s_snapshot, 0, sizeof(s_snapshot));
        s_snapshot.monitor_enabled = true;
        s_snapshot.config_write_supported = true;
        s_snapshot.config_writes_enabled = false;
        s_snapshot.last_update_age_ms = UINT32_MAX;
        s_snapshot.int_gpio_level = CHARGER_GPIO_INVALID_LEVEL;
        s_snapshot.int_last_irq_age_ms = UINT32_MAX;
        s_snapshot.pg_gpio_level = CHARGER_GPIO_INVALID_LEVEL;
        s_snapshot.qon_gpio_level = CHARGER_GPIO_INVALID_LEVEL;
        s_snapshot.last_error = ESP_OK;
        state_unlock();
    }

    const BaseType_t created = xTaskCreatePinnedToCore(
        charger_task, "bq25792", CHARGER_TASK_STACK_WORDS, NULL,
        CHARGER_TASK_PRIORITY, &s_task_handle, CHARGER_TASK_CORE);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create BQ25792 task");
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    return ESP_OK;
}

void charger_service_get_snapshot(charger_service_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    if (state_lock(pdMS_TO_TICKS(50))) {
        *snapshot = s_snapshot;
        const uint32_t now_ms = ticks_to_ms();
        snapshot->last_update_age_ms = elapsed_since(now_ms, s_last_update_ms);
        update_snapshot_gpio_fields(snapshot, now_ms);
        state_unlock();
    } else {
        memset(snapshot, 0, sizeof(*snapshot));
        snapshot->last_update_age_ms = UINT32_MAX;
        snapshot->int_last_irq_age_ms = UINT32_MAX;
        snapshot->int_gpio_level = CHARGER_GPIO_INVALID_LEVEL;
        snapshot->pg_gpio_level = CHARGER_GPIO_INVALID_LEVEL;
        snapshot->qon_gpio_level = CHARGER_GPIO_INVALID_LEVEL;
        snapshot->last_error = ESP_ERR_TIMEOUT;
    }
}

void charger_service_format_raw_hex(const charger_service_snapshot_t *snapshot,
                                    char *buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0) {
        return;
    }
    buffer[0] = '\0';
    if (snapshot == NULL || !snapshot->raw_valid) {
        return;
    }

    size_t offset = 0;
    for (size_t i = 0; i < CHARGER_SERVICE_REGISTER_MAP_SIZE; ++i) {
        if (offset + 3U > buffer_size) {
            break;
        }
        const int written =
            snprintf(&buffer[offset], buffer_size - offset, "%02X",
                     snapshot->raw[i]);
        if (written < 0) {
            buffer[0] = '\0';
            return;
        }
        offset += (size_t)written;
    }
}
