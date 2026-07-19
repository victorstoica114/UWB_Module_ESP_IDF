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
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "i2c_bus_service.h"
#include "nvs.h"
#include "sdkconfig.h"

static const char *TAG = "charger_service";

enum {
    CHARGER_TASK_STACK_WORDS = 3072,
    CHARGER_TASK_PRIORITY = 4,
    CHARGER_I2C_TIMEOUT_MS = APP_BQ25792_I2C_TRANSACTION_TIMEOUT_MS,
    CHARGER_I2C_PROBE_TIMEOUT_MS = APP_BQ25792_I2C_PROBE_TIMEOUT_MS,
    CHARGER_I2C_LOCK_TIMEOUT_MS = 1000,
    CHARGER_LOG_INTERVAL_MS = 5000,
    CHARGER_GPIO_INVALID_LEVEL = -1,
    REG00_MINIMAL_SYSTEM_VOLTAGE = 0x00,
    REG01_CHARGE_VOLTAGE_LIMIT = 0x01,
    REG03_CHARGE_CURRENT_LIMIT = 0x03,
    REG05_INPUT_VOLTAGE_LIMIT = 0x05,
    REG06_INPUT_CURRENT_LIMIT = 0x06,
    REG09_TERMINATION_CONTROL = 0x09,
    REG0A_RECHARGE_CONTROL = 0x0A,
    REG0D_IOTG_REGULATION = 0x0D,
    REG0E_TIMER_CONTROL = 0x0E,
    REG0F_CHARGER_CONTROL_0 = 0x0F,
    REG10_CHARGER_CONTROL_1 = 0x10,
    REG14_CHARGER_CONTROL_5 = 0x14,
    REG16_TEMPERATURE_CONTROL = 0x16,
    REG17_NTC_CONTROL_0 = 0x17,
    REG18_NTC_CONTROL_1 = 0x18,
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

typedef enum {
    CHARGER_READ_KIND_FULL,
    CHARGER_READ_KIND_QUICK,
} charger_read_kind_t;

#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
#define CHARGER_TASK_CORE 0
#else
#define CHARGER_TASK_CORE 0
#endif

#define CHARGER_NVS_NAMESPACE "charger"
#define KEY_WATCHDOG_DISABLED "wd_dis"
#define KEY_CHARGE_ENABLED "chg_en"
#define KEY_ADC_ENABLED "adc_en"
#define KEY_ADC_CONTINUOUS "adc_cont"
#define KEY_ADC_SAMPLE "adc_samp"
#define KEY_ADC_AVG "adc_avg"
#define KEY_VSYSMIN_MV "vsysmin"
#define KEY_VREG_MV "vreg"
#define KEY_ICHG_MA "ichg"
#define KEY_VINDPM_MV "vindpm"
#define KEY_IINDPM_MA "iindpm"
#define KEY_EXT_ILIM_EN "ext_ilim"
#define KEY_TERM_EN "term_en"
#define KEY_ITERM_MA "iterm"
#define KEY_VRECHG_MV "vrechg"
#define KEY_TRECHG_MS "trechg"
#define KEY_TOPOFF_TIMER_MIN "top_min"
#define KEY_TRICKLE_TIMER_EN "tri_tmr"
#define KEY_PRECHG_TIMER_EN "pre_en"
#define KEY_FAST_TIMER_EN "chg_tmr"
#define KEY_FAST_TIMER_HOURS "chg_hr"
#define KEY_TIMER_2X_EN "tmr2x"
#define KEY_PRECHG_TIMER_MIN "pre_min"

typedef struct {
    bool has_watchdog_disabled;
    bool watchdog_disabled;
    bool has_charge_enabled;
    bool charge_enabled;
    bool has_adc;
    bool adc_enabled;
    bool adc_continuous;
    uint8_t adc_sample;
    bool adc_running_average;
    bool has_minimal_system_voltage_mv;
    uint16_t minimal_system_voltage_mv;
    bool has_charge_voltage_limit_mv;
    uint16_t charge_voltage_limit_mv;
    bool has_charge_current_limit_ma;
    uint16_t charge_current_limit_ma;
    bool has_input_voltage_limit_mv;
    uint16_t input_voltage_limit_mv;
    bool has_input_current_limit_ma;
    uint16_t input_current_limit_ma;
    bool has_external_input_current_limit_enabled;
    bool external_input_current_limit_enabled;
    bool has_termination_recharge;
    bool termination_enabled;
    uint16_t termination_current_ma;
    uint16_t recharge_threshold_offset_mv;
    uint16_t recharge_deglitch_ms;
    bool has_safety_timers;
    uint16_t topoff_timer_minutes;
    bool trickle_timer_enabled;
    bool precharge_timer_enabled;
    bool fast_charge_timer_enabled;
    uint8_t fast_charge_timer_hours;
    bool timer_2x_enabled;
    uint16_t precharge_timer_minutes;
    uint32_t field_count;
} charger_policy_t;

static bool s_started;
static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_i2c_dev;
static SemaphoreHandle_t s_state_mutex;
static TaskHandle_t s_task_handle;
static uint32_t s_last_update_ms;
static volatile uint32_t s_int_irq_count;
static volatile uint32_t s_last_int_irq_ms;
static volatile bool s_full_refresh_requested;
static charger_service_snapshot_t s_snapshot;
static uint32_t s_last_write_ms;
static bool s_applying_saved_policy;
static int s_i2c_scl_measure_error = ESP_ERR_NOT_SUPPORTED;
static uint32_t s_i2c_scl_edges;
static uint32_t s_i2c_scl_elapsed_us;
static uint32_t s_i2c_scl_measured_hz;

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

static uint16_t bq_vsysmin_mv_from_raw(uint8_t raw)
{
    return (uint16_t)(2500U + ((uint16_t)(raw & 0x3FU) * 250U));
}

static uint16_t bq_vreg_mv_from_raw(uint16_t raw)
{
    return (uint16_t)((raw & 0x07FFU) * 10U);
}

static uint16_t bq_ichg_ma_from_raw(uint16_t raw)
{
    return (uint16_t)((raw & 0x01FFU) * 10U);
}

static uint16_t bq_vindpm_mv_from_raw(uint8_t raw)
{
    return (uint16_t)raw * 100U;
}

static uint16_t bq_iindpm_ma_from_raw(uint16_t raw)
{
    return (uint16_t)((raw & 0x01FFU) * 10U);
}

static uint16_t bq_iterm_ma_from_raw(uint8_t raw)
{
    const uint16_t ma = (uint16_t)((raw & 0x1FU) * 40U);
    return ma > 0U ? ma : 40U;
}

static uint16_t bq_vrechg_mv_from_raw(uint8_t raw)
{
    return (uint16_t)(((uint16_t)(raw & 0x0FU) + 1U) * 50U);
}

static uint16_t bq_trechg_ms_from_code(uint8_t code)
{
    static const uint16_t ms[] = {64U, 256U, 1024U, 2048U};
    return ms[code & 0x03U];
}

static bool bq_trechg_code_from_ms(uint16_t ms, uint8_t *code)
{
    if (code == NULL) {
        return false;
    }
    switch (ms) {
    case 64:
        *code = 0U;
        return true;
    case 256:
        *code = 1U;
        return true;
    case 1024:
        *code = 2U;
        return true;
    case 2048:
        *code = 3U;
        return true;
    default:
        return false;
    }
}

static uint8_t bq_fast_charge_timer_hours_from_code(uint8_t code)
{
    static const uint8_t hours[] = {5U, 8U, 12U, 24U};
    return hours[code & 0x03U];
}

static bool bq_fast_charge_timer_code_from_hours(uint8_t hours, uint8_t *code)
{
    if (code == NULL) {
        return false;
    }
    switch (hours) {
    case 5:
        *code = 0U;
        return true;
    case 8:
        *code = 1U;
        return true;
    case 12:
        *code = 2U;
        return true;
    case 24:
        *code = 3U;
        return true;
    default:
        return false;
    }
}

static bool bq_topoff_timer_code_from_minutes(uint16_t minutes, uint8_t *code)
{
    if (code == NULL) {
        return false;
    }
    switch (minutes) {
    case 0:
        *code = 0U;
        return true;
    case 15:
        *code = 1U;
        return true;
    case 30:
        *code = 2U;
        return true;
    case 45:
        *code = 3U;
        return true;
    default:
        return false;
    }
}

static uint8_t bq_battery_soc_percent_from_mv(uint16_t mv)
{
    static const struct {
        uint16_t mv;
        uint8_t percent;
    } table[] = {
        {3300, 0},  {3500, 5},   {3600, 10}, {3700, 20},
        {3750, 30}, {3800, 40},  {3850, 50}, {3900, 60},
        {3970, 70}, {4050, 80},  {4110, 90}, {4200, 100},
    };

    if (mv <= table[0].mv) {
        return table[0].percent;
    }
    const size_t last = (sizeof(table) / sizeof(table[0])) - 1U;
    if (mv >= table[last].mv) {
        return table[last].percent;
    }

    for (size_t i = 1; i <= last; ++i) {
        if (mv <= table[i].mv) {
            const uint16_t low_mv = table[i - 1U].mv;
            const uint16_t high_mv = table[i].mv;
            const uint8_t low_pct = table[i - 1U].percent;
            const uint8_t high_pct = table[i].percent;
            const uint32_t numerator =
                (uint32_t)(mv - low_mv) * (uint32_t)(high_pct - low_pct);
            const uint32_t denominator = (uint32_t)(high_mv - low_mv);
            return (uint8_t)(low_pct +
                             ((numerator + (denominator / 2U)) /
                              denominator));
        }
    }

    return table[last].percent;
}

static bool mv_in_range(uint16_t mv, uint16_t min_mv, uint16_t max_mv)
{
    return mv >= min_mv && mv <= max_mv;
}

static bool ma_in_range(uint16_t ma, uint16_t min_ma, uint16_t max_ma)
{
    return ma >= min_ma && ma <= max_ma;
}

static esp_err_t charger_policy_write_u8(const char *key, uint8_t value)
{
    if (s_applying_saved_policy) {
        return ESP_OK;
    }

    nvs_handle_t handle = 0;
    ESP_RETURN_ON_ERROR(nvs_open(CHARGER_NVS_NAMESPACE, NVS_READWRITE,
                                 &handle),
                        TAG, "open charger NVS failed");

    esp_err_t err = nvs_set_u8(handle, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "charger NVS write %s failed: %s", key,
                 esp_err_to_name(err));
    }
    return err;
}

static esp_err_t charger_policy_write_u16(const char *key, uint16_t value)
{
    if (s_applying_saved_policy) {
        return ESP_OK;
    }

    nvs_handle_t handle = 0;
    ESP_RETURN_ON_ERROR(nvs_open(CHARGER_NVS_NAMESPACE, NVS_READWRITE,
                                 &handle),
                        TAG, "open charger NVS failed");

    esp_err_t err = nvs_set_u16(handle, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "charger NVS write %s failed: %s", key,
                 esp_err_to_name(err));
    }
    return err;
}

static bool charger_policy_read_u8(nvs_handle_t handle, const char *key,
                                   uint8_t *value)
{
    const esp_err_t err = nvs_get_u8(handle, key, value);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "charger NVS read %s failed: %s", key,
                 esp_err_to_name(err));
    }
    return err == ESP_OK;
}

static bool charger_policy_read_u16(nvs_handle_t handle, const char *key,
                                    uint16_t *value)
{
    const esp_err_t err = nvs_get_u16(handle, key, value);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "charger NVS read %s failed: %s", key,
                 esp_err_to_name(err));
    }
    return err == ESP_OK;
}

static esp_err_t charger_policy_load(charger_policy_t *policy)
{
    if (policy == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(policy, 0, sizeof(*policy));
    policy->topoff_timer_minutes = 0U;
    policy->trickle_timer_enabled = true;
    policy->precharge_timer_enabled = true;
    policy->fast_charge_timer_enabled = true;
    policy->fast_charge_timer_hours = 12U;
    policy->timer_2x_enabled = true;
    policy->precharge_timer_minutes = 120U;
    policy->termination_enabled = true;
    policy->termination_current_ma = 200U;
    policy->recharge_threshold_offset_mv = 200U;
    policy->recharge_deglitch_ms = 1024U;

    nvs_handle_t handle = 0;
    const esp_err_t open_err =
        nvs_open(CHARGER_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (open_err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }
    ESP_RETURN_ON_ERROR(open_err, TAG, "open charger NVS failed");

    uint8_t u8 = 0;
    uint16_t u16 = 0;
    if (charger_policy_read_u8(handle, KEY_WATCHDOG_DISABLED, &u8)) {
        policy->has_watchdog_disabled = true;
        policy->watchdog_disabled = u8 != 0U;
        policy->field_count++;
    }
    if (charger_policy_read_u8(handle, KEY_CHARGE_ENABLED, &u8)) {
        policy->has_charge_enabled = true;
        policy->charge_enabled = u8 != 0U;
        policy->field_count++;
    }
    if (charger_policy_read_u8(handle, KEY_ADC_ENABLED, &u8)) {
        policy->has_adc = true;
        policy->adc_enabled = u8 != 0U;
        policy->field_count++;
    }
    if (charger_policy_read_u8(handle, KEY_ADC_CONTINUOUS, &u8)) {
        policy->has_adc = true;
        policy->adc_continuous = u8 != 0U;
    }
    if (charger_policy_read_u8(handle, KEY_ADC_SAMPLE, &u8)) {
        policy->has_adc = true;
        policy->adc_sample = u8 <= 3U ? u8 : 2U;
    }
    if (charger_policy_read_u8(handle, KEY_ADC_AVG, &u8)) {
        policy->has_adc = true;
        policy->adc_running_average = u8 != 0U;
    }
    if (charger_policy_read_u16(handle, KEY_VSYSMIN_MV, &u16)) {
        policy->has_minimal_system_voltage_mv = true;
        policy->minimal_system_voltage_mv = u16;
        policy->field_count++;
    }
    if (charger_policy_read_u16(handle, KEY_VREG_MV, &u16)) {
        policy->has_charge_voltage_limit_mv = true;
        policy->charge_voltage_limit_mv = u16;
        policy->field_count++;
    }
    if (charger_policy_read_u16(handle, KEY_ICHG_MA, &u16)) {
        policy->has_charge_current_limit_ma = true;
        policy->charge_current_limit_ma = u16;
        policy->field_count++;
    }
    if (charger_policy_read_u16(handle, KEY_VINDPM_MV, &u16)) {
        policy->has_input_voltage_limit_mv = true;
        policy->input_voltage_limit_mv = u16;
        policy->field_count++;
    }
    if (charger_policy_read_u16(handle, KEY_IINDPM_MA, &u16)) {
        policy->has_input_current_limit_ma = true;
        policy->input_current_limit_ma = u16;
        policy->field_count++;
    }
    if (charger_policy_read_u8(handle, KEY_EXT_ILIM_EN, &u8)) {
        policy->has_external_input_current_limit_enabled = true;
        policy->external_input_current_limit_enabled = u8 != 0U;
        policy->field_count++;
    }
    if (charger_policy_read_u8(handle, KEY_TERM_EN, &u8)) {
        policy->has_termination_recharge = true;
        policy->termination_enabled = u8 != 0U;
        policy->field_count++;
    }
    if (charger_policy_read_u16(handle, KEY_ITERM_MA, &u16)) {
        policy->has_termination_recharge = true;
        policy->termination_current_ma = u16;
    }
    if (charger_policy_read_u16(handle, KEY_VRECHG_MV, &u16)) {
        policy->has_termination_recharge = true;
        policy->recharge_threshold_offset_mv = u16;
    }
    if (charger_policy_read_u16(handle, KEY_TRECHG_MS, &u16)) {
        policy->has_termination_recharge = true;
        policy->recharge_deglitch_ms = u16;
    }
    if (charger_policy_read_u8(handle, KEY_TOPOFF_TIMER_MIN, &u8)) {
        policy->has_safety_timers = true;
        policy->topoff_timer_minutes = u8;
        policy->field_count++;
    }
    if (charger_policy_read_u8(handle, KEY_TRICKLE_TIMER_EN, &u8)) {
        policy->has_safety_timers = true;
        policy->trickle_timer_enabled = u8 != 0U;
    }
    if (charger_policy_read_u8(handle, KEY_PRECHG_TIMER_EN, &u8)) {
        policy->has_safety_timers = true;
        policy->precharge_timer_enabled = u8 != 0U;
    }
    if (charger_policy_read_u8(handle, KEY_FAST_TIMER_EN, &u8)) {
        policy->has_safety_timers = true;
        policy->fast_charge_timer_enabled = u8 != 0U;
    }
    if (charger_policy_read_u8(handle, KEY_FAST_TIMER_HOURS, &u8)) {
        policy->has_safety_timers = true;
        policy->fast_charge_timer_hours = u8;
    }
    if (charger_policy_read_u8(handle, KEY_TIMER_2X_EN, &u8)) {
        policy->has_safety_timers = true;
        policy->timer_2x_enabled = u8 != 0U;
    }
    if (charger_policy_read_u8(handle, KEY_PRECHG_TIMER_MIN, &u8)) {
        policy->has_safety_timers = true;
        policy->precharge_timer_minutes = u8;
    }

    nvs_close(handle);
    return policy->field_count > 0U || policy->has_adc ||
                   policy->has_safety_timers ||
                   policy->has_termination_recharge
               ? ESP_OK
               : ESP_ERR_NOT_FOUND;
}

static esp_err_t charger_policy_save_adc(bool enabled, bool continuous,
                                         uint8_t sample,
                                         bool running_average)
{
    ESP_RETURN_ON_ERROR(charger_policy_write_u8(KEY_ADC_ENABLED,
                                                enabled ? 1U : 0U),
                        TAG, "persist ADC enable failed");
    ESP_RETURN_ON_ERROR(charger_policy_write_u8(KEY_ADC_CONTINUOUS,
                                                continuous ? 1U : 0U),
                        TAG, "persist ADC mode failed");
    ESP_RETURN_ON_ERROR(charger_policy_write_u8(KEY_ADC_SAMPLE, sample), TAG,
                        "persist ADC sample failed");
    return charger_policy_write_u8(KEY_ADC_AVG,
                                   running_average ? 1U : 0U);
}

static esp_err_t charger_policy_save_safety_timers(
    uint16_t topoff_timer_minutes, bool trickle_timer_enabled,
    bool precharge_timer_enabled, bool fast_charge_timer_enabled,
    uint8_t fast_charge_timer_hours, bool timer_2x_enabled,
    uint16_t precharge_timer_minutes)
{
    ESP_RETURN_ON_ERROR(
        charger_policy_write_u8(KEY_TOPOFF_TIMER_MIN,
                                (uint8_t)topoff_timer_minutes),
        TAG, "persist top-off timer failed");
    ESP_RETURN_ON_ERROR(
        charger_policy_write_u8(KEY_TRICKLE_TIMER_EN,
                                trickle_timer_enabled ? 1U : 0U),
        TAG, "persist trickle timer failed");
    ESP_RETURN_ON_ERROR(
        charger_policy_write_u8(KEY_PRECHG_TIMER_EN,
                                precharge_timer_enabled ? 1U : 0U),
        TAG, "persist precharge timer enable failed");
    ESP_RETURN_ON_ERROR(
        charger_policy_write_u8(KEY_FAST_TIMER_EN,
                                fast_charge_timer_enabled ? 1U : 0U),
        TAG, "persist fast timer enable failed");
    ESP_RETURN_ON_ERROR(
        charger_policy_write_u8(KEY_FAST_TIMER_HOURS,
                                fast_charge_timer_hours),
        TAG, "persist fast timer hours failed");
    ESP_RETURN_ON_ERROR(
        charger_policy_write_u8(KEY_TIMER_2X_EN,
                                timer_2x_enabled ? 1U : 0U),
        TAG, "persist timer 2x failed");
    return charger_policy_write_u8(KEY_PRECHG_TIMER_MIN,
                                   (uint8_t)precharge_timer_minutes);
}

static esp_err_t charger_policy_save_termination_recharge(
    bool termination_enabled, uint16_t termination_current_ma,
    uint16_t recharge_threshold_offset_mv, uint16_t recharge_deglitch_ms)
{
    ESP_RETURN_ON_ERROR(
        charger_policy_write_u8(KEY_TERM_EN, termination_enabled ? 1U : 0U),
        TAG, "persist termination enable failed");
    ESP_RETURN_ON_ERROR(charger_policy_write_u16(KEY_ITERM_MA,
                                                 termination_current_ma),
                        TAG, "persist termination current failed");
    ESP_RETURN_ON_ERROR(charger_policy_write_u16(KEY_VRECHG_MV,
                                                 recharge_threshold_offset_mv),
                        TAG, "persist recharge threshold failed");
    return charger_policy_write_u16(KEY_TRECHG_MS, recharge_deglitch_ms);
}

static void charger_request_refresh_from_task_context(void)
{
    s_full_refresh_requested = true;
    const TaskHandle_t task = s_task_handle;
    if (task != NULL) {
        xTaskNotifyGive(task);
    }
}

static bool charger_take_full_refresh_request(void)
{
    if (!s_full_refresh_requested) {
        return false;
    }
    s_full_refresh_requested = false;
    return true;
}

static void charger_record_write_result(
    const charger_service_write_result_t *result)
{
    if (result == NULL) {
        return;
    }

    const uint32_t now_ms = ticks_to_ms();
    if (state_lock(pdMS_TO_TICKS(50))) {
        s_snapshot.config_write_supported = true;
        s_snapshot.config_writes_enabled = true;
        if (result->err == ESP_OK) {
            s_snapshot.write_count++;
            s_last_write_ms = now_ms;
        } else {
            s_snapshot.write_error_count++;
        }
        s_snapshot.last_write_reg = result->reg;
        s_snapshot.last_write_requested_value = result->requested_value;
        s_snapshot.last_write_mask = result->mask;
        s_snapshot.last_write_before = result->before;
        s_snapshot.last_write_after = result->after;
        s_snapshot.last_write_mask_used = result->mask_used;
        s_snapshot.last_write_changed = result->changed;
        s_snapshot.last_write_error = result->err;
        s_snapshot.last_write_age_ms = elapsed_since(now_ms, s_last_write_ms);
        state_unlock();
    }
}

static void charger_init_write_result(charger_service_write_result_t *result,
                                      uint8_t reg, uint8_t value)
{
    if (result == NULL) {
        return;
    }
    memset(result, 0, sizeof(*result));
    result->reg = reg;
    result->requested_value = value;
    result->err = ESP_OK;
}

static esp_err_t charger_store_result(
    charger_service_write_result_t *result,
    const charger_service_write_result_t *local)
{
    if (result != NULL && local != NULL) {
        *result = *local;
    }
    charger_record_write_result(local);
    return local != NULL ? (esp_err_t)local->err : ESP_ERR_INVALID_ARG;
}

static bool append_result(charger_service_write_result_t *results,
                          size_t result_count, size_t *written_count,
                          const charger_service_write_result_t *item)
{
    if (written_count == NULL || item == NULL) {
        return false;
    }
    if (*written_count < result_count && results != NULL) {
        results[*written_count] = *item;
    }
    (*written_count)++;
    return item->err == ESP_OK;
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

    if (!i2c_bus_service_lock_background(pdMS_TO_TICKS(CHARGER_I2C_LOCK_TIMEOUT_MS))) {
        return ESP_ERR_TIMEOUT;
    }
    err = i2c_master_probe(s_i2c_bus, APP_BQ25792_I2C_ADDRESS,
                           CHARGER_I2C_PROBE_TIMEOUT_MS);
    i2c_bus_service_unlock();
    return err;
}

static uint32_t charger_i2c_estimate_us(uint64_t bits)
{
    if (APP_BQ25792_I2C_CLOCK_HZ == 0) {
        return UINT32_MAX;
    }
    const uint64_t us =
        (bits * 1000000ULL + APP_BQ25792_I2C_CLOCK_HZ - 1ULL) /
            APP_BQ25792_I2C_CLOCK_HZ +
        APP_I2C_BACKGROUND_TRANSFER_MARGIN_US;
    return us > UINT32_MAX ? UINT32_MAX : (uint32_t)us;
}

static uint32_t charger_i2c_read_estimate_us(size_t data_len)
{
    return charger_i2c_estimate_us(27ULL + (uint64_t)data_len * 9ULL);
}

static uint32_t charger_i2c_write_estimate_us(size_t write_len)
{
    return charger_i2c_estimate_us(9ULL + (uint64_t)write_len * 9ULL);
}

static size_t charger_configured_read_chunk_len(void)
{
    size_t chunk_len = APP_BQ25792_REGISTER_READ_CHUNK_BYTES;
    if (chunk_len == 0 || chunk_len > 16U) {
        chunk_len = 16U;
    }
    return chunk_len;
}

static size_t charger_adaptive_read_chunk_len(size_t remaining)
{
    size_t chunk_len = charger_configured_read_chunk_len();
    if (chunk_len > remaining) {
        chunk_len = remaining;
    }

    const int32_t window_us = i2c_bus_service_background_window_us();
    if (window_us == INT32_MAX) {
        return chunk_len;
    }
    if (window_us <= 0) {
        return 1U;
    }

    while (chunk_len > 1U &&
           charger_i2c_read_estimate_us(chunk_len) > (uint32_t)window_us) {
        chunk_len--;
    }
    return chunk_len;
}

static void charger_record_scl_measure(
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

static esp_err_t charger_measure_scl_start(
    i2c_bus_service_scl_measure_t *measure)
{
    if (!APP_BQ25792_I2C_MEASURE_SCL_ENABLED) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return i2c_bus_service_scl_measure_start(BOARD_CONFIG_BQ25792_SCL_GPIO,
                                             measure);
}

static void charger_measure_scl_stop(
    i2c_bus_service_scl_measure_t *measure, uint32_t elapsed_us,
    esp_err_t start_err)
{
    i2c_bus_service_scl_measure_result_t result = {
        .error = start_err,
    };
    if (measure != NULL && measure->unit != NULL) {
        result.error = i2c_bus_service_scl_measure_stop(
            measure, elapsed_us, &result);
    }
    charger_record_scl_measure(&result);
}

static esp_err_t charger_read_bytes(uint8_t start_reg, uint8_t *data,
                                    size_t data_len)
{
    if (data == NULL || data_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_ERR_TIMEOUT;
    for (uint32_t attempt = 0; attempt <= APP_BQ25792_I2C_READ_RETRIES;
         ++attempt) {
        if (!i2c_bus_service_lock_background_for(
                pdMS_TO_TICKS(CHARGER_I2C_LOCK_TIMEOUT_MS),
                charger_i2c_read_estimate_us(data_len))) {
            err = ESP_ERR_TIMEOUT;
        } else {
            i2c_bus_service_scl_measure_t measure = {0};
            const esp_err_t measure_err = charger_measure_scl_start(&measure);
            const int64_t start_us = esp_timer_get_time();
            err = i2c_master_transmit_receive(
                s_i2c_dev, &start_reg, sizeof(start_reg), data, data_len,
                CHARGER_I2C_TIMEOUT_MS);
            const int64_t elapsed_raw = esp_timer_get_time() - start_us;
            charger_measure_scl_stop(
                &measure, elapsed_raw > 0 ? (uint32_t)elapsed_raw : 0U,
                measure_err);
            i2c_bus_service_unlock();
        }
        if (err == ESP_OK || attempt == APP_BQ25792_I2C_READ_RETRIES) {
            return err;
        }
        taskYIELD();
    }
    return err;
}

static esp_err_t charger_write_byte(uint8_t reg, uint8_t value)
{
    if (s_i2c_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[2] = {reg, value};
    esp_err_t err = ESP_ERR_TIMEOUT;
    for (uint32_t attempt = 0; attempt <= APP_BQ25792_I2C_WRITE_RETRIES;
         ++attempt) {
        if (!i2c_bus_service_lock_background_for(
                pdMS_TO_TICKS(CHARGER_I2C_LOCK_TIMEOUT_MS),
                charger_i2c_write_estimate_us(sizeof(data)))) {
            err = ESP_ERR_TIMEOUT;
        } else {
            i2c_bus_service_scl_measure_t measure = {0};
            const esp_err_t measure_err = charger_measure_scl_start(&measure);
            const int64_t start_us = esp_timer_get_time();
            err = i2c_master_transmit(s_i2c_dev, data, sizeof(data),
                                      CHARGER_I2C_TIMEOUT_MS);
            const int64_t elapsed_raw = esp_timer_get_time() - start_us;
            charger_measure_scl_stop(
                &measure, elapsed_raw > 0 ? (uint32_t)elapsed_raw : 0U,
                measure_err);
            i2c_bus_service_unlock();
        }
        if (err == ESP_OK || attempt == APP_BQ25792_I2C_WRITE_RETRIES) {
            return err;
        }
        taskYIELD();
    }
    return err;
}

static esp_err_t charger_read_register_range_chunked(
    uint8_t start_reg, uint8_t *data, size_t data_len)
{
    size_t offset = 0;
    while (offset < data_len) {
        const size_t remaining = data_len - offset;
        const size_t chunk_len = charger_adaptive_read_chunk_len(remaining);

        const esp_err_t err =
            charger_read_bytes((uint8_t)(start_reg + offset), &data[offset],
                               chunk_len);
        if (err != ESP_OK) {
            return err;
        }

        offset += chunk_len;
        if (offset < data_len &&
            APP_BQ25792_REGISTER_READ_CHUNK_GAP_MS > 0) {
            vTaskDelay(pdMS_TO_TICKS(APP_BQ25792_REGISTER_READ_CHUNK_GAP_MS));
        }
    }
    return ESP_OK;
}

static esp_err_t charger_read_register_map(
    uint8_t raw[CHARGER_SERVICE_REGISTER_MAP_SIZE])
{
    return charger_read_register_range_chunked(
        0, raw, CHARGER_SERVICE_REGISTER_MAP_SIZE);
}

static esp_err_t charger_read_quick_registers(
    uint8_t raw[CHARGER_SERVICE_REGISTER_MAP_SIZE])
{
    const uint8_t start_reg = REG1B_CHARGER_STATUS_0;
    const uint8_t end_reg = REG45_DM_ADC;
    const size_t read_len = (size_t)(end_reg - start_reg + 1U);
    return charger_read_register_range_chunked(start_reg, &raw[start_reg],
                                               read_len);
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
    snapshot->qon_asserted = gpio_level_is_active(
        snapshot->qon_gpio_level, BOARD_CONFIG_BQ25792_QON_ACTIVE_LEVEL);
}

static void update_snapshot_from_raw(const uint8_t raw[CHARGER_SERVICE_REGISTER_MAP_SIZE],
                                     esp_err_t read_err,
                                     charger_read_kind_t read_kind,
                                     uint32_t read_duration_ms)
{
    const uint32_t now_ms = ticks_to_ms();

    if (state_lock(pdMS_TO_TICKS(50))) {
        s_snapshot.monitor_enabled = APP_BQ25792_MONITOR_ENABLED_DEFAULT != 0;
        s_snapshot.config_write_supported = true;
        s_snapshot.config_writes_enabled = true;
        s_snapshot.last_error = read_err;
        s_snapshot.i2c_clock_hz = APP_BQ25792_I2C_CLOCK_HZ;
        s_snapshot.i2c_scl_measure_error = s_i2c_scl_measure_error;
        s_snapshot.i2c_scl_edges = s_i2c_scl_edges;
        s_snapshot.i2c_scl_elapsed_us = s_i2c_scl_elapsed_us;
        s_snapshot.i2c_scl_measured_hz = s_i2c_scl_measured_hz;
        update_snapshot_gpio_fields(&s_snapshot, now_ms);
        s_snapshot.last_write_age_ms = elapsed_since(now_ms, s_last_write_ms);
        if (read_err == ESP_OK) {
            memcpy(s_snapshot.raw, raw, CHARGER_SERVICE_REGISTER_MAP_SIZE);
            s_snapshot.present = true;
            s_snapshot.read_ok = true;
            s_snapshot.raw_valid = true;
            s_snapshot.read_count++;
            if (read_kind == CHARGER_READ_KIND_FULL) {
                s_snapshot.full_read_count++;
                s_snapshot.last_read_full = true;
            } else {
                s_snapshot.quick_read_count++;
                s_snapshot.last_read_full = false;
            }
            s_snapshot.last_read_duration_ms = read_duration_ms;
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
            s_snapshot.reg09_termination_control =
                raw[REG09_TERMINATION_CONTROL];
            s_snapshot.reg0a_recharge_control = raw[REG0A_RECHARGE_CONTROL];
            s_snapshot.reg0d_iotg_regulation = raw[REG0D_IOTG_REGULATION];
            s_snapshot.reg0e_timer_control = raw[REG0E_TIMER_CONTROL];
            s_snapshot.reg16_temperature_control =
                raw[REG16_TEMPERATURE_CONTROL];
            s_snapshot.reg17_ntc_control_0 = raw[REG17_NTC_CONTROL_0];
            s_snapshot.reg18_ntc_control_1 = raw[REG18_NTC_CONTROL_1];
            s_snapshot.reg0f_charger_control_0 = raw[REG0F_CHARGER_CONTROL_0];
            s_snapshot.reg10_charger_control_1 = raw[REG10_CHARGER_CONTROL_1];
            s_snapshot.reg14_charger_control_5 = raw[REG14_CHARGER_CONTROL_5];
            s_snapshot.reg2e_adc_control = raw[REG2E_ADC_CONTROL];
            s_snapshot.reg2f_adc_disable_0 = raw[REG2F_ADC_DISABLE_0];
            s_snapshot.reg30_adc_disable_1 = raw[REG30_ADC_DISABLE_1];
            s_snapshot.minimal_system_voltage_mv =
                bq_vsysmin_mv_from_raw(raw[REG00_MINIMAL_SYSTEM_VOLTAGE]);
            s_snapshot.charge_voltage_limit_mv =
                bq_vreg_mv_from_raw(read_be_u16(raw, REG01_CHARGE_VOLTAGE_LIMIT));
            s_snapshot.charge_current_limit_ma =
                bq_ichg_ma_from_raw(read_be_u16(raw, REG03_CHARGE_CURRENT_LIMIT));
            s_snapshot.input_voltage_limit_mv =
                bq_vindpm_mv_from_raw(raw[REG05_INPUT_VOLTAGE_LIMIT]);
            s_snapshot.input_current_limit_ma =
                bq_iindpm_ma_from_raw(read_be_u16(raw, REG06_INPUT_CURRENT_LIMIT));
            s_snapshot.charge_enabled =
                (raw[REG0F_CHARGER_CONTROL_0] & 0x20U) != 0U;
            s_snapshot.termination_enabled =
                (raw[REG0F_CHARGER_CONTROL_0] & 0x02U) != 0U;
            s_snapshot.termination_current_ma =
                bq_iterm_ma_from_raw(raw[REG09_TERMINATION_CONTROL]);
            s_snapshot.recharge_threshold_offset_mv =
                bq_vrechg_mv_from_raw(raw[REG0A_RECHARGE_CONTROL]);
            s_snapshot.recharge_threshold_mv =
                s_snapshot.charge_voltage_limit_mv >
                        s_snapshot.recharge_threshold_offset_mv
                    ? (uint16_t)(s_snapshot.charge_voltage_limit_mv -
                                 s_snapshot.recharge_threshold_offset_mv)
                    : 0U;
            s_snapshot.recharge_deglitch_ms =
                bq_trechg_ms_from_code(
                    (uint8_t)((raw[REG0A_RECHARGE_CONTROL] >> 4U) & 0x03U));
            s_snapshot.iindpm_active =
                (raw[REG1B_CHARGER_STATUS_0] & 0x80U) != 0U;
            s_snapshot.vindpm_active =
                (raw[REG1B_CHARGER_STATUS_0] & 0x40U) != 0U;
            s_snapshot.charge_status_code =
                (uint8_t)((raw[REG1B_CHARGER_STATUS_0 + 1U] >> 5U) & 0x07U);
            s_snapshot.vbus_status_code =
                (uint8_t)((raw[REG1B_CHARGER_STATUS_0 + 1U] >> 1U) & 0x0FU);
            s_snapshot.vsys_regulation_active =
                (raw[REG1B_CHARGER_STATUS_0 + 3U] & 0x10U) != 0U;
            s_snapshot.charge_safety_timer_expired =
                (raw[REG1B_CHARGER_STATUS_0 + 3U] & 0x08U) != 0U;
            s_snapshot.battery_overvoltage_active =
                (raw[REG20_FAULT_STATUS_0] & 0x20U) != 0U;
            s_snapshot.topoff_timer_flag =
                (raw[REG22_CHARGER_FLAG_0 + 2U] & 0x01U) != 0U;
            s_snapshot.precharge_timer_flag =
                (raw[REG22_CHARGER_FLAG_0 + 2U] & 0x02U) != 0U;
            s_snapshot.trickle_timer_flag =
                (raw[REG22_CHARGER_FLAG_0 + 2U] & 0x04U) != 0U;
            s_snapshot.fast_charge_timer_flag =
                (raw[REG22_CHARGER_FLAG_0 + 2U] & 0x08U) != 0U;
            s_snapshot.watchdog_setting =
                (uint8_t)(raw[REG10_CHARGER_CONTROL_1] & 0x07U);
            s_snapshot.watchdog_disabled = s_snapshot.watchdog_setting == 0U;
            s_snapshot.topoff_timer_minutes =
                (uint16_t)(((raw[REG0E_TIMER_CONTROL] >> 6U) & 0x03U) * 15U);
            s_snapshot.trickle_timer_enabled =
                (raw[REG0E_TIMER_CONTROL] & 0x20U) != 0U;
            s_snapshot.precharge_timer_enabled =
                (raw[REG0E_TIMER_CONTROL] & 0x10U) != 0U;
            s_snapshot.fast_charge_timer_enabled =
                (raw[REG0E_TIMER_CONTROL] & 0x08U) != 0U;
            s_snapshot.fast_charge_timer_hours =
                bq_fast_charge_timer_hours_from_code(
                    (uint8_t)((raw[REG0E_TIMER_CONTROL] >> 1U) & 0x03U));
            s_snapshot.timer_2x_enabled =
                (raw[REG0E_TIMER_CONTROL] & 0x01U) != 0U;
            s_snapshot.precharge_timer_minutes =
                (raw[REG0D_IOTG_REGULATION] & 0x80U) != 0U ? 30U : 120U;
            s_snapshot.external_input_current_limit_enabled =
                (raw[REG14_CHARGER_CONTROL_5] & 0x02U) != 0U;
            s_snapshot.adc_sample =
                (uint8_t)((raw[REG2E_ADC_CONTROL] >> 4U) & 0x03U);
            s_snapshot.adc_continuous =
                (raw[REG2E_ADC_CONTROL] & 0x40U) == 0U;
            s_snapshot.adc_running_average =
                (raw[REG2E_ADC_CONTROL] & 0x08U) != 0U;
            s_snapshot.ibat_discharge_sense_enabled =
                (raw[REG14_CHARGER_CONTROL_5] & 0x20U) != 0U;
            s_snapshot.adc_enabled = (raw[REG2E_ADC_CONTROL] & 0x80U) != 0;
            s_snapshot.ibus_ma = read_be_i16(raw, REG31_IBUS_ADC);
            s_snapshot.ibat_ma = read_be_i16(raw, REG33_IBAT_ADC);
            s_snapshot.vbus_mv = read_be_u16(raw, REG35_VBUS_ADC);
            s_snapshot.vac1_mv = read_be_u16(raw, REG37_VAC1_ADC);
            s_snapshot.vac2_mv = read_be_u16(raw, REG39_VAC2_ADC);
            s_snapshot.vbat_mv = read_be_u16(raw, REG3B_VBAT_ADC);
            s_snapshot.vsys_mv = read_be_u16(raw, REG3D_VSYS_ADC);
            s_snapshot.battery_soc_valid = s_snapshot.vbat_mv > 0U;
            s_snapshot.battery_soc_percent =
                s_snapshot.battery_soc_valid
                    ? bq_battery_soc_percent_from_mv(s_snapshot.vbat_mv)
                    : 0U;
            s_snapshot.ts_percent =
                (double)read_be_u16(raw, REG3F_TS_ADC) * 0.0976563;
            s_snapshot.ts_ignore = (raw[REG18_NTC_CONTROL_1] & 0x01U) != 0U;
            s_snapshot.ts_cold_active =
                (raw[REG1B_CHARGER_STATUS_0 + 4U] & 0x08U) != 0U;
            s_snapshot.ts_cool_active =
                (raw[REG1B_CHARGER_STATUS_0 + 4U] & 0x04U) != 0U;
            s_snapshot.ts_warm_active =
                (raw[REG1B_CHARGER_STATUS_0 + 4U] & 0x02U) != 0U;
            s_snapshot.ts_hot_active =
                (raw[REG1B_CHARGER_STATUS_0 + 4U] & 0x01U) != 0U;
            s_snapshot.ts_cold_flag =
                (raw[REG22_CHARGER_FLAG_0 + 3U] & 0x08U) != 0U;
            s_snapshot.ts_cool_flag =
                (raw[REG22_CHARGER_FLAG_0 + 3U] & 0x04U) != 0U;
            s_snapshot.ts_warm_flag =
                (raw[REG22_CHARGER_FLAG_0 + 3U] & 0x02U) != 0U;
            s_snapshot.ts_hot_flag =
                (raw[REG22_CHARGER_FLAG_0 + 3U] & 0x01U) != 0U;
            s_snapshot.tdie_c =
                (double)read_be_i16(raw, REG41_TDIE_ADC) * 0.5;
            s_snapshot.dp_mv = read_be_u16(raw, REG43_DP_ADC);
            s_snapshot.dm_mv = read_be_u16(raw, REG45_DM_ADC);
        } else {
            s_snapshot.present = false;
            s_snapshot.read_ok = false;
            s_snapshot.error_count++;
            s_snapshot.last_read_full = read_kind == CHARGER_READ_KIND_FULL;
            s_snapshot.last_read_duration_ms = read_duration_ms;
        }
        state_unlock();
    }
}

static esp_err_t charger_perform_read(charger_read_kind_t read_kind)
{
    uint8_t raw[CHARGER_SERVICE_REGISTER_MAP_SIZE] = {0};

    if (read_kind == CHARGER_READ_KIND_QUICK && state_lock(pdMS_TO_TICKS(50))) {
        if (!s_snapshot.raw_valid) {
            read_kind = CHARGER_READ_KIND_FULL;
        }
        memcpy(raw, s_snapshot.raw, CHARGER_SERVICE_REGISTER_MAP_SIZE);
        state_unlock();
    }

    const uint32_t start_ms = ticks_to_ms();
    esp_err_t err = ESP_ERR_INVALID_STATE;
    if (s_i2c_dev != NULL) {
        err = read_kind == CHARGER_READ_KIND_FULL
                  ? charger_read_register_map(raw)
                  : charger_read_quick_registers(raw);
    }
    const uint32_t duration_ms = ticks_to_ms() - start_ms;
    update_snapshot_from_raw(raw, err, read_kind, duration_ms);
    return err;
}

static void charger_note_policy_result(const char *name, esp_err_t err,
                                       esp_err_t *first_err,
                                       uint32_t *applied_count)
{
    if (err == ESP_OK) {
        if (applied_count != NULL) {
            (*applied_count)++;
        }
        return;
    }
    ESP_LOGW(TAG, "BQ25792 saved policy apply failed for %s: %s", name,
             esp_err_to_name(err));
    if (first_err != NULL && *first_err == ESP_OK) {
        *first_err = err;
    }
}

static esp_err_t charger_apply_saved_policy(void)
{
    charger_policy_t policy = {0};
    const esp_err_t load_err = charger_policy_load(&policy);
    if (load_err == ESP_ERR_NOT_FOUND) {
        ESP_LOGI(TAG, "BQ25792 has no saved charger policy in NVS");
        return ESP_OK;
    }
    if (load_err != ESP_OK) {
        ESP_LOGW(TAG, "BQ25792 saved policy load failed: %s",
                 esp_err_to_name(load_err));
        return load_err;
    }

    s_applying_saved_policy = true;
    esp_err_t first_err = ESP_OK;
    uint32_t applied_count = 0;

    if (policy.has_watchdog_disabled && policy.watchdog_disabled) {
        charger_service_write_result_t result = {0};
        charger_note_policy_result(
            "watchdog", charger_service_set_watchdog_disabled(&result),
            &first_err, &applied_count);
    }
    if (policy.has_adc) {
        charger_service_write_result_t results[6] = {0};
        size_t written_count = 0;
        (void)written_count;
        charger_note_policy_result(
            "adc", charger_service_set_adc(policy.adc_enabled,
                                            policy.adc_continuous,
                                            policy.adc_sample,
                                            policy.adc_running_average,
                                            results, 6, &written_count),
            &first_err, &applied_count);
    }
    if (policy.has_minimal_system_voltage_mv) {
        charger_service_write_result_t results[4] = {0};
        size_t written_count = 0;
        charger_note_policy_result(
            "VSYSMIN",
            charger_service_set_minimal_system_voltage_mv(
                policy.minimal_system_voltage_mv, results, 4, &written_count),
            &first_err, &applied_count);
    }
    if (policy.has_charge_voltage_limit_mv) {
        charger_service_write_result_t results[4] = {0};
        size_t written_count = 0;
        charger_note_policy_result(
            "VREG",
            charger_service_set_charge_voltage_limit_mv(
                policy.charge_voltage_limit_mv, results, 4, &written_count),
            &first_err, &applied_count);
    }
    if (policy.has_charge_current_limit_ma) {
        charger_service_write_result_t results[4] = {0};
        size_t written_count = 0;
        charger_note_policy_result(
            "ICHG",
            charger_service_set_charge_current_limit_ma(
                policy.charge_current_limit_ma, results, 4, &written_count),
            &first_err, &applied_count);
    }
    if (policy.has_input_voltage_limit_mv) {
        charger_service_write_result_t result = {0};
        charger_note_policy_result(
            "VINDPM",
            charger_service_set_input_voltage_limit_mv(
                policy.input_voltage_limit_mv, &result),
            &first_err, &applied_count);
    }
    if (policy.has_external_input_current_limit_enabled) {
        charger_service_write_result_t result = {0};
        charger_note_policy_result(
            "EN_EXTILIM",
            charger_service_set_external_input_current_limit_enabled(
                policy.external_input_current_limit_enabled, &result),
            &first_err, &applied_count);
    }
    if (policy.has_input_current_limit_ma) {
        charger_service_write_result_t results[4] = {0};
        size_t written_count = 0;
        charger_note_policy_result(
            "IINDPM",
            charger_service_set_input_current_limit_ma(
                policy.input_current_limit_ma, results, 4, &written_count),
            &first_err, &applied_count);
    }
    if (policy.has_termination_recharge) {
        charger_service_write_result_t results[4] = {0};
        size_t written_count = 0;
        charger_note_policy_result(
            "termination/recharge",
            charger_service_set_termination_recharge(
                policy.termination_enabled, policy.termination_current_ma,
                policy.recharge_threshold_offset_mv,
                policy.recharge_deglitch_ms, results, 4, &written_count),
            &first_err, &applied_count);
    }
    if (policy.has_safety_timers) {
        charger_service_write_result_t results[4] = {0};
        size_t written_count = 0;
        charger_note_policy_result(
            "safety timers",
            charger_service_set_safety_timers(
                policy.topoff_timer_minutes, policy.trickle_timer_enabled,
                policy.precharge_timer_enabled,
                policy.fast_charge_timer_enabled,
                policy.fast_charge_timer_hours, policy.timer_2x_enabled,
                policy.precharge_timer_minutes, results, 4,
                &written_count),
            &first_err, &applied_count);
    }
    if (policy.has_charge_enabled) {
        charger_service_write_result_t result = {0};
        charger_note_policy_result(
            "EN_CHG",
            charger_service_set_charge_enabled(policy.charge_enabled, &result),
            &first_err, &applied_count);
    }

    s_applying_saved_policy = false;
    s_full_refresh_requested = false;
    (void)ulTaskNotifyTake(pdTRUE, 0);

    ESP_LOGI(TAG, "BQ25792 saved policy applied: fields=%lu applied=%lu err=%s",
             (unsigned long)policy.field_count,
             (unsigned long)applied_count,
             esp_err_to_name(first_err));
    return first_err;
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
             "BQ25792 REG48=0x%02X PN=%u rev=%u adc=%s VBAT=%umV SOC=%u%% VSYS=%umV VBUS=%umV IBUS=%dmA IBAT=%dmA TDIE=%.1fC PG=%d INT=%d irq=%lu QON=%d reads=%lu",
             snapshot.part_info, (unsigned)snapshot.part_number,
             (unsigned)snapshot.device_revision,
             snapshot.adc_enabled ? "on" : "off",
             (unsigned)snapshot.vbat_mv,
             (unsigned)(snapshot.battery_soc_valid
                            ? snapshot.battery_soc_percent
                            : 0U),
             (unsigned)snapshot.vsys_mv,
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
                                 init_err, CHARGER_READ_KIND_FULL, 0);
    }

    if (s_i2c_dev != NULL) {
        (void)charger_apply_saved_policy();
        (void)charger_perform_read(CHARGER_READ_KIND_FULL);
        charger_log_summary_if_needed();
    }

    while (true) {
        const uint32_t notified = ulTaskNotifyTake(
            pdTRUE, pdMS_TO_TICKS(APP_BQ25792_READ_INTERVAL_MS));
        const bool full_requested = charger_take_full_refresh_request();
        const charger_read_kind_t read_kind =
            (notified == 0U || full_requested) ? CHARGER_READ_KIND_FULL
                                               : CHARGER_READ_KIND_QUICK;
        (void)charger_perform_read(read_kind);
        charger_log_summary_if_needed();
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
        s_snapshot.config_writes_enabled = true;
        s_snapshot.last_update_age_ms = UINT32_MAX;
        s_snapshot.last_write_age_ms = UINT32_MAX;
        s_snapshot.i2c_clock_hz = APP_BQ25792_I2C_CLOCK_HZ;
        s_snapshot.i2c_scl_measure_error = ESP_ERR_NOT_SUPPORTED;
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
        snapshot->last_write_age_ms = elapsed_since(now_ms, s_last_write_ms);
        update_snapshot_gpio_fields(snapshot, now_ms);
        state_unlock();
    } else {
        memset(snapshot, 0, sizeof(*snapshot));
        snapshot->last_update_age_ms = UINT32_MAX;
        snapshot->last_write_age_ms = UINT32_MAX;
        snapshot->int_last_irq_age_ms = UINT32_MAX;
        snapshot->int_gpio_level = CHARGER_GPIO_INVALID_LEVEL;
        snapshot->pg_gpio_level = CHARGER_GPIO_INVALID_LEVEL;
        snapshot->qon_gpio_level = CHARGER_GPIO_INVALID_LEVEL;
        snapshot->last_error = ESP_ERR_TIMEOUT;
        snapshot->i2c_clock_hz = APP_BQ25792_I2C_CLOCK_HZ;
        snapshot->i2c_scl_measure_error = ESP_ERR_TIMEOUT;
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

esp_err_t charger_service_read_register(uint8_t reg, uint8_t *value)
{
    if (value == NULL || reg >= CHARGER_SERVICE_REGISTER_MAP_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_i2c_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return charger_read_bytes(reg, value, 1);
}

esp_err_t charger_service_write_register(
    uint8_t reg, uint8_t value, charger_service_write_result_t *result)
{
    charger_service_write_result_t local = {0};
    charger_init_write_result(&local, reg, value);
    if (reg >= CHARGER_SERVICE_REGISTER_MAP_SIZE) {
        local.err = ESP_ERR_INVALID_ARG;
        return charger_store_result(result, &local);
    }

    esp_err_t err = charger_service_read_register(reg, &local.before);
    if (err != ESP_OK) {
        local.err = err;
        return charger_store_result(result, &local);
    }

    err = charger_write_byte(reg, value);
    if (err != ESP_OK) {
        local.err = err;
        return charger_store_result(result, &local);
    }

    err = charger_service_read_register(reg, &local.after);
    if (err != ESP_OK) {
        local.err = err;
        return charger_store_result(result, &local);
    }

    local.changed = local.before != local.after;
    charger_request_refresh_from_task_context();
    return charger_store_result(result, &local);
}

esp_err_t charger_service_update_register_bits(
    uint8_t reg, uint8_t mask, uint8_t value,
    charger_service_write_result_t *result)
{
    charger_service_write_result_t local = {0};
    charger_init_write_result(&local, reg, value);
    local.mask = mask;
    local.mask_used = true;
    if (reg >= CHARGER_SERVICE_REGISTER_MAP_SIZE || mask == 0U) {
        local.err = ESP_ERR_INVALID_ARG;
        return charger_store_result(result, &local);
    }

    esp_err_t err = charger_service_read_register(reg, &local.before);
    if (err != ESP_OK) {
        local.err = err;
        return charger_store_result(result, &local);
    }

    const uint8_t next = (uint8_t)((local.before & (uint8_t)~mask) |
                                  (value & mask));
    err = charger_write_byte(reg, next);
    if (err != ESP_OK) {
        local.err = err;
        return charger_store_result(result, &local);
    }

    err = charger_service_read_register(reg, &local.after);
    if (err != ESP_OK) {
        local.err = err;
        return charger_store_result(result, &local);
    }

    local.requested_value = next;
    local.changed = local.before != local.after;
    charger_request_refresh_from_task_context();
    return charger_store_result(result, &local);
}

esp_err_t charger_service_set_watchdog_disabled(
    charger_service_write_result_t *result)
{
    const esp_err_t err = charger_service_update_register_bits(
        REG10_CHARGER_CONTROL_1, 0x07U, 0x00U, result);
    if (err != ESP_OK) {
        return err;
    }
    return charger_policy_write_u8(KEY_WATCHDOG_DISABLED, 1U);
}

esp_err_t charger_service_set_charge_enabled(
    bool enabled, charger_service_write_result_t *result)
{
    charger_service_write_result_t watchdog_result = {0};
    esp_err_t err = charger_service_set_watchdog_disabled(&watchdog_result);
    if (err != ESP_OK) {
        return err;
    }

    err = charger_service_update_register_bits(
        REG0F_CHARGER_CONTROL_0, 0x20U, enabled ? 0x20U : 0x00U, result);
    if (err != ESP_OK) {
        return err;
    }
    return charger_policy_write_u8(KEY_CHARGE_ENABLED, enabled ? 1U : 0U);
}

esp_err_t charger_service_set_adc(bool enabled, bool continuous,
                                  uint8_t sample, bool running_average,
                                  charger_service_write_result_t *results,
                                  size_t result_count,
                                  size_t *written_count)
{
    if (written_count == NULL || sample > 3U) {
        return ESP_ERR_INVALID_ARG;
    }
    *written_count = 0;

    charger_service_write_result_t item = {0};
    esp_err_t first_err = charger_service_set_watchdog_disabled(&item);
    append_result(results, result_count, written_count, &item);
    if (first_err != ESP_OK) {
        return first_err;
    }

    if (!enabled) {
        first_err = charger_service_update_register_bits(REG2E_ADC_CONTROL,
                                                         0x80U, 0x00U, &item);
        append_result(results, result_count, written_count, &item);
        if (first_err != ESP_OK) {
            return first_err;
        }
        return charger_policy_save_adc(false, continuous, sample,
                                       running_average);
    }

    first_err = charger_service_write_register(REG2F_ADC_DISABLE_0, 0x00U,
                                               &item);
    append_result(results, result_count, written_count, &item);
    if (first_err != ESP_OK) {
        return first_err;
    }

    first_err = charger_service_write_register(REG30_ADC_DISABLE_1, 0x00U,
                                               &item);
    append_result(results, result_count, written_count, &item);
    if (first_err != ESP_OK) {
        return first_err;
    }

    first_err = charger_service_update_register_bits(REG14_CHARGER_CONTROL_5,
                                                     0x20U, 0x20U, &item);
    append_result(results, result_count, written_count, &item);
    if (first_err != ESP_OK) {
        return first_err;
    }

    uint8_t adc_control = 0x80U | ((uint8_t)(sample & 0x03U) << 4U);
    if (!continuous) {
        adc_control |= 0x40U;
    }
    if (running_average) {
        adc_control |= 0x0CU;
    }
    first_err = charger_service_write_register(REG2E_ADC_CONTROL, adc_control,
                                               &item);
    append_result(results, result_count, written_count, &item);
    if (first_err != ESP_OK) {
        return first_err;
    }
    return charger_policy_save_adc(enabled, continuous, sample,
                                   running_average);
}

esp_err_t charger_service_set_minimal_system_voltage_mv(
    uint16_t mv, charger_service_write_result_t *results, size_t result_count,
    size_t *written_count)
{
    if (written_count == NULL || !mv_in_range(mv, 2500U, 16000U)) {
        return ESP_ERR_INVALID_ARG;
    }
    *written_count = 0;

    charger_service_write_result_t item = {0};
    esp_err_t err = charger_service_set_watchdog_disabled(&item);
    append_result(results, result_count, written_count, &item);
    if (err != ESP_OK) {
        return err;
    }

    const uint8_t raw = (uint8_t)((mv - 2500U + 125U) / 250U);
    err = charger_service_update_register_bits(REG00_MINIMAL_SYSTEM_VOLTAGE,
                                               0x3FU, raw, &item);
    append_result(results, result_count, written_count, &item);
    if (err != ESP_OK) {
        return err;
    }
    return charger_policy_write_u16(KEY_VSYSMIN_MV, mv);
}

esp_err_t charger_service_set_charge_voltage_limit_mv(
    uint16_t mv, charger_service_write_result_t *results, size_t result_count,
    size_t *written_count)
{
    if (written_count == NULL || !mv_in_range(mv, 3000U, 18800U)) {
        return ESP_ERR_INVALID_ARG;
    }
    *written_count = 0;

    charger_service_write_result_t item = {0};
    esp_err_t err = charger_service_set_watchdog_disabled(&item);
    append_result(results, result_count, written_count, &item);
    if (err != ESP_OK) {
        return err;
    }

    const uint16_t raw = (uint16_t)((mv + 5U) / 10U) & 0x07FFU;
    err = charger_service_write_register(REG01_CHARGE_VOLTAGE_LIMIT,
                                         (uint8_t)((raw >> 8U) & 0x07U),
                                         &item);
    append_result(results, result_count, written_count, &item);
    if (err != ESP_OK) {
        return err;
    }

    err = charger_service_write_register(REG01_CHARGE_VOLTAGE_LIMIT + 1U,
                                         (uint8_t)(raw & 0xFFU), &item);
    append_result(results, result_count, written_count, &item);
    if (err != ESP_OK) {
        return err;
    }
    return charger_policy_write_u16(KEY_VREG_MV, mv);
}

esp_err_t charger_service_set_charge_current_limit_ma(
    uint16_t ma, charger_service_write_result_t *results, size_t result_count,
    size_t *written_count)
{
    if (written_count == NULL || !ma_in_range(ma, 50U, 5000U)) {
        return ESP_ERR_INVALID_ARG;
    }
    *written_count = 0;

    charger_service_write_result_t item = {0};
    esp_err_t err = charger_service_set_watchdog_disabled(&item);
    append_result(results, result_count, written_count, &item);
    if (err != ESP_OK) {
        return err;
    }

    const uint16_t raw = (uint16_t)((ma + 5U) / 10U) & 0x01FFU;
    err = charger_service_write_register(REG03_CHARGE_CURRENT_LIMIT,
                                         (uint8_t)((raw >> 8U) & 0x01U),
                                         &item);
    append_result(results, result_count, written_count, &item);
    if (err != ESP_OK) {
        return err;
    }

    err = charger_service_write_register(REG03_CHARGE_CURRENT_LIMIT + 1U,
                                         (uint8_t)(raw & 0xFFU), &item);
    append_result(results, result_count, written_count, &item);
    if (err != ESP_OK) {
        return err;
    }
    return charger_policy_write_u16(KEY_ICHG_MA, ma);
}

esp_err_t charger_service_set_input_voltage_limit_mv(
    uint16_t mv, charger_service_write_result_t *result)
{
    if (!mv_in_range(mv, 3600U, 22000U)) {
        charger_service_write_result_t local = {0};
        charger_init_write_result(&local, REG05_INPUT_VOLTAGE_LIMIT, 0);
        local.err = ESP_ERR_INVALID_ARG;
        return charger_store_result(result, &local);
    }

    charger_service_write_result_t watchdog_result = {0};
    esp_err_t err = charger_service_set_watchdog_disabled(&watchdog_result);
    if (err != ESP_OK) {
        return err;
    }

    const uint8_t raw = (uint8_t)((mv + 50U) / 100U);
    err = charger_service_write_register(REG05_INPUT_VOLTAGE_LIMIT, raw,
                                         result);
    if (err != ESP_OK) {
        return err;
    }
    return charger_policy_write_u16(KEY_VINDPM_MV, mv);
}

esp_err_t charger_service_set_input_current_limit_ma(
    uint16_t ma, charger_service_write_result_t *results, size_t result_count,
    size_t *written_count)
{
    if (written_count == NULL || !ma_in_range(ma, 100U, 3300U)) {
        return ESP_ERR_INVALID_ARG;
    }
    *written_count = 0;

    charger_service_write_result_t item = {0};
    esp_err_t err = charger_service_set_watchdog_disabled(&item);
    append_result(results, result_count, written_count, &item);
    if (err != ESP_OK) {
        return err;
    }

    const uint16_t raw = (uint16_t)((ma + 5U) / 10U) & 0x01FFU;
    err = charger_service_write_register(REG06_INPUT_CURRENT_LIMIT,
                                         (uint8_t)((raw >> 8U) & 0x01U),
                                         &item);
    append_result(results, result_count, written_count, &item);
    if (err != ESP_OK) {
        return err;
    }

    err = charger_service_write_register(REG06_INPUT_CURRENT_LIMIT + 1U,
                                         (uint8_t)(raw & 0xFFU), &item);
    append_result(results, result_count, written_count, &item);
    if (err != ESP_OK) {
        return err;
    }
    return charger_policy_write_u16(KEY_IINDPM_MA, ma);
}

esp_err_t charger_service_set_external_input_current_limit_enabled(
    bool enabled, charger_service_write_result_t *result)
{
    charger_service_write_result_t watchdog_result = {0};
    esp_err_t err = charger_service_set_watchdog_disabled(&watchdog_result);
    if (err != ESP_OK) {
        return err;
    }

    err = charger_service_update_register_bits(REG14_CHARGER_CONTROL_5, 0x02U,
                                               enabled ? 0x02U : 0x00U,
                                               result);
    if (err != ESP_OK) {
        return err;
    }
    return charger_policy_write_u8(KEY_EXT_ILIM_EN, enabled ? 1U : 0U);
}

esp_err_t charger_service_set_termination_recharge(
    bool termination_enabled, uint16_t termination_current_ma,
    uint16_t recharge_threshold_offset_mv, uint16_t recharge_deglitch_ms,
    charger_service_write_result_t *results, size_t result_count,
    size_t *written_count)
{
    if (written_count == NULL ||
        !ma_in_range(termination_current_ma, 40U, 1000U) ||
        !mv_in_range(recharge_threshold_offset_mv, 50U, 800U)) {
        return ESP_ERR_INVALID_ARG;
    }
    *written_count = 0;

    uint8_t trechg_code = 0;
    if (!bq_trechg_code_from_ms(recharge_deglitch_ms, &trechg_code)) {
        return ESP_ERR_INVALID_ARG;
    }

    charger_service_write_result_t item = {0};
    esp_err_t err = charger_service_set_watchdog_disabled(&item);
    append_result(results, result_count, written_count, &item);
    if (err != ESP_OK) {
        return err;
    }

    err = charger_service_update_register_bits(
        REG0F_CHARGER_CONTROL_0, 0x02U,
        termination_enabled ? 0x02U : 0x00U, &item);
    append_result(results, result_count, written_count, &item);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t iterm_raw = (uint8_t)((termination_current_ma + 20U) / 40U);
    if (iterm_raw == 0U) {
        iterm_raw = 1U;
    } else if (iterm_raw > 25U) {
        iterm_raw = 25U;
    }
    err = charger_service_update_register_bits(REG09_TERMINATION_CONTROL,
                                               0x1FU, iterm_raw, &item);
    append_result(results, result_count, written_count, &item);
    if (err != ESP_OK) {
        return err;
    }

    const uint8_t vrechg_raw =
        (uint8_t)(((recharge_threshold_offset_mv + 25U) / 50U) - 1U);
    const uint8_t recharge_bits =
        (uint8_t)(((trechg_code & 0x03U) << 4U) | (vrechg_raw & 0x0FU));
    err = charger_service_update_register_bits(REG0A_RECHARGE_CONTROL, 0x3FU,
                                               recharge_bits, &item);
    append_result(results, result_count, written_count, &item);
    if (err != ESP_OK) {
        return err;
    }

    return charger_policy_save_termination_recharge(
        termination_enabled, termination_current_ma,
        recharge_threshold_offset_mv, recharge_deglitch_ms);
}

esp_err_t charger_service_set_safety_timers(
    uint16_t topoff_timer_minutes, bool trickle_timer_enabled,
    bool precharge_timer_enabled, bool fast_charge_timer_enabled,
    uint8_t fast_charge_timer_hours, bool timer_2x_enabled,
    uint16_t precharge_timer_minutes, charger_service_write_result_t *results,
    size_t result_count, size_t *written_count)
{
    if (written_count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *written_count = 0;

    uint8_t topoff_code = 0;
    uint8_t fast_code = 0;
    if (!bq_topoff_timer_code_from_minutes(topoff_timer_minutes,
                                           &topoff_code) ||
        !bq_fast_charge_timer_code_from_hours(fast_charge_timer_hours,
                                              &fast_code) ||
        (precharge_timer_minutes != 30U && precharge_timer_minutes != 120U)) {
        return ESP_ERR_INVALID_ARG;
    }

    charger_service_write_result_t item = {0};
    esp_err_t err = charger_service_set_watchdog_disabled(&item);
    append_result(results, result_count, written_count, &item);
    if (err != ESP_OK) {
        return err;
    }

    const uint8_t precharge_short_bit =
        precharge_timer_minutes == 30U ? 0x80U : 0x00U;
    err = charger_service_update_register_bits(REG0D_IOTG_REGULATION, 0x80U,
                                               precharge_short_bit, &item);
    append_result(results, result_count, written_count, &item);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t timer_control = (uint8_t)((topoff_code & 0x03U) << 6U);
    if (trickle_timer_enabled) {
        timer_control |= 0x20U;
    }
    if (precharge_timer_enabled) {
        timer_control |= 0x10U;
    }
    if (fast_charge_timer_enabled) {
        timer_control |= 0x08U;
    }
    timer_control |= (uint8_t)((fast_code & 0x03U) << 1U);
    if (timer_2x_enabled) {
        timer_control |= 0x01U;
    }

    err = charger_service_write_register(REG0E_TIMER_CONTROL, timer_control,
                                         &item);
    append_result(results, result_count, written_count, &item);
    if (err != ESP_OK) {
        return err;
    }

    return charger_policy_save_safety_timers(
        topoff_timer_minutes, trickle_timer_enabled, precharge_timer_enabled,
        fast_charge_timer_enabled, fast_charge_timer_hours, timer_2x_enabled,
        precharge_timer_minutes);
}

void charger_service_request_refresh(void)
{
    charger_request_refresh_from_task_context();
}
