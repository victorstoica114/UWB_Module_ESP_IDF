#include "max77958_service.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "board_config.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "i2c_bus_service.h"
#include "nvs.h"

static const char *TAG = "max77958_service";

enum {
    MAX77958_TASK_STACK_WORDS = 4096,
    MAX77958_TASK_PRIORITY = 4,
    MAX77958_I2C_TIMEOUT_MS = APP_MAX77958_I2C_TRANSACTION_TIMEOUT_MS,
    MAX77958_I2C_PROBE_TIMEOUT_MS = APP_MAX77958_I2C_PROBE_TIMEOUT_MS,
    MAX77958_I2C_LOCK_TIMEOUT_MS = 1000,
    MAX77958_START_DELAY_MS = 2000,
    MAX77958_AP_POLL_MS = 2,
    MAX77958_LOG_INTERVAL_MS = 10000,

    REG_DEVICE_ID = 0x00,
    REG_DEVICE_REV = 0x01,
    REG_FW_REV = 0x02,
    REG_FW_SUB_VER = 0x03,
    REG_UIC_INT = 0x04,
    REG_CC_INT = 0x05,
    REG_PD_INT = 0x06,
    REG_ACTION_INT = 0x07,
    REG_USBC_STATUS1 = 0x08,
    REG_USBC_STATUS2 = 0x09,
    REG_BC_STATUS = 0x0A,
    REG_DP_STATUS = 0x0B,
    REG_CC_STATUS0 = 0x0C,
    REG_CC_STATUS1 = 0x0D,
    REG_PD_STATUS0 = 0x0E,
    REG_PD_STATUS1 = 0x0F,
    REG_UIC_INT_M = 0x10,
    REG_CC_INT_M = 0x11,
    REG_PD_INT_M = 0x12,
    REG_ACTION_INT_M = 0x13,
    REG_AP_DATAOUT0 = 0x21,
    REG_AP_DATAIN0 = 0x51,
    REG_SW_RESET = 0x80,
    REG_I2C_CNFG = 0xE0,
    REG_I2C_CNFG_HS_EXT_EN = 0x01,

    OP_BC_CTRL1_WRITE = 0x02,
    OP_CTRL1_READ = 0x05,
    OP_CTRL1_WRITE = 0x06,
    OP_CURRENT_SRC_CAP = 0x30,
    OP_SRC_CAP_REQUEST = 0x32,
    OP_APDO_SRC_CAP_REQUEST = 0x3A,
    OP_SET_PPS = 0x3C,
    OP_SNK_PDO_REQUEST = 0x3E,
    OP_SNK_PDO_SET = 0x3F,

    MAX77958_DEVICE_ID_EXPECTED = 0x58,
    MAX77958_DIRECT_READ_MAX_BYTES = 32,
    MAX77958_DIRECT_WRITE_MAX_BYTES = 30,
};

#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
#define MAX77958_TASK_CORE 0
#else
#define MAX77958_TASK_CORE 0
#endif

#define MAX77958_NVS_NAMESPACE "max77958"
#define KEY_USB2_CLOSED "usb2"
#define KEY_SOURCE_POS "src_pos"
#define KEY_SINK_COUNT "snk_cnt"
#define KEY_SINK_PDO_PREFIX "snk"
#define KEY_PPS_ENABLED "pps_en"
#define KEY_PPS_MV "pps_mv"
#define KEY_PPS_MA "pps_ma"

typedef struct {
    bool has_usb2_switch_closed;
    bool usb2_switch_closed;
    bool has_source_position;
    uint8_t source_position;
    bool has_sink_pdos;
    uint8_t sink_pdo_count;
    uint32_t sink_pdos[MAX77958_SERVICE_MAX_SINK_PDOS];
    bool has_pps_default;
    bool pps_enabled;
    uint16_t pps_mv;
    uint16_t pps_ma;
    uint32_t field_count;
} max77958_policy_t;

typedef struct {
    uint8_t start_reg;
    uint8_t length;
} register_range_t;

static bool s_started;
static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_i2c_dev;
static SemaphoreHandle_t s_state_mutex;
static TaskHandle_t s_task_handle;
static volatile bool s_refresh_requested;
static max77958_service_snapshot_t s_snapshot;
static uint32_t s_last_update_ms;
static uint32_t s_last_operation_ms;
static bool s_hs_ext_active;
static uint32_t s_hs_direct_read_count;
static uint32_t s_hs_direct_write_count;
static uint32_t s_hs_direct_error_count;
static uint32_t s_hs_direct_write_verify_count;
static uint32_t s_hs_direct_write_verify_error_count;
static uint32_t s_hs_direct_last_elapsed_us;
static int s_hs_direct_scl_measure_error = ESP_ERR_NOT_SUPPORTED;
static uint32_t s_hs_direct_scl_edges;
static uint32_t s_hs_direct_scl_elapsed_us;
static uint32_t s_hs_direct_scl_measured_hz;

static uint32_t ticks_to_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
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

const char *max77958_service_sys_msg_to_string(uint8_t value)
{
    switch (value) {
    case 0x00:
        return "none";
    case 0x03:
        return "boot_wdt";
    case 0x04:
        return "boot_sw_reset_req";
    case 0x05:
        return "boot_por";
    case 0x31:
        return "apcmd_unknown";
    case 0x32:
        return "apcmd_in_progress";
    case 0x33:
        return "apcmd_fail";
    default:
        return "other";
    }
}

const char *max77958_service_chg_typ_to_string(uint8_t value)
{
    switch (value & 0x03U) {
    case 0:
        return "none";
    case 1:
        return "SDP";
    case 2:
        return "CDP";
    default:
        return "DCP";
    }
}

const char *max77958_service_cc_pin_to_string(uint8_t value)
{
    switch (value & 0x03U) {
    case 0:
        return "none";
    case 1:
        return "CC1";
    case 2:
        return "CC2";
    default:
        return "RFU";
    }
}

const char *max77958_service_cci_to_string(uint8_t value)
{
    switch (value & 0x03U) {
    case 0:
        return "not_ufp";
    case 1:
        return "500mA";
    case 2:
        return "1.5A";
    case 3:
        return "3.0A";
    default:
        return "?";
    }
}

const char *max77958_service_cc_stat_to_string(uint8_t value)
{
    switch (value & 0x07U) {
    case 0:
        return "no_connection";
    case 1:
        return "sink";
    case 2:
        return "source";
    case 3:
        return "audio_accessory";
    case 4:
        return "debug_source";
    case 5:
        return "error";
    case 6:
        return "disabled";
    case 7:
        return "debug_sink";
    default:
        return "?";
    }
}

const char *max77958_service_apdo_result_to_string(uint8_t value)
{
    switch (value) {
    case 0x00:
        return "sent";
    case 0x01:
        return "invalid_apdo_position";
    case 0x02:
        return "invalid_voltage";
    case 0x03:
        return "invalid_current";
    case 0x04:
        return "pps_off";
    case 0x05:
        return "not_snk_ready";
    case 0x06:
        return "pd2_contract";
    case 0x07:
        return "sink_tx_ng";
    default:
        return "unknown";
    }
}

static uint8_t decode_vbadc(uint8_t usbc_status1)
{
    return (uint8_t)((usbc_status1 >> 3U) & 0x1FU);
}

static void decode_vbadc_range(uint8_t code, uint16_t *min_mv,
                               uint16_t *max_mv, uint16_t *mid_mv,
                               bool *above_range)
{
    if (min_mv != NULL) {
        *min_mv = 0;
    }
    if (max_mv != NULL) {
        *max_mv = 0;
    }
    if (mid_mv != NULL) {
        *mid_mv = 0;
    }
    if (above_range != NULL) {
        *above_range = false;
    }

    if (code == 0U) {
        if (max_mv != NULL) {
            *max_mv = 3500;
        }
        if (mid_mv != NULL) {
            *mid_mv = 1750;
        }
        return;
    }
    if (code >= 0x19U) {
        if (min_mv != NULL) {
            *min_mv = 27500;
        }
        if (mid_mv != NULL) {
            *mid_mv = 27500;
        }
        if (above_range != NULL) {
            *above_range = true;
        }
        return;
    }

    const uint16_t low = (uint16_t)(2500U + ((uint16_t)code * 1000U));
    if (min_mv != NULL) {
        *min_mv = low;
    }
    if (max_mv != NULL) {
        *max_mv = (uint16_t)(low + 1000U);
    }
    if (mid_mv != NULL) {
        *mid_mv = (uint16_t)(low + 500U);
    }
}

static uint32_t read_le_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static void write_le_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static bool fixed_pdo_from_mv_ma(uint16_t mv, uint16_t ma, uint32_t *pdo)
{
    if (pdo == NULL || mv == 0U || ma == 0U || (mv % 50U) != 0U ||
        (ma % 10U) != 0U) {
        return false;
    }

    const uint32_t voltage_units = (uint32_t)mv / 50U;
    const uint32_t current_units = (uint32_t)ma / 10U;
    if (voltage_units > 0x3FFU || current_units > 0x3FFU) {
        return false;
    }

    *pdo = (voltage_units << 10U) | current_units;
    return true;
}

static esp_err_t policy_write_u8(const char *key, uint8_t value)
{
    nvs_handle_t handle = 0;
    ESP_RETURN_ON_ERROR(nvs_open(MAX77958_NVS_NAMESPACE, NVS_READWRITE,
                                 &handle),
                        TAG, "open MAX77958 NVS failed");
    const esp_err_t err = nvs_set_u8(handle, key, value);
    if (err == ESP_OK) {
        (void)nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

static esp_err_t policy_write_u16(const char *key, uint16_t value)
{
    nvs_handle_t handle = 0;
    ESP_RETURN_ON_ERROR(nvs_open(MAX77958_NVS_NAMESPACE, NVS_READWRITE,
                                 &handle),
                        TAG, "open MAX77958 NVS failed");
    const esp_err_t err = nvs_set_u16(handle, key, value);
    if (err == ESP_OK) {
        (void)nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

static bool policy_read_u8(nvs_handle_t handle, const char *key, uint8_t *value)
{
    return value != NULL && nvs_get_u8(handle, key, value) == ESP_OK;
}

static bool policy_read_u16(nvs_handle_t handle, const char *key,
                            uint16_t *value)
{
    return value != NULL && nvs_get_u16(handle, key, value) == ESP_OK;
}

static bool policy_read_u32(nvs_handle_t handle, const char *key,
                            uint32_t *value)
{
    return value != NULL && nvs_get_u32(handle, key, value) == ESP_OK;
}

static void sink_pdo_key(size_t index, char *buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0) {
        return;
    }
    (void)snprintf(buffer, buffer_size, "%s%u", KEY_SINK_PDO_PREFIX,
                   (unsigned)index);
}

static esp_err_t policy_load(max77958_policy_t *policy)
{
    if (policy == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(policy, 0, sizeof(*policy));

    nvs_handle_t handle = 0;
    const esp_err_t open_err =
        nvs_open(MAX77958_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (open_err != ESP_OK) {
        return open_err;
    }

    uint8_t u8 = 0;
    uint16_t u16 = 0;
    uint32_t u32 = 0;

    if (policy_read_u8(handle, KEY_USB2_CLOSED, &u8)) {
        policy->has_usb2_switch_closed = true;
        policy->usb2_switch_closed = u8 != 0U;
        policy->field_count++;
    }
    if (policy_read_u8(handle, KEY_SOURCE_POS, &u8)) {
        policy->has_source_position = true;
        policy->source_position = u8;
        policy->field_count++;
    }
    if (policy_read_u8(handle, KEY_SINK_COUNT, &u8) && u8 > 0U &&
        u8 <= MAX77958_SERVICE_MAX_SINK_PDOS) {
        policy->has_sink_pdos = true;
        policy->sink_pdo_count = u8;
        policy->field_count++;
        for (size_t i = 0; i < policy->sink_pdo_count; ++i) {
            char key[8] = {0};
            sink_pdo_key(i, key, sizeof(key));
            if (policy_read_u32(handle, key, &u32)) {
                policy->sink_pdos[i] = u32;
            }
        }
    }
    if (policy_read_u8(handle, KEY_PPS_ENABLED, &u8)) {
        policy->has_pps_default = true;
        policy->pps_enabled = u8 != 0U;
        policy->field_count++;
    }
    if (policy_read_u16(handle, KEY_PPS_MV, &u16)) {
        policy->has_pps_default = true;
        policy->pps_mv = u16;
    }
    if (policy_read_u16(handle, KEY_PPS_MA, &u16)) {
        policy->has_pps_default = true;
        policy->pps_ma = u16;
    }

    nvs_close(handle);
    return policy->field_count > 0U || policy->has_pps_default
               ? ESP_OK
               : ESP_ERR_NOT_FOUND;
}

static esp_err_t policy_save_sink_pdos(const uint32_t *pdos, size_t count)
{
    if (pdos == NULL || count == 0 ||
        count > MAX77958_SERVICE_MAX_SINK_PDOS) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle = 0;
    ESP_RETURN_ON_ERROR(nvs_open(MAX77958_NVS_NAMESPACE, NVS_READWRITE,
                                 &handle),
                        TAG, "open MAX77958 NVS failed");
    esp_err_t err = nvs_set_u8(handle, KEY_SINK_COUNT, (uint8_t)count);
    for (size_t i = 0; err == ESP_OK && i < count; ++i) {
        char key[8] = {0};
        sink_pdo_key(i, key, sizeof(key));
        err = nvs_set_u32(handle, key, pdos[i]);
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

static esp_err_t policy_save_pps(bool enabled, uint16_t mv, uint16_t ma)
{
    ESP_RETURN_ON_ERROR(policy_write_u8(KEY_PPS_ENABLED, enabled ? 1U : 0U),
                        TAG, "persist PPS enable failed");
    ESP_RETURN_ON_ERROR(policy_write_u16(KEY_PPS_MV, mv), TAG,
                        "persist PPS voltage failed");
    return policy_write_u16(KEY_PPS_MA, ma);
}

static esp_err_t max77958_i2c_init(void)
{
    esp_err_t err = i2c_bus_service_get(&s_i2c_bus);
    if (err != ESP_OK) {
        return err;
    }

    if (!i2c_bus_service_lock_background(pdMS_TO_TICKS(MAX77958_I2C_LOCK_TIMEOUT_MS))) {
        return ESP_ERR_TIMEOUT;
    }

    const i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = APP_MAX77958_I2C_ADDRESS,
        .scl_speed_hz = APP_MAX77958_I2C_CLOCK_HZ,
        .scl_wait_us = 20000,
    };
    err = i2c_master_bus_add_device(s_i2c_bus, &dev_config, &s_i2c_dev);
    if (err != ESP_OK) {
        i2c_bus_service_unlock();
        return err;
    }

    err = i2c_master_probe(s_i2c_bus, APP_MAX77958_I2C_ADDRESS,
                           MAX77958_I2C_PROBE_TIMEOUT_MS);
    if (err != ESP_OK) {
        (void)i2c_master_bus_rm_device(s_i2c_dev);
        s_i2c_dev = NULL;
        s_hs_ext_active = false;
    }
    i2c_bus_service_unlock();
    return err;
}

static bool max77958_hs_direct_reads_active(void)
{
    return APP_MAX77958_I2C_HS_DIRECT_READS_ENABLED != 0 && s_hs_ext_active;
}

static bool max77958_hs_direct_writes_active(void)
{
    return APP_MAX77958_I2C_HS_DIRECT_WRITES_ENABLED != 0 && s_hs_ext_active;
}

static uint32_t max77958_i2c_estimate_us_for(uint64_t bits, uint32_t clock_hz)
{
    if (clock_hz == 0) {
        return UINT32_MAX;
    }
    const uint64_t us =
        (bits * 1000000ULL + clock_hz - 1ULL) / clock_hz +
        APP_I2C_BACKGROUND_TRANSFER_MARGIN_US;
    return us > UINT32_MAX ? UINT32_MAX : (uint32_t)us;
}

static uint32_t max77958_i2c_standard_read_estimate_us(size_t data_len)
{
    return max77958_i2c_estimate_us_for(
        27ULL + (uint64_t)data_len * 9ULL, APP_MAX77958_I2C_CLOCK_HZ);
}

static uint32_t max77958_i2c_read_estimate_us(size_t data_len)
{
    if (!max77958_hs_direct_reads_active()) {
        return max77958_i2c_standard_read_estimate_us(data_len);
    }

    const uint32_t entry_us = max77958_i2c_estimate_us_for(
        18ULL, APP_MAX77958_I2C_HS_ENTRY_CLOCK_HZ);
    const uint32_t read_us = max77958_i2c_estimate_us_for(
        27ULL + (uint64_t)data_len * 9ULL,
        APP_MAX77958_I2C_HS_DIRECT_CLOCK_HZ);
    if (entry_us == UINT32_MAX || read_us == UINT32_MAX) {
        return UINT32_MAX;
    }
    return entry_us > UINT32_MAX - read_us ? UINT32_MAX : entry_us + read_us;
}

static uint32_t max77958_i2c_write_estimate_us(size_t write_len)
{
    return max77958_i2c_estimate_us_for(
        9ULL + (uint64_t)write_len * 9ULL, APP_MAX77958_I2C_CLOCK_HZ);
}

static uint32_t max77958_i2c_direct_write_estimate_us(size_t data_len)
{
    const uint32_t entry_us = max77958_i2c_estimate_us_for(
        18ULL, APP_MAX77958_I2C_HS_ENTRY_CLOCK_HZ);
    const uint32_t write_us = max77958_i2c_estimate_us_for(
        9ULL + ((uint64_t)data_len + 1ULL) * 9ULL,
        APP_MAX77958_I2C_HS_DIRECT_CLOCK_HZ);
    if (entry_us == UINT32_MAX || write_us == UINT32_MAX) {
        return UINT32_MAX;
    }
    return entry_us > UINT32_MAX - write_us ? UINT32_MAX : entry_us + write_us;
}

static size_t max77958_configured_read_chunk_len(void)
{
    size_t chunk_len = APP_MAX77958_REGISTER_READ_CHUNK_BYTES;
    if (chunk_len == 0 || chunk_len > MAX77958_SERVICE_AP_DATA_BYTES) {
        chunk_len = MAX77958_SERVICE_AP_DATA_BYTES;
    }
    if (max77958_hs_direct_reads_active() &&
        chunk_len > APP_MAX77958_I2C_HS_DIRECT_READ_CHUNK_BYTES) {
        chunk_len = APP_MAX77958_I2C_HS_DIRECT_READ_CHUNK_BYTES;
    }
    if (chunk_len > MAX77958_DIRECT_READ_MAX_BYTES) {
        chunk_len = MAX77958_DIRECT_READ_MAX_BYTES;
    }
    if (chunk_len == 0) {
        chunk_len = 1U;
    }
    return chunk_len;
}

static size_t max77958_configured_write_chunk_len(void)
{
    size_t chunk_len = APP_MAX77958_I2C_HS_DIRECT_WRITE_CHUNK_BYTES;
    if (chunk_len == 0 || chunk_len > MAX77958_DIRECT_WRITE_MAX_BYTES) {
        chunk_len = MAX77958_DIRECT_WRITE_MAX_BYTES;
    }
    return chunk_len == 0 ? 1U : chunk_len;
}

static esp_err_t read_bytes_direct_hs(uint8_t start_reg, uint8_t *data,
                                      size_t data_len)
{
    const i2c_bus_service_direct_config_t direct_config = {
        .clock_hz = APP_MAX77958_I2C_HS_DIRECT_CLOCK_HZ,
        .timeout_us = (uint32_t)MAX77958_I2C_TIMEOUT_MS * 1000U,
        .estimated_transfer_us = max77958_i2c_read_estimate_us(data_len),
        .manual_timing = true,
        .scl_low_period = APP_MAX77958_I2C_HS_DIRECT_LOW_PERIOD,
        .scl_high_period = APP_MAX77958_I2C_HS_DIRECT_HIGH_PERIOD,
        .scl_wait_high_period = APP_MAX77958_I2C_HS_DIRECT_WAIT_HIGH_PERIOD,
        .hs_master_code = true,
        .hs_master_stop = true,
        .hs_entry_clock_hz = APP_MAX77958_I2C_HS_ENTRY_CLOCK_HZ,
        .measure_scl_gpio =
            APP_MAX77958_I2C_HS_DIRECT_MEASURE_SCL_ENABLED
                ? BOARD_CONFIG_MAX77958_SCL_GPIO
                : -1,
    };
    i2c_bus_service_direct_result_t result = {0};
    const esp_err_t err = i2c_bus_service_direct_read_reg(
        APP_MAX77958_I2C_ADDRESS, start_reg, data, data_len, &direct_config,
        &result);
    s_hs_direct_last_elapsed_us =
        result.hs_master_elapsed_us + result.elapsed_us;
    s_hs_direct_scl_measure_error = result.scl_measure_error;
    s_hs_direct_scl_edges = result.scl_measure_edges;
    s_hs_direct_scl_elapsed_us = result.scl_measure_elapsed_us;
    s_hs_direct_scl_measured_hz = result.scl_measure_hz;
    if (err == ESP_OK) {
        s_hs_direct_read_count++;
    } else {
        s_hs_direct_error_count++;
    }
    return err;
}

static esp_err_t write_bytes_direct_hs(uint8_t start_reg, const uint8_t *data,
                                       size_t data_len)
{
    const i2c_bus_service_direct_config_t direct_config = {
        .clock_hz = APP_MAX77958_I2C_HS_DIRECT_CLOCK_HZ,
        .timeout_us = (uint32_t)MAX77958_I2C_TIMEOUT_MS * 1000U,
        .estimated_transfer_us = max77958_i2c_direct_write_estimate_us(data_len),
        .manual_timing = true,
        .scl_low_period = APP_MAX77958_I2C_HS_DIRECT_LOW_PERIOD,
        .scl_high_period = APP_MAX77958_I2C_HS_DIRECT_HIGH_PERIOD,
        .scl_wait_high_period = APP_MAX77958_I2C_HS_DIRECT_WAIT_HIGH_PERIOD,
        .hs_master_code = true,
        .hs_master_stop = true,
        .hs_entry_clock_hz = APP_MAX77958_I2C_HS_ENTRY_CLOCK_HZ,
        .measure_scl_gpio =
            APP_MAX77958_I2C_HS_DIRECT_MEASURE_SCL_ENABLED
                ? BOARD_CONFIG_MAX77958_SCL_GPIO
                : -1,
    };
    i2c_bus_service_direct_result_t result = {0};
    const esp_err_t err = i2c_bus_service_direct_write_reg(
        APP_MAX77958_I2C_ADDRESS, start_reg, data, data_len, &direct_config,
        &result);
    s_hs_direct_last_elapsed_us =
        result.hs_master_elapsed_us + result.elapsed_us;
    s_hs_direct_scl_measure_error = result.scl_measure_error;
    s_hs_direct_scl_edges = result.scl_measure_edges;
    s_hs_direct_scl_elapsed_us = result.scl_measure_elapsed_us;
    s_hs_direct_scl_measured_hz = result.scl_measure_hz;
    if (err == ESP_OK) {
        s_hs_direct_write_count++;
    } else {
        s_hs_direct_error_count++;
    }
    return err;
}

static size_t max77958_adaptive_read_chunk_len(size_t remaining)
{
    size_t chunk_len = max77958_configured_read_chunk_len();
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
           max77958_i2c_read_estimate_us(chunk_len) > (uint32_t)window_us) {
        chunk_len--;
    }
    return chunk_len;
}

static esp_err_t read_bytes(uint8_t start_reg, uint8_t *data, size_t data_len)
{
    if (data == NULL || data_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_i2c_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ESP_ERR_TIMEOUT;
    for (uint32_t attempt = 0; attempt <= APP_MAX77958_I2C_READ_RETRIES;
         ++attempt) {
        if (max77958_hs_direct_reads_active() &&
            data_len <= MAX77958_DIRECT_READ_MAX_BYTES) {
            err = read_bytes_direct_hs(start_reg, data, data_len);
            if (err == ESP_OK) {
                return ESP_OK;
            }
        }

        {
            if (!i2c_bus_service_lock_background_for(
                    pdMS_TO_TICKS(MAX77958_I2C_LOCK_TIMEOUT_MS),
                    max77958_i2c_standard_read_estimate_us(data_len))) {
                err = ESP_ERR_TIMEOUT;
            } else {
                err = i2c_master_transmit_receive(
                    s_i2c_dev, &start_reg, sizeof(start_reg), data, data_len,
                    MAX77958_I2C_TIMEOUT_MS);
                i2c_bus_service_unlock();
            }
        }
        if (err == ESP_OK || attempt == APP_MAX77958_I2C_READ_RETRIES) {
            return err;
        }
        taskYIELD();
    }
    return err;
}

static esp_err_t write_bytes(uint8_t start_reg, const uint8_t *data,
                             size_t data_len)
{
    if (data == NULL || data_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_i2c_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buffer[MAX77958_SERVICE_AP_DATA_BYTES + 1U] = {0};
    if (data_len > MAX77958_SERVICE_AP_DATA_BYTES) {
        return ESP_ERR_INVALID_SIZE;
    }
    buffer[0] = start_reg;
    memcpy(&buffer[1], data, data_len);

    const uint32_t retries =
        data_len == 1U ? APP_MAX77958_I2C_WRITE_RETRIES : 0U;
    esp_err_t err = ESP_ERR_TIMEOUT;
    for (uint32_t attempt = 0; attempt <= retries; ++attempt) {
        if (max77958_hs_direct_writes_active() &&
            data_len <= max77958_configured_write_chunk_len()) {
            err = write_bytes_direct_hs(start_reg, data, data_len);
            if (err == ESP_OK) {
                return ESP_OK;
            }
        }

        if (!i2c_bus_service_lock_background_for(
                pdMS_TO_TICKS(MAX77958_I2C_LOCK_TIMEOUT_MS),
                max77958_i2c_write_estimate_us(data_len + 1U))) {
            err = ESP_ERR_TIMEOUT;
        } else {
            err = i2c_master_transmit(s_i2c_dev, buffer, data_len + 1U,
                                      MAX77958_I2C_TIMEOUT_MS);
            i2c_bus_service_unlock();
        }
        if (err == ESP_OK || attempt == retries) {
            return err;
        }
        taskYIELD();
    }
    return err;
}

static esp_err_t write_bytes_direct_hs_chunked(uint8_t start_reg,
                                               const uint8_t *data,
                                               size_t data_len)
{
    if (!max77958_hs_direct_writes_active()) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (data == NULL || data_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const size_t max_chunk = max77958_configured_write_chunk_len();
    size_t offset = 0;
    while (offset < data_len) {
        const size_t remaining = data_len - offset;
        const size_t chunk_len = remaining > max_chunk ? max_chunk : remaining;
        const esp_err_t err =
            write_bytes_direct_hs((uint8_t)(start_reg + offset),
                                  &data[offset], chunk_len);
        if (err != ESP_OK) {
            return err;
        }
        offset += chunk_len;
        taskYIELD();
    }
    return ESP_OK;
}

static esp_err_t verify_bytes_single_reads(uint8_t start_reg,
                                           const uint8_t *expected,
                                           size_t data_len)
{
    if (expected == NULL || data_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t offset = 0; offset < data_len; ++offset) {
        uint8_t actual = 0;
        const uint8_t reg = (uint8_t)(start_reg + offset);
        const esp_err_t err = read_bytes(reg, &actual, sizeof(actual));
        if (err != ESP_OK) {
            s_hs_direct_write_verify_error_count++;
            ESP_LOGW(TAG, "MAX77958 read-back verify failed reg=0x%02X: %s",
                     reg, esp_err_to_name(err));
            return err;
        }
        if (actual != expected[offset]) {
            s_hs_direct_write_verify_error_count++;
            ESP_LOGW(TAG,
                     "MAX77958 read-back mismatch reg=0x%02X expected=0x%02X actual=0x%02X",
                     reg, expected[offset], actual);
            return ESP_ERR_INVALID_RESPONSE;
        }
    }

    s_hs_direct_write_verify_count++;
    return ESP_OK;
}

static esp_err_t ap_write_command_direct_hs_verified(
    const uint8_t command[MAX77958_SERVICE_AP_DATA_BYTES], bool *latched)
{
    if (latched != NULL) {
        *latched = false;
    }
    if (!max77958_hs_direct_writes_active()) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    /*
     * AP_DATAOUT32 (0x41) latches the whole AP command. Verify everything up
     * to 0x40 first, then write the latch byte separately and verify it after
     * the command has been handed to the USBC block.
     */
    esp_err_t err =
        write_bytes_direct_hs_chunked(REG_AP_DATAOUT0, command, 30U);
    if (err != ESP_OK) {
        return err;
    }
    err = verify_bytes_single_reads(REG_AP_DATAOUT0, command, 30U);
    if (err != ESP_OK) {
        return err;
    }

    err = write_bytes_direct_hs_chunked((uint8_t)(REG_AP_DATAOUT0 + 30U),
                                        &command[30], 2U);
    if (err != ESP_OK) {
        return err;
    }
    err = verify_bytes_single_reads((uint8_t)(REG_AP_DATAOUT0 + 30U),
                                    &command[30], 2U);
    if (err != ESP_OK) {
        return err;
    }

    err = write_bytes_direct_hs((uint8_t)(REG_AP_DATAOUT0 + 32U),
                                &command[32], 1U);
    if (err != ESP_OK) {
        return err;
    }
    if (latched != NULL) {
        *latched = true;
    }
    return verify_bytes_single_reads((uint8_t)(REG_AP_DATAOUT0 + 32U),
                                     &command[32], 1U);
}

static esp_err_t max77958_read_device_id(uint8_t *device_id)
{
    if (device_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return read_bytes(REG_DEVICE_ID, device_id, sizeof(*device_id));
}

static bool max77958_validate_device_id(const char *stage)
{
    uint8_t device_id = 0;
    const esp_err_t err = max77958_read_device_id(&device_id);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "MAX77958 DEVICE_ID read failed during %s: %s",
                 stage != NULL ? stage : "validation", esp_err_to_name(err));
        return false;
    }
    if (device_id != MAX77958_DEVICE_ID_EXPECTED) {
        ESP_LOGW(TAG, "MAX77958 DEVICE_ID mismatch during %s: 0x%02X",
                 stage != NULL ? stage : "validation", device_id);
        return false;
    }
    return true;
}

static esp_err_t max77958_set_hs_ext(bool enabled)
{
    uint8_t cnfg = 0;
    esp_err_t err = read_bytes(REG_I2C_CNFG, &cnfg, sizeof(cnfg));
    if (err != ESP_OK) {
        return err;
    }

    const uint8_t next =
        enabled ? (uint8_t)(cnfg | REG_I2C_CNFG_HS_EXT_EN)
                : (uint8_t)(cnfg & (uint8_t)~REG_I2C_CNFG_HS_EXT_EN);
    if (next != cnfg) {
        err = write_bytes(REG_I2C_CNFG, &next, sizeof(next));
        if (err != ESP_OK) {
            return err;
        }
    }

    uint8_t verify = 0;
    err = read_bytes(REG_I2C_CNFG, &verify, sizeof(verify));
    if (err != ESP_OK) {
        return err;
    }
    if (((verify & REG_I2C_CNFG_HS_EXT_EN) != 0U) != enabled) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (!enabled) {
        s_hs_ext_active = false;
    }
    ESP_LOGI(TAG, "MAX77958 HS_EXT bit %s (I2C_CNFG=0x%02X)",
             enabled ? "set" : "cleared", verify);
    return ESP_OK;
}

static esp_err_t max77958_enter_hs_ext_if_enabled(void)
{
    s_hs_ext_active = false;
    if (!APP_MAX77958_I2C_HS_EXT_ENABLED ||
        (!APP_MAX77958_I2C_HS_DIRECT_READS_ENABLED &&
         !APP_MAX77958_I2C_HS_DIRECT_WRITES_ENABLED)) {
        return ESP_OK;
    }

    if (!max77958_validate_device_id("HS entry")) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    esp_err_t err = max77958_set_hs_ext(true);
    if (err != ESP_OK) {
        return err;
    }

    s_hs_ext_active = true;
    if (!max77958_validate_device_id("direct HS")) {
        s_hs_ext_active = false;
        (void)max77958_set_hs_ext(false);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGI(TAG,
             "MAX77958 direct HS active: clock=%uHz timing=%u/%u/%u entry=%uHz reads=%s writes=%s",
             (unsigned)APP_MAX77958_I2C_HS_DIRECT_CLOCK_HZ,
             (unsigned)APP_MAX77958_I2C_HS_DIRECT_LOW_PERIOD,
             (unsigned)APP_MAX77958_I2C_HS_DIRECT_HIGH_PERIOD,
             (unsigned)APP_MAX77958_I2C_HS_DIRECT_WAIT_HIGH_PERIOD,
             (unsigned)APP_MAX77958_I2C_HS_ENTRY_CLOCK_HZ,
             APP_MAX77958_I2C_HS_DIRECT_READS_ENABLED ? "on" : "off",
             APP_MAX77958_I2C_HS_DIRECT_WRITES_ENABLED ? "on" : "off");
    return ESP_OK;
}

static esp_err_t read_register_range_chunked(uint8_t start_reg, uint8_t *data,
                                             size_t data_len)
{
    size_t offset = 0;
    while (offset < data_len) {
        const size_t remaining = data_len - offset;
        const size_t chunk_len = max77958_adaptive_read_chunk_len(remaining);

        const esp_err_t err =
            read_bytes((uint8_t)(start_reg + offset), &data[offset], chunk_len);
        if (err != ESP_OK) {
            return err;
        }

        offset += chunk_len;
        taskYIELD();
    }
    return ESP_OK;
}

static esp_err_t read_register_map(
    uint8_t raw[MAX77958_SERVICE_REGISTER_MAP_SIZE])
{
    static const register_range_t ranges[] = {
        {0x00, 0x14},
        {REG_AP_DATAOUT0, MAX77958_SERVICE_AP_DATA_BYTES},
        {REG_AP_DATAIN0, MAX77958_SERVICE_AP_DATA_BYTES},
        {REG_I2C_CNFG, 1},
    };

    memset(raw, 0, MAX77958_SERVICE_REGISTER_MAP_SIZE);
    for (size_t i = 0; i < sizeof(ranges) / sizeof(ranges[0]); ++i) {
        const esp_err_t err =
            read_register_range_chunked(ranges[i].start_reg,
                                        &raw[ranges[i].start_reg],
                                        ranges[i].length);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

static esp_err_t wait_uic_mask(uint8_t mask, uint32_t timeout_ms,
                               uint8_t *last_uic)
{
    const uint32_t start_ms = ticks_to_ms();
    uint8_t uic = 0;
    while ((uint32_t)(ticks_to_ms() - start_ms) < timeout_ms) {
        const esp_err_t err = read_bytes(REG_UIC_INT, &uic, 1);
        if (err != ESP_OK) {
            return err;
        }
        if (last_uic != NULL) {
            *last_uic = uic;
        }
        if ((uic & mask) != 0U) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(MAX77958_AP_POLL_MS));
    }
    return ESP_ERR_TIMEOUT;
}

static void decode_source_caps_from_response(
    max77958_service_snapshot_t *snapshot, const uint8_t response[33])
{
    if (snapshot == NULL || response == NULL || response[0] != OP_CURRENT_SRC_CAP) {
        return;
    }

    const uint8_t count = response[1] & 0x07U;
    snapshot->source_pdo_count =
        count > MAX77958_SERVICE_MAX_SOURCE_PDOS
            ? MAX77958_SERVICE_MAX_SOURCE_PDOS
            : count;
    snapshot->selected_source_pdo_pos = (uint8_t)((response[1] >> 3U) & 0x07U);
    memset(snapshot->source_pdos, 0, sizeof(snapshot->source_pdos));
    for (size_t i = 0; i < snapshot->source_pdo_count; ++i) {
        snapshot->source_pdos[i] = read_le_u32(&response[2U + (i * 4U)]);
    }
    snapshot->source_caps_valid = true;
}

static void decode_sink_pdos_from_response(
    max77958_service_snapshot_t *snapshot, const uint8_t response[33],
    bool mtp)
{
    if (snapshot == NULL || response == NULL || response[0] != OP_SNK_PDO_REQUEST) {
        return;
    }

    const uint8_t count = response[1] & 0x07U;
    snapshot->sink_pdo_count =
        count > MAX77958_SERVICE_MAX_SINK_PDOS
            ? MAX77958_SERVICE_MAX_SINK_PDOS
            : count;
    memset(snapshot->sink_pdos, 0, sizeof(snapshot->sink_pdos));
    for (size_t i = 0; i < snapshot->sink_pdo_count; ++i) {
        snapshot->sink_pdos[i] = read_le_u32(&response[2U + (i * 4U)]);
    }
    snapshot->sink_pdos_valid = true;
    snapshot->sink_pdos_from_mtp = mtp;
}

static void decode_ctrl1_from_response(max77958_service_snapshot_t *snapshot,
                                       const uint8_t response[33])
{
    if (snapshot == NULL || response == NULL || response[0] != OP_CTRL1_READ) {
        return;
    }

    snapshot->ctrl1_raw = response[1];
    snapshot->ctrl1_comp2_sw = (uint8_t)((response[1] >> 3U) & 0x07U);
    snapshot->ctrl1_comn1_sw = (uint8_t)(response[1] & 0x07U);
    snapshot->usb2_switch_closed =
        snapshot->ctrl1_comp2_sw == 1U && snapshot->ctrl1_comn1_sw == 1U;
    snapshot->ctrl1_valid = true;
}

static void record_ap_result(const max77958_service_ap_result_t *result,
                             bool source_caps, bool sink_pdos, bool sink_mtp,
                             bool ctrl1)
{
    if (result == NULL) {
        return;
    }

    const uint32_t now_ms = ticks_to_ms();
    if (state_lock(pdMS_TO_TICKS(50))) {
        s_snapshot.config_write_supported = true;
        s_snapshot.config_writes_enabled = true;
        if (result->err == ESP_OK && result->ok) {
            s_snapshot.operation_count++;
            s_last_operation_ms = now_ms;
        } else {
            s_snapshot.operation_error_count++;
        }
        s_snapshot.last_opcode = result->opcode;
        s_snapshot.last_response_opcode = result->response_opcode;
        s_snapshot.last_result_code = result->result_code;
        s_snapshot.last_operation_error = result->err;
        memcpy(s_snapshot.last_response, result->response,
               sizeof(s_snapshot.last_response));
        s_snapshot.last_operation_age_ms =
            elapsed_since(now_ms, s_last_operation_ms);
        if (source_caps) {
            decode_source_caps_from_response(&s_snapshot, result->response);
        }
        if (sink_pdos) {
            decode_sink_pdos_from_response(&s_snapshot, result->response,
                                           sink_mtp);
        }
        if (ctrl1) {
            decode_ctrl1_from_response(&s_snapshot, result->response);
        }
        state_unlock();
    }
}

static esp_err_t ap_send33(uint8_t opcode, const uint8_t payload32[32],
                           max77958_service_ap_result_t *result)
{
    max77958_service_ap_result_t local = {0};
    local.opcode = opcode;
    local.err = ESP_OK;

    uint8_t command[MAX77958_SERVICE_AP_DATA_BYTES] = {0};
    command[0] = opcode;
    if (payload32 != NULL) {
        memcpy(&command[1], payload32, 32);
    }

    bool command_latched = false;
    esp_err_t err = ap_write_command_direct_hs_verified(command,
                                                        &command_latched);
    if (err != ESP_OK && !command_latched) {
        if (err != ESP_ERR_NOT_SUPPORTED) {
            ESP_LOGW(TAG,
                     "MAX77958 direct-HS AP write/verify failed before latch, falling back: %s",
                     esp_err_to_name(err));
        }
        err = write_bytes(REG_AP_DATAOUT0, command, sizeof(command));
    } else if (err != ESP_OK) {
        ESP_LOGW(TAG,
                 "MAX77958 direct-HS AP write verification failed after latch; not retrying: %s",
                 esp_err_to_name(err));
    }
    if (err == ESP_OK) {
        uint8_t last_uic = 0;
        err = wait_uic_mask(0x80U, APP_MAX77958_AP_CMD_TIMEOUT_MS,
                            &last_uic);
        if (err == ESP_OK) {
            err = read_bytes(REG_AP_DATAIN0, local.response,
                             sizeof(local.response));
        }
    }

    local.err = err;
    local.response_opcode = local.response[0];
    local.result_code = local.response[1];
    local.ok = err == ESP_OK && local.response_opcode == opcode;
    if (result != NULL) {
        *result = local;
    }
    return err == ESP_OK && local.ok ? ESP_OK
                                     : (err != ESP_OK ? err : ESP_FAIL);
}

static void update_snapshot_from_raw(
    const uint8_t raw[MAX77958_SERVICE_REGISTER_MAP_SIZE], esp_err_t read_err,
    uint32_t read_duration_ms)
{
    const uint32_t now_ms = ticks_to_ms();
    if (state_lock(pdMS_TO_TICKS(50))) {
        s_snapshot.monitor_enabled = APP_MAX77958_MONITOR_ENABLED_DEFAULT != 0;
        s_snapshot.config_write_supported = true;
        s_snapshot.config_writes_enabled = true;
        s_snapshot.last_error = read_err;
        s_snapshot.i2c_clock_hz = APP_MAX77958_I2C_CLOCK_HZ;
        s_snapshot.i2c_hs_direct_clock_hz =
            APP_MAX77958_I2C_HS_DIRECT_CLOCK_HZ;
        s_snapshot.i2c_hs_ext_requested =
            APP_MAX77958_I2C_HS_EXT_ENABLED != 0;
        s_snapshot.i2c_hs_ext_active = s_hs_ext_active;
        s_snapshot.i2c_hs_direct_reads_enabled =
            APP_MAX77958_I2C_HS_DIRECT_READS_ENABLED != 0;
        s_snapshot.i2c_hs_direct_writes_enabled =
            APP_MAX77958_I2C_HS_DIRECT_WRITES_ENABLED != 0;
        s_snapshot.i2c_hs_direct_read_count = s_hs_direct_read_count;
        s_snapshot.i2c_hs_direct_write_count = s_hs_direct_write_count;
        s_snapshot.i2c_hs_direct_error_count = s_hs_direct_error_count;
        s_snapshot.i2c_hs_direct_write_verify_count =
            s_hs_direct_write_verify_count;
        s_snapshot.i2c_hs_direct_write_verify_error_count =
            s_hs_direct_write_verify_error_count;
        s_snapshot.i2c_hs_direct_last_elapsed_us =
            s_hs_direct_last_elapsed_us;
        s_snapshot.i2c_hs_direct_scl_measure_error =
            s_hs_direct_scl_measure_error;
        s_snapshot.i2c_hs_direct_scl_edges = s_hs_direct_scl_edges;
        s_snapshot.i2c_hs_direct_scl_elapsed_us =
            s_hs_direct_scl_elapsed_us;
        s_snapshot.i2c_hs_direct_scl_measured_hz =
            s_hs_direct_scl_measured_hz;
        s_snapshot.last_operation_age_ms =
            elapsed_since(now_ms, s_last_operation_ms);
        if (read_err == ESP_OK) {
            memcpy(s_snapshot.raw, raw, MAX77958_SERVICE_REGISTER_MAP_SIZE);
            s_snapshot.present = true;
            s_snapshot.read_ok = true;
            s_snapshot.raw_valid = true;
            s_snapshot.read_count++;
            s_snapshot.last_read_duration_ms = read_duration_ms;
            s_last_update_ms = now_ms;
            s_snapshot.last_update_age_ms = 0;

            s_snapshot.device_id = raw[REG_DEVICE_ID];
            s_snapshot.device_rev = raw[REG_DEVICE_REV];
            s_snapshot.fw_rev = raw[REG_FW_REV];
            s_snapshot.fw_sub_ver = raw[REG_FW_SUB_VER];
            s_snapshot.uic_int = raw[REG_UIC_INT];
            s_snapshot.cc_int = raw[REG_CC_INT];
            s_snapshot.pd_int = raw[REG_PD_INT];
            s_snapshot.action_int = raw[REG_ACTION_INT];
            s_snapshot.usbc_status1 = raw[REG_USBC_STATUS1];
            s_snapshot.usbc_status2 = raw[REG_USBC_STATUS2];
            s_snapshot.bc_status = raw[REG_BC_STATUS];
            s_snapshot.dp_status = raw[REG_DP_STATUS];
            s_snapshot.cc_status0 = raw[REG_CC_STATUS0];
            s_snapshot.cc_status1 = raw[REG_CC_STATUS1];
            s_snapshot.pd_status0 = raw[REG_PD_STATUS0];
            s_snapshot.pd_status1 = raw[REG_PD_STATUS1];
            s_snapshot.uic_int_mask = raw[REG_UIC_INT_M];
            s_snapshot.cc_int_mask = raw[REG_CC_INT_M];
            s_snapshot.pd_int_mask = raw[REG_PD_INT_M];
            s_snapshot.action_int_mask = raw[REG_ACTION_INT_M];
            s_snapshot.sw_reset = raw[REG_SW_RESET];
            s_snapshot.i2c_cnfg = raw[REG_I2C_CNFG];

            s_snapshot.vbadc_code = decode_vbadc(s_snapshot.usbc_status1);
            decode_vbadc_range(s_snapshot.vbadc_code,
                                &s_snapshot.vbus_min_mv,
                                &s_snapshot.vbus_max_mv,
                                &s_snapshot.vbus_mid_mv,
                                &s_snapshot.vbus_above_range);
            s_snapshot.vbus_detected = (s_snapshot.bc_status & 0x80U) != 0U;
            s_snapshot.chg_typ = (uint8_t)(s_snapshot.bc_status & 0x03U);
            s_snapshot.dcd_timeout = (s_snapshot.bc_status & 0x04U) != 0U;
            s_snapshot.pr_chg_typ =
                (uint8_t)((s_snapshot.bc_status >> 3U) & 0x07U);
            s_snapshot.cc_pin =
                (uint8_t)((s_snapshot.cc_status0 >> 6U) & 0x03U);
            s_snapshot.cci =
                (uint8_t)((s_snapshot.cc_status0 >> 4U) & 0x03U);
            s_snapshot.vconn_enabled =
                (s_snapshot.cc_status0 & 0x08U) != 0U;
            s_snapshot.cc_stat = (uint8_t)(s_snapshot.cc_status0 & 0x07U);
            s_snapshot.det_abrt = (s_snapshot.cc_status1 & 0x04U) != 0U;
            s_snapshot.data_role_dfp =
                (s_snapshot.pd_status1 & 0x80U) != 0U;
            s_snapshot.power_role_source =
                (s_snapshot.pd_status1 & 0x40U) != 0U;
            s_snapshot.vconn_source = (s_snapshot.pd_status1 & 0x20U) != 0U;
            s_snapshot.psrdy_as_sink = (s_snapshot.pd_status1 & 0x10U) != 0U;
        } else {
            s_snapshot.present = false;
            s_snapshot.read_ok = false;
            s_snapshot.error_count++;
            s_snapshot.last_read_duration_ms = read_duration_ms;
        }
        state_unlock();
    }
}

static esp_err_t perform_status_read(void)
{
    uint8_t raw[MAX77958_SERVICE_REGISTER_MAP_SIZE] = {0};
    const uint32_t start_ms = ticks_to_ms();
    const esp_err_t err =
        s_i2c_dev != NULL ? read_register_map(raw) : ESP_ERR_INVALID_STATE;
    const uint32_t duration_ms = ticks_to_ms() - start_ms;
    update_snapshot_from_raw(raw, err, duration_ms);
    return err;
}

static esp_err_t refresh_discovery_state(void)
{
    max77958_service_ap_result_t result = {0};
    esp_err_t first_err = max77958_service_read_ctrl1(&result);
    result = (max77958_service_ap_result_t){0};
    const esp_err_t src_err =
        max77958_service_read_current_source_caps(&result);
    if (first_err == ESP_OK && src_err != ESP_OK) {
        first_err = src_err;
    }
    result = (max77958_service_ap_result_t){0};
    const esp_err_t snk_err = max77958_service_read_sink_pdos(false, &result);
    if (first_err == ESP_OK && snk_err != ESP_OK) {
        first_err = snk_err;
    }
    return first_err;
}

static void note_policy_result(const char *name, esp_err_t err,
                               esp_err_t *first_err,
                               uint32_t *applied_count)
{
    if (err == ESP_OK) {
        if (applied_count != NULL) {
            (*applied_count)++;
        }
        return;
    }
    ESP_LOGW(TAG, "MAX77958 saved policy apply failed for %s: %s", name,
             esp_err_to_name(err));
    if (first_err != NULL && *first_err == ESP_OK) {
        *first_err = err;
    }
}

static esp_err_t apply_saved_policy(void)
{
    max77958_policy_t policy = {0};
    const esp_err_t load_err = policy_load(&policy);
    if (load_err == ESP_ERR_NOT_FOUND) {
        ESP_LOGI(TAG, "MAX77958 has no saved PD policy in NVS");
        return ESP_OK;
    }
    if (load_err != ESP_OK) {
        ESP_LOGW(TAG, "MAX77958 saved policy load failed: %s",
                 esp_err_to_name(load_err));
        return load_err;
    }

    esp_err_t first_err = ESP_OK;
    uint32_t applied_count = 0;
    max77958_service_ap_result_t result = {0};

    if (policy.has_usb2_switch_closed) {
        note_policy_result(
            "USB2 switch",
            max77958_service_set_usb2_switch(policy.usb2_switch_closed,
                                             &result),
            &first_err, &applied_count);
    }
    if (policy.has_sink_pdos) {
        uint16_t voltages[MAX77958_SERVICE_MAX_SINK_PDOS] = {0};
        uint16_t currents[MAX77958_SERVICE_MAX_SINK_PDOS] = {0};
        for (size_t i = 0; i < policy.sink_pdo_count; ++i) {
            const uint32_t pdo = policy.sink_pdos[i];
            voltages[i] = (uint16_t)(((pdo >> 10U) & 0x3FFU) * 50U);
            currents[i] = (uint16_t)((pdo & 0x3FFU) * 10U);
        }
        result = (max77958_service_ap_result_t){0};
        note_policy_result(
            "sink PDOs",
            max77958_service_set_sink_fixed_pdos(
                voltages, currents, policy.sink_pdo_count, false, &result),
            &first_err, &applied_count);
    }
    if (policy.has_pps_default) {
        result = (max77958_service_ap_result_t){0};
        note_policy_result(
            "PPS default",
            max77958_service_set_pps_default(policy.pps_enabled,
                                             policy.pps_mv, policy.pps_ma,
                                             &result),
            &first_err, &applied_count);
    }
    if (policy.has_source_position) {
        result = (max77958_service_ap_result_t){0};
        note_policy_result(
            "source PDO request",
            max77958_service_request_source_pdo(policy.source_position,
                                                &result),
            &first_err, &applied_count);
    }

    ESP_LOGI(TAG, "MAX77958 saved policy applied: fields=%lu applied=%lu err=%s",
             (unsigned long)policy.field_count,
             (unsigned long)applied_count,
             esp_err_to_name(first_err));
    return first_err;
}

static void log_summary_if_needed(void)
{
    static uint32_t last_log_ms;
    const uint32_t now_ms = ticks_to_ms();
    if ((uint32_t)(now_ms - last_log_ms) < MAX77958_LOG_INTERVAL_MS) {
        return;
    }
    last_log_ms = now_ms;

    max77958_service_snapshot_t snapshot = {0};
    max77958_service_get_snapshot(&snapshot);
    if (!snapshot.present) {
        ESP_LOGW(TAG, "MAX77958 not present/read failed: err=%s reads=%lu errors=%lu",
                 esp_err_to_name((esp_err_t)snapshot.last_error),
                 (unsigned long)snapshot.read_count,
                 (unsigned long)snapshot.error_count);
        return;
    }

    ESP_LOGI(TAG,
             "MAX77958 id=0x%02X rev=0x%02X fw=%u.%u VBUS~%umV BC=%s CC=%s/%s PD=%s/%s src_pdos=%u selected=%u sink_pdos=%u reads=%lu hs=%s direct r/w/err=%lu/%lu/%lu",
             (unsigned)snapshot.device_id, (unsigned)snapshot.device_rev,
             (unsigned)snapshot.fw_rev, (unsigned)snapshot.fw_sub_ver,
             (unsigned)snapshot.vbus_mid_mv,
             max77958_service_chg_typ_to_string(snapshot.chg_typ),
             max77958_service_cc_pin_to_string(snapshot.cc_pin),
             max77958_service_cci_to_string(snapshot.cci),
             snapshot.power_role_source ? "source" : "sink",
             snapshot.data_role_dfp ? "DFP" : "UFP",
             (unsigned)snapshot.source_pdo_count,
             (unsigned)snapshot.selected_source_pdo_pos,
             (unsigned)snapshot.sink_pdo_count,
             (unsigned long)snapshot.read_count,
             snapshot.i2c_hs_ext_active ? "on" : "off",
             (unsigned long)snapshot.i2c_hs_direct_read_count,
             (unsigned long)snapshot.i2c_hs_direct_write_count,
             (unsigned long)snapshot.i2c_hs_direct_error_count);
}

static void max77958_task(void *arg)
{
    (void)arg;
    s_task_handle = xTaskGetCurrentTaskHandle();

    vTaskDelay(pdMS_TO_TICKS(MAX77958_START_DELAY_MS));

    ESP_LOGI(TAG,
             "MAX77958 monitor: SDA=%d SCL=%d addr=0x%02X clock=%u Hz interval=%ums core=%d",
             BOARD_CONFIG_MAX77958_SDA_GPIO, BOARD_CONFIG_MAX77958_SCL_GPIO,
             APP_MAX77958_I2C_ADDRESS, (unsigned)APP_MAX77958_I2C_CLOCK_HZ,
             (unsigned)APP_MAX77958_READ_INTERVAL_MS, MAX77958_TASK_CORE);

    const esp_err_t init_err = max77958_i2c_init();
    if (init_err != ESP_OK) {
        ESP_LOGW(TAG, "MAX77958 I2C init/probe failed: %s",
                 esp_err_to_name(init_err));
        update_snapshot_from_raw((uint8_t[MAX77958_SERVICE_REGISTER_MAP_SIZE]){0},
                                 init_err, 0);
    }

    if (init_err == ESP_OK) {
        const esp_err_t hs_err = max77958_enter_hs_ext_if_enabled();
        if (hs_err != ESP_OK) {
            ESP_LOGW(TAG, "MAX77958 direct HS disabled: %s",
                     esp_err_to_name(hs_err));
        }
    }

    if (init_err == ESP_OK && s_i2c_dev != NULL) {
        (void)perform_status_read();
        (void)apply_saved_policy();
        (void)refresh_discovery_state();
        (void)perform_status_read();
        log_summary_if_needed();
    }

    while (true) {
        (void)ulTaskNotifyTake(pdTRUE,
                               pdMS_TO_TICKS(APP_MAX77958_READ_INTERVAL_MS));
        const bool refresh = s_refresh_requested;
        s_refresh_requested = false;
        if (s_i2c_dev == NULL) {
            if (max77958_i2c_init() == ESP_OK) {
                (void)max77958_enter_hs_ext_if_enabled();
            }
        }
        (void)perform_status_read();
        if (refresh && s_i2c_dev != NULL) {
            (void)refresh_discovery_state();
        }
        log_summary_if_needed();
    }
}

esp_err_t max77958_service_start(void)
{
    if (!APP_MAX77958_MONITOR_ENABLED_DEFAULT) {
        ESP_LOGI(TAG, "MAX77958 monitor disabled by config");
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
        s_snapshot.last_operation_age_ms = UINT32_MAX;
        s_snapshot.last_error = ESP_OK;
        s_snapshot.last_operation_error = ESP_OK;
        s_snapshot.i2c_clock_hz = APP_MAX77958_I2C_CLOCK_HZ;
        s_snapshot.i2c_hs_direct_clock_hz =
            APP_MAX77958_I2C_HS_DIRECT_CLOCK_HZ;
        s_snapshot.i2c_hs_ext_requested =
            APP_MAX77958_I2C_HS_EXT_ENABLED != 0;
        s_snapshot.i2c_hs_direct_reads_enabled =
            APP_MAX77958_I2C_HS_DIRECT_READS_ENABLED != 0;
        s_snapshot.i2c_hs_direct_writes_enabled =
            APP_MAX77958_I2C_HS_DIRECT_WRITES_ENABLED != 0;
        state_unlock();
    }

    const BaseType_t created = xTaskCreatePinnedToCore(
        max77958_task, "max77958", MAX77958_TASK_STACK_WORDS, NULL,
        MAX77958_TASK_PRIORITY, &s_task_handle, MAX77958_TASK_CORE);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create MAX77958 task");
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    return ESP_OK;
}

void max77958_service_get_snapshot(max77958_service_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    if (state_lock(pdMS_TO_TICKS(50))) {
        *snapshot = s_snapshot;
        const uint32_t now_ms = ticks_to_ms();
        snapshot->last_update_age_ms = elapsed_since(now_ms, s_last_update_ms);
        snapshot->last_operation_age_ms =
            elapsed_since(now_ms, s_last_operation_ms);
        state_unlock();
    } else {
        memset(snapshot, 0, sizeof(*snapshot));
        snapshot->last_update_age_ms = UINT32_MAX;
        snapshot->last_operation_age_ms = UINT32_MAX;
        snapshot->last_error = ESP_ERR_TIMEOUT;
        snapshot->last_operation_error = ESP_ERR_TIMEOUT;
    }
}

void max77958_service_format_raw_hex(
    const max77958_service_snapshot_t *snapshot, char *buffer,
    size_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0) {
        return;
    }
    buffer[0] = '\0';
    if (snapshot == NULL || !snapshot->raw_valid) {
        return;
    }

    size_t offset = 0;
    for (size_t i = 0; i < MAX77958_SERVICE_REGISTER_MAP_SIZE; ++i) {
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

esp_err_t max77958_service_read_register(uint8_t reg, uint8_t *value)
{
    if (value == NULL || reg >= MAX77958_SERVICE_REGISTER_MAP_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    return read_bytes(reg, value, 1);
}

esp_err_t max77958_service_write_register(uint8_t reg, uint8_t value)
{
    if (reg >= MAX77958_SERVICE_REGISTER_MAP_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    const esp_err_t err = write_bytes(reg, &value, 1);
    if (err == ESP_OK) {
        max77958_service_request_refresh();
    }
    return err;
}

void max77958_service_request_refresh(void)
{
    s_refresh_requested = true;
    const TaskHandle_t task = s_task_handle;
    if (task != NULL) {
        xTaskNotifyGive(task);
    }
}

esp_err_t max77958_service_trigger_bc_detection(
    max77958_service_ap_result_t *result)
{
    uint8_t payload[32] = {0};
    payload[0] = 0x01U;
    esp_err_t err = ap_send33(OP_BC_CTRL1_WRITE, payload, result);
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(5));
        payload[0] = 0x03U;
        err = ap_send33(OP_BC_CTRL1_WRITE, payload, result);
        if (err == ESP_OK) {
            (void)wait_uic_mask(1U << 1U, APP_MAX77958_BC_TRIGGER_TIMEOUT_MS,
                                NULL);
        }
    }
    if (result != NULL) {
        record_ap_result(result, false, false, false, false);
    }
    max77958_service_request_refresh();
    return err;
}

esp_err_t max77958_service_read_ctrl1(max77958_service_ap_result_t *result)
{
    uint8_t payload[32] = {0};
    const esp_err_t err = ap_send33(OP_CTRL1_READ, payload, result);
    if (result != NULL) {
        record_ap_result(result, false, false, false, true);
    }
    return err;
}

esp_err_t max77958_service_set_usb2_switch(
    bool closed, max77958_service_ap_result_t *result)
{
    uint8_t payload[32] = {0};
    const uint8_t value = closed ? 0x01U : 0x00U;
    payload[0] = (uint8_t)((value << 3U) | value);
    const esp_err_t err = ap_send33(OP_CTRL1_WRITE, payload, result);
    if (result != NULL) {
        record_ap_result(result, false, false, false, false);
    }
    if (err == ESP_OK) {
        (void)policy_write_u8(KEY_USB2_CLOSED, closed ? 1U : 0U);
        (void)max77958_service_read_ctrl1(NULL);
    }
    max77958_service_request_refresh();
    return err;
}

esp_err_t max77958_service_read_current_source_caps(
    max77958_service_ap_result_t *result)
{
    uint8_t payload[32] = {0};
    const esp_err_t err = ap_send33(OP_CURRENT_SRC_CAP, payload, result);
    if (result != NULL) {
        record_ap_result(result, true, false, false, false);
    }
    return err;
}

esp_err_t max77958_service_request_source_pdo(
    uint8_t position, max77958_service_ap_result_t *result)
{
    if (position == 0U || position > MAX77958_SERVICE_MAX_SOURCE_PDOS) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t payload[32] = {0};
    payload[0] = position & 0x07U;
    const esp_err_t err = ap_send33(OP_SRC_CAP_REQUEST, payload, result);
    if (result != NULL) {
        record_ap_result(result, false, false, false, false);
    }
    if (err == ESP_OK) {
        (void)policy_write_u8(KEY_SOURCE_POS, position);
        (void)max77958_service_read_current_source_caps(NULL);
    }
    max77958_service_request_refresh();
    return err;
}

esp_err_t max77958_service_read_sink_pdos(
    bool mtp, max77958_service_ap_result_t *result)
{
    uint8_t payload[32] = {0};
    payload[0] = mtp ? 0x80U : 0x00U;
    const esp_err_t err = ap_send33(OP_SNK_PDO_REQUEST, payload, result);
    if (result != NULL) {
        record_ap_result(result, false, true, mtp, false);
    }
    return err;
}

esp_err_t max77958_service_set_sink_fixed_pdos(
    const uint16_t *voltages_mv, const uint16_t *currents_ma, size_t count,
    bool mtp, max77958_service_ap_result_t *result)
{
    if (voltages_mv == NULL || currents_ma == NULL || count == 0 ||
        count > MAX77958_SERVICE_MAX_SINK_PDOS) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t pdos[MAX77958_SERVICE_MAX_SINK_PDOS] = {0};
    uint8_t payload[32] = {0};
    payload[0] = (uint8_t)(count & 0x07U);
    if (mtp) {
        payload[0] |= 0x80U;
    }
    for (size_t i = 0; i < count; ++i) {
        if (!fixed_pdo_from_mv_ma(voltages_mv[i], currents_ma[i], &pdos[i])) {
            return ESP_ERR_INVALID_ARG;
        }
        write_le_u32(&payload[1U + (i * 4U)], pdos[i]);
    }

    const esp_err_t err = ap_send33(OP_SNK_PDO_SET, payload, result);
    if (result != NULL) {
        record_ap_result(result, false, false, false, false);
    }
    if (err == ESP_OK) {
        (void)policy_save_sink_pdos(pdos, count);
        (void)max77958_service_read_sink_pdos(false, NULL);
    }
    max77958_service_request_refresh();
    return err;
}

esp_err_t max77958_service_set_pps_default(
    bool enabled, uint16_t voltage_mv, uint16_t current_ma,
    max77958_service_ap_result_t *result)
{
    if ((voltage_mv % 20U) != 0U || (current_ma % 50U) != 0U ||
        voltage_mv > 20000U || current_ma > 6200U) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t payload[32] = {0};
    payload[0] = enabled ? 0x01U : 0x00U;
    const uint16_t voltage_units = (uint16_t)(voltage_mv / 20U);
    payload[1] = (uint8_t)(voltage_units & 0xFFU);
    payload[2] = (uint8_t)((voltage_units >> 8U) & 0xFFU);
    payload[3] = (uint8_t)(current_ma / 50U);

    const esp_err_t err = ap_send33(OP_SET_PPS, payload, result);
    if (result != NULL) {
        record_ap_result(result, false, false, false, false);
    }
    if (err == ESP_OK) {
        (void)policy_save_pps(enabled, voltage_mv, current_ma);
        if (state_lock(pdMS_TO_TICKS(50))) {
            s_snapshot.pps_default_valid = true;
            s_snapshot.pps_default_enabled = enabled;
            s_snapshot.pps_default_mv = voltage_mv;
            s_snapshot.pps_default_ma = current_ma;
            state_unlock();
        }
    }
    max77958_service_request_refresh();
    return err;
}

esp_err_t max77958_service_request_apdo(
    uint8_t position, uint16_t voltage_mv, uint16_t current_ma,
    max77958_service_ap_result_t *result)
{
    if (position == 0U || (voltage_mv % 20U) != 0U ||
        (current_ma % 50U) != 0U || voltage_mv > 20000U ||
        current_ma > 6200U) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t payload[32] = {0};
    payload[0] = position;
    const uint16_t voltage_units = (uint16_t)(voltage_mv / 20U);
    payload[1] = (uint8_t)(voltage_units & 0xFFU);
    payload[2] = (uint8_t)((voltage_units >> 8U) & 0xFFU);
    payload[3] = (uint8_t)(current_ma / 50U);

    const esp_err_t err = ap_send33(OP_APDO_SRC_CAP_REQUEST, payload, result);
    if (result != NULL) {
        record_ap_result(result, false, false, false, false);
    }
    max77958_service_request_refresh();
    return err;
}
