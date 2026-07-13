#include "ota_service.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include "esp_app_desc.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "app_identity.h"
#include "app_runtime_config.h"
#include "bno085_service.h"
#include "charger_service.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps_service.h"
#include "uwb_config.h"
#include "uwb_dw3000.h"
#include "wifi_service.h"
#include "wireless_log_service.h"
#include "wireless_telemetry_service.h"

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef APP_OTA_PASSWORD
#define APP_OTA_PASSWORD ""
#endif

static const char *TAG = "ota_service";

enum {
    OTA_SERVICE_TASK_STACK_WORDS = 4096,
    OTA_SERVICE_TASK_PRIORITY = 5,
    OTA_SERVICE_RESTART_TASK_STACK_WORDS = 2048,
    OTA_SERVICE_RESTART_TASK_PRIORITY = 5,
    OTA_SERVICE_CHUNK_SIZE = 4096,
    OTA_SERVICE_WIFI_WAIT_MS = 500,
    OTA_SERVICE_REBOOT_DELAY_MS = 1200,
    OTA_SERVICE_MAX_TOKEN_LEN = 128,
    OTA_SERVICE_MAX_QUERY_LEN = 768,
    OTA_SERVICE_STATUS_RESPONSE_SIZE = 16000,
};

#define OTA_SERVICE_TOKEN_HEADER "X-OTA-Token"

static httpd_handle_t s_http_server;
static bool s_started;
static bool s_running;
static volatile bool s_ota_in_progress;
static volatile enum ota_service_status s_status = OTA_SERVICE_STATUS_IDLE;
static uint8_t s_ota_buffer[OTA_SERVICE_CHUNK_SIZE];

static void reboot_task(void *arg);

static const char *ota_status_to_string(enum ota_service_status status)
{
    switch (status) {
    case OTA_SERVICE_STATUS_IDLE:
        return "idle";
    case OTA_SERVICE_STATUS_WAITING_FOR_WIFI:
        return "waiting_for_wifi";
    case OTA_SERVICE_STATUS_RUNNING:
        return "running";
    case OTA_SERVICE_STATUS_UPDATING:
        return "updating";
    case OTA_SERVICE_STATUS_REBOOTING:
        return "rebooting";
    case OTA_SERVICE_STATUS_FAILED:
        return "failed";
    default:
        return "unknown";
    }
}

static const char *partition_label_or_unknown(const esp_partition_t *partition)
{
    return partition != NULL ? partition->label : "unknown";
}

static bool ota_token_configured(void)
{
    return strlen(APP_OTA_PASSWORD) > 0;
}

static bool constant_time_string_equal(const char *a, const char *b)
{
    const size_t a_len = strlen(a);
    const size_t b_len = strlen(b);
    const size_t max_len = a_len > b_len ? a_len : b_len;
    unsigned char diff = (unsigned char)(a_len ^ b_len);

    for (size_t i = 0; i < max_len; ++i) {
        const unsigned char ca = i < a_len ? (unsigned char)a[i] : 0;
        const unsigned char cb = i < b_len ? (unsigned char)b[i] : 0;
        diff |= (unsigned char)(ca ^ cb);
    }

    return diff == 0;
}

static bool ota_request_authorized(httpd_req_t *req)
{
    if (!ota_token_configured()) {
        return false;
    }

    const size_t token_len =
        httpd_req_get_hdr_value_len(req, OTA_SERVICE_TOKEN_HEADER);
    if (token_len == 0 || token_len >= OTA_SERVICE_MAX_TOKEN_LEN) {
        return false;
    }

    char token[OTA_SERVICE_MAX_TOKEN_LEN];
    if (httpd_req_get_hdr_value_str(req, OTA_SERVICE_TOKEN_HEADER, token,
                                    sizeof(token)) != ESP_OK) {
        return false;
    }

    return constant_time_string_equal(token, APP_OTA_PASSWORD);
}

static bool ota_parse_u16(const char *text, uint16_t *value)
{
    if (text == NULL || text[0] == '\0' || value == NULL) {
        return false;
    }

    errno = 0;
    char *end = NULL;
    const unsigned long parsed = strtoul(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || parsed > 0xFFFFUL) {
        return false;
    }

    *value = (uint16_t)parsed;
    return true;
}

static bool ota_parse_u8(const char *text, uint8_t *value)
{
    if (text == NULL || text[0] == '\0' || value == NULL) {
        return false;
    }

    errno = 0;
    char *end = NULL;
    const unsigned long parsed = strtoul(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || parsed > 0xFFUL) {
        return false;
    }

    *value = (uint8_t)parsed;
    return true;
}

static bool ota_parse_u32(const char *text, uint32_t *value)
{
    if (text == NULL || text[0] == '\0' || value == NULL) {
        return false;
    }

    errno = 0;
    char *end = NULL;
    const unsigned long parsed = strtoul(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' ||
        parsed > 0xFFFFFFFFUL) {
        return false;
    }

    *value = (uint32_t)parsed;
    return true;
}

static bool ota_parse_bool_text(const char *text, bool *value)
{
    if (text == NULL || value == NULL) {
        return false;
    }
    if (strcmp(text, "1") == 0 || strcmp(text, "true") == 0 ||
        strcmp(text, "on") == 0 || strcmp(text, "yes") == 0) {
        *value = true;
        return true;
    }
    if (strcmp(text, "0") == 0 || strcmp(text, "false") == 0 ||
        strcmp(text, "off") == 0 || strcmp(text, "no") == 0) {
        *value = false;
        return true;
    }
    return false;
}

static bool ota_parse_bno085_sample_hz(const char *text, uint32_t *interval_ms)
{
    uint32_t hz = 0;
    if (!ota_parse_u32(text, &hz) || hz == 0 || hz > 500U ||
        interval_ms == NULL) {
        return false;
    }

    uint32_t ms = (1000U + (hz / 2U)) / hz;
    if (ms < 2U) {
        ms = 2U;
    }
    *interval_ms = ms;
    return true;
}

static bool ota_parse_u8_list(const char *text, uint8_t *values,
                              size_t max_count, uint8_t *count)
{
    if (text == NULL || text[0] == '\0' || values == NULL ||
        count == NULL || max_count == 0) {
        return false;
    }

    const char *cursor = text;
    uint8_t parsed_count = 0;

    while (*cursor != '\0') {
        while (*cursor == ' ') {
            cursor++;
        }

        errno = 0;
        char *end = NULL;
        const unsigned long parsed = strtoul(cursor, &end, 0);
        if (errno != 0 || end == cursor || parsed > 0xFFUL ||
            parsed_count >= max_count) {
            return false;
        }

        values[parsed_count++] = (uint8_t)parsed;
        cursor = end;
        while (*cursor == ' ') {
            cursor++;
        }

        if (*cursor == '\0') {
            break;
        }
        if (*cursor != ',' && *cursor != ';' && *cursor != ':') {
            return false;
        }
        cursor++;
    }

    if (parsed_count == 0) {
        return false;
    }

    *count = parsed_count;
    return true;
}

static bool ota_parse_calibration_method(const char *text, uint8_t *value)
{
    uint8_t parsed = 0;
    if (ota_parse_u8(text, &parsed) &&
        (parsed == APP_UWB_CALIBRATION_METHOD_TWO_MODULE ||
         parsed == APP_UWB_CALIBRATION_METHOD_THREE_MODULE)) {
        *value = parsed;
        return true;
    }

    if (strcmp(text, "two") == 0 || strcmp(text, "two_module") == 0) {
        *value = APP_UWB_CALIBRATION_METHOD_TWO_MODULE;
        return true;
    }
    if (strcmp(text, "three") == 0 || strcmp(text, "three_module") == 0 ||
        strcmp(text, "three_module_edm") == 0 ||
        strcmp(text, "edm") == 0) {
        *value = APP_UWB_CALIBRATION_METHOD_THREE_MODULE;
        return true;
    }

    return false;
}

static bool ota_query_option_enabled(const char *query, const char *key)
{
    char value[8] = {0};
    return httpd_query_key_value(query, key, value, sizeof(value)) == ESP_OK &&
           strcmp(value, "1") == 0;
}

static bool ota_query_has_key(const char *query, const char *key)
{
    char value[8] = {0};
    return httpd_query_key_value(query, key, value, sizeof(value)) == ESP_OK;
}

static bool runtime_config_reboot_recommended(
    const app_runtime_config_t *before, const app_runtime_config_t *after)
{
    if (before == NULL || after == NULL) {
        return true;
    }

    return before->runtime_mode != after->runtime_mode ||
           before->tag_id != after->tag_id ||
           before->anchor_count != after->anchor_count ||
           memcmp(before->anchor_ids, after->anchor_ids,
                  sizeof(before->anchor_ids)) != 0 ||
           before->anchor_survey_coordinator_id !=
               after->anchor_survey_coordinator_id ||
           before->uwb_enabled != after->uwb_enabled ||
           before->radio_channel != after->radio_channel;
}

static uint8_t runtime_radio_channel(const app_runtime_config_t *config)
{
    return config != NULL && config->radio_channel == 9U ? 9U : 5U;
}

static uint8_t runtime_radio_rf_channel_bit(const app_runtime_config_t *config)
{
    return runtime_radio_channel(config) == 9U ? 1U : 0U;
}

static uint8_t runtime_radio_profile(const app_runtime_config_t *config)
{
    return runtime_radio_channel(config) == 9U
               ? APP_UWB_RADIO_PROFILE_LEGACY_CH9_6M8_PLEN128
               : APP_UWB_RADIO_PROFILE_LEGACY_CH5_6M8_PLEN128;
}

static uint32_t runtime_radio_rf_tx_ctrl_2(const app_runtime_config_t *config)
{
    return runtime_radio_channel(config) == 9U
               ? APP_UWB_RADIO_RF_TX_CTRL_2_CH9
               : APP_UWB_RADIO_RF_TX_CTRL_2_CH5;
}

static uint32_t runtime_radio_pll_cfg_final(const app_runtime_config_t *config)
{
    return runtime_radio_channel(config) == 9U
               ? APP_UWB_RADIO_PLL_CFG_FINAL_CH9
               : APP_UWB_RADIO_PLL_CFG_FINAL_CH5;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    static const char response[] =
        "uwb_esp_idf\n"
        "GET  /status\n"
        "POST /ota    raw firmware image, requires X-OTA-Token header\n"
        "POST /config/antenna-delay?value=0x4018[&reboot=1]\n"
        "POST /config/antenna-delay?clear=1[&reboot=1]\n"
        "POST /config/runtime?mode=ranging&tag=1&anchors=2,3,4,5[&reboot=1]\n"
        "POST /config/runtime?clear=1[&reboot=1]\n"
        "POST /config/charger?adc=1&adc_sample=2\n"
        "POST /config/charger?charge_current_ma=500&charge_voltage_mv=4200\n"
        "POST /config/charger?fast_charge_timer_enabled=1&fast_charge_timer_hours=12\n";

    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_get_handler(httpd_req_t *req)
{
    const esp_app_desc_t *app = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *boot = esp_ota_get_boot_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    const uint16_t active_antenna_delay = uwb_dw3000_get_antenna_delay();
    const uint16_t configured_antenna_delay =
        app_identity_get_uwb_antenna_delay();
    const app_runtime_config_t *runtime_config = app_runtime_config_get();
    gps_service_snapshot_t gps_snapshot = {0};
    gps_service_get_snapshot(&gps_snapshot);
    charger_service_snapshot_t charger_snapshot = {0};
    charger_service_get_snapshot(&charger_snapshot);
    char charger_raw_hex[(CHARGER_SERVICE_REGISTER_MAP_SIZE * 2U) + 1U] = {0};
    charger_service_format_raw_hex(&charger_snapshot, charger_raw_hex,
                                   sizeof(charger_raw_hex));

    char *response = malloc(OTA_SERVICE_STATUS_RESPONSE_SIZE);
    if (response == NULL) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "status allocation failed");
    }

    const int len = snprintf(
        response, OTA_SERVICE_STATUS_RESPONSE_SIZE,
        "{"
        "\"project\":\"%s\","
        "\"version\":\"%s\","
        "\"idf\":\"%s\","
        "\"hostname\":\"%s\","
        "\"module_id\":%u,"
        "\"module_id_from_nvs\":%s,"
        "\"module_id_provisioned_this_boot\":%s,"
        "\"uwb_role\":%u,"
        "\"uwb_role_name\":\"%s\","
        "\"uwb_role_from_nvs\":%s,"
        "\"uwb_role_provisioned_this_boot\":%s,"
        "\"ota_status\":\"%s\","
        "\"wifi_connected\":%s,"
        "\"ip\":\"%s\","
        "\"wifi_disconnect_count\":%lu,"
        "\"wifi_last_disconnect_reason\":%u,"
        "\"wifi_last_disconnect_reason_name\":\"%s\","
        "\"wifi_connected_bssid\":\"%s\","
        "\"wifi_connected_channel\":%u,"
        "\"wifi_connected_rssi\":%d,"
        "\"wifi_scan_ap_count\":%u,"
        "\"wifi_scan_best_bssid\":\"%s\","
        "\"wifi_scan_best_channel\":%u,"
        "\"wifi_scan_best_rssi\":%d,"
        "\"running_partition\":\"%s\","
        "\"boot_partition\":\"%s\","
        "\"next_update_partition\":\"%s\","
        "\"ota_in_progress\":%s,"
        "\"ota_auth_configured\":%s,"
        "\"runtime_config_from_nvs\":%s,"
        "\"runtime_mode\":%u,"
        "\"runtime_mode_name\":\"%s\","
        "\"runtime_tag_id\":%u,"
        "\"runtime_anchor_count\":%u,"
        "\"runtime_anchor_ids\":[%u,%u,%u,%u],"
        "\"runtime_anchor_survey_coordinator_id\":%u,"
        "\"runtime_anchor_survey_rx_slice_ms\":%lu,"
        "\"runtime_anchor_survey_command_delay_ms\":%lu,"
        "\"runtime_anchor_survey_slot_ms\":%lu,"
        "\"runtime_anchor_survey_round_gap_ms\":%lu,"
        "\"runtime_anchor_survey_passive_tag_log_every\":%lu,"
        "\"runtime_ranging_slot_ms\":%lu,"
        "\"runtime_ranging_round_gap_ms\":%lu,"
        "\"runtime_ranging_rx_slice_ms\":%lu,"
        "\"runtime_distance_test_peer_id\":%u,"
        "\"runtime_distance_test_initiator_id\":%u,"
        "\"runtime_distance_test_responder_id\":%u,"
        "\"runtime_distance_test_interval_ms\":%lu,"
        "\"runtime_distance_test_rx_timeout_ms\":%lu,"
        "\"runtime_distance_test_resp_delay_ms\":%lu,"
        "\"runtime_distance_test_final_delay_ms\":%lu,"
        "\"runtime_distance_test_report_delay_ms\":%lu,"
        "\"runtime_distance_test_auto_rx_delay_uus\":%lu,"
        "\"runtime_calibration_method\":%u,"
        "\"runtime_calibration_reference_id\":%u,"
        "\"runtime_calibration_dut_id\":%u,"
        "\"runtime_calibration_three_ids\":[%u,%u,%u],"
        "\"runtime_calibration_known_distance_mm\":%lu,"
        "\"runtime_calibration_three_distance_0_1_mm\":%lu,"
        "\"runtime_calibration_three_distance_0_2_mm\":%lu,"
        "\"runtime_calibration_three_distance_1_2_mm\":%lu,"
        "\"runtime_calibration_sample_count\":%lu,"
        "\"runtime_calibration_summary_every\":%lu,"
        "\"runtime_calibration_min_interval_ms\":%lu,"
        "\"runtime_calibration_max_interval_ms\":%lu,"
        "\"runtime_calibration_rx_slice_ms\":%lu,"
        "\"runtime_calibration_slot_guard_us\":%lu,"
        "\"runtime_uwb_enabled\":%s,"
        "\"runtime_bno085_accel_enabled\":%s,"
        "\"runtime_bno085_accel_interval_ms\":%lu,"
        "\"runtime_bno085_log_interval_ms\":%lu,"
        "\"runtime_gps_enabled\":%s,"
        "\"gps_powered\":%s,"
        "\"gps_task_running\":%s,"
        "\"gps_uart_ready\":%s,"
        "\"gps_last_error\":%d,"
        "\"gps_last_error_name\":\"%s\","
        "\"gps_fix_valid\":%s,"
        "\"gps_fix_quality\":%d,"
        "\"gps_fix_quality_text\":\"%s\","
        "\"gps_fix_type\":%u,"
        "\"gps_satellites\":%u,"
        "\"gps_satellites_in_view\":%u,"
        "\"gps_hdop\":%.2f,"
        "\"gps_latitude_deg\":%.8f,"
        "\"gps_longitude_deg\":%.8f,"
        "\"gps_altitude_m\":%.2f,"
        "\"gps_speed_mps\":%.2f,"
        "\"gps_course_deg\":%.1f,"
        "\"gps_rmc_status\":\"%c\","
        "\"gps_rmc_mode\":\"%c\","
        "\"gps_utc_time\":\"%s\","
        "\"gps_utc_date\":\"%s\","
        "\"gps_last_sentence\":\"%s\","
        "\"gps_last_rx_age_ms\":%lu,"
        "\"gps_last_fix_age_ms\":%lu,"
        "\"gps_byte_count\":%lu,"
        "\"gps_sentence_count\":%lu,"
        "\"gps_gga_count\":%lu,"
        "\"gps_rmc_count\":%lu,"
        "\"gps_gsa_count\":%lu,"
        "\"gps_gsv_count\":%lu,"
        "\"gps_psti030_count\":%lu,"
        "\"gps_rtk_age_s\":%.2f,"
        "\"gps_rtk_ratio\":%.2f,"
        "\"gps_checksum_errors\":%lu,"
        "\"gps_parse_errors\":%lu,"
        "\"charger_monitor_enabled\":%s,"
        "\"charger_present\":%s,"
        "\"charger_read_ok\":%s,"
        "\"charger_raw_valid\":%s,"
        "\"charger_config_write_supported\":%s,"
        "\"charger_config_writes_enabled\":%s,"
        "\"charger_last_error\":%d,"
        "\"charger_last_error_name\":\"%s\","
        "\"charger_read_count\":%lu,"
        "\"charger_full_read_count\":%lu,"
        "\"charger_quick_read_count\":%lu,"
        "\"charger_error_count\":%lu,"
        "\"charger_last_read_duration_ms\":%lu,"
        "\"charger_last_read_full\":%s,"
        "\"charger_last_update_age_ms\":%lu,"
        "\"charger_int_gpio_level\":%d,"
        "\"charger_int_irq_count\":%lu,"
        "\"charger_int_last_irq_age_ms\":%lu,"
        "\"charger_pg_gpio_level\":%d,"
        "\"charger_pg_asserted\":%s,"
        "\"charger_pg_stat\":%s,"
        "\"charger_qon_gpio_level\":%d,"
        "\"charger_qon_asserted\":%s,"
        "\"charger_part_info\":\"0x%02x\","
        "\"charger_part_number\":%u,"
        "\"charger_device_revision\":%u,"
        "\"charger_adc_enabled\":%s,"
        "\"charger_reg0d_iotg_regulation\":\"0x%02x\","
        "\"charger_reg0e_timer_control\":\"0x%02x\","
        "\"charger_reg16_temperature_control\":\"0x%02x\","
        "\"charger_reg17_ntc_control_0\":\"0x%02x\","
        "\"charger_reg18_ntc_control_1\":\"0x%02x\","
        "\"charger_reg0f_charger_control_0\":\"0x%02x\","
        "\"charger_reg10_charger_control_1\":\"0x%02x\","
        "\"charger_reg14_charger_control_5\":\"0x%02x\","
        "\"charger_reg2e_adc_control\":\"0x%02x\","
        "\"charger_reg2f_adc_disable_0\":\"0x%02x\","
        "\"charger_reg30_adc_disable_1\":\"0x%02x\","
        "\"charger_minimal_system_voltage_mv\":%u,"
        "\"charger_charge_voltage_limit_mv\":%u,"
        "\"charger_charge_current_limit_ma\":%u,"
        "\"charger_input_voltage_limit_mv\":%u,"
        "\"charger_input_current_limit_ma\":%u,"
        "\"charger_external_input_current_limit_enabled\":%s,"
        "\"charger_charge_enabled\":%s,"
        "\"charger_charge_status_code\":%u,"
        "\"charger_vbus_status_code\":%u,"
        "\"charger_iindpm_active\":%s,"
        "\"charger_vindpm_active\":%s,"
        "\"charger_vsys_regulation_active\":%s,"
        "\"charger_battery_overvoltage_active\":%s,"
        "\"charger_charge_safety_timer_expired\":%s,"
        "\"charger_topoff_timer_flag\":%s,"
        "\"charger_trickle_timer_flag\":%s,"
        "\"charger_precharge_timer_flag\":%s,"
        "\"charger_fast_charge_timer_flag\":%s,"
        "\"charger_watchdog_setting\":%u,"
        "\"charger_watchdog_disabled\":%s,"
        "\"charger_topoff_timer_minutes\":%u,"
        "\"charger_trickle_timer_enabled\":%s,"
        "\"charger_precharge_timer_enabled\":%s,"
        "\"charger_fast_charge_timer_enabled\":%s,"
        "\"charger_fast_charge_timer_hours\":%u,"
        "\"charger_timer_2x_enabled\":%s,"
        "\"charger_precharge_timer_minutes\":%u,"
        "\"charger_adc_sample\":%u,"
        "\"charger_adc_continuous\":%s,"
        "\"charger_adc_running_average\":%s,"
        "\"charger_ibat_discharge_sense_enabled\":%s,"
        "\"charger_status\":[%u,%u,%u,%u,%u],"
        "\"charger_fault_status\":[%u,%u],"
        "\"charger_flag\":[%u,%u,%u,%u],"
        "\"charger_fault_flag\":[%u,%u],"
        "\"charger_ibus_ma\":%d,"
        "\"charger_ibat_ma\":%d,"
        "\"charger_vbus_mv\":%u,"
        "\"charger_vac1_mv\":%u,"
        "\"charger_vac2_mv\":%u,"
        "\"charger_vbat_mv\":%u,"
        "\"charger_vsys_mv\":%u,"
        "\"charger_battery_soc_valid\":%s,"
        "\"charger_battery_soc_percent\":%u,"
        "\"charger_ts_percent\":%.4f,"
        "\"charger_ts_ignore\":%s,"
        "\"charger_ts_cold_active\":%s,"
        "\"charger_ts_cool_active\":%s,"
        "\"charger_ts_warm_active\":%s,"
        "\"charger_ts_hot_active\":%s,"
        "\"charger_ts_cold_flag\":%s,"
        "\"charger_ts_cool_flag\":%s,"
        "\"charger_ts_warm_flag\":%s,"
        "\"charger_ts_hot_flag\":%s,"
        "\"charger_tdie_c\":%.1f,"
        "\"charger_dp_mv\":%u,"
        "\"charger_dm_mv\":%u,"
        "\"charger_write_count\":%lu,"
        "\"charger_write_error_count\":%lu,"
        "\"charger_last_write_age_ms\":%lu,"
        "\"charger_last_write_reg\":\"0x%02x\","
        "\"charger_last_write_requested_value\":\"0x%02x\","
        "\"charger_last_write_mask\":\"0x%02x\","
        "\"charger_last_write_before\":\"0x%02x\","
        "\"charger_last_write_after\":\"0x%02x\","
        "\"charger_last_write_mask_used\":%s,"
        "\"charger_last_write_changed\":%s,"
        "\"charger_last_write_error\":%d,"
        "\"charger_last_write_error_name\":\"%s\","
        "\"charger_raw_hex\":\"%s\","
        "\"runtime_radio_channel\":%u,"
        "\"runtime_wireless_telemetry_port\":%lu,"
        "\"uwb_status\":\"%s\","
        "\"uwb_radio_profile\":%u,"
        "\"uwb_radio_channel\":%u,"
        "\"uwb_radio_rf_channel_bit\":%u,"
        "\"uwb_radio_preamble_len_code\":%u,"
        "\"uwb_radio_preamble_code\":%u,"
        "\"uwb_radio_pac\":%u,"
        "\"uwb_radio_data_rate\":%u,"
        "\"uwb_radio_phr_mode\":%u,"
        "\"uwb_radio_phr_rate\":%u,"
        "\"uwb_radio_sfd_type\":%u,"
        "\"uwb_radio_tx_pg_delay\":\"0x%02x\","
        "\"uwb_radio_tx_power\":\"0x%08lx\","
        "\"uwb_radio_rf_tx_ctrl_2\":\"0x%08lx\","
        "\"uwb_radio_pll_cfg_final\":\"0x%04lx\","
        "\"uwb_sts_mode\":%u,"
        "\"uwb_sts_length_symbols\":%u,"
        "\"uwb_diagnostics_enabled\":%s,"
        "\"uwb_diagnostics_log_every\":%u,"
        "\"uwb_event_counters_enabled\":%s,"
        "\"uwb_event_counters_log_every\":%u,"
        "\"uwb_device_id\":\"0x%08lx\","
        "\"uwb_source_id\":%u,"
        "\"uwb_active_antenna_delay\":%u,"
        "\"uwb_active_antenna_delay_hex\":\"0x%04x\","
        "\"uwb_configured_antenna_delay\":%u,"
        "\"uwb_configured_antenna_delay_hex\":\"0x%04x\","
        "\"uwb_antenna_delay_from_nvs\":%s,"
        "\"uwb_antenna_delay_reboot_required\":%s,"
        "\"uwb_tx_count\":%lu,"
        "\"uwb_tx_error_count\":%lu,"
        "\"uwb_rx_count\":%lu,"
        "\"uwb_rx_error_count\":%lu,"
        "\"uwb_rx_ignored_count\":%lu,"
        "\"uwb_last_rx_source_id\":%u,"
        "\"uwb_last_rx_sequence\":%lu,"
        "\"wireless_log_status\":\"%s\","
        "\"wireless_log_connected\":%s,"
        "\"wireless_log_target\":\"%s\","
        "\"wireless_log_port\":%u,"
        "\"wireless_log_dropped\":%lu,"
        "\"wireless_telemetry_status\":\"%s\","
        "\"wireless_telemetry_connected\":%s,"
        "\"wireless_telemetry_target\":\"%s\","
        "\"wireless_telemetry_port\":%u,"
        "\"wireless_telemetry_dropped\":%lu,"
        "\"wireless_telemetry_drop_full\":%lu,"
        "\"wireless_telemetry_drop_mutex\":%lu,"
        "\"wireless_telemetry_drop_format\":%lu,"
        "\"wireless_telemetry_queue_high_water\":%lu,"
        "\"wireless_telemetry_binary_frames\":%lu,"
        "\"wireless_telemetry_binary_samples\":%lu,"
        "\"wireless_telemetry_text_frames\":%lu,"
        "\"wireless_telemetry_last_error\":%d"
        "}\n",
        app->project_name, app->version, app->idf_ver,
        app_identity_get_hostname(), (unsigned)app_identity_get_module_id(),
        app_identity_module_id_from_nvs() ? "true" : "false",
        app_identity_module_id_provisioned_this_boot() ? "true" : "false",
        (unsigned)app_identity_get_uwb_role(),
        app_identity_uwb_role_to_string(app_identity_get_uwb_role()),
        app_identity_uwb_role_from_nvs() ? "true" : "false",
        app_identity_uwb_role_provisioned_this_boot() ? "true" : "false",
        ota_status_to_string(s_status),
        wifi_service_is_connected() ? "true" : "false",
        wifi_service_get_ip_address(),
        (unsigned long)wifi_service_get_disconnect_count(),
        (unsigned)wifi_service_get_last_disconnect_reason(),
        wifi_service_get_last_disconnect_reason_name(),
        wifi_service_get_connected_bssid(),
        (unsigned)wifi_service_get_connected_channel(),
        wifi_service_get_connected_rssi(),
        (unsigned)wifi_service_get_last_scan_ap_count(),
        wifi_service_get_last_scan_best_bssid(),
        (unsigned)wifi_service_get_last_scan_best_channel(),
        wifi_service_get_last_scan_best_rssi(),
        partition_label_or_unknown(running),
        partition_label_or_unknown(boot), partition_label_or_unknown(next),
        s_ota_in_progress ? "true" : "false",
        ota_token_configured() ? "true" : "false",
        runtime_config->from_nvs ? "true" : "false",
        (unsigned)runtime_config->runtime_mode,
        app_runtime_config_runtime_mode_to_string(
            runtime_config->runtime_mode),
        (unsigned)runtime_config->tag_id,
        (unsigned)runtime_config->anchor_count,
        (unsigned)runtime_config->anchor_ids[0],
        (unsigned)runtime_config->anchor_ids[1],
        (unsigned)runtime_config->anchor_ids[2],
        (unsigned)runtime_config->anchor_ids[3],
        (unsigned)runtime_config->anchor_survey_coordinator_id,
        (unsigned long)runtime_config->anchor_survey_rx_slice_ms,
        (unsigned long)runtime_config->anchor_survey_command_delay_ms,
        (unsigned long)runtime_config->anchor_survey_slot_ms,
        (unsigned long)runtime_config->anchor_survey_round_gap_ms,
        (unsigned long)runtime_config->anchor_survey_passive_tag_log_every,
        (unsigned long)runtime_config->ranging_slot_ms,
        (unsigned long)runtime_config->ranging_round_gap_ms,
        (unsigned long)runtime_config->ranging_rx_slice_ms,
        (unsigned)runtime_config->distance_test_peer_id,
        (unsigned)runtime_config->distance_test_initiator_id,
        (unsigned)runtime_config->distance_test_responder_id,
        (unsigned long)runtime_config->distance_test_interval_ms,
        (unsigned long)runtime_config->distance_test_rx_timeout_ms,
        (unsigned long)runtime_config->distance_test_resp_delay_ms,
        (unsigned long)runtime_config->distance_test_final_delay_ms,
        (unsigned long)runtime_config->distance_test_report_delay_ms,
        (unsigned long)runtime_config->distance_test_auto_rx_delay_uus,
        (unsigned)runtime_config->calibration_method,
        (unsigned)runtime_config->calibration_reference_id,
        (unsigned)runtime_config->calibration_dut_id,
        (unsigned)runtime_config->calibration_three_ids[0],
        (unsigned)runtime_config->calibration_three_ids[1],
        (unsigned)runtime_config->calibration_three_ids[2],
        (unsigned long)runtime_config->calibration_known_distance_mm,
        (unsigned long)runtime_config->calibration_three_distance_0_1_mm,
        (unsigned long)runtime_config->calibration_three_distance_0_2_mm,
        (unsigned long)runtime_config->calibration_three_distance_1_2_mm,
        (unsigned long)runtime_config->calibration_sample_count,
        (unsigned long)runtime_config->calibration_summary_every,
        (unsigned long)runtime_config->calibration_min_interval_ms,
        (unsigned long)runtime_config->calibration_max_interval_ms,
        (unsigned long)runtime_config->calibration_rx_slice_ms,
        (unsigned long)runtime_config->calibration_slot_guard_us,
        runtime_config->uwb_enabled ? "true" : "false",
        runtime_config->bno085_accel_enabled ? "true" : "false",
        (unsigned long)runtime_config->bno085_accel_interval_ms,
        (unsigned long)runtime_config->bno085_log_interval_ms,
        runtime_config->gps_enabled ? "true" : "false",
        gps_snapshot.powered ? "true" : "false",
        gps_snapshot.task_running ? "true" : "false",
        gps_snapshot.uart_ready ? "true" : "false",
        gps_snapshot.last_error,
        esp_err_to_name((esp_err_t)gps_snapshot.last_error),
        gps_snapshot.fix_valid ? "true" : "false",
        gps_snapshot.fix_quality,
        gps_service_fix_quality_to_string(gps_snapshot.fix_quality),
        (unsigned)gps_snapshot.fix_type,
        (unsigned)gps_snapshot.satellites,
        (unsigned)gps_snapshot.satellites_in_view,
        gps_snapshot.hdop,
        gps_snapshot.latitude_deg,
        gps_snapshot.longitude_deg,
        gps_snapshot.altitude_m,
        gps_snapshot.speed_mps,
        gps_snapshot.course_deg,
        gps_snapshot.rmc_status != '\0' ? gps_snapshot.rmc_status : '-',
        gps_snapshot.rmc_mode != '\0' ? gps_snapshot.rmc_mode : '-',
        gps_snapshot.utc_time,
        gps_snapshot.utc_date,
        gps_snapshot.last_sentence_id,
        (unsigned long)gps_snapshot.last_rx_age_ms,
        (unsigned long)gps_snapshot.last_fix_age_ms,
        (unsigned long)gps_snapshot.byte_count,
        (unsigned long)gps_snapshot.sentence_count,
        (unsigned long)gps_snapshot.gga_count,
        (unsigned long)gps_snapshot.rmc_count,
        (unsigned long)gps_snapshot.gsa_count,
        (unsigned long)gps_snapshot.gsv_count,
        (unsigned long)gps_snapshot.psti030_count,
        gps_snapshot.rtk_age_s,
        gps_snapshot.rtk_ratio,
        (unsigned long)gps_snapshot.checksum_error_count,
        (unsigned long)gps_snapshot.parse_error_count,
        charger_snapshot.monitor_enabled ? "true" : "false",
        charger_snapshot.present ? "true" : "false",
        charger_snapshot.read_ok ? "true" : "false",
        charger_snapshot.raw_valid ? "true" : "false",
        charger_snapshot.config_write_supported ? "true" : "false",
        charger_snapshot.config_writes_enabled ? "true" : "false",
        charger_snapshot.last_error,
        esp_err_to_name((esp_err_t)charger_snapshot.last_error),
        (unsigned long)charger_snapshot.read_count,
        (unsigned long)charger_snapshot.full_read_count,
        (unsigned long)charger_snapshot.quick_read_count,
        (unsigned long)charger_snapshot.error_count,
        (unsigned long)charger_snapshot.last_read_duration_ms,
        charger_snapshot.last_read_full ? "true" : "false",
        (unsigned long)charger_snapshot.last_update_age_ms,
        charger_snapshot.int_gpio_level,
        (unsigned long)charger_snapshot.int_irq_count,
        (unsigned long)charger_snapshot.int_last_irq_age_ms,
        charger_snapshot.pg_gpio_level,
        charger_snapshot.pg_asserted ? "true" : "false",
        charger_snapshot.pg_stat ? "true" : "false",
        charger_snapshot.qon_gpio_level,
        charger_snapshot.qon_asserted ? "true" : "false",
        (unsigned)charger_snapshot.part_info,
        (unsigned)charger_snapshot.part_number,
        (unsigned)charger_snapshot.device_revision,
        charger_snapshot.adc_enabled ? "true" : "false",
        (unsigned)charger_snapshot.reg0d_iotg_regulation,
        (unsigned)charger_snapshot.reg0e_timer_control,
        (unsigned)charger_snapshot.reg16_temperature_control,
        (unsigned)charger_snapshot.reg17_ntc_control_0,
        (unsigned)charger_snapshot.reg18_ntc_control_1,
        (unsigned)charger_snapshot.reg0f_charger_control_0,
        (unsigned)charger_snapshot.reg10_charger_control_1,
        (unsigned)charger_snapshot.reg14_charger_control_5,
        (unsigned)charger_snapshot.reg2e_adc_control,
        (unsigned)charger_snapshot.reg2f_adc_disable_0,
        (unsigned)charger_snapshot.reg30_adc_disable_1,
        (unsigned)charger_snapshot.minimal_system_voltage_mv,
        (unsigned)charger_snapshot.charge_voltage_limit_mv,
        (unsigned)charger_snapshot.charge_current_limit_ma,
        (unsigned)charger_snapshot.input_voltage_limit_mv,
        (unsigned)charger_snapshot.input_current_limit_ma,
        charger_snapshot.external_input_current_limit_enabled ? "true"
                                                              : "false",
        charger_snapshot.charge_enabled ? "true" : "false",
        (unsigned)charger_snapshot.charge_status_code,
        (unsigned)charger_snapshot.vbus_status_code,
        charger_snapshot.iindpm_active ? "true" : "false",
        charger_snapshot.vindpm_active ? "true" : "false",
        charger_snapshot.vsys_regulation_active ? "true" : "false",
        charger_snapshot.battery_overvoltage_active ? "true" : "false",
        charger_snapshot.charge_safety_timer_expired ? "true" : "false",
        charger_snapshot.topoff_timer_flag ? "true" : "false",
        charger_snapshot.trickle_timer_flag ? "true" : "false",
        charger_snapshot.precharge_timer_flag ? "true" : "false",
        charger_snapshot.fast_charge_timer_flag ? "true" : "false",
        (unsigned)charger_snapshot.watchdog_setting,
        charger_snapshot.watchdog_disabled ? "true" : "false",
        (unsigned)charger_snapshot.topoff_timer_minutes,
        charger_snapshot.trickle_timer_enabled ? "true" : "false",
        charger_snapshot.precharge_timer_enabled ? "true" : "false",
        charger_snapshot.fast_charge_timer_enabled ? "true" : "false",
        (unsigned)charger_snapshot.fast_charge_timer_hours,
        charger_snapshot.timer_2x_enabled ? "true" : "false",
        (unsigned)charger_snapshot.precharge_timer_minutes,
        (unsigned)charger_snapshot.adc_sample,
        charger_snapshot.adc_continuous ? "true" : "false",
        charger_snapshot.adc_running_average ? "true" : "false",
        charger_snapshot.ibat_discharge_sense_enabled ? "true" : "false",
        (unsigned)charger_snapshot.charger_status[0],
        (unsigned)charger_snapshot.charger_status[1],
        (unsigned)charger_snapshot.charger_status[2],
        (unsigned)charger_snapshot.charger_status[3],
        (unsigned)charger_snapshot.charger_status[4],
        (unsigned)charger_snapshot.fault_status[0],
        (unsigned)charger_snapshot.fault_status[1],
        (unsigned)charger_snapshot.charger_flag[0],
        (unsigned)charger_snapshot.charger_flag[1],
        (unsigned)charger_snapshot.charger_flag[2],
        (unsigned)charger_snapshot.charger_flag[3],
        (unsigned)charger_snapshot.fault_flag[0],
        (unsigned)charger_snapshot.fault_flag[1],
        (int)charger_snapshot.ibus_ma,
        (int)charger_snapshot.ibat_ma,
        (unsigned)charger_snapshot.vbus_mv,
        (unsigned)charger_snapshot.vac1_mv,
        (unsigned)charger_snapshot.vac2_mv,
        (unsigned)charger_snapshot.vbat_mv,
        (unsigned)charger_snapshot.vsys_mv,
        charger_snapshot.battery_soc_valid ? "true" : "false",
        (unsigned)charger_snapshot.battery_soc_percent,
        charger_snapshot.ts_percent,
        charger_snapshot.ts_ignore ? "true" : "false",
        charger_snapshot.ts_cold_active ? "true" : "false",
        charger_snapshot.ts_cool_active ? "true" : "false",
        charger_snapshot.ts_warm_active ? "true" : "false",
        charger_snapshot.ts_hot_active ? "true" : "false",
        charger_snapshot.ts_cold_flag ? "true" : "false",
        charger_snapshot.ts_cool_flag ? "true" : "false",
        charger_snapshot.ts_warm_flag ? "true" : "false",
        charger_snapshot.ts_hot_flag ? "true" : "false",
        charger_snapshot.tdie_c,
        (unsigned)charger_snapshot.dp_mv,
        (unsigned)charger_snapshot.dm_mv,
        (unsigned long)charger_snapshot.write_count,
        (unsigned long)charger_snapshot.write_error_count,
        (unsigned long)charger_snapshot.last_write_age_ms,
        (unsigned)charger_snapshot.last_write_reg,
        (unsigned)charger_snapshot.last_write_requested_value,
        (unsigned)charger_snapshot.last_write_mask,
        (unsigned)charger_snapshot.last_write_before,
        (unsigned)charger_snapshot.last_write_after,
        charger_snapshot.last_write_mask_used ? "true" : "false",
        charger_snapshot.last_write_changed ? "true" : "false",
        charger_snapshot.last_write_error,
        esp_err_to_name((esp_err_t)charger_snapshot.last_write_error),
        charger_raw_hex,
        (unsigned)runtime_radio_channel(runtime_config),
        (unsigned long)runtime_config->wireless_telemetry_port,
        uwb_dw3000_status_to_string(uwb_dw3000_get_status()),
        (unsigned)runtime_radio_profile(runtime_config),
        (unsigned)runtime_radio_channel(runtime_config),
        (unsigned)runtime_radio_rf_channel_bit(runtime_config),
        (unsigned)APP_UWB_RADIO_PREAMBLE_LEN_CODE,
        (unsigned)APP_UWB_RADIO_PREAMBLE_CODE,
        (unsigned)APP_UWB_RADIO_PAC,
        (unsigned)APP_UWB_RADIO_DATA_RATE,
        (unsigned)APP_UWB_RADIO_PHR_MODE,
        (unsigned)APP_UWB_RADIO_PHR_RATE,
        (unsigned)APP_UWB_RADIO_SFD_TYPE,
        (unsigned)APP_UWB_RADIO_TX_PG_DELAY,
        (unsigned long)APP_UWB_RADIO_TX_POWER,
        (unsigned long)runtime_radio_rf_tx_ctrl_2(runtime_config),
        (unsigned long)runtime_radio_pll_cfg_final(runtime_config),
        (unsigned)APP_UWB_STS_MODE,
        (unsigned)APP_UWB_STS_LENGTH_SYMBOLS,
        APP_UWB_DIAGNOSTICS_ENABLED ? "true" : "false",
        (unsigned)APP_UWB_DIAGNOSTICS_LOG_EVERY,
        APP_UWB_EVENT_COUNTERS_ENABLED ? "true" : "false",
        (unsigned)APP_UWB_EVENT_COUNTERS_LOG_EVERY,
        (unsigned long)uwb_dw3000_get_device_id(),
        (unsigned)uwb_dw3000_get_source_id(),
        (unsigned)active_antenna_delay, (unsigned)active_antenna_delay,
        (unsigned)configured_antenna_delay,
        (unsigned)configured_antenna_delay,
        app_identity_uwb_antenna_delay_from_nvs() ? "true" : "false",
        active_antenna_delay != configured_antenna_delay ? "true" : "false",
        (unsigned long)uwb_dw3000_get_tx_count(),
        (unsigned long)uwb_dw3000_get_tx_error_count(),
        (unsigned long)uwb_dw3000_get_rx_count(),
        (unsigned long)uwb_dw3000_get_rx_error_count(),
        (unsigned long)uwb_dw3000_get_rx_ignored_count(),
        (unsigned)uwb_dw3000_get_last_rx_source_id(),
        (unsigned long)uwb_dw3000_get_last_rx_sequence(),
        wireless_log_service_status_to_string(
            wireless_log_service_get_status()),
        wireless_log_service_is_connected() ? "true" : "false",
        wireless_log_service_get_target(),
        (unsigned)wireless_log_service_get_port(),
        (unsigned long)wireless_log_service_get_dropped_count(),
        wireless_telemetry_service_status_to_string(
            wireless_telemetry_service_get_status()),
        wireless_telemetry_service_is_connected() ? "true" : "false",
        wireless_telemetry_service_get_target(),
        (unsigned)wireless_telemetry_service_get_port(),
        (unsigned long)wireless_telemetry_service_get_dropped_count(),
        (unsigned long)wireless_telemetry_service_get_drop_full_count(),
        (unsigned long)wireless_telemetry_service_get_drop_mutex_count(),
        (unsigned long)wireless_telemetry_service_get_drop_format_count(),
        (unsigned long)wireless_telemetry_service_get_queue_high_water(),
        (unsigned long)wireless_telemetry_service_get_binary_frame_count(),
        (unsigned long)wireless_telemetry_service_get_binary_sample_count(),
        (unsigned long)wireless_telemetry_service_get_text_frame_count(),
        wireless_telemetry_service_get_last_error());

    if (len < 0 || len >= OTA_SERVICE_STATUS_RESPONSE_SIZE) {
        free(response);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "status too long");
    }

    httpd_resp_set_type(req, "application/json");
    const esp_err_t response_err = httpd_resp_send(req, response, (ssize_t)len);
    free(response);
    return response_err;
}

static esp_err_t antenna_delay_post_handler(httpd_req_t *req)
{
    if (!ota_request_authorized(req)) {
        ESP_LOGW(TAG,
                 "Rejected antenna delay config: missing or invalid token");
        return httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED,
                                   "Missing or invalid OTA token");
    }

    if (s_ota_in_progress) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "OTA already in progress");
    }

    const size_t query_len = httpd_req_get_url_query_len(req);
    if (query_len == 0 || query_len >= OTA_SERVICE_MAX_QUERY_LEN) {
        return httpd_resp_send_err(
            req, HTTPD_400_BAD_REQUEST,
            "Use ?value=0x4018 or ?clear=1");
    }

    char query[OTA_SERVICE_MAX_QUERY_LEN] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid query string");
    }

    const bool clear_requested = ota_query_option_enabled(query, "clear");
    const bool reboot_requested = ota_query_option_enabled(query, "reboot");
    esp_err_t err = ESP_OK;

    if (clear_requested) {
        err = app_identity_clear_uwb_antenna_delay();
    } else {
        char value_text[24] = {0};
        if (httpd_query_key_value(query, "value", value_text,
                                  sizeof(value_text)) != ESP_OK) {
            return httpd_resp_send_err(
                req, HTTPD_400_BAD_REQUEST,
                "Missing antenna delay value; use ?value=0x4018");
        }

        uint16_t delay = 0;
        if (!ota_parse_u16(value_text, &delay)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid antenna delay value");
        }

        err = app_identity_set_uwb_antenna_delay(delay);
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Antenna delay config failed: %s",
                 esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Antenna delay config failed");
    }

    const uint16_t active_delay = uwb_dw3000_get_antenna_delay();
    const uint16_t configured_delay = app_identity_get_uwb_antenna_delay();
    const bool reboot_required = active_delay != configured_delay;
    ESP_LOGW(TAG,
             "Antenna delay config: active=0x%04x configured=0x%04x source=%s reboot_required=%s reboot_requested=%s",
             (unsigned)active_delay, (unsigned)configured_delay,
             app_identity_uwb_antenna_delay_from_nvs() ? "nvs" : "fallback",
             reboot_required ? "true" : "false",
             reboot_requested ? "true" : "false");

    char response[420];
    const int len = snprintf(
        response, sizeof(response),
        "{"
        "\"ok\":true,"
        "\"uwb_active_antenna_delay\":%u,"
        "\"uwb_active_antenna_delay_hex\":\"0x%04x\","
        "\"uwb_configured_antenna_delay\":%u,"
        "\"uwb_configured_antenna_delay_hex\":\"0x%04x\","
        "\"uwb_antenna_delay_from_nvs\":%s,"
        "\"reboot_required\":%s,"
        "\"rebooting\":%s"
        "}\n",
        (unsigned)active_delay, (unsigned)active_delay,
        (unsigned)configured_delay, (unsigned)configured_delay,
        app_identity_uwb_antenna_delay_from_nvs() ? "true" : "false",
        reboot_required ? "true" : "false",
        reboot_requested ? "true" : "false");

    if (len < 0 || len >= (int)sizeof(response)) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "response too long");
    }

    httpd_resp_set_type(req, "application/json");
    const esp_err_t response_err = httpd_resp_send(req, response, len);

    if (reboot_requested) {
        xTaskCreate(reboot_task, "cfg_reboot",
                    OTA_SERVICE_RESTART_TASK_STACK_WORDS, NULL,
                    OTA_SERVICE_RESTART_TASK_PRIORITY, NULL);
    }

    return response_err;
}

static esp_err_t charger_config_apply_u16(
    const char *query, const char *key, esp_err_t (*setter)(
                                        uint16_t,
                                        charger_service_write_result_t *,
                                        size_t, size_t *),
    bool *handled, uint32_t *operation_count, esp_err_t *first_error)
{
    char value_text[32] = {0};
    const esp_err_t query_err =
        httpd_query_key_value(query, key, value_text, sizeof(value_text));
    if (query_err == ESP_ERR_NOT_FOUND) {
        return ESP_OK;
    }
    if (query_err != ESP_OK) {
        return query_err;
    }

    uint16_t value = 0;
    if (!ota_parse_u16(value_text, &value)) {
        return ESP_ERR_INVALID_ARG;
    }

    charger_service_write_result_t results[4] = {0};
    size_t written_count = 0;
    const esp_err_t err = setter(value, results, 4, &written_count);
    *handled = true;
    *operation_count += (uint32_t)written_count;
    if (err != ESP_OK && *first_error == ESP_OK) {
        *first_error = err;
    }
    return ESP_OK;
}

static esp_err_t charger_config_post_handler(httpd_req_t *req)
{
    if (!ota_request_authorized(req)) {
        ESP_LOGW(TAG, "Rejected charger config: missing or invalid token");
        return httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED,
                                   "Missing or invalid OTA token");
    }

    if (s_ota_in_progress) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "OTA already in progress");
    }

    const size_t query_len = httpd_req_get_url_query_len(req);
    if (query_len == 0 || query_len >= OTA_SERVICE_MAX_QUERY_LEN) {
        return httpd_resp_send_err(
            req, HTTPD_400_BAD_REQUEST,
            "Use charger config query parameters");
    }

    char query[OTA_SERVICE_MAX_QUERY_LEN] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid query string");
    }

    bool handled = false;
    uint32_t operation_count = 0;
    esp_err_t first_error = ESP_OK;
    esp_err_t query_err = ESP_OK;

    if (ota_query_option_enabled(query, "refresh")) {
        charger_service_request_refresh();
        handled = true;
    }

    if (ota_query_option_enabled(query, "disable_watchdog") ||
        ota_query_option_enabled(query, "watchdog_disable")) {
        charger_service_write_result_t result = {0};
        const esp_err_t err = charger_service_set_watchdog_disabled(&result);
        operation_count++;
        handled = true;
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    }

    char charge_enabled_text[16] = {0};
    query_err = httpd_query_key_value(query, "charge_enabled",
                                      charge_enabled_text,
                                      sizeof(charge_enabled_text));
    if (query_err == ESP_ERR_NOT_FOUND) {
        query_err = httpd_query_key_value(query, "charging",
                                          charge_enabled_text,
                                          sizeof(charge_enabled_text));
    }
    if (query_err == ESP_ERR_NOT_FOUND) {
        query_err = httpd_query_key_value(query, "en_chg", charge_enabled_text,
                                          sizeof(charge_enabled_text));
    }
    if (query_err == ESP_OK) {
        bool charge_enabled = false;
        if (!ota_parse_bool_text(charge_enabled_text, &charge_enabled)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid charge_enabled value");
        }
        charger_service_write_result_t result = {0};
        const esp_err_t err =
            charger_service_set_charge_enabled(charge_enabled, &result);
        operation_count += 2U;
        handled = true;
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    } else if (query_err != ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid charge_enabled value");
    }

    char adc_text[16] = {0};
    query_err = httpd_query_key_value(query, "adc", adc_text, sizeof(adc_text));
    if (query_err == ESP_OK) {
        bool adc_enabled = false;
        if (!ota_parse_bool_text(adc_text, &adc_enabled)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid adc value");
        }

        uint8_t adc_sample = 2;
        char sample_text[16] = {0};
        query_err = httpd_query_key_value(query, "adc_sample", sample_text,
                                          sizeof(sample_text));
        if (query_err == ESP_OK) {
            if (!ota_parse_u8(sample_text, &adc_sample) || adc_sample > 3U) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid adc_sample");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid adc_sample");
        }

        bool continuous = true;
        char rate_text[16] = {0};
        query_err = httpd_query_key_value(query, "adc_rate", rate_text,
                                          sizeof(rate_text));
        if (query_err == ESP_OK) {
            if (strcmp(rate_text, "continuous") == 0 ||
                strcmp(rate_text, "cont") == 0 || strcmp(rate_text, "0") == 0) {
                continuous = true;
            } else if (strcmp(rate_text, "oneshot") == 0 ||
                       strcmp(rate_text, "one_shot") == 0 ||
                       strcmp(rate_text, "1") == 0) {
                continuous = false;
            } else {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid adc_rate");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid adc_rate");
        }

        bool running_average = false;
        char avg_text[16] = {0};
        query_err = httpd_query_key_value(query, "adc_avg", avg_text,
                                          sizeof(avg_text));
        if (query_err == ESP_OK) {
            if (!ota_parse_bool_text(avg_text, &running_average)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid adc_avg");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid adc_avg");
        }

        charger_service_write_result_t results[6] = {0};
        size_t written_count = 0;
        const esp_err_t err =
            charger_service_set_adc(adc_enabled, continuous, adc_sample,
                                    running_average, results, 6,
                                    &written_count);
        operation_count += (uint32_t)written_count;
        handled = true;
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    } else if (query_err != ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid adc value");
    }

    esp_err_t apply_err = charger_config_apply_u16(
        query, "minimal_system_voltage_mv",
        charger_service_set_minimal_system_voltage_mv, &handled,
        &operation_count, &first_error);
    if (apply_err != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid minimal_system_voltage_mv");
    }

    apply_err = charger_config_apply_u16(
        query, "charge_voltage_mv",
        charger_service_set_charge_voltage_limit_mv, &handled,
        &operation_count, &first_error);
    if (apply_err != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid charge_voltage_mv");
    }

    apply_err = charger_config_apply_u16(
        query, "charge_current_ma",
        charger_service_set_charge_current_limit_ma, &handled,
        &operation_count, &first_error);
    if (apply_err != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid charge_current_ma");
    }

    char input_voltage_text[32] = {0};
    query_err = httpd_query_key_value(query, "input_voltage_mv",
                                      input_voltage_text,
                                      sizeof(input_voltage_text));
    if (query_err == ESP_OK) {
        uint16_t mv = 0;
        if (!ota_parse_u16(input_voltage_text, &mv)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid input_voltage_mv");
        }
        charger_service_write_result_t result = {0};
        const esp_err_t err =
            charger_service_set_input_voltage_limit_mv(mv, &result);
        operation_count += 2U;
        handled = true;
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    } else if (query_err != ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid input_voltage_mv");
    }

    char ext_ilim_text[16] = {0};
    query_err = httpd_query_key_value(
        query, "external_input_current_limit_enabled", ext_ilim_text,
        sizeof(ext_ilim_text));
    if (query_err == ESP_ERR_NOT_FOUND) {
        query_err = httpd_query_key_value(query, "external_ilim_enabled",
                                          ext_ilim_text,
                                          sizeof(ext_ilim_text));
    }
    if (query_err == ESP_ERR_NOT_FOUND) {
        query_err = httpd_query_key_value(query, "en_extilim", ext_ilim_text,
                                          sizeof(ext_ilim_text));
    }
    if (query_err == ESP_OK) {
        bool enabled = false;
        if (!ota_parse_bool_text(ext_ilim_text, &enabled)) {
            return httpd_resp_send_err(
                req, HTTPD_400_BAD_REQUEST,
                "Invalid external_input_current_limit_enabled");
        }
        charger_service_write_result_t result = {0};
        const esp_err_t err =
            charger_service_set_external_input_current_limit_enabled(enabled,
                                                                     &result);
        operation_count += 2U;
        handled = true;
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    } else if (query_err != ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(
            req, HTTPD_400_BAD_REQUEST,
            "Invalid external_input_current_limit_enabled");
    }

    apply_err = charger_config_apply_u16(
        query, "input_current_ma",
        charger_service_set_input_current_limit_ma, &handled,
        &operation_count, &first_error);
    if (apply_err != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid input_current_ma");
    }

    apply_err = charger_config_apply_u16(
        query, "iindpm_ma",
        charger_service_set_input_current_limit_ma, &handled,
        &operation_count, &first_error);
    if (apply_err != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid iindpm_ma");
    }

    const bool timer_requested =
        ota_query_has_key(query, "topoff_timer_minutes") ||
        ota_query_has_key(query, "trickle_timer_enabled") ||
        ota_query_has_key(query, "precharge_timer_enabled") ||
        ota_query_has_key(query, "fast_charge_timer_enabled") ||
        ota_query_has_key(query, "fast_charge_timer_hours") ||
        ota_query_has_key(query, "timer_2x_enabled") ||
        ota_query_has_key(query, "precharge_timer_minutes");
    if (timer_requested) {
        charger_service_snapshot_t current = {0};
        charger_service_get_snapshot(&current);

        uint16_t topoff_timer_minutes = current.topoff_timer_minutes;
        bool trickle_timer_enabled = current.trickle_timer_enabled;
        bool precharge_timer_enabled = current.precharge_timer_enabled;
        bool fast_charge_timer_enabled = current.fast_charge_timer_enabled;
        uint8_t fast_charge_timer_hours = current.fast_charge_timer_hours;
        bool timer_2x_enabled = current.timer_2x_enabled;
        uint16_t precharge_timer_minutes = current.precharge_timer_minutes;

        char text[32] = {0};
        query_err = httpd_query_key_value(query, "topoff_timer_minutes", text,
                                          sizeof(text));
        if (query_err == ESP_OK) {
            if (!ota_parse_u16(text, &topoff_timer_minutes)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid topoff_timer_minutes");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid topoff_timer_minutes");
        }

        query_err = httpd_query_key_value(query, "trickle_timer_enabled", text,
                                          sizeof(text));
        if (query_err == ESP_OK) {
            if (!ota_parse_bool_text(text, &trickle_timer_enabled)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid trickle_timer_enabled");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid trickle_timer_enabled");
        }

        query_err = httpd_query_key_value(query, "precharge_timer_enabled",
                                          text, sizeof(text));
        if (query_err == ESP_OK) {
            if (!ota_parse_bool_text(text, &precharge_timer_enabled)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid precharge_timer_enabled");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid precharge_timer_enabled");
        }

        query_err = httpd_query_key_value(query, "fast_charge_timer_enabled",
                                          text, sizeof(text));
        if (query_err == ESP_OK) {
            if (!ota_parse_bool_text(text, &fast_charge_timer_enabled)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid fast_charge_timer_enabled");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid fast_charge_timer_enabled");
        }

        query_err = httpd_query_key_value(query, "fast_charge_timer_hours",
                                          text, sizeof(text));
        if (query_err == ESP_OK) {
            if (!ota_parse_u8(text, &fast_charge_timer_hours)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid fast_charge_timer_hours");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid fast_charge_timer_hours");
        }

        query_err = httpd_query_key_value(query, "timer_2x_enabled", text,
                                          sizeof(text));
        if (query_err == ESP_OK) {
            if (!ota_parse_bool_text(text, &timer_2x_enabled)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid timer_2x_enabled");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid timer_2x_enabled");
        }

        query_err = httpd_query_key_value(query, "precharge_timer_minutes",
                                          text, sizeof(text));
        if (query_err == ESP_OK) {
            if (!ota_parse_u16(text, &precharge_timer_minutes)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid precharge_timer_minutes");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid precharge_timer_minutes");
        }

        charger_service_write_result_t results[4] = {0};
        size_t written_count = 0;
        const esp_err_t err = charger_service_set_safety_timers(
            topoff_timer_minutes, trickle_timer_enabled,
            precharge_timer_enabled, fast_charge_timer_enabled,
            fast_charge_timer_hours, timer_2x_enabled, precharge_timer_minutes,
            results, 4, &written_count);
        operation_count += (uint32_t)written_count;
        handled = true;
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    }

    char reg_text[16] = {0};
    query_err = httpd_query_key_value(query, "reg", reg_text,
                                      sizeof(reg_text));
    if (query_err == ESP_OK) {
        if (!ota_query_option_enabled(query, "confirm")) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Raw register write needs confirm=1");
        }

        uint8_t reg = 0;
        if (!ota_parse_u8(reg_text, &reg) ||
            reg >= CHARGER_SERVICE_REGISTER_MAP_SIZE) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid reg");
        }

        charger_service_write_result_t result = {0};
        const esp_err_t watchdog_err =
            charger_service_set_watchdog_disabled(&result);
        operation_count++;
        if (watchdog_err != ESP_OK && first_error == ESP_OK) {
            first_error = watchdog_err;
        }

        char mask_text[16] = {0};
        char value_text[16] = {0};
        const esp_err_t mask_err =
            httpd_query_key_value(query, "mask", mask_text, sizeof(mask_text));
        if (mask_err == ESP_OK) {
            char bits_text[16] = {0};
            if (httpd_query_key_value(query, "bits", bits_text,
                                      sizeof(bits_text)) != ESP_OK &&
                httpd_query_key_value(query, "value", bits_text,
                                      sizeof(bits_text)) != ESP_OK) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Missing bits/value");
            }
            uint8_t mask = 0;
            uint8_t bits = 0;
            if (!ota_parse_u8(mask_text, &mask) ||
                !ota_parse_u8(bits_text, &bits)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid mask/bits");
            }
            const esp_err_t err =
                charger_service_update_register_bits(reg, mask, bits, &result);
            operation_count++;
            if (err != ESP_OK && first_error == ESP_OK) {
                first_error = err;
            }
        } else if (mask_err == ESP_ERR_NOT_FOUND) {
            if (httpd_query_key_value(query, "value", value_text,
                                      sizeof(value_text)) != ESP_OK) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Missing raw value");
            }
            uint8_t value = 0;
            if (!ota_parse_u8(value_text, &value)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid value");
            }
            const esp_err_t err =
                charger_service_write_register(reg, value, &result);
            operation_count++;
            if (err != ESP_OK && first_error == ESP_OK) {
                first_error = err;
            }
        } else {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid mask");
        }
        handled = true;
    } else if (query_err != ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid reg");
    }

    if (!handled) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "No charger operation requested");
    }

    charger_service_request_refresh();
    vTaskDelay(pdMS_TO_TICKS(40));

    charger_service_snapshot_t snapshot = {0};
    charger_service_get_snapshot(&snapshot);

    ESP_LOGW(TAG,
             "Charger config: ops=%lu err=%s ADC=%s CHG=%s REG0F=0x%02X VREG=%umV ICHG=%umA VINDPM=%umV IINDPM=%umA EXTILIM=%s fast_tmr=%s/%uh pre_tmr=%s/%umin WD=%u",
             (unsigned long)operation_count, esp_err_to_name(first_error),
             snapshot.adc_enabled ? "on" : "off",
             snapshot.charge_enabled ? "on" : "off",
             (unsigned)snapshot.reg0f_charger_control_0,
             (unsigned)snapshot.charge_voltage_limit_mv,
             (unsigned)snapshot.charge_current_limit_ma,
             (unsigned)snapshot.input_voltage_limit_mv,
             (unsigned)snapshot.input_current_limit_ma,
             snapshot.external_input_current_limit_enabled ? "on" : "off",
             snapshot.fast_charge_timer_enabled ? "on" : "off",
             (unsigned)snapshot.fast_charge_timer_hours,
             snapshot.precharge_timer_enabled ? "on" : "off",
             (unsigned)snapshot.precharge_timer_minutes,
             (unsigned)snapshot.watchdog_setting);

    char response[2600];
    const int len = snprintf(
        response, sizeof(response),
        "{"
        "\"ok\":%s,"
        "\"operation_count\":%lu,"
        "\"error\":%d,"
        "\"error_name\":\"%s\","
        "\"charger_present\":%s,"
        "\"charger_adc_enabled\":%s,"
        "\"charger_adc_sample\":%u,"
        "\"charger_adc_continuous\":%s,"
        "\"charger_adc_running_average\":%s,"
        "\"charger_charge_enabled\":%s,"
        "\"charger_reg0f_charger_control_0\":\"0x%02x\","
        "\"charger_reg0d_iotg_regulation\":\"0x%02x\","
        "\"charger_reg0e_timer_control\":\"0x%02x\","
        "\"charger_reg16_temperature_control\":\"0x%02x\","
        "\"charger_reg17_ntc_control_0\":\"0x%02x\","
        "\"charger_reg18_ntc_control_1\":\"0x%02x\","
        "\"charger_watchdog_setting\":%u,"
        "\"charger_watchdog_disabled\":%s,"
        "\"charger_topoff_timer_minutes\":%u,"
        "\"charger_trickle_timer_enabled\":%s,"
        "\"charger_precharge_timer_enabled\":%s,"
        "\"charger_fast_charge_timer_enabled\":%s,"
        "\"charger_fast_charge_timer_hours\":%u,"
        "\"charger_timer_2x_enabled\":%s,"
        "\"charger_precharge_timer_minutes\":%u,"
        "\"charger_charge_safety_timer_expired\":%s,"
        "\"charger_topoff_timer_flag\":%s,"
        "\"charger_trickle_timer_flag\":%s,"
        "\"charger_precharge_timer_flag\":%s,"
        "\"charger_fast_charge_timer_flag\":%s,"
        "\"charger_minimal_system_voltage_mv\":%u,"
        "\"charger_charge_voltage_limit_mv\":%u,"
        "\"charger_charge_current_limit_ma\":%u,"
        "\"charger_input_voltage_limit_mv\":%u,"
        "\"charger_input_current_limit_ma\":%u,"
        "\"charger_external_input_current_limit_enabled\":%s,"
        "\"charger_vbat_mv\":%u,"
        "\"charger_vsys_mv\":%u,"
        "\"charger_vbus_mv\":%u,"
        "\"charger_battery_soc_valid\":%s,"
        "\"charger_battery_soc_percent\":%u,"
        "\"charger_ibus_ma\":%d,"
        "\"charger_ibat_ma\":%d,"
        "\"charger_ts_percent\":%.4f,"
        "\"charger_ts_ignore\":%s,"
        "\"charger_ts_cold_active\":%s,"
        "\"charger_ts_cool_active\":%s,"
        "\"charger_ts_warm_active\":%s,"
        "\"charger_ts_hot_active\":%s,"
        "\"charger_tdie_c\":%.1f,"
        "\"charger_write_count\":%lu,"
        "\"charger_write_error_count\":%lu,"
        "\"charger_last_write_reg\":\"0x%02x\","
        "\"charger_last_write_before\":\"0x%02x\","
        "\"charger_last_write_after\":\"0x%02x\","
        "\"charger_last_write_error_name\":\"%s\""
        "}\n",
        first_error == ESP_OK ? "true" : "false",
        (unsigned long)operation_count, first_error,
        esp_err_to_name(first_error),
        snapshot.present ? "true" : "false",
        snapshot.adc_enabled ? "true" : "false",
        (unsigned)snapshot.adc_sample,
        snapshot.adc_continuous ? "true" : "false",
        snapshot.adc_running_average ? "true" : "false",
        snapshot.charge_enabled ? "true" : "false",
        (unsigned)snapshot.reg0f_charger_control_0,
        (unsigned)snapshot.reg0d_iotg_regulation,
        (unsigned)snapshot.reg0e_timer_control,
        (unsigned)snapshot.reg16_temperature_control,
        (unsigned)snapshot.reg17_ntc_control_0,
        (unsigned)snapshot.reg18_ntc_control_1,
        (unsigned)snapshot.watchdog_setting,
        snapshot.watchdog_disabled ? "true" : "false",
        (unsigned)snapshot.topoff_timer_minutes,
        snapshot.trickle_timer_enabled ? "true" : "false",
        snapshot.precharge_timer_enabled ? "true" : "false",
        snapshot.fast_charge_timer_enabled ? "true" : "false",
        (unsigned)snapshot.fast_charge_timer_hours,
        snapshot.timer_2x_enabled ? "true" : "false",
        (unsigned)snapshot.precharge_timer_minutes,
        snapshot.charge_safety_timer_expired ? "true" : "false",
        snapshot.topoff_timer_flag ? "true" : "false",
        snapshot.trickle_timer_flag ? "true" : "false",
        snapshot.precharge_timer_flag ? "true" : "false",
        snapshot.fast_charge_timer_flag ? "true" : "false",
        (unsigned)snapshot.minimal_system_voltage_mv,
        (unsigned)snapshot.charge_voltage_limit_mv,
        (unsigned)snapshot.charge_current_limit_ma,
        (unsigned)snapshot.input_voltage_limit_mv,
        (unsigned)snapshot.input_current_limit_ma,
        snapshot.external_input_current_limit_enabled ? "true" : "false",
        (unsigned)snapshot.vbat_mv,
        (unsigned)snapshot.vsys_mv,
        (unsigned)snapshot.vbus_mv,
        snapshot.battery_soc_valid ? "true" : "false",
        (unsigned)snapshot.battery_soc_percent,
        (int)snapshot.ibus_ma,
        (int)snapshot.ibat_ma,
        snapshot.ts_percent,
        snapshot.ts_ignore ? "true" : "false",
        snapshot.ts_cold_active ? "true" : "false",
        snapshot.ts_cool_active ? "true" : "false",
        snapshot.ts_warm_active ? "true" : "false",
        snapshot.ts_hot_active ? "true" : "false",
        snapshot.tdie_c,
        (unsigned long)snapshot.write_count,
        (unsigned long)snapshot.write_error_count,
        (unsigned)snapshot.last_write_reg,
        (unsigned)snapshot.last_write_before,
        (unsigned)snapshot.last_write_after,
        esp_err_to_name((esp_err_t)snapshot.last_write_error));

    if (len < 0 || len >= (int)sizeof(response)) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "response too long");
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, response, len);
}

static esp_err_t runtime_config_post_handler(httpd_req_t *req)
{
    if (!ota_request_authorized(req)) {
        ESP_LOGW(TAG, "Rejected runtime config: missing or invalid token");
        return httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED,
                                   "Missing or invalid OTA token");
    }

    if (s_ota_in_progress) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "OTA already in progress");
    }

    const size_t query_len = httpd_req_get_url_query_len(req);
    if (query_len == 0 || query_len >= OTA_SERVICE_MAX_QUERY_LEN) {
        return httpd_resp_send_err(
            req, HTTPD_400_BAD_REQUEST,
            "Use runtime config query parameters");
    }

    char query[OTA_SERVICE_MAX_QUERY_LEN] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid query string");
    }

    const app_runtime_config_t before_config = *app_runtime_config_get();
    const bool clear_requested = ota_query_option_enabled(query, "clear");
    const bool reboot_requested = ota_query_option_enabled(query, "reboot");
    bool changed = false;
    bool cleared = false;
    esp_err_t err = ESP_OK;

    if (clear_requested) {
        err = app_runtime_config_clear();
        changed = true;
        cleared = true;
    } else {
        app_runtime_config_t config = before_config;

        char mode_text[40] = {0};
        esp_err_t query_err = httpd_query_key_value(
            query, "mode", mode_text, sizeof(mode_text));
        if (query_err == ESP_OK) {
            bool mode_ok = false;
            config.runtime_mode =
                app_runtime_config_runtime_mode_from_string(mode_text,
                                                            &mode_ok);
            if (!mode_ok) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid mode");
            }
            changed = true;
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid mode");
        }

        char anchors_text[80] = {0};
        query_err = httpd_query_key_value(query, "anchors", anchors_text,
                                          sizeof(anchors_text));
        if (query_err == ESP_OK) {
            uint8_t ids[APP_RUNTIME_CONFIG_MAX_ANCHORS] = {0};
            uint8_t count = 0;
            if (!ota_parse_u8_list(anchors_text, ids,
                                   APP_RUNTIME_CONFIG_MAX_ANCHORS, &count)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid anchors");
            }
            memset(config.anchor_ids, 0, sizeof(config.anchor_ids));
            memcpy(config.anchor_ids, ids, count);
            config.anchor_count = count;
            changed = true;
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid anchors");
        }

        char cal_three_text[64] = {0};
        query_err = httpd_query_key_value(query, "cal_three",
                                          cal_three_text,
                                          sizeof(cal_three_text));
        if (query_err == ESP_OK) {
            uint8_t ids[APP_RUNTIME_CONFIG_CAL_THREE_COUNT] = {0};
            uint8_t count = 0;
            if (!ota_parse_u8_list(cal_three_text, ids,
                                   APP_RUNTIME_CONFIG_CAL_THREE_COUNT,
                                   &count) ||
                count != APP_RUNTIME_CONFIG_CAL_THREE_COUNT) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid cal_three");
            }
            memcpy(config.calibration_three_ids, ids,
                   sizeof(config.calibration_three_ids));
            changed = true;
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid cal_three");
        }

        char cal_method_text[32] = {0};
        query_err = httpd_query_key_value(query, "cal_method",
                                          cal_method_text,
                                          sizeof(cal_method_text));
        if (query_err == ESP_OK) {
            if (!ota_parse_calibration_method(cal_method_text,
                                              &config.calibration_method)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid cal_method");
            }
            changed = true;
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid cal_method");
        }

#define APPLY_U8_PARAM(KEY, FIELD)                                      \
        do {                                                            \
            char value_text[32] = {0};                                  \
            esp_err_t key_err = httpd_query_key_value(                  \
                query, KEY, value_text, sizeof(value_text));            \
            if (key_err == ESP_OK) {                                    \
                uint8_t parsed = 0;                                     \
                if (!ota_parse_u8(value_text, &parsed)) {               \
                    return httpd_resp_send_err(req,                     \
                                               HTTPD_400_BAD_REQUEST,   \
                                               "Invalid " KEY);         \
                }                                                       \
                config.FIELD = parsed;                                  \
                changed = true;                                         \
            } else if (key_err != ESP_ERR_NOT_FOUND) {                  \
                return httpd_resp_send_err(req,                         \
                                           HTTPD_400_BAD_REQUEST,       \
                                           "Invalid " KEY);             \
            }                                                           \
        } while (0)

#define APPLY_U32_PARAM(KEY, FIELD)                                     \
        do {                                                            \
            char value_text[32] = {0};                                  \
            esp_err_t key_err = httpd_query_key_value(                  \
                query, KEY, value_text, sizeof(value_text));            \
            if (key_err == ESP_OK) {                                    \
                uint32_t parsed = 0;                                    \
                if (!ota_parse_u32(value_text, &parsed)) {              \
                    return httpd_resp_send_err(req,                     \
                                               HTTPD_400_BAD_REQUEST,   \
                                               "Invalid " KEY);         \
                }                                                       \
                config.FIELD = parsed;                                  \
                changed = true;                                         \
            } else if (key_err != ESP_ERR_NOT_FOUND) {                  \
                return httpd_resp_send_err(req,                         \
                                           HTTPD_400_BAD_REQUEST,       \
                                           "Invalid " KEY);             \
            }                                                           \
        } while (0)

#define APPLY_BOOL_PARAM(KEY, FIELD)                                    \
        do {                                                            \
            char value_text[8] = {0};                                   \
            esp_err_t key_err = httpd_query_key_value(                  \
                query, KEY, value_text, sizeof(value_text));            \
            if (key_err == ESP_OK) {                                    \
                if (strcmp(value_text, "1") == 0 ||                    \
                    strcmp(value_text, "true") == 0 ||                 \
                    strcmp(value_text, "on") == 0) {                   \
                    config.FIELD = true;                                \
                } else if (strcmp(value_text, "0") == 0 ||             \
                           strcmp(value_text, "false") == 0 ||         \
                           strcmp(value_text, "off") == 0) {           \
                    config.FIELD = false;                               \
                } else {                                                \
                    return httpd_resp_send_err(req,                     \
                                               HTTPD_400_BAD_REQUEST,   \
                                               "Invalid " KEY);         \
                }                                                       \
                changed = true;                                         \
            } else if (key_err != ESP_ERR_NOT_FOUND) {                  \
                return httpd_resp_send_err(req,                         \
                                           HTTPD_400_BAD_REQUEST,       \
                                           "Invalid " KEY);             \
            }                                                           \
        } while (0)

        APPLY_U8_PARAM("tag", tag_id);
        APPLY_U8_PARAM("anchor_count", anchor_count);
        APPLY_U8_PARAM("coordinator", anchor_survey_coordinator_id);
        APPLY_U8_PARAM("coord", anchor_survey_coordinator_id);
        APPLY_U32_PARAM("survey_rx_ms", anchor_survey_rx_slice_ms);
        APPLY_U32_PARAM("survey_delay_ms",
                        anchor_survey_command_delay_ms);
        APPLY_U32_PARAM("survey_slot_ms", anchor_survey_slot_ms);
        APPLY_U32_PARAM("survey_gap_ms", anchor_survey_round_gap_ms);
        APPLY_U32_PARAM("survey_log_every",
                        anchor_survey_passive_tag_log_every);
        APPLY_U32_PARAM("ranging_slot_ms", ranging_slot_ms);
        APPLY_U32_PARAM("ranging_gap_ms", ranging_round_gap_ms);
        APPLY_U32_PARAM("ranging_rx_ms", ranging_rx_slice_ms);
        APPLY_U8_PARAM("dt_peer", distance_test_peer_id);
        APPLY_U8_PARAM("dt_initiator", distance_test_initiator_id);
        APPLY_U8_PARAM("dt_responder", distance_test_responder_id);
        APPLY_U32_PARAM("dt_interval_ms", distance_test_interval_ms);
        APPLY_U32_PARAM("dt_rx_timeout_ms", distance_test_rx_timeout_ms);
        APPLY_U32_PARAM("dt_resp_delay_ms", distance_test_resp_delay_ms);
        APPLY_U32_PARAM("dt_final_delay_ms", distance_test_final_delay_ms);
        APPLY_U32_PARAM("dt_report_delay_ms", distance_test_report_delay_ms);
        APPLY_U32_PARAM("dt_auto_rx_delay_uus",
                        distance_test_auto_rx_delay_uus);
        APPLY_U8_PARAM("cal_ref", calibration_reference_id);
        APPLY_U8_PARAM("cal_dut", calibration_dut_id);
        APPLY_U32_PARAM("cal_known_mm", calibration_known_distance_mm);
        APPLY_U32_PARAM("cal_d01_mm", calibration_three_distance_0_1_mm);
        APPLY_U32_PARAM("cal_d02_mm", calibration_three_distance_0_2_mm);
        APPLY_U32_PARAM("cal_d12_mm", calibration_three_distance_1_2_mm);
        APPLY_U32_PARAM("cal_samples", calibration_sample_count);
        APPLY_U32_PARAM("cal_summary", calibration_summary_every);
        APPLY_U32_PARAM("cal_slot_ms", calibration_min_interval_ms);
        APPLY_U32_PARAM("cal_min_ms", calibration_min_interval_ms);
        APPLY_U32_PARAM("cal_round_gap_ms", calibration_max_interval_ms);
        APPLY_U32_PARAM("cal_max_ms", calibration_max_interval_ms);
        APPLY_U32_PARAM("cal_rx_ms", calibration_rx_slice_ms);
        APPLY_U32_PARAM("cal_guard", calibration_slot_guard_us);
        APPLY_U32_PARAM("cal_guard_us", calibration_slot_guard_us);
        APPLY_BOOL_PARAM("uwb", uwb_enabled);
        APPLY_BOOL_PARAM("bno085", bno085_accel_enabled);
        APPLY_BOOL_PARAM("bno085_accel", bno085_accel_enabled);
        APPLY_U32_PARAM("bno085_accel_interval_ms",
                        bno085_accel_interval_ms);
        APPLY_U32_PARAM("bno085_log_interval_ms", bno085_log_interval_ms);
        {
            char value_text[32] = {0};
            esp_err_t key_err = httpd_query_key_value(
                query, "bno085_sample_hz", value_text, sizeof(value_text));
            if (key_err == ESP_OK) {
                uint32_t interval_ms = 0;
                if (!ota_parse_bno085_sample_hz(value_text, &interval_ms)) {
                    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                               "Invalid bno085_sample_hz");
                }
                config.bno085_accel_interval_ms = interval_ms;
                changed = true;
            } else if (key_err != ESP_ERR_NOT_FOUND) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid bno085_sample_hz");
            }
        }
        {
            char value_text[32] = {0};
            esp_err_t key_err = httpd_query_key_value(
                query, "bno085_sample_ms", value_text, sizeof(value_text));
            if (key_err == ESP_OK) {
                uint32_t parsed = 0;
                if (!ota_parse_u32(value_text, &parsed)) {
                    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                               "Invalid bno085_sample_ms");
                }
                config.bno085_accel_interval_ms = parsed;
                changed = true;
            } else if (key_err != ESP_ERR_NOT_FOUND) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid bno085_sample_ms");
            }
        }
        APPLY_BOOL_PARAM("gps", gps_enabled);
        APPLY_U8_PARAM("radio_channel", radio_channel);
        APPLY_U8_PARAM("uwb_channel", radio_channel);
        APPLY_U32_PARAM("telemetry_port", wireless_telemetry_port);
        APPLY_U32_PARAM("tel_port", wireless_telemetry_port);

#undef APPLY_BOOL_PARAM
#undef APPLY_U32_PARAM
#undef APPLY_U8_PARAM

        if (!app_runtime_config_validate(&config)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid runtime config");
        }

        if (changed) {
            err = app_runtime_config_save(&config);
        }
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Runtime config failed: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Runtime config failed");
    }

    const app_runtime_config_t *active_config = app_runtime_config_get();
    if (changed && before_config.gps_enabled != active_config->gps_enabled) {
        const esp_err_t gps_err = gps_service_apply_runtime_config();
        if (gps_err != ESP_OK) {
            ESP_LOGW(TAG, "GPS runtime apply failed: %s",
                     esp_err_to_name(gps_err));
        }
    }

    if (changed &&
        (before_config.bno085_accel_enabled !=
             active_config->bno085_accel_enabled ||
         before_config.bno085_accel_interval_ms !=
             active_config->bno085_accel_interval_ms ||
         before_config.bno085_log_interval_ms !=
             active_config->bno085_log_interval_ms)) {
        const esp_err_t bno_err = bno085_service_apply_runtime_config();
        if (bno_err != ESP_OK) {
            ESP_LOGW(TAG, "BNO085 runtime apply failed: %s",
                     esp_err_to_name(bno_err));
        }
    }

    const bool reboot_recommended =
        runtime_config_reboot_recommended(&before_config, active_config);
    ESP_LOGW(TAG,
             "Runtime config: changed=%s cleared=%s mode=%s(%u) tag=%u anchors=[%u,%u,%u,%u] count=%u coord=%u reboot_recommended=%s reboot_requested=%s",
             changed ? "true" : "false", cleared ? "true" : "false",
             app_runtime_config_runtime_mode_to_string(
                 active_config->runtime_mode),
             (unsigned)active_config->runtime_mode,
             (unsigned)active_config->tag_id,
             (unsigned)active_config->anchor_ids[0],
             (unsigned)active_config->anchor_ids[1],
             (unsigned)active_config->anchor_ids[2],
             (unsigned)active_config->anchor_ids[3],
             (unsigned)active_config->anchor_count,
             (unsigned)active_config->anchor_survey_coordinator_id,
             reboot_recommended ? "true" : "false",
             reboot_requested ? "true" : "false");

    char response[1600];
    const int len = snprintf(
        response, sizeof(response),
        "{"
        "\"ok\":true,"
        "\"changed\":%s,"
        "\"cleared\":%s,"
        "\"runtime_config_from_nvs\":%s,"
        "\"runtime_mode\":%u,"
        "\"runtime_mode_name\":\"%s\","
        "\"runtime_tag_id\":%u,"
        "\"runtime_anchor_count\":%u,"
        "\"runtime_anchor_ids\":[%u,%u,%u,%u],"
        "\"runtime_anchor_survey_coordinator_id\":%u,"
        "\"runtime_ranging_slot_ms\":%lu,"
        "\"runtime_ranging_round_gap_ms\":%lu,"
        "\"runtime_anchor_survey_slot_ms\":%lu,"
        "\"runtime_anchor_survey_round_gap_ms\":%lu,"
        "\"runtime_calibration_method\":%u,"
        "\"runtime_calibration_three_ids\":[%u,%u,%u],"
        "\"runtime_uwb_enabled\":%s,"
        "\"runtime_bno085_accel_enabled\":%s,"
        "\"runtime_bno085_accel_interval_ms\":%lu,"
        "\"runtime_bno085_log_interval_ms\":%lu,"
        "\"runtime_gps_enabled\":%s,"
        "\"runtime_radio_channel\":%u,"
        "\"runtime_wireless_telemetry_port\":%lu,"
        "\"reboot_recommended\":%s,"
        "\"rebooting\":%s"
        "}\n",
        changed ? "true" : "false", cleared ? "true" : "false",
        active_config->from_nvs ? "true" : "false",
        (unsigned)active_config->runtime_mode,
        app_runtime_config_runtime_mode_to_string(
            active_config->runtime_mode),
        (unsigned)active_config->tag_id,
        (unsigned)active_config->anchor_count,
        (unsigned)active_config->anchor_ids[0],
        (unsigned)active_config->anchor_ids[1],
        (unsigned)active_config->anchor_ids[2],
        (unsigned)active_config->anchor_ids[3],
        (unsigned)active_config->anchor_survey_coordinator_id,
        (unsigned long)active_config->ranging_slot_ms,
        (unsigned long)active_config->ranging_round_gap_ms,
        (unsigned long)active_config->anchor_survey_slot_ms,
        (unsigned long)active_config->anchor_survey_round_gap_ms,
        (unsigned)active_config->calibration_method,
        (unsigned)active_config->calibration_three_ids[0],
        (unsigned)active_config->calibration_three_ids[1],
        (unsigned)active_config->calibration_three_ids[2],
        active_config->uwb_enabled ? "true" : "false",
        active_config->bno085_accel_enabled ? "true" : "false",
        (unsigned long)active_config->bno085_accel_interval_ms,
        (unsigned long)active_config->bno085_log_interval_ms,
        active_config->gps_enabled ? "true" : "false",
        (unsigned)runtime_radio_channel(active_config),
        (unsigned long)active_config->wireless_telemetry_port,
        reboot_recommended ? "true" : "false",
        reboot_requested ? "true" : "false");

    if (len < 0 || len >= (int)sizeof(response)) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "response too long");
    }

    httpd_resp_set_type(req, "application/json");
    const esp_err_t response_err = httpd_resp_send(req, response, len);

    if (reboot_requested) {
        s_status = OTA_SERVICE_STATUS_REBOOTING;
        xTaskCreate(reboot_task, "runtime_reboot",
                    OTA_SERVICE_RESTART_TASK_STACK_WORDS, NULL,
                    OTA_SERVICE_RESTART_TASK_PRIORITY, NULL);
    }

    return response_err;
}

static void reboot_task(void *arg)
{
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(OTA_SERVICE_REBOOT_DELAY_MS));
    ESP_LOGI(TAG, "Restarting device");
    esp_restart();
}

static esp_err_t send_ota_error(httpd_req_t *req, httpd_err_code_t code,
                                const char *message)
{
    ESP_LOGE(TAG, "%s", message);
    return httpd_resp_send_err(req, code, message);
}

static esp_err_t ota_post_handler(httpd_req_t *req)
{
    if (!ota_request_authorized(req)) {
        ESP_LOGW(TAG, "Rejected OTA upload: missing or invalid token");
        return httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED,
                                   "Missing or invalid OTA token");
    }

    if (s_ota_in_progress) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "OTA already in progress");
    }

    const esp_partition_t *update_partition =
        esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        s_status = OTA_SERVICE_STATUS_FAILED;
        return send_ota_error(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                              "No OTA update partition available");
    }

    if (req->content_len <= 0) {
        s_status = OTA_SERVICE_STATUS_FAILED;
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Missing firmware body");
    }

    if ((size_t)req->content_len > update_partition->size) {
        s_status = OTA_SERVICE_STATUS_FAILED;
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Firmware image is larger than OTA partition");
    }

    s_ota_in_progress = true;
    s_status = OTA_SERVICE_STATUS_UPDATING;

    ESP_LOGI(TAG, "OTA upload start: %d bytes -> partition %s @ 0x%lx",
             req->content_len, update_partition->label,
             (unsigned long)update_partition->address);

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES,
                                  &ota_handle);
    if (err != ESP_OK) {
        s_ota_in_progress = false;
        s_status = OTA_SERVICE_STATUS_FAILED;
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "esp_ota_begin failed");
    }

    int remaining = req->content_len;
    size_t written = 0;
    size_t next_progress_log = 256 * 1024;

    while (remaining > 0) {
        const size_t to_read = remaining < OTA_SERVICE_CHUNK_SIZE
                                   ? (size_t)remaining
                                   : OTA_SERVICE_CHUNK_SIZE;
        const int received =
            httpd_req_recv(req, (char *)s_ota_buffer, to_read);

        if (received == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }

        if (received <= 0) {
            esp_ota_abort(ota_handle);
            s_ota_in_progress = false;
            s_status = OTA_SERVICE_STATUS_FAILED;
            return send_ota_error(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                  "Failed to receive OTA data");
        }

        err = esp_ota_write(ota_handle, s_ota_buffer, (size_t)received);
        if (err != ESP_OK) {
            esp_ota_abort(ota_handle);
            s_ota_in_progress = false;
            s_status = OTA_SERVICE_STATUS_FAILED;
            ESP_LOGE(TAG, "esp_ota_write failed after %u bytes: %s",
                     (unsigned)written, esp_err_to_name(err));
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                       "esp_ota_write failed");
        }

        remaining -= received;
        written += (size_t)received;

        if (written >= next_progress_log || remaining == 0) {
            ESP_LOGI(TAG, "OTA received %u/%d bytes", (unsigned)written,
                     req->content_len);
            next_progress_log += 256 * 1024;
        }
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        s_ota_in_progress = false;
        s_status = OTA_SERVICE_STATUS_FAILED;
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "esp_ota_end failed");
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        s_ota_in_progress = false;
        s_status = OTA_SERVICE_STATUS_FAILED;
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s",
                 esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "esp_ota_set_boot_partition failed");
    }

    s_status = OTA_SERVICE_STATUS_REBOOTING;
    ESP_LOGI(TAG, "OTA upload complete, next boot partition: %s",
             update_partition->label);

    httpd_resp_set_type(req, "application/json");
    const esp_err_t response_err =
        httpd_resp_sendstr(req, "{\"ok\":true,\"rebooting\":true}\n");

    xTaskCreate(reboot_task, "ota_reboot", OTA_SERVICE_RESTART_TASK_STACK_WORDS,
                NULL, OTA_SERVICE_RESTART_TASK_PRIORITY, NULL);

    return response_err;
}

static esp_err_t start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.stack_size = 8192;
    config.max_uri_handlers = 6;

    esp_err_t err = httpd_start(&s_http_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return err;
    }

    const httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = status_get_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t ota_uri = {
        .uri = "/ota",
        .method = HTTP_POST,
        .handler = ota_post_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t antenna_delay_uri = {
        .uri = "/config/antenna-delay",
        .method = HTTP_POST,
        .handler = antenna_delay_post_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t runtime_config_uri = {
        .uri = "/config/runtime",
        .method = HTTP_POST,
        .handler = runtime_config_post_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t charger_config_uri = {
        .uri = "/config/charger",
        .method = HTTP_POST,
        .handler = charger_config_post_handler,
        .user_ctx = NULL,
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &root_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &status_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &ota_uri));
    ESP_ERROR_CHECK(
        httpd_register_uri_handler(s_http_server, &antenna_delay_uri));
    ESP_ERROR_CHECK(
        httpd_register_uri_handler(s_http_server, &runtime_config_uri));
    ESP_ERROR_CHECK(
        httpd_register_uri_handler(s_http_server, &charger_config_uri));

    s_running = true;
    s_status = OTA_SERVICE_STATUS_RUNNING;

    ESP_LOGI(TAG, "HTTP OTA server ready: http://%s/status",
             wifi_service_get_ip_address());
    return ESP_OK;
}

static void ota_service_task(void *arg)
{
    (void)arg;

    s_status = OTA_SERVICE_STATUS_WAITING_FOR_WIFI;
    while (!wifi_service_is_connected()) {
        vTaskDelay(pdMS_TO_TICKS(OTA_SERVICE_WIFI_WAIT_MS));
    }

    if (start_http_server() != ESP_OK) {
        s_status = OTA_SERVICE_STATUS_FAILED;
    }

    vTaskDelete(NULL);
}

esp_err_t ota_service_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    const BaseType_t created = xTaskCreatePinnedToCore(
        ota_service_task, "ota_service", OTA_SERVICE_TASK_STACK_WORDS, NULL,
        OTA_SERVICE_TASK_PRIORITY, NULL, 0);
    if (created != pdPASS) {
        s_status = OTA_SERVICE_STATUS_FAILED;
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    return ESP_OK;
}

bool ota_service_is_running(void)
{
    return s_running;
}

enum ota_service_status ota_service_get_status(void)
{
    return s_status;
}
