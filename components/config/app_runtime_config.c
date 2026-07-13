#include "app_runtime_config.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_config.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs.h"
#include "uwb_config.h"

static const char *TAG = "app_runtime_config";

#define APP_RUNTIME_CONFIG_NVS_NAMESPACE "runtime"
#define KEY_MODE "mode"
#define KEY_TAG "tag"
#define KEY_ACOUNT "acount"
#define KEY_A0 "a0"
#define KEY_A1 "a1"
#define KEY_A2 "a2"
#define KEY_A3 "a3"
#define KEY_COORD "coord"
#define KEY_AS_RX "as_rx"
#define KEY_AS_CMD "as_cmd"
#define KEY_AS_SLOT "as_slot"
#define KEY_AS_GAP "as_gap"
#define KEY_AS_LOG "as_log"
#define KEY_RNG_SLOT "rng_slot"
#define KEY_RNG_GAP "rng_gap"
#define KEY_RNG_RX "rng_rx"
#define KEY_DT_PEER "dt_peer"
#define KEY_DT_INIT "dt_init"
#define KEY_DT_RESP "dt_resp"
#define KEY_DT_INT "dt_int"
#define KEY_DT_RX "dt_rx"
#define KEY_DT_RESPD "dt_respd"
#define KEY_DT_FINAL "dt_final"
#define KEY_DT_REPORT "dt_report"
#define KEY_DT_ARX "dt_arx"
#define KEY_CAL_METHOD "cal_method"
#define KEY_CAL_REF "cal_ref"
#define KEY_CAL_DUT "cal_dut"
#define KEY_CAL_ID0 "cal_id0"
#define KEY_CAL_ID1 "cal_id1"
#define KEY_CAL_ID2 "cal_id2"
#define KEY_CAL_KNOWN "cal_known"
#define KEY_CAL_D01 "cal_d01"
#define KEY_CAL_D02 "cal_d02"
#define KEY_CAL_D12 "cal_d12"
#define KEY_CAL_SAMPLES "cal_samples"
#define KEY_CAL_SUMMARY "cal_summary"
#define KEY_CAL_MIN "cal_min"
#define KEY_CAL_MAX "cal_max"
#define KEY_CAL_RX "cal_rx"
#define KEY_CAL_GUARD "cal_guard"
#define KEY_UWB_ENABLED "uwb_enabled"
#define KEY_BNO085_ACCEL "bno085_accel"
#define KEY_BNO085_RATE "bno_rate"
#define KEY_BNO085_LOG "bno_log"
#define KEY_GPS_ENABLED "gps_enabled"
#define KEY_RADIO_CH "radio_ch"
#define KEY_TEL_PORT "tel_port"

static bool s_initialized;
static app_runtime_config_t s_config;

static bool id_valid(uint8_t id)
{
    return id != 0;
}

static bool ms_valid(uint32_t value)
{
    return value > 0 && value <= 60000U;
}

static bool us_valid(uint32_t value)
{
    return value > 0 && value <= 1000000U;
}

static bool bno085_ms_valid(uint32_t value)
{
    return value >= 2U && value <= 60000U;
}

static bool count_valid(uint8_t count)
{
    return count > 0 && count <= APP_RUNTIME_CONFIG_MAX_ANCHORS;
}

static bool radio_channel_valid(uint8_t channel)
{
    return channel == 5U || channel == 9U;
}

static bool tcp_port_valid(uint32_t port)
{
    return port > 0U && port <= 65535U;
}

static bool anchor_ids_valid(const app_runtime_config_t *config)
{
    if (config == NULL || !count_valid(config->anchor_count)) {
        return false;
    }

    for (size_t i = 0; i < config->anchor_count; ++i) {
        if (!id_valid(config->anchor_ids[i])) {
            return false;
        }
        for (size_t j = i + 1U; j < config->anchor_count; ++j) {
            if (config->anchor_ids[i] == config->anchor_ids[j]) {
                return false;
            }
        }
    }

    return true;
}

bool app_runtime_config_runtime_mode_valid(uint8_t mode)
{
    return mode == APP_RUNTIME_MODE_UWB_BEACON_SMOKE ||
           mode == APP_RUNTIME_MODE_UWB_DISTANCE_TEST ||
           mode == APP_RUNTIME_MODE_UWB_ANTENNA_DELAY_CALIBRATION ||
           mode == APP_RUNTIME_MODE_UWB_RANGING ||
           mode == APP_RUNTIME_MODE_UWB_ANCHOR_SURVEY ||
           mode == APP_RUNTIME_MODE_UWB_DS_TWR_TDOA;
}

const char *app_runtime_config_runtime_mode_to_string(uint8_t mode)
{
    switch (mode) {
    case APP_RUNTIME_MODE_UWB_BEACON_SMOKE:
        return "uwb_beacon_smoke";
    case APP_RUNTIME_MODE_UWB_DISTANCE_TEST:
        return "uwb_distance_test";
    case APP_RUNTIME_MODE_UWB_ANTENNA_DELAY_CALIBRATION:
        return "uwb_antenna_delay_calibration";
    case APP_RUNTIME_MODE_UWB_RANGING:
        return "uwb_ranging";
    case APP_RUNTIME_MODE_UWB_ANCHOR_SURVEY:
        return "uwb_anchor_survey";
    case APP_RUNTIME_MODE_UWB_DS_TWR_TDOA:
        return "uwb_ds_twr_tdoa";
    default:
        return "unknown";
    }
}

static bool string_equal(const char *a, const char *b)
{
    return a != NULL && b != NULL && strcmp(a, b) == 0;
}

static bool parse_u8_text(const char *text, uint8_t *value)
{
    if (text == NULL || text[0] == '\0' || value == NULL) {
        return false;
    }

    errno = 0;
    char *end = NULL;
    const unsigned long parsed = strtoul(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || parsed > 255UL) {
        return false;
    }

    *value = (uint8_t)parsed;
    return true;
}

uint8_t app_runtime_config_runtime_mode_from_string(const char *text, bool *ok)
{
    uint8_t parsed = 0;
    bool parsed_ok = false;

    if (parse_u8_text(text, &parsed) &&
        app_runtime_config_runtime_mode_valid(parsed)) {
        parsed_ok = true;
    } else if (string_equal(text, "beacon") ||
               string_equal(text, "uwb_beacon_smoke")) {
        parsed = APP_RUNTIME_MODE_UWB_BEACON_SMOKE;
        parsed_ok = true;
    } else if (string_equal(text, "distance") ||
               string_equal(text, "distance_test") ||
               string_equal(text, "uwb_distance_test")) {
        parsed = APP_RUNTIME_MODE_UWB_DISTANCE_TEST;
        parsed_ok = true;
    } else if (string_equal(text, "calibration") ||
               string_equal(text, "antenna_delay_calibration") ||
               string_equal(text, "uwb_antenna_delay_calibration")) {
        parsed = APP_RUNTIME_MODE_UWB_ANTENNA_DELAY_CALIBRATION;
        parsed_ok = true;
    } else if (string_equal(text, "ranging") ||
               string_equal(text, "uwb_ranging")) {
        parsed = APP_RUNTIME_MODE_UWB_RANGING;
        parsed_ok = true;
    } else if (string_equal(text, "survey") ||
               string_equal(text, "anchor_survey") ||
               string_equal(text, "uwb_anchor_survey")) {
        parsed = APP_RUNTIME_MODE_UWB_ANCHOR_SURVEY;
        parsed_ok = true;
    } else if (string_equal(text, "ds_twr_tdoa") ||
               string_equal(text, "dstwr_tdoa") ||
               string_equal(text, "ds-twr-tdoa") ||
               string_equal(text, "uwb_ds_twr_tdoa")) {
        parsed = APP_RUNTIME_MODE_UWB_DS_TWR_TDOA;
        parsed_ok = true;
    }

    if (ok != NULL) {
        *ok = parsed_ok;
    }
    return parsed;
}

void app_runtime_config_defaults(app_runtime_config_t *config)
{
    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->runtime_mode = (uint8_t)APP_RUNTIME_MODE;
    config->tag_id = (uint8_t)APP_UWB_TAG_ID;
    config->anchor_count = (uint8_t)APP_UWB_ANCHOR_COUNT;
    config->anchor_ids[0] = (uint8_t)APP_UWB_ANCHOR_0_ID;
    config->anchor_ids[1] = (uint8_t)APP_UWB_ANCHOR_1_ID;
    config->anchor_ids[2] = (uint8_t)APP_UWB_ANCHOR_2_ID;
    config->anchor_ids[3] = (uint8_t)APP_UWB_ANCHOR_3_ID;
    config->anchor_survey_coordinator_id =
        (uint8_t)APP_UWB_ANCHOR_SURVEY_COORDINATOR_ID;
    config->anchor_survey_rx_slice_ms = APP_UWB_ANCHOR_SURVEY_RX_SLICE_MS;
    config->anchor_survey_command_delay_ms =
        APP_UWB_ANCHOR_SURVEY_COMMAND_DELAY_MS;
    config->anchor_survey_slot_ms = APP_UWB_ANCHOR_SURVEY_SLOT_MS;
    config->anchor_survey_round_gap_ms = APP_UWB_ANCHOR_SURVEY_ROUND_GAP_MS;
    config->anchor_survey_passive_tag_log_every =
        APP_UWB_ANCHOR_SURVEY_PASSIVE_TAG_LOG_EVERY;
    config->ranging_slot_ms = APP_UWB_RANGING_SLOT_MS;
    config->ranging_round_gap_ms = APP_UWB_RANGING_ROUND_GAP_MS;
    config->ranging_rx_slice_ms = APP_UWB_RANGING_RX_SLICE_MS;
    config->distance_test_peer_id = (uint8_t)APP_UWB_DISTANCE_TEST_PEER_ID;
    config->distance_test_initiator_id =
        (uint8_t)APP_UWB_DISTANCE_TEST_INITIATOR_ID;
    config->distance_test_responder_id =
        (uint8_t)APP_UWB_DISTANCE_TEST_RESPONDER_ID;
    config->distance_test_interval_ms = APP_UWB_DISTANCE_TEST_INTERVAL_MS;
    config->distance_test_rx_timeout_ms = APP_UWB_DISTANCE_TEST_RX_TIMEOUT_MS;
    config->distance_test_resp_delay_ms = APP_UWB_DISTANCE_TEST_RESP_DELAY_MS;
    config->distance_test_final_delay_ms = APP_UWB_DISTANCE_TEST_FINAL_DELAY_MS;
    config->distance_test_report_delay_ms =
        APP_UWB_DISTANCE_TEST_REPORT_DELAY_MS;
    config->distance_test_auto_rx_delay_uus =
        APP_UWB_DISTANCE_TEST_AUTO_RX_DELAY_UUS;
    config->calibration_method = (uint8_t)APP_UWB_CALIBRATION_METHOD;
    config->calibration_reference_id =
        (uint8_t)APP_UWB_CALIBRATION_REFERENCE_ID;
    config->calibration_dut_id = (uint8_t)APP_UWB_CALIBRATION_DUT_ID;
    config->calibration_three_ids[0] =
        (uint8_t)APP_UWB_CALIBRATION_THREE_ID_0;
    config->calibration_three_ids[1] =
        (uint8_t)APP_UWB_CALIBRATION_THREE_ID_1;
    config->calibration_three_ids[2] =
        (uint8_t)APP_UWB_CALIBRATION_THREE_ID_2;
    config->calibration_known_distance_mm =
        APP_UWB_CALIBRATION_KNOWN_DISTANCE_MM;
    config->calibration_three_distance_0_1_mm =
        APP_UWB_CALIBRATION_THREE_DISTANCE_0_1_MM;
    config->calibration_three_distance_0_2_mm =
        APP_UWB_CALIBRATION_THREE_DISTANCE_0_2_MM;
    config->calibration_three_distance_1_2_mm =
        APP_UWB_CALIBRATION_THREE_DISTANCE_1_2_MM;
    config->calibration_sample_count = APP_UWB_CALIBRATION_SAMPLE_COUNT;
    config->calibration_summary_every = APP_UWB_CALIBRATION_SUMMARY_EVERY;
    config->calibration_min_interval_ms = APP_UWB_CALIBRATION_MIN_INTERVAL_MS;
    config->calibration_max_interval_ms = APP_UWB_CALIBRATION_MAX_INTERVAL_MS;
    config->calibration_rx_slice_ms = APP_UWB_CALIBRATION_RX_SLICE_MS;
    config->calibration_slot_guard_us = APP_UWB_CALIBRATION_SLOT_GUARD_US;
    config->uwb_enabled = APP_UWB_ENABLED != 0;
    config->bno085_accel_enabled = APP_BNO085_ACCEL_ENABLED_DEFAULT != 0;
    config->bno085_accel_interval_ms = APP_BNO085_ACCEL_INTERVAL_MS;
    config->bno085_log_interval_ms = APP_BNO085_LOG_INTERVAL_MS;
    config->gps_enabled = APP_GPS_ENABLED_DEFAULT != 0;
    config->radio_channel = (uint8_t)APP_UWB_RADIO_CHANNEL;
    config->wireless_telemetry_port = APP_WIRELESS_TELEMETRY_PORT;
    config->from_nvs = false;
}

bool app_runtime_config_validate(const app_runtime_config_t *config)
{
    if (config == NULL ||
        !app_runtime_config_runtime_mode_valid(config->runtime_mode) ||
        !id_valid(config->tag_id) || !anchor_ids_valid(config) ||
        !id_valid(config->anchor_survey_coordinator_id) ||
        !ms_valid(config->anchor_survey_rx_slice_ms) ||
        !ms_valid(config->anchor_survey_command_delay_ms) ||
        !ms_valid(config->anchor_survey_slot_ms) ||
        !ms_valid(config->anchor_survey_round_gap_ms) ||
        !ms_valid(config->ranging_slot_ms) ||
        !ms_valid(config->ranging_round_gap_ms) ||
        !ms_valid(config->ranging_rx_slice_ms) ||
        !id_valid(config->distance_test_initiator_id) ||
        !id_valid(config->distance_test_responder_id) ||
        config->distance_test_initiator_id ==
            config->distance_test_responder_id ||
        !ms_valid(config->distance_test_interval_ms) ||
        !ms_valid(config->distance_test_rx_timeout_ms) ||
        !ms_valid(config->distance_test_resp_delay_ms) ||
        !ms_valid(config->distance_test_final_delay_ms) ||
        !ms_valid(config->distance_test_report_delay_ms) ||
        config->distance_test_auto_rx_delay_uus == 0 ||
        config->calibration_sample_count == 0 ||
        config->calibration_summary_every == 0 ||
        !ms_valid(config->calibration_min_interval_ms) ||
        !ms_valid(config->calibration_max_interval_ms) ||
        !ms_valid(config->calibration_rx_slice_ms) ||
        !us_valid(config->calibration_slot_guard_us) ||
        !bno085_ms_valid(config->bno085_accel_interval_ms) ||
        !bno085_ms_valid(config->bno085_log_interval_ms) ||
        !radio_channel_valid(config->radio_channel) ||
        !tcp_port_valid(config->wireless_telemetry_port)) {
        return false;
    }

    if (config->calibration_method != APP_UWB_CALIBRATION_METHOD_TWO_MODULE &&
        config->calibration_method != APP_UWB_CALIBRATION_METHOD_THREE_MODULE) {
        return false;
    }

    if (!id_valid(config->calibration_reference_id) ||
        !id_valid(config->calibration_dut_id) ||
        config->calibration_reference_id == config->calibration_dut_id) {
        return false;
    }

    for (size_t i = 0; i < APP_RUNTIME_CONFIG_CAL_THREE_COUNT; ++i) {
        if (!id_valid(config->calibration_three_ids[i])) {
            return false;
        }
        for (size_t j = i + 1U; j < APP_RUNTIME_CONFIG_CAL_THREE_COUNT; ++j) {
            if (config->calibration_three_ids[i] ==
                config->calibration_three_ids[j]) {
                return false;
            }
        }
    }

    return true;
}

static bool read_u8(nvs_handle_t handle, const char *key, uint8_t *value)
{
    uint8_t stored = 0;
    const esp_err_t err = nvs_get_u8(handle, key, &stored);
    if (err == ESP_OK) {
        *value = stored;
        return true;
    }
    return false;
}

static bool read_u32(nvs_handle_t handle, const char *key, uint32_t *value)
{
    uint32_t stored = 0;
    const esp_err_t err = nvs_get_u32(handle, key, &stored);
    if (err == ESP_OK) {
        *value = stored;
        return true;
    }
    return false;
}

static bool read_bool(nvs_handle_t handle, const char *key, bool *value)
{
    uint8_t stored = 0;
    const esp_err_t err = nvs_get_u8(handle, key, &stored);
    if (err == ESP_OK) {
        *value = stored != 0;
        return true;
    }
    return false;
}

static void read_config_from_nvs(app_runtime_config_t *config)
{
    nvs_handle_t handle = 0;
    const esp_err_t err =
        nvs_open(APP_RUNTIME_CONFIG_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return;
    }

    bool found = false;
    found |= read_u8(handle, KEY_MODE, &config->runtime_mode);
    found |= read_u8(handle, KEY_TAG, &config->tag_id);
    found |= read_u8(handle, KEY_ACOUNT, &config->anchor_count);
    found |= read_u8(handle, KEY_A0, &config->anchor_ids[0]);
    found |= read_u8(handle, KEY_A1, &config->anchor_ids[1]);
    found |= read_u8(handle, KEY_A2, &config->anchor_ids[2]);
    found |= read_u8(handle, KEY_A3, &config->anchor_ids[3]);
    found |= read_u8(handle, KEY_COORD, &config->anchor_survey_coordinator_id);
    found |= read_u32(handle, KEY_AS_RX, &config->anchor_survey_rx_slice_ms);
    found |= read_u32(handle, KEY_AS_CMD,
                      &config->anchor_survey_command_delay_ms);
    found |= read_u32(handle, KEY_AS_SLOT, &config->anchor_survey_slot_ms);
    found |= read_u32(handle, KEY_AS_GAP, &config->anchor_survey_round_gap_ms);
    found |= read_u32(handle, KEY_AS_LOG,
                      &config->anchor_survey_passive_tag_log_every);
    found |= read_u32(handle, KEY_RNG_SLOT, &config->ranging_slot_ms);
    found |= read_u32(handle, KEY_RNG_GAP, &config->ranging_round_gap_ms);
    found |= read_u32(handle, KEY_RNG_RX, &config->ranging_rx_slice_ms);
    found |= read_u8(handle, KEY_DT_PEER, &config->distance_test_peer_id);
    found |= read_u8(handle, KEY_DT_INIT, &config->distance_test_initiator_id);
    found |= read_u8(handle, KEY_DT_RESP, &config->distance_test_responder_id);
    found |= read_u32(handle, KEY_DT_INT, &config->distance_test_interval_ms);
    found |= read_u32(handle, KEY_DT_RX, &config->distance_test_rx_timeout_ms);
    found |= read_u32(handle, KEY_DT_RESPD,
                      &config->distance_test_resp_delay_ms);
    found |= read_u32(handle, KEY_DT_FINAL,
                      &config->distance_test_final_delay_ms);
    found |= read_u32(handle, KEY_DT_REPORT,
                      &config->distance_test_report_delay_ms);
    found |= read_u32(handle, KEY_DT_ARX,
                      &config->distance_test_auto_rx_delay_uus);
    found |= read_u8(handle, KEY_CAL_METHOD, &config->calibration_method);
    found |= read_u8(handle, KEY_CAL_REF, &config->calibration_reference_id);
    found |= read_u8(handle, KEY_CAL_DUT, &config->calibration_dut_id);
    found |= read_u8(handle, KEY_CAL_ID0, &config->calibration_three_ids[0]);
    found |= read_u8(handle, KEY_CAL_ID1, &config->calibration_three_ids[1]);
    found |= read_u8(handle, KEY_CAL_ID2, &config->calibration_three_ids[2]);
    found |= read_u32(handle, KEY_CAL_KNOWN,
                      &config->calibration_known_distance_mm);
    found |= read_u32(handle, KEY_CAL_D01,
                      &config->calibration_three_distance_0_1_mm);
    found |= read_u32(handle, KEY_CAL_D02,
                      &config->calibration_three_distance_0_2_mm);
    found |= read_u32(handle, KEY_CAL_D12,
                      &config->calibration_three_distance_1_2_mm);
    found |= read_u32(handle, KEY_CAL_SAMPLES,
                      &config->calibration_sample_count);
    found |= read_u32(handle, KEY_CAL_SUMMARY,
                      &config->calibration_summary_every);
    found |= read_u32(handle, KEY_CAL_MIN,
                      &config->calibration_min_interval_ms);
    found |= read_u32(handle, KEY_CAL_MAX,
                      &config->calibration_max_interval_ms);
    found |= read_u32(handle, KEY_CAL_RX, &config->calibration_rx_slice_ms);
    found |= read_u32(handle, KEY_CAL_GUARD,
                      &config->calibration_slot_guard_us);
    found |= read_bool(handle, KEY_UWB_ENABLED, &config->uwb_enabled);
    found |= read_bool(handle, KEY_BNO085_ACCEL,
                       &config->bno085_accel_enabled);
    found |= read_u32(handle, KEY_BNO085_RATE,
                      &config->bno085_accel_interval_ms);
    found |= read_u32(handle, KEY_BNO085_LOG,
                      &config->bno085_log_interval_ms);
    found |= read_bool(handle, KEY_GPS_ENABLED, &config->gps_enabled);
    found |= read_u8(handle, KEY_RADIO_CH, &config->radio_channel);
    found |= read_u32(handle, KEY_TEL_PORT,
                      &config->wireless_telemetry_port);
    config->from_nvs = found;

    nvs_close(handle);
}

esp_err_t app_runtime_config_reload(void)
{
    app_runtime_config_t loaded = {0};
    app_runtime_config_defaults(&loaded);
    read_config_from_nvs(&loaded);

    if (!app_runtime_config_validate(&loaded)) {
        ESP_LOGE(TAG, "Invalid runtime config in NVS; using firmware defaults");
        app_runtime_config_defaults(&loaded);
    }

    s_config = loaded;
    s_initialized = true;
    ESP_LOGI(TAG,
             "Runtime config ready: mode=%s(%u) tag=%u anchors=[%u,%u,%u,%u] count=%u coord=%u uwb=%s bno085=%s bno_rate=%lu ms bno_log=%lu ms gps=%s radio_ch=%u tel_port=%lu source=%s",
             app_runtime_config_runtime_mode_to_string(s_config.runtime_mode),
             (unsigned)s_config.runtime_mode, (unsigned)s_config.tag_id,
             (unsigned)s_config.anchor_ids[0],
             (unsigned)s_config.anchor_ids[1],
             (unsigned)s_config.anchor_ids[2],
             (unsigned)s_config.anchor_ids[3],
             (unsigned)s_config.anchor_count,
             (unsigned)s_config.anchor_survey_coordinator_id,
             s_config.uwb_enabled ? "on" : "off",
             s_config.bno085_accel_enabled ? "on" : "off",
             (unsigned long)s_config.bno085_accel_interval_ms,
             (unsigned long)s_config.bno085_log_interval_ms,
             s_config.gps_enabled ? "on" : "off",
             (unsigned)s_config.radio_channel,
             (unsigned long)s_config.wireless_telemetry_port,
             s_config.from_nvs ? "nvs" : "firmware");
    return ESP_OK;
}

esp_err_t app_runtime_config_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }
    return app_runtime_config_reload();
}

const app_runtime_config_t *app_runtime_config_get(void)
{
    if (!s_initialized) {
        (void)app_runtime_config_init();
    }
    return &s_config;
}

static esp_err_t write_u8(nvs_handle_t handle, const char *key, uint8_t value)
{
    return nvs_set_u8(handle, key, value);
}

static esp_err_t write_u32(nvs_handle_t handle, const char *key, uint32_t value)
{
    return nvs_set_u32(handle, key, value);
}

static esp_err_t write_bool(nvs_handle_t handle, const char *key, bool value)
{
    return nvs_set_u8(handle, key, value ? 1U : 0U);
}

esp_err_t app_runtime_config_save(const app_runtime_config_t *config)
{
    if (!app_runtime_config_validate(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle = 0;
    ESP_RETURN_ON_ERROR(
        nvs_open(APP_RUNTIME_CONFIG_NVS_NAMESPACE, NVS_READWRITE, &handle),
        TAG, "open runtime NVS failed");

    esp_err_t err = ESP_OK;
#define WRITE_OR_GOTO(expr)       \
    do {                          \
        err = (expr);             \
        if (err != ESP_OK) {      \
            goto done;            \
        }                         \
    } while (0)

    WRITE_OR_GOTO(write_u8(handle, KEY_MODE, config->runtime_mode));
    WRITE_OR_GOTO(write_u8(handle, KEY_TAG, config->tag_id));
    WRITE_OR_GOTO(write_u8(handle, KEY_ACOUNT, config->anchor_count));
    WRITE_OR_GOTO(write_u8(handle, KEY_A0, config->anchor_ids[0]));
    WRITE_OR_GOTO(write_u8(handle, KEY_A1, config->anchor_ids[1]));
    WRITE_OR_GOTO(write_u8(handle, KEY_A2, config->anchor_ids[2]));
    WRITE_OR_GOTO(write_u8(handle, KEY_A3, config->anchor_ids[3]));
    WRITE_OR_GOTO(write_u8(handle, KEY_COORD,
                           config->anchor_survey_coordinator_id));
    WRITE_OR_GOTO(write_u32(handle, KEY_AS_RX,
                            config->anchor_survey_rx_slice_ms));
    WRITE_OR_GOTO(write_u32(handle, KEY_AS_CMD,
                            config->anchor_survey_command_delay_ms));
    WRITE_OR_GOTO(write_u32(handle, KEY_AS_SLOT,
                            config->anchor_survey_slot_ms));
    WRITE_OR_GOTO(write_u32(handle, KEY_AS_GAP,
                            config->anchor_survey_round_gap_ms));
    WRITE_OR_GOTO(write_u32(handle, KEY_AS_LOG,
                            config->anchor_survey_passive_tag_log_every));
    WRITE_OR_GOTO(write_u32(handle, KEY_RNG_SLOT, config->ranging_slot_ms));
    WRITE_OR_GOTO(write_u32(handle, KEY_RNG_GAP,
                            config->ranging_round_gap_ms));
    WRITE_OR_GOTO(write_u32(handle, KEY_RNG_RX, config->ranging_rx_slice_ms));
    WRITE_OR_GOTO(write_u8(handle, KEY_DT_PEER,
                           config->distance_test_peer_id));
    WRITE_OR_GOTO(write_u8(handle, KEY_DT_INIT,
                           config->distance_test_initiator_id));
    WRITE_OR_GOTO(write_u8(handle, KEY_DT_RESP,
                           config->distance_test_responder_id));
    WRITE_OR_GOTO(write_u32(handle, KEY_DT_INT,
                            config->distance_test_interval_ms));
    WRITE_OR_GOTO(write_u32(handle, KEY_DT_RX,
                            config->distance_test_rx_timeout_ms));
    WRITE_OR_GOTO(write_u32(handle, KEY_DT_RESPD,
                            config->distance_test_resp_delay_ms));
    WRITE_OR_GOTO(write_u32(handle, KEY_DT_FINAL,
                            config->distance_test_final_delay_ms));
    WRITE_OR_GOTO(write_u32(handle, KEY_DT_REPORT,
                            config->distance_test_report_delay_ms));
    WRITE_OR_GOTO(write_u32(handle, KEY_DT_ARX,
                            config->distance_test_auto_rx_delay_uus));
    WRITE_OR_GOTO(write_u8(handle, KEY_CAL_METHOD,
                           config->calibration_method));
    WRITE_OR_GOTO(write_u8(handle, KEY_CAL_REF,
                           config->calibration_reference_id));
    WRITE_OR_GOTO(write_u8(handle, KEY_CAL_DUT, config->calibration_dut_id));
    WRITE_OR_GOTO(write_u8(handle, KEY_CAL_ID0,
                           config->calibration_three_ids[0]));
    WRITE_OR_GOTO(write_u8(handle, KEY_CAL_ID1,
                           config->calibration_three_ids[1]));
    WRITE_OR_GOTO(write_u8(handle, KEY_CAL_ID2,
                           config->calibration_three_ids[2]));
    WRITE_OR_GOTO(write_u32(handle, KEY_CAL_KNOWN,
                            config->calibration_known_distance_mm));
    WRITE_OR_GOTO(write_u32(handle, KEY_CAL_D01,
                            config->calibration_three_distance_0_1_mm));
    WRITE_OR_GOTO(write_u32(handle, KEY_CAL_D02,
                            config->calibration_three_distance_0_2_mm));
    WRITE_OR_GOTO(write_u32(handle, KEY_CAL_D12,
                            config->calibration_three_distance_1_2_mm));
    WRITE_OR_GOTO(write_u32(handle, KEY_CAL_SAMPLES,
                            config->calibration_sample_count));
    WRITE_OR_GOTO(write_u32(handle, KEY_CAL_SUMMARY,
                            config->calibration_summary_every));
    WRITE_OR_GOTO(write_u32(handle, KEY_CAL_MIN,
                            config->calibration_min_interval_ms));
    WRITE_OR_GOTO(write_u32(handle, KEY_CAL_MAX,
                            config->calibration_max_interval_ms));
    WRITE_OR_GOTO(write_u32(handle, KEY_CAL_RX,
                            config->calibration_rx_slice_ms));
    WRITE_OR_GOTO(write_u32(handle, KEY_CAL_GUARD,
                            config->calibration_slot_guard_us));
    WRITE_OR_GOTO(write_bool(handle, KEY_UWB_ENABLED, config->uwb_enabled));
    WRITE_OR_GOTO(write_bool(handle, KEY_BNO085_ACCEL,
                             config->bno085_accel_enabled));
    WRITE_OR_GOTO(write_u32(handle, KEY_BNO085_RATE,
                            config->bno085_accel_interval_ms));
    WRITE_OR_GOTO(write_u32(handle, KEY_BNO085_LOG,
                            config->bno085_log_interval_ms));
    WRITE_OR_GOTO(write_bool(handle, KEY_GPS_ENABLED, config->gps_enabled));
    WRITE_OR_GOTO(write_u8(handle, KEY_RADIO_CH, config->radio_channel));
    WRITE_OR_GOTO(write_u32(handle, KEY_TEL_PORT,
                            config->wireless_telemetry_port));

    err = nvs_commit(handle);

done:
#undef WRITE_OR_GOTO
    nvs_close(handle);
    if (err == ESP_OK) {
        s_config = *config;
        s_config.from_nvs = true;
        ESP_LOGW(TAG, "Runtime config saved to NVS");
    }
    return err;
}

esp_err_t app_runtime_config_clear(void)
{
    nvs_handle_t handle = 0;
    ESP_RETURN_ON_ERROR(
        nvs_open(APP_RUNTIME_CONFIG_NVS_NAMESPACE, NVS_READWRITE, &handle),
        TAG, "open runtime NVS failed");

    esp_err_t err = nvs_erase_all(handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err == ESP_OK) {
        app_runtime_config_defaults(&s_config);
        s_initialized = true;
        ESP_LOGW(TAG, "Runtime config cleared; using firmware defaults");
    }
    return err;
}

size_t app_runtime_config_get_anchor_ids(
    uint8_t ids[APP_RUNTIME_CONFIG_MAX_ANCHORS])
{
    const app_runtime_config_t *config = app_runtime_config_get();
    if (ids != NULL) {
        for (size_t i = 0; i < APP_RUNTIME_CONFIG_MAX_ANCHORS; ++i) {
            ids[i] = config->anchor_ids[i];
        }
    }
    return config->anchor_count;
}

void app_runtime_config_format_anchors(char *buffer, size_t buffer_size,
                                       const app_runtime_config_t *config)
{
    if (buffer == NULL || buffer_size == 0) {
        return;
    }
    if (config == NULL) {
        config = app_runtime_config_get();
    }

    (void)snprintf(buffer, buffer_size, "[%u,%u,%u,%u]",
                   (unsigned)config->anchor_ids[0],
                   (unsigned)config->anchor_ids[1],
                   (unsigned)config->anchor_ids[2],
                   (unsigned)config->anchor_ids[3]);
}
