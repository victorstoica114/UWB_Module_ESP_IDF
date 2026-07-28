#ifndef APP_RUNTIME_CONFIG_H
#define APP_RUNTIME_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_RUNTIME_CONFIG_MAX_ANCHORS 10U
#define APP_RUNTIME_CONFIG_MAX_ANCHOR_PAIRS \
    ((APP_RUNTIME_CONFIG_MAX_ANCHORS * \
      (APP_RUNTIME_CONFIG_MAX_ANCHORS - 1U)) / 2U)
#define APP_RUNTIME_CONFIG_CAL_THREE_COUNT 3U
#define APP_RUNTIME_CONFIG_FLEX_MAX_SLOTS 10U
#define APP_RUNTIME_PASSIVE_DS_FAST_STAR 0U
#define APP_RUNTIME_PASSIVE_DS_ROBUST_ROTATING 1U
#define APP_RUNTIME_PASSIVE_DS_PIPELINE_LEGACY 0U
#define APP_RUNTIME_PASSIVE_DS_PIPELINE_DEADLINE 1U
#define APP_RUNTIME_PASSIVE_DS_SOLVE_FRAME 0U
#define APP_RUNTIME_PASSIVE_DS_SOLVE_ROLLING_ALL 1U
#define APP_RUNTIME_PASSIVE_DS_SOLVE_ROLLING_INDEPENDENT 2U
#define APP_RUNTIME_PASSIVE_DS_SOLVE_ROLLING_SUPERFRAME 3U

typedef struct {
    uint8_t runtime_mode;
    uint8_t tag_id;
    uint8_t anchor_count;
    uint8_t anchor_ids[APP_RUNTIME_CONFIG_MAX_ANCHORS];
    uint8_t flex_tdoa_responder_count;
    uint8_t flex_tdoa_slot_count;
    uint8_t flex_tdoa_slot_initiator_ids[APP_RUNTIME_CONFIG_FLEX_MAX_SLOTS];
    uint16_t flex_tdoa_slot_responder_masks[APP_RUNTIME_CONFIG_FLEX_MAX_SLOTS];
    uint32_t flex_tdoa_config_generation;
    uint32_t flex_tdoa_guard_us;
    uint32_t flex_tdoa_request_subslot_us;
    uint32_t flex_tdoa_request_process_us;
    uint32_t flex_tdoa_response_subslot_us;
    uint32_t flex_tdoa_response_process_us;
    bool flex_tdoa_geometry_fixed;
    uint32_t flex_tdoa_geometry_generation;
    int32_t flex_tdoa_anchor_x_mm[APP_RUNTIME_CONFIG_MAX_ANCHORS];
    int32_t flex_tdoa_anchor_y_mm[APP_RUNTIME_CONFIG_MAX_ANCHORS];
    uint8_t anchor_survey_coordinator_id;
    uint32_t anchor_survey_rx_slice_ms;
    uint32_t anchor_survey_command_delay_ms;
    uint32_t anchor_survey_slot_ms;
    uint32_t anchor_survey_round_gap_ms;
    uint32_t anchor_survey_passive_tag_log_every;
    uint32_t ranging_slot_ms;
    uint32_t ranging_round_gap_ms;
    uint32_t ranging_rx_slice_ms;
    uint32_t ranging_rx_timeout_ms;
    uint32_t ranging_resp_delay_ms;
    uint32_t ranging_final_delay_ms;
    uint32_t ranging_auto_rx_delay_uus;
    uint8_t passive_ds_schedule;
    uint32_t passive_ds_slot_ms;
    uint32_t passive_ds_round_gap_ms;
    uint32_t passive_ds_rx_slice_ms;
    uint32_t passive_ds_rx_timeout_ms;
    uint32_t passive_ds_resp_delay_us;
    uint32_t passive_ds_final_delay_us;
    uint32_t passive_ds_auto_rx_delay_uus;
    uint8_t passive_ds_pipeline_mode;
    uint8_t passive_ds_solve_mode;
    uint32_t passive_ds_rolling_max_hz;
    bool passive_ds_calibration_enabled;
    uint32_t passive_ds_calibration_generation;
    int32_t passive_ds_anchor_bias_mm[APP_RUNTIME_CONFIG_MAX_ANCHORS];
    int32_t passive_ds_range_bias_mm[
        APP_RUNTIME_CONFIG_MAX_ANCHOR_PAIRS];
    uint8_t distance_test_peer_id;
    uint8_t distance_test_initiator_id;
    uint8_t distance_test_responder_id;
    uint32_t distance_test_interval_ms;
    uint32_t distance_test_rx_timeout_ms;
    uint32_t distance_test_resp_delay_ms;
    uint32_t distance_test_final_delay_ms;
    uint32_t distance_test_report_delay_ms;
    uint32_t distance_test_auto_rx_delay_uus;
    uint8_t calibration_method;
    uint8_t calibration_reference_id;
    uint8_t calibration_dut_id;
    uint8_t calibration_three_ids[APP_RUNTIME_CONFIG_CAL_THREE_COUNT];
    uint32_t calibration_known_distance_mm;
    uint32_t calibration_three_distance_0_1_mm;
    uint32_t calibration_three_distance_0_2_mm;
    uint32_t calibration_three_distance_1_2_mm;
    uint32_t calibration_sample_count;
    uint32_t calibration_summary_every;
    uint32_t calibration_min_interval_ms;
    uint32_t calibration_max_interval_ms;
    uint32_t calibration_rx_slice_ms;
    uint32_t calibration_slot_guard_us;
    bool uwb_enabled;
    bool bno085_accel_enabled;
    uint32_t bno085_accel_interval_ms;
    uint32_t bno085_log_interval_ms;
    bool gps_enabled;
    uint8_t radio_channel;
    uint32_t wireless_telemetry_port;
    bool from_nvs;
} app_runtime_config_t;

esp_err_t app_runtime_config_init(void);
const app_runtime_config_t *app_runtime_config_get(void);
esp_err_t app_runtime_config_reload(void);
esp_err_t app_runtime_config_save(const app_runtime_config_t *config);
esp_err_t app_runtime_config_clear(void);
void app_runtime_config_defaults(app_runtime_config_t *config);
void app_runtime_config_reset_flex_tdoa(app_runtime_config_t *config);
void app_runtime_config_reset_passive_ds_calibration(
    app_runtime_config_t *config);
bool app_runtime_config_validate(const app_runtime_config_t *config);
bool app_runtime_config_runtime_mode_valid(uint8_t mode);
const char *app_runtime_config_runtime_mode_to_string(uint8_t mode);
uint8_t app_runtime_config_runtime_mode_from_string(const char *text,
                                                    bool *ok);
size_t app_runtime_config_get_anchor_ids(uint8_t *ids, size_t capacity);
size_t app_runtime_config_anchor_pair_index(size_t first_index,
                                            size_t second_index);
void app_runtime_config_format_anchors(char *buffer, size_t buffer_size,
                                       const app_runtime_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* APP_RUNTIME_CONFIG_H */
