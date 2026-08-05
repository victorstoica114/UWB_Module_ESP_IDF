#ifndef UWB_DW3000_H
#define UWB_DW3000_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

enum uwb_dw3000_status {
    UWB_DW3000_STATUS_IDLE = 0,
    UWB_DW3000_STATUS_INITIALIZING,
    UWB_DW3000_STATUS_PROBING,
    UWB_DW3000_STATUS_READY,
    UWB_DW3000_STATUS_FAILED,
};

struct uwb_passive_ds_stage_stats {
    uint32_t count;
    uint32_t failure_count;
    uint32_t last_duration_us;
    uint32_t max_duration_us;
    uint64_t total_duration_us;
    int64_t last_event_host_us;
};

struct uwb_passive_ds_pipeline_stats {
    bool deadline_pipeline_active;
    uint32_t completed_exchange_count;
    uint32_t response_timeout_count;
    uint32_t final_timeout_count;
    uint32_t invalid_frame_count;
    uint32_t state_collision_count;
    uint32_t schedule_alarm_count;
    uint32_t schedule_overrun_count;
    uint32_t rx_rearm_failure_count;
    struct uwb_passive_ds_stage_stats poll_tx;
    struct uwb_passive_ds_stage_stats response_tx;
    struct uwb_passive_ds_stage_stats final_tx;
    struct uwb_passive_ds_stage_stats final_rx;
    struct uwb_passive_ds_stage_stats cia_read;
    struct uwb_passive_ds_stage_stats rx_rearm;
};

#define UWB_NATIVE_DS_PIPELINE_MAX_ANCHORS 10U

struct uwb_native_ds_tag_anchor_stats {
    uint8_t anchor_id;
    uint32_t attempt_count;
    uint32_t poll_tx_error_count;
    uint32_t response_timeout_count;
    uint32_t response_rx_error_count;
    uint32_t final_tx_error_count;
    uint32_t result_timeout_count;
    uint32_t result_rx_error_count;
    uint32_t completed_range_count;
};

struct uwb_native_ds_pipeline_stats {
    uint32_t poll_tx_count;
    uint32_t poll_rx_count;
    uint32_t response_tx_count;
    uint32_t response_rx_count;
    uint32_t final_tx_count;
    uint32_t final_rx_count;
    uint32_t result_tx_count;
    uint32_t result_rx_count;
    uint32_t completed_range_count;
    uint32_t rx_timeout_count;
    uint32_t poll_tx_error_count;
    uint32_t response_timeout_count;
    uint32_t response_rx_error_count;
    uint32_t response_tx_error_count;
    uint32_t final_timeout_count;
    uint32_t final_rx_error_count;
    uint32_t final_tx_error_count;
    uint32_t result_timeout_count;
    uint32_t result_rx_error_count;
    uint32_t result_tx_error_count;
    uint32_t invalid_frame_count;
    uint32_t crc_error_count;
    uint32_t delayed_tx_error_count;
    uint32_t rejected_range_count;
    uint32_t slot_overrun_count;
    uint32_t recovered_rx_error_count;
    uint32_t complete_frame_count;
    uint32_t incomplete_frame_count;
    uint32_t last_frame_missing_anchor_mask;
    uint32_t rx_phy_error_count;
    uint32_t rx_frame_sync_loss_count;
    uint32_t rx_phr_error_count;
    uint32_t rx_fcs_error_count;
    uint32_t rx_overrun_count;
    uint32_t rx_cia_error_count;
    uint32_t rx_filter_rejection_count;
    uint32_t rx_cp_error_count;
    uint32_t rx_timestamp_cia_invalid_count;
    int32_t last_distance_mm;
    uint8_t tag_anchor_count;
    struct uwb_native_ds_tag_anchor_stats
        tag_anchors[UWB_NATIVE_DS_PIPELINE_MAX_ANCHORS];
};

esp_err_t uwb_dw3000_start(void);
esp_err_t uwb_dw3000_start_distance_test(void);
esp_err_t uwb_dw3000_start_calibration(void);
esp_err_t uwb_dw3000_start_anchor_survey(void);
esp_err_t uwb_dw3000_start_ranging(void);
esp_err_t uwb_dw3000_start_flex_tdoa(void);
esp_err_t uwb_dw3000_start_passive_ds_twr(void);
bool uwb_dw3000_hot_switch_mode_supported(uint8_t app_runtime_mode);
esp_err_t uwb_dw3000_request_runtime_switch(uint8_t app_runtime_mode);
bool uwb_dw3000_runtime_switch_in_progress(void);
uint32_t uwb_dw3000_get_runtime_switch_count(void);
uint32_t uwb_dw3000_get_last_runtime_switch_ms(void);
esp_err_t uwb_dw3000_hold_in_reset(void);
bool uwb_dw3000_is_ready(void);
enum uwb_dw3000_status uwb_dw3000_get_status(void);
const char *uwb_dw3000_status_to_string(enum uwb_dw3000_status status);
uint32_t uwb_dw3000_get_device_id(void);
uint32_t uwb_dw3000_get_spi_clock_hz(void);
uint8_t uwb_dw3000_get_source_id(void);
uint32_t uwb_dw3000_get_tx_count(void);
uint32_t uwb_dw3000_get_tx_error_count(void);
uint32_t uwb_dw3000_get_rx_count(void);
uint32_t uwb_dw3000_get_rx_error_count(void);
uint32_t uwb_dw3000_get_rx_ignored_count(void);
uint8_t uwb_dw3000_get_last_rx_source_id(void);
uint32_t uwb_dw3000_get_last_rx_sequence(void);
uint16_t uwb_dw3000_get_antenna_delay(void);
void uwb_dw3000_get_passive_ds_pipeline_stats(
    struct uwb_passive_ds_pipeline_stats *stats);
void uwb_dw3000_get_native_ds_pipeline_stats(
    struct uwb_native_ds_pipeline_stats *stats);

#ifdef __cplusplus
}
#endif

#endif
