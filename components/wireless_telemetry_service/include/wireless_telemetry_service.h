#ifndef WIRELESS_TELEMETRY_SERVICE_H
#define WIRELESS_TELEMETRY_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

enum wireless_telemetry_status {
    WIRELESS_TELEMETRY_STATUS_DISABLED = 0,
    WIRELESS_TELEMETRY_STATUS_IDLE,
    WIRELESS_TELEMETRY_STATUS_WAITING_FOR_WIFI,
    WIRELESS_TELEMETRY_STATUS_CONNECTING,
    WIRELESS_TELEMETRY_STATUS_CONNECTED,
    WIRELESS_TELEMETRY_STATUS_FAILED,
};

esp_err_t wireless_telemetry_service_start(void);
enum wireless_telemetry_status wireless_telemetry_service_get_status(void);
const char *wireless_telemetry_service_status_to_string(
    enum wireless_telemetry_status status);
bool wireless_telemetry_service_is_connected(void);
const char *wireless_telemetry_service_get_target(void);
uint16_t wireless_telemetry_service_get_port(void);
uint32_t wireless_telemetry_service_get_dropped_count(void);
uint32_t wireless_telemetry_service_get_drop_full_count(void);
uint32_t wireless_telemetry_service_get_drop_mutex_count(void);
uint32_t wireless_telemetry_service_get_drop_format_count(void);
uint32_t wireless_telemetry_service_get_queue_depth(void);
uint32_t wireless_telemetry_service_get_queue_high_water(void);
uint32_t wireless_telemetry_service_get_binary_frame_count(void);
uint32_t wireless_telemetry_service_get_binary_sample_count(void);
uint32_t wireless_telemetry_service_get_text_frame_count(void);
uint32_t wireless_telemetry_service_get_connect_count(void);
uint32_t wireless_telemetry_service_get_send_failure_count(void);
uint32_t wireless_telemetry_service_get_send_timeout_count(void);
uint32_t wireless_telemetry_service_get_socket_close_count(void);
uint32_t wireless_telemetry_service_get_last_send_ms(void);
uint32_t wireless_telemetry_service_get_max_send_ms(void);
int wireless_telemetry_service_get_last_error(void);
bool wireless_telemetry_service_submit(const char *topic, const char *format,
                                       ...)
    __attribute__((format(printf, 2, 3)));
bool wireless_telemetry_service_submit_bno085_accel(
    int32_t x_milli_mps2, int32_t y_milli_mps2, int32_t z_milli_mps2,
    uint8_t accuracy, uint32_t report_count);
bool wireless_telemetry_service_submit_flex_tdoa_observation(
    uint8_t tag_id, uint8_t initiator_id, uint8_t responder_id,
    uint8_t responder_index, uint16_t sequence, uint32_t slot_id,
    int32_t diff_mm, int32_t raw_diff_mm, int32_t anchor_distance_mm,
    int32_t cfo_correction_mm, int32_t raw_cfo_ppb,
    int32_t estimated_cfo_ppb, int32_t applied_cfo_ppb,
    uint32_t processing_dtu, uint16_t cfo_sample_count,
    uint8_t cfo_flags);
bool wireless_telemetry_service_submit_flex_anchor_range(
    uint8_t initiator_id, uint8_t responder_id, uint16_t sequence,
    uint32_t slot_id, int32_t distance_mm, int32_t raw_distance_mm);
bool wireless_telemetry_service_submit_flex_position(
    uint8_t tag_id, uint32_t slot_id, int32_t x_mm, int32_t y_mm,
    int32_t sigma_mm, int32_t rms_mm, uint16_t observation_count,
    uint8_t anchor_count, uint32_t geometry_version);
bool wireless_telemetry_service_submit_flex_geometry(
    uint8_t tag_id, uint8_t anchor_id, uint8_t anchor_count,
    uint32_t geometry_version, int32_t x_mm, int32_t y_mm,
    int32_t fit_rms_mm, bool dynamic, bool all_rtk_fixed);
bool wireless_telemetry_service_submit_passive_ds_observation(
    uint8_t tag_id, uint8_t initiator_id, uint8_t responder_id,
    uint8_t responder_index, uint16_t sequence, uint32_t slot_id,
    int32_t diff_mm, int32_t raw_diff_mm, int32_t anchor_distance_mm,
    int32_t cfo_correction_mm, int32_t clock_offset_ppb,
    uint32_t reply_delay_us, uint8_t range_source,
    uint16_t range_age_slots);
bool wireless_telemetry_service_submit_passive_ds_anchor_range(
    uint8_t initiator_id, uint8_t responder_id, uint16_t sequence,
    uint32_t slot_id, int32_t distance_mm, int32_t raw_distance_mm);
bool wireless_telemetry_service_submit_native_ds_anchor_range(
    uint8_t initiator_id, uint8_t responder_id, uint16_t sequence,
    uint32_t slot_id, int32_t distance_mm, int32_t raw_distance_mm);
bool wireless_telemetry_service_submit_native_ds_tag_range(
    uint8_t tag_id, uint8_t anchor_id, uint16_t sequence,
    uint32_t slot_id, int32_t distance_mm, int32_t raw_distance_mm);
bool wireless_telemetry_service_submit_native_ds_position(
    uint8_t tag_id, uint32_t frame_id, int32_t x_mm, int32_t y_mm,
    int32_t sigma_mm, int32_t rms_mm, uint16_t observation_count,
    uint8_t anchor_count, uint32_t geometry_version);
bool wireless_telemetry_service_submit_native_ds_geometry(
    uint8_t anchor_id, uint8_t anchor_count, uint32_t geometry_version,
    int32_t x_mm, int32_t y_mm, int32_t fit_rms_mm, bool dynamic,
    bool all_rtk_fixed);
bool wireless_telemetry_service_submit_passive_ds_position(
    uint8_t tag_id, uint32_t slot_id, int32_t filtered_x_mm,
    int32_t filtered_y_mm, int32_t raw_x_mm, int32_t raw_y_mm,
    int32_t sigma_mm, int32_t rms_mm, uint16_t observation_count,
    uint8_t anchor_count, uint32_t geometry_version,
    bool independent_frame, bool complete_superframe,
    bool filter_correction, uint32_t solver_update_count,
    uint32_t independent_frame_count, uint16_t batch_span_ms,
    uint16_t batch_max_age_ms, uint16_t observation_mask,
    uint16_t rejection_reason_mask, uint16_t rejected_since_last,
    uint32_t position_rejected_count, bool unfiltered_esp_solver);
bool wireless_telemetry_service_submit_passive_ds_geometry(
    uint8_t anchor_id, uint8_t anchor_count, uint32_t geometry_version,
    int32_t x_mm, int32_t y_mm, int32_t fit_rms_mm, bool dynamic,
    bool all_rtk_fixed);

#ifdef __cplusplus
}
#endif

#endif /* WIRELESS_TELEMETRY_SERVICE_H */
