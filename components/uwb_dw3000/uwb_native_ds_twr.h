#ifndef UWB_NATIVE_DS_TWR_H
#define UWB_NATIVE_DS_TWR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "uwb_dw3000.h"

#define UWB_NATIVE_DS_MAX_FRAME_LEN 64U
#define UWB_NATIVE_DS_MAX_ANCHORS UWB_NATIVE_DS_PIPELINE_MAX_ANCHORS

struct uwb_native_ds_rx_frame {
    uint8_t payload[UWB_NATIVE_DS_MAX_FRAME_LEN];
    uint16_t payload_len;
    uint64_t rx_timestamp;
};

struct uwb_native_ds_radio_ops {
    void *context;
    esp_err_t (*send_immediate_expect_rx)(
        void *context, const uint8_t *payload, size_t payload_len,
        uint32_t rx_after_tx_delay_uus, uint32_t rx_timeout_ms,
        uint64_t *tx_timestamp);
    esp_err_t (*send_delayed)(
        void *context, const uint8_t *payload, size_t payload_len,
        uint64_t due_timestamp, uint64_t *programmed_tx_timestamp,
        uint64_t *actual_tx_timestamp);
    esp_err_t (*send_delayed_expect_rx)(
        void *context, const uint8_t *payload, size_t payload_len,
        uint64_t due_timestamp, uint32_t rx_after_tx_delay_uus,
        uint32_t rx_timeout_ms, uint64_t *programmed_tx_timestamp,
        uint64_t *actual_tx_timestamp);
    esp_err_t (*receive)(void *context, struct uwb_native_ds_rx_frame *frame,
                         uint32_t timeout_ms);
    uint64_t (*add_delay_ms)(void *context, uint64_t timestamp,
                             uint32_t delay_ms);
    uint64_t (*programmed_tx_timestamp)(void *context,
                                        uint64_t due_timestamp);
    int64_t (*now_us)(void *context);
    void (*delay_ms)(void *context, uint32_t delay_ms);
    bool (*stop_requested)(void *context);
    void (*set_ready)(void *context);
    void (*consume_report)(void *context, bool tag_range,
                           uint8_t initiator_id, uint8_t responder_id,
                           uint32_t frame_id, double distance_m);
};

struct uwb_native_ds_config {
    uint8_t source_id;
    uint8_t tag_id;
    uint8_t anchor_count;
    uint8_t anchor_ids[UWB_NATIVE_DS_MAX_ANCHORS];
    uint32_t slot_ms;
    uint32_t round_gap_ms;
    uint32_t rx_slice_ms;
    uint32_t rx_timeout_ms;
    uint32_t response_delay_ms;
    uint32_t final_delay_ms;
    uint32_t auto_rx_delay_uus;
    double maximum_distance_m;
    bool fixed_geometry;
    uint32_t geometry_version;
    int32_t anchor_x_mm[UWB_NATIVE_DS_MAX_ANCHORS];
    int32_t anchor_y_mm[UWB_NATIVE_DS_MAX_ANCHORS];
    bool range_calibration_enabled;
    uint32_t range_calibration_generation;
    /* Measured range minus truth; subtracted only by the tag solver. */
    int32_t anchor_range_bias_mm[UWB_NATIVE_DS_MAX_ANCHORS];
};

esp_err_t uwb_native_ds_twr_run(const struct uwb_native_ds_config *config,
                                const struct uwb_native_ds_radio_ops *radio);
void uwb_native_ds_twr_get_stats(
    struct uwb_native_ds_pipeline_stats *stats);

#endif
