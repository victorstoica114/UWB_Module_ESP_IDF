#ifndef UWB_NATIVE_DS_POSITION_SOLVER_H
#define UWB_NATIVE_DS_POSITION_SOLVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UWB_NATIVE_DS_POSITION_MAX_ANCHORS 10U

struct uwb_native_ds_position_output {
    bool geometry_updated;
    bool position_valid;
    uint8_t tag_id;
    uint8_t anchor_count;
    uint16_t frame_id;
    uint32_t geometry_version;
    float anchor_x_m[UWB_NATIVE_DS_POSITION_MAX_ANCHORS];
    float anchor_y_m[UWB_NATIVE_DS_POSITION_MAX_ANCHORS];
    float geometry_fit_rms_m;
    float x_m;
    float y_m;
    float sigma_m;
    float rms_m;
    uint8_t observation_count;
    uint8_t iteration_count;
};

struct uwb_native_ds_position_solver {
    uint8_t tag_id;
    uint8_t anchor_count;
    uint8_t anchor_ids[UWB_NATIVE_DS_POSITION_MAX_ANCHORS];
    bool pair_valid[UWB_NATIVE_DS_POSITION_MAX_ANCHORS]
                   [UWB_NATIVE_DS_POSITION_MAX_ANCHORS];
    float pair_range_m[UWB_NATIVE_DS_POSITION_MAX_ANCHORS]
                      [UWB_NATIVE_DS_POSITION_MAX_ANCHORS];
    uint16_t pair_frame_id[UWB_NATIVE_DS_POSITION_MAX_ANCHORS]
                          [UWB_NATIVE_DS_POSITION_MAX_ANCHORS];
    uint64_t pair_cycle_mask;
    bool geometry_ready;
    uint32_t geometry_version;
    float anchor_x_m[UWB_NATIVE_DS_POSITION_MAX_ANCHORS];
    float anchor_y_m[UWB_NATIVE_DS_POSITION_MAX_ANCHORS];
    float geometry_fit_rms_m;
    bool tag_range_valid[UWB_NATIVE_DS_POSITION_MAX_ANCHORS];
    float tag_range_m[UWB_NATIVE_DS_POSITION_MAX_ANCHORS];
    uint16_t tag_frame_id[UWB_NATIVE_DS_POSITION_MAX_ANCHORS];
    bool position_valid;
    uint16_t last_position_frame_id;
    float position_x_m;
    float position_y_m;
};

bool uwb_native_ds_position_solver_init(
    struct uwb_native_ds_position_solver *solver, uint8_t tag_id,
    const uint8_t *anchor_ids, size_t anchor_count);

bool uwb_native_ds_position_solver_submit_anchor_range(
    struct uwb_native_ds_position_solver *solver, uint8_t initiator_id,
    uint8_t responder_id, uint16_t frame_id, float distance_m,
    struct uwb_native_ds_position_output *output);

bool uwb_native_ds_position_solver_submit_tag_range(
    struct uwb_native_ds_position_solver *solver, uint8_t tag_id,
    uint8_t anchor_id, uint16_t frame_id, float distance_m,
    struct uwb_native_ds_position_output *output);

#ifdef __cplusplus
}
#endif

#endif
