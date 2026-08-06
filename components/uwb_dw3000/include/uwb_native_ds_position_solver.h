#ifndef UWB_NATIVE_DS_POSITION_SOLVER_H
#define UWB_NATIVE_DS_POSITION_SOLVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UWB_NATIVE_DS_POSITION_MAX_ANCHORS 10U

/*
 * Native DS-TWR positions use the fixed survey frame supplied by the
 * application (currently GPS RTK ENU).  Anchor-to-anchor ranges never move
 * this geometry; they are retained only to report a ranging residual.
 */
struct uwb_native_ds_position_output {
    bool geometry_updated;
    bool position_valid;
    uint8_t tag_id;
    uint8_t anchor_count;
    uint32_t frame_id;
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
    uint32_t geometry_version;
    float anchor_x_m[UWB_NATIVE_DS_POSITION_MAX_ANCHORS];
    float anchor_y_m[UWB_NATIVE_DS_POSITION_MAX_ANCHORS];

    bool pair_valid[UWB_NATIVE_DS_POSITION_MAX_ANCHORS]
                   [UWB_NATIVE_DS_POSITION_MAX_ANCHORS];
    float pair_range_m[UWB_NATIVE_DS_POSITION_MAX_ANCHORS]
                      [UWB_NATIVE_DS_POSITION_MAX_ANCHORS];
    uint64_t pair_cycle_mask;

    bool tag_frame_active;
    bool tag_frame_emitted;
    uint32_t tag_frame_id;
    uint16_t tag_range_mask;
    float tag_range_m[UWB_NATIVE_DS_POSITION_MAX_ANCHORS];
};

bool uwb_native_ds_position_solver_init(
    struct uwb_native_ds_position_solver *solver, uint8_t tag_id,
    const uint8_t *anchor_ids, const float *anchor_x_m,
    const float *anchor_y_m, size_t anchor_count,
    uint32_t geometry_version);

/* Returns the current fixed or packet-time mobile geometry. */
bool uwb_native_ds_position_solver_geometry(
    const struct uwb_native_ds_position_solver *solver,
    struct uwb_native_ds_position_output *output);

bool uwb_native_ds_position_solver_update_geometry(
    struct uwb_native_ds_position_solver *solver,
    const float *anchor_x_m, const float *anchor_y_m,
    uint32_t geometry_version);

bool uwb_native_ds_position_solver_submit_anchor_range(
    struct uwb_native_ds_position_solver *solver, uint8_t initiator_id,
    uint8_t responder_id, uint32_t frame_id, float distance_m,
    struct uwb_native_ds_position_output *output);

bool uwb_native_ds_position_solver_submit_tag_range(
    struct uwb_native_ds_position_solver *solver, uint8_t tag_id,
    uint8_t anchor_id, uint32_t frame_id, float distance_m,
    struct uwb_native_ds_position_output *output);

#ifdef __cplusplus
}
#endif

#endif
