#ifndef UWB_FLEX_TDOA_RUNTIME_H
#define UWB_FLEX_TDOA_RUNTIME_H

#include <stdint.h>

#include "esp_err.h"
#include "uwb_anchor_range_cache.h"

#ifdef __cplusplus
extern "C" {
#endif

void uwb_flex_tdoa_runtime_reset(void);
esp_err_t uwb_flex_tdoa_runtime_start_solver(void);
bool uwb_flex_tdoa_runtime_reload_geometry(void);
bool uwb_flex_tdoa_runtime_submit_observation(
    uint8_t tag_id, uint8_t initiator_id, uint8_t responder_id,
    uint32_t slot_id, int32_t difference_mm, bool dynamic_geometry,
    int32_t initiator_x_mm, int32_t initiator_y_mm,
    int32_t responder_x_mm, int32_t responder_y_mm,
    uint32_t geometry_version, int32_t geometry_fit_rms_mm,
    bool geometry_all_rtk_fixed);
int32_t uwb_flex_tdoa_runtime_store_anchor_range(
    uint8_t anchor_a_id, uint8_t anchor_b_id, int32_t distance_mm,
    int32_t raw_distance_mm, uint32_t slot_id, uint16_t sequence);
int32_t uwb_flex_tdoa_runtime_anchor_range_mm(
    uint8_t anchor_a_id, uint8_t anchor_b_id);
uint32_t uwb_flex_tdoa_runtime_anchor_range_slot(
    uint8_t anchor_a_id, uint8_t anchor_b_id);
bool uwb_flex_tdoa_runtime_next_anchor_range(
    uint8_t source_id, struct uwb_anchor_range_entry *range);

#ifdef __cplusplus
}
#endif

#endif
