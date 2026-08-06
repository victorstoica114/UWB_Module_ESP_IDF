#include "uwb_flex_tdoa_runtime.h"

#include "flextdoa_solver_service.h"

/* FlexTDOA owns this cache. No other protocol can observe or reset it. */
static struct uwb_anchor_range_cache s_anchor_range_cache;

void uwb_flex_tdoa_runtime_reset(void)
{
    uwb_anchor_range_cache_reset(&s_anchor_range_cache);
    (void)flextdoa_solver_service_reset();
}

esp_err_t uwb_flex_tdoa_runtime_start_solver(void)
{
    return flextdoa_solver_service_start();
}

bool uwb_flex_tdoa_runtime_reload_geometry(void)
{
    return flextdoa_solver_service_reload_geometry();
}

bool uwb_flex_tdoa_runtime_submit_observation(
    uint8_t tag_id, uint8_t initiator_id, uint8_t responder_id,
    uint32_t slot_id, int32_t difference_mm, bool dynamic_geometry,
    int32_t initiator_x_mm, int32_t initiator_y_mm,
    int32_t responder_x_mm, int32_t responder_y_mm,
    uint32_t geometry_version, int32_t geometry_fit_rms_mm,
    bool geometry_all_rtk_fixed)
{
    return flextdoa_solver_service_submit_observation(
        tag_id, initiator_id, responder_id, slot_id, difference_mm,
        dynamic_geometry, initiator_x_mm, initiator_y_mm,
        responder_x_mm, responder_y_mm, geometry_version,
        geometry_fit_rms_mm, geometry_all_rtk_fixed);
}

int32_t uwb_flex_tdoa_runtime_store_anchor_range(
    uint8_t anchor_a_id, uint8_t anchor_b_id, int32_t distance_mm,
    int32_t raw_distance_mm, uint32_t slot_id, uint16_t sequence)
{
    return uwb_anchor_range_cache_store(
        &s_anchor_range_cache, anchor_a_id, anchor_b_id, sequence,
        slot_id, distance_mm, raw_distance_mm);
}

int32_t uwb_flex_tdoa_runtime_anchor_range_mm(
    uint8_t anchor_a_id, uint8_t anchor_b_id)
{
    return uwb_anchor_range_cache_get_mm(
        &s_anchor_range_cache, anchor_a_id, anchor_b_id);
}

uint32_t uwb_flex_tdoa_runtime_anchor_range_slot(
    uint8_t anchor_a_id, uint8_t anchor_b_id)
{
    return uwb_anchor_range_cache_get_slot_id(
        &s_anchor_range_cache, anchor_a_id, anchor_b_id);
}

bool uwb_flex_tdoa_runtime_next_anchor_range(
    uint8_t source_id, struct uwb_anchor_range_entry *range)
{
    return uwb_anchor_range_cache_next_from(
        &s_anchor_range_cache, source_id, range);
}
