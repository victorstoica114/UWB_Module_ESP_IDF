#include "uwb_passive_ds_runtime.h"

#include <stddef.h>
#include <string.h>

#include "passive_ds_solver_service.h"

struct uwb_passive_ds_runtime_state {
    struct uwb_anchor_range_cache anchor_range_cache;
    struct uwb_passive_ds_completed_exchange completed_exchange;
    struct uwb_passive_ds_pipeline_stats pipeline_stats;
};

/* Passive DS-TWR owns all mutable cross-frame state in this context. */
static struct uwb_passive_ds_runtime_state s_runtime;

void uwb_passive_ds_runtime_reset(bool deadline_pipeline_active)
{
    memset(&s_runtime, 0, sizeof(s_runtime));
    s_runtime.pipeline_stats.deadline_pipeline_active =
        deadline_pipeline_active;
    (void)passive_ds_solver_service_reset();
}

esp_err_t uwb_passive_ds_runtime_start_solver(void)
{
    return passive_ds_solver_service_start();
}

bool uwb_passive_ds_runtime_submit_anchor_range(
    uint8_t anchor_a_id, uint8_t anchor_b_id, uint32_t slot_id,
    int32_t distance_mm)
{
    return passive_ds_solver_service_submit_anchor_range(
        anchor_a_id, anchor_b_id, slot_id, distance_mm);
}

bool uwb_passive_ds_runtime_submit_observation(
    uint8_t tag_id, uint8_t initiator_id, uint8_t responder_id,
    uint32_t slot_id, int32_t difference_mm, uint16_t delay_ratio_q15)
{
    return passive_ds_solver_service_submit_observation(
        tag_id, initiator_id, responder_id, slot_id, difference_mm,
        delay_ratio_q15);
}

int32_t uwb_passive_ds_runtime_store_anchor_range(
    uint8_t anchor_a_id, uint8_t anchor_b_id, int32_t distance_mm,
    int32_t raw_distance_mm, uint32_t slot_id, uint16_t sequence)
{
    return uwb_anchor_range_cache_store(
        &s_runtime.anchor_range_cache, anchor_a_id, anchor_b_id,
        sequence, slot_id, distance_mm, raw_distance_mm);
}

int32_t uwb_passive_ds_runtime_anchor_range_mm(
    uint8_t anchor_a_id, uint8_t anchor_b_id)
{
    return uwb_anchor_range_cache_get_mm(
        &s_runtime.anchor_range_cache, anchor_a_id, anchor_b_id);
}

bool uwb_passive_ds_runtime_next_anchor_range(
    uint8_t source_id, struct uwb_anchor_range_entry *range)
{
    return uwb_anchor_range_cache_next_from(
        &s_runtime.anchor_range_cache, source_id, range);
}

void uwb_passive_ds_runtime_store_completed_exchange(
    const struct uwb_passive_ds_completed_exchange *exchange)
{
    if (exchange != NULL) {
        s_runtime.completed_exchange = *exchange;
    }
}

bool uwb_passive_ds_runtime_get_completed_exchange(
    struct uwb_passive_ds_completed_exchange *exchange)
{
    if (exchange == NULL || !s_runtime.completed_exchange.valid) {
        return false;
    }
    *exchange = s_runtime.completed_exchange;
    return true;
}

static struct uwb_passive_ds_stage_stats *stage_stats(
    enum uwb_passive_ds_runtime_stage stage)
{
    switch (stage) {
    case UWB_PASSIVE_DS_RUNTIME_STAGE_POLL_TX:
        return &s_runtime.pipeline_stats.poll_tx;
    case UWB_PASSIVE_DS_RUNTIME_STAGE_RESPONSE_TX:
        return &s_runtime.pipeline_stats.response_tx;
    case UWB_PASSIVE_DS_RUNTIME_STAGE_FINAL_TX:
        return &s_runtime.pipeline_stats.final_tx;
    case UWB_PASSIVE_DS_RUNTIME_STAGE_FINAL_RX:
        return &s_runtime.pipeline_stats.final_rx;
    case UWB_PASSIVE_DS_RUNTIME_STAGE_CIA_READ:
        return &s_runtime.pipeline_stats.cia_read;
    case UWB_PASSIVE_DS_RUNTIME_STAGE_RX_REARM:
        return &s_runtime.pipeline_stats.rx_rearm;
    default:
        return NULL;
    }
}

void uwb_passive_ds_runtime_record_stage(
    enum uwb_passive_ds_runtime_stage stage, int64_t start_host_us,
    int64_t end_host_us, bool success)
{
    struct uwb_passive_ds_stage_stats *stats = stage_stats(stage);
    if (stats == NULL || start_host_us <= 0 || end_host_us <= 0) {
        return;
    }

    const int64_t elapsed_us = end_host_us - start_host_us;
    const uint32_t duration_us =
        elapsed_us <= 0
            ? 0U
            : elapsed_us > (int64_t)UINT32_MAX
                  ? UINT32_MAX
                  : (uint32_t)elapsed_us;
    stats->count++;
    if (!success) {
        stats->failure_count++;
    }
    stats->last_duration_us = duration_us;
    if (duration_us > stats->max_duration_us) {
        stats->max_duration_us = duration_us;
    }
    stats->total_duration_us += duration_us;
    stats->last_event_host_us = end_host_us;
}

void uwb_passive_ds_runtime_increment(
    enum uwb_passive_ds_runtime_counter counter)
{
    switch (counter) {
    case UWB_PASSIVE_DS_RUNTIME_COUNTER_RX_REARM_FAILURE:
        s_runtime.pipeline_stats.rx_rearm_failure_count++;
        break;
    case UWB_PASSIVE_DS_RUNTIME_COUNTER_RESPONSE_TIMEOUT:
        s_runtime.pipeline_stats.response_timeout_count++;
        break;
    case UWB_PASSIVE_DS_RUNTIME_COUNTER_FINAL_TIMEOUT:
        s_runtime.pipeline_stats.final_timeout_count++;
        break;
    case UWB_PASSIVE_DS_RUNTIME_COUNTER_INVALID_FRAME:
        s_runtime.pipeline_stats.invalid_frame_count++;
        break;
    case UWB_PASSIVE_DS_RUNTIME_COUNTER_STATE_COLLISION:
        s_runtime.pipeline_stats.state_collision_count++;
        break;
    case UWB_PASSIVE_DS_RUNTIME_COUNTER_SCHEDULE_ALARM:
        s_runtime.pipeline_stats.schedule_alarm_count++;
        break;
    case UWB_PASSIVE_DS_RUNTIME_COUNTER_SCHEDULE_OVERRUN:
        s_runtime.pipeline_stats.schedule_overrun_count++;
        break;
    case UWB_PASSIVE_DS_RUNTIME_COUNTER_COMPLETED_EXCHANGE:
        s_runtime.pipeline_stats.completed_exchange_count++;
        break;
    default:
        break;
    }
}

void uwb_passive_ds_runtime_get_stats(
    struct uwb_passive_ds_pipeline_stats *stats)
{
    if (stats != NULL) {
        memcpy(stats, &s_runtime.pipeline_stats, sizeof(*stats));
    }
}
