#ifndef UWB_PASSIVE_DS_RUNTIME_H
#define UWB_PASSIVE_DS_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "uwb_anchor_range_cache.h"
#include "uwb_dw3000.h"
#include "uwb_passive_ds_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

struct uwb_passive_ds_completed_exchange {
    bool valid;
    uint8_t initiator_id;
    uint16_t sequence;
    uint32_t slot_id;
    uint32_t responder_exchange_dtu;
    int32_t distance_mm;
    int32_t raw_distance_mm;
};

enum uwb_passive_ds_runtime_stage {
    UWB_PASSIVE_DS_RUNTIME_STAGE_POLL_TX = 0,
    UWB_PASSIVE_DS_RUNTIME_STAGE_RESPONSE_TX,
    UWB_PASSIVE_DS_RUNTIME_STAGE_FINAL_TX,
    UWB_PASSIVE_DS_RUNTIME_STAGE_FINAL_RX,
    UWB_PASSIVE_DS_RUNTIME_STAGE_CIA_READ,
    UWB_PASSIVE_DS_RUNTIME_STAGE_RX_REARM,
};

enum uwb_passive_ds_runtime_counter {
    UWB_PASSIVE_DS_RUNTIME_COUNTER_RX_REARM_FAILURE = 0,
    UWB_PASSIVE_DS_RUNTIME_COUNTER_RESPONSE_TIMEOUT,
    UWB_PASSIVE_DS_RUNTIME_COUNTER_FINAL_TIMEOUT,
    UWB_PASSIVE_DS_RUNTIME_COUNTER_INVALID_FRAME,
    UWB_PASSIVE_DS_RUNTIME_COUNTER_STATE_COLLISION,
    UWB_PASSIVE_DS_RUNTIME_COUNTER_SCHEDULE_ALARM,
    UWB_PASSIVE_DS_RUNTIME_COUNTER_SCHEDULE_OVERRUN,
    UWB_PASSIVE_DS_RUNTIME_COUNTER_COMPLETED_EXCHANGE,
    UWB_PASSIVE_DS_RUNTIME_COUNTER_RX_PHY_RETRY,
    UWB_PASSIVE_DS_RUNTIME_COUNTER_RX_RECOVERED_AFTER_PHY,
    UWB_PASSIVE_DS_RUNTIME_COUNTER_RX_TIMEOUT_AFTER_PHY,
};

void uwb_passive_ds_runtime_reset(bool deadline_pipeline_active);
esp_err_t uwb_passive_ds_runtime_start_solver(void);
bool uwb_passive_ds_runtime_submit_anchor_range(
    uint8_t anchor_a_id, uint8_t anchor_b_id, uint32_t slot_id,
    int32_t distance_mm);
bool uwb_passive_ds_runtime_submit_observation(
    uint8_t tag_id, uint8_t initiator_id, uint8_t responder_id,
    uint32_t session_id, uint32_t frame_id, int32_t difference_mm,
    uint16_t delay_ratio_q15,
    const struct uwb_passive_ds_anchor_position *initiator_position,
    const struct uwb_passive_ds_anchor_position *responder_position);
int32_t uwb_passive_ds_runtime_store_anchor_range(
    uint8_t anchor_a_id, uint8_t anchor_b_id, int32_t distance_mm,
    int32_t raw_distance_mm, uint32_t slot_id, uint16_t sequence);
int32_t uwb_passive_ds_runtime_anchor_range_mm(
    uint8_t anchor_a_id, uint8_t anchor_b_id);
bool uwb_passive_ds_runtime_next_anchor_range(
    uint8_t source_id, struct uwb_anchor_range_entry *range);
void uwb_passive_ds_runtime_store_completed_exchange(
    const struct uwb_passive_ds_completed_exchange *exchange);
bool uwb_passive_ds_runtime_get_completed_exchange(
    struct uwb_passive_ds_completed_exchange *exchange);
void uwb_passive_ds_runtime_record_stage(
    enum uwb_passive_ds_runtime_stage stage, int64_t start_host_us,
    int64_t end_host_us, bool success);
void uwb_passive_ds_runtime_increment(
    enum uwb_passive_ds_runtime_counter counter);
void uwb_passive_ds_runtime_get_stats(
    struct uwb_passive_ds_pipeline_stats *stats);

#ifdef __cplusplus
}
#endif

#endif
