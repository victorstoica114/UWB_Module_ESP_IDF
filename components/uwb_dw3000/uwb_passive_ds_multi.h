#ifndef UWB_PASSIVE_DS_MULTI_H
#define UWB_PASSIVE_DS_MULTI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "uwb_config.h"

#define UWB_PASSIVE_DS_MULTI_MAX_ANCHORS 4U
#define UWB_PASSIVE_DS_MULTI_MAX_RESPONDERS \
    (UWB_PASSIVE_DS_MULTI_MAX_ANCHORS - 1U)
#define UWB_PASSIVE_DS_MULTI_RESPONSE_SPACING_US \
    APP_UWB_PASSIVE_DS_MULTI_RESPONSE_SPACING_US

/* One broadcast POLL, N-1 staggered RESP frames and one broadcast FINAL. */
struct uwb_passive_ds_multi_plan {
    uint32_t frame_id;
    uint8_t initiator_id;
    uint8_t responder_count;
    uint8_t responder_ids[UWB_PASSIVE_DS_MULTI_MAX_RESPONDERS];
};

struct uwb_passive_ds_multi_final_entry {
    uint8_t responder_id;
    uint64_t initiator_response_rx;
};

struct uwb_passive_ds_multi_final {
    uint32_t frame_id;
    uint64_t initiator_poll_tx;
    uint64_t initiator_final_tx;
    uint8_t responder_count;
    struct uwb_passive_ds_multi_final_entry
        responders[UWB_PASSIVE_DS_MULTI_MAX_RESPONDERS];
};

bool uwb_passive_ds_multi_build_plan(
    const uint8_t *anchor_ids, size_t anchor_count, uint32_t frame_id,
    struct uwb_passive_ds_multi_plan *plan);
int uwb_passive_ds_multi_responder_index(
    const struct uwb_passive_ds_multi_plan *plan, uint8_t responder_id);
uint32_t uwb_passive_ds_multi_response_delay_us(
    uint32_t first_response_delay_us, uint8_t responder_index);
uint32_t uwb_passive_ds_multi_final_delay_from_poll_us(
    uint32_t first_response_delay_us, uint8_t responder_count,
    uint32_t final_delay_us);

size_t uwb_passive_ds_multi_encode_poll(
    uint8_t *payload, size_t capacity, size_t application_offset,
    uint32_t frame_id);
bool uwb_passive_ds_multi_decode_poll(
    const uint8_t *payload, size_t payload_len, size_t application_offset,
    uint32_t *frame_id);
size_t uwb_passive_ds_multi_encode_response(
    uint8_t *payload, size_t capacity, size_t application_offset,
    uint32_t frame_id, uint8_t responder_index, uint32_t reply_dtu);
bool uwb_passive_ds_multi_decode_response(
    const uint8_t *payload, size_t payload_len, size_t application_offset,
    uint32_t *frame_id, uint8_t *responder_index, uint32_t *reply_dtu);
size_t uwb_passive_ds_multi_encode_final(
    uint8_t *payload, size_t capacity, size_t application_offset,
    const struct uwb_passive_ds_multi_final *final);
bool uwb_passive_ds_multi_decode_final(
    const uint8_t *payload, size_t payload_len, size_t application_offset,
    struct uwb_passive_ds_multi_final *final);

size_t uwb_passive_ds_multi_poll_payload_size(size_t application_offset);
size_t uwb_passive_ds_multi_response_payload_size(
    size_t application_offset);
size_t uwb_passive_ds_multi_final_payload_size(
    size_t application_offset, uint8_t responder_count);

#endif
