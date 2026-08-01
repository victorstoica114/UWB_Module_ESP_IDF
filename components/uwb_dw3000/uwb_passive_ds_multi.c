#include "uwb_passive_ds_multi.h"

#include <string.h>

enum {
    MULTI_POLL_FRAME_ID = 0,
    MULTI_POLL_LEN = 4,
    MULTI_RESPONSE_FRAME_ID = 0,
    MULTI_RESPONSE_INDEX = 4,
    MULTI_RESPONSE_REPLY_DTU = 5,
    MULTI_RESPONSE_LEN = 9,
    MULTI_FINAL_FRAME_ID = 0,
    MULTI_FINAL_POLL_TX = 4,
    MULTI_FINAL_FINAL_TX = 9,
    MULTI_FINAL_RESPONDER_COUNT = 14,
    MULTI_FINAL_RESPONDERS = 15,
    MULTI_FINAL_ENTRY_LEN = 6,
};

static void put_u32(uint8_t *payload, size_t offset, uint32_t value)
{
    payload[offset] = (uint8_t)(value & 0xffU);
    payload[offset + 1U] = (uint8_t)((value >> 8U) & 0xffU);
    payload[offset + 2U] = (uint8_t)((value >> 16U) & 0xffU);
    payload[offset + 3U] = (uint8_t)((value >> 24U) & 0xffU);
}

static uint32_t get_u32(const uint8_t *payload, size_t offset)
{
    return (uint32_t)payload[offset] |
           ((uint32_t)payload[offset + 1U] << 8U) |
           ((uint32_t)payload[offset + 2U] << 16U) |
           ((uint32_t)payload[offset + 3U] << 24U);
}

static void put_ts40(uint8_t *payload, size_t offset, uint64_t timestamp)
{
    for (size_t index = 0U; index < 5U; ++index) {
        payload[offset + index] =
            (uint8_t)((timestamp >> (8U * index)) & 0xffU);
    }
}

static uint64_t get_ts40(const uint8_t *payload, size_t offset)
{
    uint64_t timestamp = 0U;
    for (size_t index = 0U; index < 5U; ++index) {
        timestamp |= ((uint64_t)payload[offset + index]) << (8U * index);
    }
    return timestamp;
}

bool uwb_passive_ds_multi_build_plan(
    const uint8_t *anchor_ids, size_t anchor_count, uint32_t frame_id,
    struct uwb_passive_ds_multi_plan *plan)
{
    if (anchor_ids == NULL || plan == NULL || anchor_count < 3U ||
        anchor_count > UWB_PASSIVE_DS_MULTI_MAX_ANCHORS) {
        return false;
    }
    for (size_t first = 0U; first < anchor_count; ++first) {
        if (anchor_ids[first] == 0U) {
            return false;
        }
        for (size_t second = first + 1U; second < anchor_count; ++second) {
            if (anchor_ids[first] == anchor_ids[second]) {
                return false;
            }
        }
    }

    memset(plan, 0, sizeof(*plan));
    plan->frame_id = frame_id;
    const size_t initiator_index = frame_id % anchor_count;
    plan->initiator_id = anchor_ids[initiator_index];
    for (size_t offset = 1U; offset < anchor_count; ++offset) {
        const size_t anchor_index =
            (initiator_index + offset) % anchor_count;
        plan->responder_ids[plan->responder_count++] =
            anchor_ids[anchor_index];
    }
    return true;
}

int uwb_passive_ds_multi_responder_index(
    const struct uwb_passive_ds_multi_plan *plan, uint8_t responder_id)
{
    if (plan == NULL || responder_id == 0U) {
        return -1;
    }
    for (uint8_t index = 0U; index < plan->responder_count; ++index) {
        if (plan->responder_ids[index] == responder_id) {
            return (int)index;
        }
    }
    return -1;
}

uint32_t uwb_passive_ds_multi_response_delay_us(
    uint32_t first_response_delay_us, uint8_t responder_index)
{
    return first_response_delay_us +
           UWB_PASSIVE_DS_MULTI_RESPONSE_SPACING_US *
               (uint32_t)responder_index;
}

uint32_t uwb_passive_ds_multi_final_delay_from_poll_us(
    uint32_t first_response_delay_us, uint8_t responder_count,
    uint32_t final_delay_us)
{
    const uint32_t response_train_us = responder_count > 0U
        ? UWB_PASSIVE_DS_MULTI_RESPONSE_SPACING_US *
              ((uint32_t)responder_count - 1U)
        : 0U;
    return first_response_delay_us + response_train_us + final_delay_us;
}

size_t uwb_passive_ds_multi_poll_payload_size(size_t application_offset)
{
    return application_offset + MULTI_POLL_LEN;
}

size_t uwb_passive_ds_multi_response_payload_size(size_t application_offset)
{
    return application_offset + MULTI_RESPONSE_LEN;
}

size_t uwb_passive_ds_multi_final_payload_size(
    size_t application_offset, uint8_t responder_count)
{
    if (responder_count > UWB_PASSIVE_DS_MULTI_MAX_RESPONDERS) {
        return 0U;
    }
    return application_offset + MULTI_FINAL_RESPONDERS +
           (size_t)responder_count * MULTI_FINAL_ENTRY_LEN;
}

size_t uwb_passive_ds_multi_encode_poll(
    uint8_t *payload, size_t capacity, size_t application_offset,
    uint32_t frame_id)
{
    const size_t length =
        uwb_passive_ds_multi_poll_payload_size(application_offset);
    if (payload == NULL || capacity < length) {
        return 0U;
    }
    put_u32(payload, application_offset + MULTI_POLL_FRAME_ID, frame_id);
    return length;
}

bool uwb_passive_ds_multi_decode_poll(
    const uint8_t *payload, size_t payload_len, size_t application_offset,
    uint32_t *frame_id)
{
    if (payload == NULL || frame_id == NULL ||
        payload_len <
            uwb_passive_ds_multi_poll_payload_size(application_offset)) {
        return false;
    }
    *frame_id = get_u32(payload, application_offset + MULTI_POLL_FRAME_ID);
    return true;
}

size_t uwb_passive_ds_multi_encode_response(
    uint8_t *payload, size_t capacity, size_t application_offset,
    uint32_t frame_id, uint8_t responder_index, uint32_t reply_dtu)
{
    const size_t length =
        uwb_passive_ds_multi_response_payload_size(application_offset);
    if (payload == NULL || capacity < length || reply_dtu == 0U ||
        responder_index >= UWB_PASSIVE_DS_MULTI_MAX_RESPONDERS) {
        return 0U;
    }
    put_u32(payload, application_offset + MULTI_RESPONSE_FRAME_ID,
            frame_id);
    payload[application_offset + MULTI_RESPONSE_INDEX] = responder_index;
    put_u32(payload, application_offset + MULTI_RESPONSE_REPLY_DTU,
            reply_dtu);
    return length;
}

bool uwb_passive_ds_multi_decode_response(
    const uint8_t *payload, size_t payload_len, size_t application_offset,
    uint32_t *frame_id, uint8_t *responder_index, uint32_t *reply_dtu)
{
    if (payload == NULL || frame_id == NULL || responder_index == NULL ||
        reply_dtu == NULL ||
        payload_len < uwb_passive_ds_multi_response_payload_size(
                          application_offset)) {
        return false;
    }
    *frame_id = get_u32(payload,
                        application_offset + MULTI_RESPONSE_FRAME_ID);
    *responder_index = payload[application_offset + MULTI_RESPONSE_INDEX];
    *reply_dtu = get_u32(payload,
                         application_offset + MULTI_RESPONSE_REPLY_DTU);
    return *responder_index < UWB_PASSIVE_DS_MULTI_MAX_RESPONDERS &&
           *reply_dtu != 0U;
}

size_t uwb_passive_ds_multi_encode_final(
    uint8_t *payload, size_t capacity, size_t application_offset,
    const struct uwb_passive_ds_multi_final *final)
{
    if (payload == NULL || final == NULL ||
        final->responder_count > UWB_PASSIVE_DS_MULTI_MAX_RESPONDERS) {
        return 0U;
    }
    const size_t length = uwb_passive_ds_multi_final_payload_size(
        application_offset, final->responder_count);
    if (capacity < length) {
        return 0U;
    }
    put_u32(payload, application_offset + MULTI_FINAL_FRAME_ID,
            final->frame_id);
    put_ts40(payload, application_offset + MULTI_FINAL_POLL_TX,
             final->initiator_poll_tx);
    put_ts40(payload, application_offset + MULTI_FINAL_FINAL_TX,
             final->initiator_final_tx);
    payload[application_offset + MULTI_FINAL_RESPONDER_COUNT] =
        final->responder_count;
    for (uint8_t index = 0U; index < final->responder_count; ++index) {
        const size_t offset = application_offset + MULTI_FINAL_RESPONDERS +
                              (size_t)index * MULTI_FINAL_ENTRY_LEN;
        payload[offset] = final->responders[index].responder_id;
        put_ts40(payload, offset + 1U,
                 final->responders[index].initiator_response_rx);
    }
    return length;
}

bool uwb_passive_ds_multi_decode_final(
    const uint8_t *payload, size_t payload_len, size_t application_offset,
    struct uwb_passive_ds_multi_final *final)
{
    if (payload == NULL || final == NULL ||
        payload_len < application_offset + MULTI_FINAL_RESPONDERS) {
        return false;
    }
    const uint8_t responder_count =
        payload[application_offset + MULTI_FINAL_RESPONDER_COUNT];
    const size_t expected_length = uwb_passive_ds_multi_final_payload_size(
        application_offset, responder_count);
    if (expected_length == 0U || payload_len < expected_length) {
        return false;
    }

    memset(final, 0, sizeof(*final));
    final->frame_id =
        get_u32(payload, application_offset + MULTI_FINAL_FRAME_ID);
    final->initiator_poll_tx =
        get_ts40(payload, application_offset + MULTI_FINAL_POLL_TX);
    final->initiator_final_tx =
        get_ts40(payload, application_offset + MULTI_FINAL_FINAL_TX);
    final->responder_count = responder_count;
    for (uint8_t index = 0U; index < responder_count; ++index) {
        const size_t offset = application_offset + MULTI_FINAL_RESPONDERS +
                              (size_t)index * MULTI_FINAL_ENTRY_LEN;
        final->responders[index].responder_id = payload[offset];
        final->responders[index].initiator_response_rx =
            get_ts40(payload, offset + 1U);
        if (final->responders[index].responder_id == 0U ||
            final->responders[index].initiator_response_rx == 0U) {
            return false;
        }
    }
    return final->initiator_poll_tx != 0U &&
           final->initiator_final_tx != 0U;
}
