#include "flextdoa_protocol.h"

#include <math.h>
#include <string.h>

const struct flextdoa_timing FLEXTDOA_PAPER_TIMING = {
    .guard_us = 250U,
    .request_subslot_us = 2000U,
    .request_process_us = 250U,
    .response_subslot_us = 250U,
    .response_process_us = 600U,
};

static void put_u16_le(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8U);
}

static void put_u32_le(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8U);
    destination[2] = (uint8_t)(value >> 16U);
    destination[3] = (uint8_t)(value >> 24U);
}

static uint16_t get_u16_le(const uint8_t *source)
{
    return (uint16_t)((uint16_t)source[0] |
                      ((uint16_t)source[1] << 8U));
}

static uint32_t get_u32_le(const uint8_t *source)
{
    return (uint32_t)source[0] |
           ((uint32_t)source[1] << 8U) |
           ((uint32_t)source[2] << 16U) |
           ((uint32_t)source[3] << 24U);
}

static bool message_type_valid(enum flextdoa_message_type type)
{
    return type == FLEXTDOA_MESSAGE_REQUEST ||
           type == FLEXTDOA_MESSAGE_RESPONSE;
}

uint32_t flextdoa_slot_duration_us(
    const struct flextdoa_timing *timing, uint8_t responder_count)
{
    if (timing == NULL || responder_count > FLEXTDOA_MAX_RESPONDERS) {
        return 0U;
    }

    const uint64_t duration =
        (uint64_t)timing->guard_us + timing->request_subslot_us +
        timing->request_process_us +
        (uint64_t)responder_count * timing->response_subslot_us +
        (uint64_t)responder_count * timing->response_process_us;
    return duration <= UINT32_MAX ? (uint32_t)duration : 0U;
}

uint32_t flextdoa_response_collection_us(
    const struct flextdoa_timing *timing, uint8_t responder_count)
{
    if (timing == NULL || responder_count > FLEXTDOA_MAX_RESPONDERS) {
        return 0U;
    }

    /*
     * Measured from reception of the request to the end of the final
     * response subslot. The paper defines response_subslot_us as the whole
     * response subslot, including the packet, so no trailing grace belongs
     * here. The rest of the slot is deliberately left for response
     * processing and the next slot's guard interval.
     */
    const uint64_t duration =
        (uint64_t)timing->request_subslot_us +
        timing->request_process_us +
        (uint64_t)responder_count * timing->response_subslot_us;
    return duration <= UINT32_MAX ? (uint32_t)duration : 0U;
}

uint32_t flextdoa_frame_duration_us(
    const struct flextdoa_timing *timing, uint8_t responder_count,
    uint8_t slot_count)
{
    const uint32_t slot_duration =
        flextdoa_slot_duration_us(timing, responder_count);
    const uint64_t frame_duration = (uint64_t)slot_duration * slot_count;
    return slot_duration != 0U && slot_count != 0U &&
                   frame_duration <= UINT32_MAX
               ? (uint32_t)frame_duration
               : 0U;
}

bool flextdoa_build_ci_cr_slot(
    const uint16_t *anchor_ids, size_t anchor_count,
    const uint16_t *slot_initiator_ids, uint8_t slot_count,
    const uint16_t *slot_responder_masks, uint8_t responder_count,
    uint32_t slot_id,
    struct flextdoa_slot_plan *plan)
{
    if (anchor_ids == NULL || slot_initiator_ids == NULL ||
        slot_responder_masks == NULL || plan == NULL ||
        anchor_count < 2U || anchor_count > FLEXTDOA_MAX_ANCHORS ||
        slot_count == 0U || responder_count == 0U ||
        responder_count >= anchor_count) {
        return false;
    }

    const uint8_t slot_index = (uint8_t)(slot_id % slot_count);
    const uint16_t initiator_id = slot_initiator_ids[slot_index];
    const uint16_t allowed_mask = slot_responder_masks[slot_index];
    size_t initiator_index = anchor_count;
    for (size_t index = 0U; index < anchor_count; ++index) {
        if (anchor_ids[index] == initiator_id) {
            initiator_index = index;
            break;
        }
    }
    if (initiator_index == anchor_count) {
        return false;
    }

    memset(plan, 0, sizeof(*plan));
    plan->slot_id = slot_id;
    plan->slot_index = slot_index;
    plan->initiator_id = initiator_id;
    plan->responder_count = responder_count;

    /*
     * CI-CR has two independent round-robin dimensions. slot_index selects
     * the initiator, while frame_index rotates the response order for that
     * same initiator across successive TDMA frames. Build a stable eligible
     * pool first; using slot_id % anchor_count directly would repeat the same
     * relative response order whenever slot_count == anchor_count.
     */
    size_t allowed_count = 0U;
    uint8_t eligible[FLEXTDOA_MAX_ANCHORS] = {0};
    for (size_t index = 0U; index < anchor_count; ++index) {
        if (index != initiator_index &&
            (allowed_mask & (uint16_t)(1U << index)) != 0U) {
            if (anchor_ids[index] > UINT8_MAX) {
                return false;
            }
            eligible[allowed_count++] = (uint8_t)anchor_ids[index];
        }
    }
    if (allowed_count < responder_count) {
        return false;
    }

    const uint32_t frame_index = slot_id / slot_count;
    const size_t rotation =
        ((size_t)(frame_index % allowed_count) + plan->slot_index) %
        allowed_count;
    for (uint8_t output_index = 0U;
         output_index < responder_count; ++output_index) {
        plan->responder_ids[output_index] =
            eligible[(rotation + output_index) % allowed_count];
    }
    return true;
}

int flextdoa_responder_index(
    const struct flextdoa_slot_plan *plan, uint16_t responder_id)
{
    if (plan == NULL) {
        return -1;
    }
    for (uint8_t index = 0U; index < plan->responder_count; ++index) {
        if (plan->responder_ids[index] == responder_id) {
            return index;
        }
    }
    return -1;
}

size_t flextdoa_packet_size(uint8_t destination_count)
{
    return destination_count <= FLEXTDOA_MAX_RESPONDERS
               ? FLEXTDOA_PACKET_FIXED_SIZE + destination_count
               : 0U;
}

size_t flextdoa_encode_packet(
    const struct flextdoa_packet *packet, uint8_t *payload,
    size_t capacity)
{
    if (packet == NULL || payload == NULL ||
        !message_type_valid(packet->type) ||
        (packet->type == FLEXTDOA_MESSAGE_REQUEST &&
         packet->destination_count == 0U) ||
        (packet->type == FLEXTDOA_MESSAGE_RESPONSE &&
         packet->destination_count != 0U)) {
        return 0U;
    }

    const size_t encoded_size =
        flextdoa_packet_size(packet->destination_count);
    if (encoded_size == 0U || capacity < encoded_size) {
        return 0U;
    }

    size_t offset = 0U;
    payload[offset++] = (uint8_t)packet->type;
    put_u32_le(&payload[offset], packet->slot_id);
    offset += 4U;
    put_u16_le(&payload[offset], packet->source_id);
    offset += 2U;
    payload[offset++] = packet->destination_count;
    memcpy(&payload[offset], packet->destination_ids,
           packet->destination_count);
    offset += packet->destination_count;
    put_u32_le(&payload[offset], packet->processing_time_dtu);
    offset += 4U;
    put_u16_le(&payload[offset], packet->previous_twr_responder_id);
    offset += 2U;
    put_u16_le(&payload[offset], packet->previous_twr_mm);
    offset += 2U;
    put_u16_le(&payload[offset], packet->previous_slot_id);
    offset += 2U;
    if (!uwb_mobile_position_encode(
            &packet->sender_position, &payload[offset],
            capacity - offset)) {
        return 0U;
    }
    offset += UWB_MOBILE_POSITION_WIRE_SIZE;
    return offset;
}

bool flextdoa_decode_packet(
    const uint8_t *payload, size_t payload_len,
    struct flextdoa_packet *packet)
{
    if (payload == NULL || packet == NULL ||
        payload_len < FLEXTDOA_PACKET_FIXED_SIZE) {
        return false;
    }

    const enum flextdoa_message_type type =
        (enum flextdoa_message_type)payload[0];
    const uint8_t destination_count = payload[7];
    const size_t expected_size = flextdoa_packet_size(destination_count);
    if (!message_type_valid(type) || expected_size == 0U ||
        payload_len != expected_size ||
        (type == FLEXTDOA_MESSAGE_REQUEST && destination_count == 0U) ||
        (type == FLEXTDOA_MESSAGE_RESPONSE && destination_count != 0U)) {
        return false;
    }

    memset(packet, 0, sizeof(*packet));
    size_t offset = 0U;
    packet->type = (enum flextdoa_message_type)payload[offset++];
    packet->slot_id = get_u32_le(&payload[offset]);
    offset += 4U;
    packet->source_id = get_u16_le(&payload[offset]);
    offset += 2U;
    packet->destination_count = payload[offset++];
    memcpy(packet->destination_ids, &payload[offset], destination_count);
    offset += destination_count;
    packet->processing_time_dtu = get_u32_le(&payload[offset]);
    offset += 4U;
    packet->previous_twr_responder_id = get_u16_le(&payload[offset]);
    offset += 2U;
    packet->previous_twr_mm = get_u16_le(&payload[offset]);
    offset += 2U;
    packet->previous_slot_id = get_u16_le(&payload[offset]);
    offset += 2U;
    return uwb_mobile_position_decode(
        &payload[offset], payload_len - offset,
        &packet->sender_position);
}

uint64_t flextdoa_timestamp_delta(uint64_t later, uint64_t earlier)
{
    const uint64_t mask = (1ULL << FLEXTDOA_TIMESTAMP_BITS) - 1ULL;
    return (later - earlier) & mask;
}

double flextdoa_dw3000_cia_scale_delta(int16_t clock_offset_raw)
{
    return -(double)clock_offset_raw / (double)(1UL << 26U);
}

bool flextdoa_compute_range_difference_m(
    const struct flextdoa_observation_input *input,
    double *range_difference_m)
{
    if (input == NULL || range_difference_m == NULL ||
        !isfinite(input->responder_to_tag_cfo_fraction) ||
        fabs(input->responder_to_tag_cfo_fraction) > 0.001 ||
        !isfinite(input->initiator_responder_tof_dtu) ||
        input->initiator_responder_tof_dtu < 0.0 ||
        !isfinite(input->dtu_seconds) || input->dtu_seconds <= 0.0 ||
        !isfinite(input->speed_of_light_mps) ||
        input->speed_of_light_mps <= 0.0) {
        return false;
    }

    const double tag_interval_dtu = (double)flextdoa_timestamp_delta(
        input->response_rx_tag_dtu, input->request_rx_tag_dtu);
    const double corrected_processing_dtu =
        (double)input->responder_processing_dtu *
        (1.0 - input->responder_to_tag_cfo_fraction);
    const double tdoa_dtu = tag_interval_dtu -
                            corrected_processing_dtu -
                            input->initiator_responder_tof_dtu;
    const double result = tdoa_dtu * input->dtu_seconds *
                          input->speed_of_light_mps;
    if (!isfinite(result)) {
        return false;
    }
    *range_difference_m = result;
    return true;
}
