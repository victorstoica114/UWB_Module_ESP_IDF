#include "uwb_passive_ds_protocol.h"

#include <math.h>
#include <string.h>

#define PASSIVE_DS_HEADER_SIZE 16U
#define PASSIVE_DS_CRC_SIZE 2U
#define PASSIVE_DS_POSITION_SIZE 15U
#define PASSIVE_DS_COMPLETED_EXCHANGE_SIZE 13U
#define PASSIVE_DS_POLL_SIZE                                          \
    (PASSIVE_DS_HEADER_SIZE + 1U +                                   \
     UWB_PASSIVE_DS_EXCHANGE_HISTORY *                               \
         PASSIVE_DS_COMPLETED_EXCHANGE_SIZE +                        \
     PASSIVE_DS_POSITION_SIZE +                                      \
     PASSIVE_DS_CRC_SIZE)
#define PASSIVE_DS_RESPONSE_SIZE                                      \
    (PASSIVE_DS_HEADER_SIZE + 6U +                                   \
     UWB_PASSIVE_DS_EXCHANGE_HISTORY *                               \
         PASSIVE_DS_COMPLETED_EXCHANGE_SIZE +                        \
     PASSIVE_DS_POSITION_SIZE +                                      \
     PASSIVE_DS_CRC_SIZE)
#define PASSIVE_DS_FINAL_FIXED_SIZE \
    (PASSIVE_DS_HEADER_SIZE + 11U + PASSIVE_DS_CRC_SIZE)
#define PASSIVE_DS_FINAL_ENTRY_SIZE 6U
#define PASSIVE_DS_TIME_UNIT_SECONDS 15.650040064102564e-12
#define PASSIVE_DS_SPEED_OF_LIGHT_MPS 299702547.0

static const uint8_t s_magic[4] = {'P', 'D', 'S', '3'};

static void put_u16(uint8_t *payload, size_t offset, uint16_t value)
{
    payload[offset] = (uint8_t)value;
    payload[offset + 1U] = (uint8_t)(value >> 8U);
}

static uint16_t get_u16(const uint8_t *payload, size_t offset)
{
    return (uint16_t)payload[offset] |
           (uint16_t)((uint16_t)payload[offset + 1U] << 8U);
}

static void put_u32(uint8_t *payload, size_t offset, uint32_t value)
{
    payload[offset] = (uint8_t)value;
    payload[offset + 1U] = (uint8_t)(value >> 8U);
    payload[offset + 2U] = (uint8_t)(value >> 16U);
    payload[offset + 3U] = (uint8_t)(value >> 24U);
}

static uint32_t get_u32(const uint8_t *payload, size_t offset)
{
    return (uint32_t)payload[offset] |
           ((uint32_t)payload[offset + 1U] << 8U) |
           ((uint32_t)payload[offset + 2U] << 16U) |
           ((uint32_t)payload[offset + 3U] << 24U);
}

static void put_i16(uint8_t *payload, size_t offset, int16_t value)
{
    put_u16(payload, offset, (uint16_t)value);
}

static int16_t get_i16(const uint8_t *payload, size_t offset)
{
    return (int16_t)get_u16(payload, offset);
}

static void put_i32(uint8_t *payload, size_t offset, int32_t value)
{
    put_u32(payload, offset, (uint32_t)value);
}

static int32_t get_i32(const uint8_t *payload, size_t offset)
{
    return (int32_t)get_u32(payload, offset);
}

static void put_ts40(uint8_t *payload, size_t offset, uint64_t value)
{
    value &= UWB_PASSIVE_DS_TIMESTAMP_MASK;
    for (size_t index = 0U; index < 5U; ++index) {
        payload[offset + index] =
            (uint8_t)(value >> (uint8_t)(8U * index));
    }
}

static uint64_t get_ts40(const uint8_t *payload, size_t offset)
{
    uint64_t value = 0U;
    for (size_t index = 0U; index < 5U; ++index) {
        value |= (uint64_t)payload[offset + index] << (8U * index);
    }
    return value & UWB_PASSIVE_DS_TIMESTAMP_MASK;
}

static uint16_t crc16_ccitt_false(const uint8_t *payload, size_t length)
{
    uint16_t crc = 0xffffU;
    for (size_t index = 0U; index < length; ++index) {
        crc ^= (uint16_t)payload[index] << 8U;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) != 0U
                      ? (uint16_t)((crc << 1U) ^ 0x1021U)
                      : (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

static bool position_valid(
    const struct uwb_passive_ds_anchor_position *position)
{
    if (position == NULL ||
        (position->flags & ~UWB_PASSIVE_DS_POSITION_KNOWN_FLAGS) != 0U) {
        return false;
    }
    if ((position->flags & UWB_PASSIVE_DS_POSITION_VALID) == 0U) {
        return position->flags == 0U && position->age_ms == 0U &&
               position->latitude_e7 == 0 &&
               position->longitude_e7 == 0 &&
               position->velocity_east_mmps == 0 &&
               position->velocity_north_mmps == 0;
    }
    return position->latitude_e7 >= -900000000 &&
           position->latitude_e7 <= 900000000 &&
           position->longitude_e7 >= -1800000000 &&
           position->longitude_e7 <= 1800000000;
}

static bool encode_position(
    const struct uwb_passive_ds_anchor_position *position,
    uint8_t *payload, size_t offset)
{
    if (!position_valid(position)) {
        return false;
    }
    payload[offset] = position->flags;
    put_u16(payload, offset + 1U, position->age_ms);
    put_i32(payload, offset + 3U, position->latitude_e7);
    put_i32(payload, offset + 7U, position->longitude_e7);
    put_i16(payload, offset + 11U, position->velocity_east_mmps);
    put_i16(payload, offset + 13U, position->velocity_north_mmps);
    return true;
}

static bool decode_position(
    const uint8_t *payload, size_t offset,
    struct uwb_passive_ds_anchor_position *position)
{
    if (payload == NULL || position == NULL) {
        return false;
    }
    *position = (struct uwb_passive_ds_anchor_position){
        .flags = payload[offset],
        .age_ms = get_u16(payload, offset + 1U),
        .latitude_e7 = get_i32(payload, offset + 3U),
        .longitude_e7 = get_i32(payload, offset + 7U),
        .velocity_east_mmps = get_i16(payload, offset + 11U),
        .velocity_north_mmps = get_i16(payload, offset + 13U),
    };
    return position_valid(position);
}

static bool encode_completed_exchanges(
    const struct uwb_passive_ds_packet *packet, uint8_t *payload,
    size_t count_offset)
{
    if (packet->completed_exchange_count >
        UWB_PASSIVE_DS_EXCHANGE_HISTORY) {
        return false;
    }
    payload[count_offset] = packet->completed_exchange_count;
    for (uint8_t index = 0U;
         index < packet->completed_exchange_count; ++index) {
        const struct uwb_passive_ds_exchange_reference *exchange =
            &packet->completed_exchanges[index];
        if (exchange->session_id == 0U ||
            exchange->initiator_id == 0U ||
            exchange->responder_exchange_dtu == 0U) {
            return false;
        }
        for (uint8_t previous = 0U; previous < index; ++previous) {
            const struct uwb_passive_ds_exchange_reference *other =
                &packet->completed_exchanges[previous];
            if (other->session_id == exchange->session_id &&
                other->frame_id == exchange->frame_id &&
                other->initiator_id == exchange->initiator_id) {
                return false;
            }
        }
        const size_t offset = count_offset + 1U +
                              (size_t)index *
                                  PASSIVE_DS_COMPLETED_EXCHANGE_SIZE;
        put_u32(payload, offset, exchange->session_id);
        put_u32(payload, offset + 4U, exchange->frame_id);
        payload[offset + 8U] = exchange->initiator_id;
        put_u32(payload, offset + 9U,
                exchange->responder_exchange_dtu);
    }
    return true;
}

static bool decode_completed_exchanges(
    const uint8_t *payload, size_t count_offset,
    struct uwb_passive_ds_packet *packet)
{
    packet->completed_exchange_count = payload[count_offset];
    if (packet->completed_exchange_count >
        UWB_PASSIVE_DS_EXCHANGE_HISTORY) {
        return false;
    }
    for (uint8_t index = 0U;
         index < packet->completed_exchange_count; ++index) {
        const size_t offset = count_offset + 1U +
                              (size_t)index *
                                  PASSIVE_DS_COMPLETED_EXCHANGE_SIZE;
        struct uwb_passive_ds_exchange_reference *exchange =
            &packet->completed_exchanges[index];
        exchange->session_id = get_u32(payload, offset);
        exchange->frame_id = get_u32(payload, offset + 4U);
        exchange->initiator_id = payload[offset + 8U];
        exchange->responder_exchange_dtu =
            get_u32(payload, offset + 9U);
        if (exchange->session_id == 0U ||
            exchange->initiator_id == 0U ||
            exchange->responder_exchange_dtu == 0U) {
            return false;
        }
        for (uint8_t previous = 0U; previous < index; ++previous) {
            const struct uwb_passive_ds_exchange_reference *other =
                &packet->completed_exchanges[previous];
            if (other->session_id == exchange->session_id &&
                other->frame_id == exchange->frame_id &&
                other->initiator_id == exchange->initiator_id) {
                return false;
            }
        }
    }
    return true;
}

size_t uwb_passive_ds_protocol_packet_size(
    enum uwb_passive_ds_message_type type, uint8_t responder_count)
{
    switch (type) {
    case UWB_PASSIVE_DS_MESSAGE_POLL:
        return responder_count == 0U ? PASSIVE_DS_POLL_SIZE : 0U;
    case UWB_PASSIVE_DS_MESSAGE_RESPONSE:
        return responder_count == 0U ? PASSIVE_DS_RESPONSE_SIZE : 0U;
    case UWB_PASSIVE_DS_MESSAGE_FINAL:
        return responder_count <= UWB_PASSIVE_DS_MAX_RESPONDERS
                   ? PASSIVE_DS_FINAL_FIXED_SIZE +
                         (size_t)responder_count *
                             PASSIVE_DS_FINAL_ENTRY_SIZE
                   : 0U;
    default:
        return 0U;
    }
}

static bool common_fields_valid(const struct uwb_passive_ds_packet *packet)
{
    return packet != NULL && packet->session_id != 0U &&
           packet->initiator_id != 0U && packet->anchor_count >= 3U &&
           packet->anchor_count <= UWB_PASSIVE_DS_MAX_ANCHORS;
}

bool uwb_passive_ds_protocol_encode(
    const struct uwb_passive_ds_packet *packet, uint8_t *payload,
    size_t capacity, size_t *payload_len)
{
    if (!common_fields_valid(packet) || payload == NULL ||
        payload_len == NULL) {
        return false;
    }
    const uint8_t final_count =
        packet->type == UWB_PASSIVE_DS_MESSAGE_FINAL
            ? packet->responder_count
            : 0U;
    const size_t length = uwb_passive_ds_protocol_packet_size(
        packet->type, final_count);
    if (length == 0U || capacity < length ||
        ((packet->type == UWB_PASSIVE_DS_MESSAGE_POLL ||
          packet->type == UWB_PASSIVE_DS_MESSAGE_RESPONSE) &&
         packet->completed_exchange_count >
             UWB_PASSIVE_DS_EXCHANGE_HISTORY) ||
        (packet->type == UWB_PASSIVE_DS_MESSAGE_FINAL &&
         packet->completed_exchange_count != 0U) ||
        (packet->type == UWB_PASSIVE_DS_MESSAGE_RESPONSE &&
         (packet->responder_index >= packet->anchor_count - 1U ||
          packet->responder_reply_dtu == 0U)) ||
        (packet->type == UWB_PASSIVE_DS_MESSAGE_FINAL &&
         packet->responder_count > packet->anchor_count - 1U)) {
        return false;
    }

    memset(payload, 0, length);
    memcpy(payload, s_magic, sizeof(s_magic));
    payload[4] = UWB_PASSIVE_DS_PROTOCOL_VERSION;
    payload[5] = (uint8_t)packet->type;
    put_u32(payload, 6U, packet->session_id);
    put_u32(payload, 10U, packet->frame_id);
    payload[14] = packet->initiator_id;
    payload[15] = packet->anchor_count;

    if (packet->type == UWB_PASSIVE_DS_MESSAGE_POLL) {
        if (!encode_completed_exchanges(packet, payload, 16U)) {
            return false;
        }
        if (!encode_position(&packet->sender_position, payload,
                             length - PASSIVE_DS_CRC_SIZE -
                                 PASSIVE_DS_POSITION_SIZE)) {
            return false;
        }
    } else if (packet->type == UWB_PASSIVE_DS_MESSAGE_RESPONSE) {
        payload[16] = packet->responder_index;
        put_u32(payload, 17U, packet->responder_reply_dtu);
        if (!encode_completed_exchanges(packet, payload, 21U)) {
            return false;
        }
        if (!encode_position(&packet->sender_position, payload,
                             length - PASSIVE_DS_CRC_SIZE -
                                 PASSIVE_DS_POSITION_SIZE)) {
            return false;
        }
    } else if (packet->type == UWB_PASSIVE_DS_MESSAGE_FINAL) {
        payload[16] = packet->responder_count;
        put_ts40(payload, 17U, packet->initiator_poll_tx);
        put_ts40(payload, 22U, packet->initiator_final_tx);
        for (uint8_t index = 0U; index < packet->responder_count;
             ++index) {
            const size_t offset = 27U +
                                  (size_t)index *
                                      PASSIVE_DS_FINAL_ENTRY_SIZE;
            const uint8_t responder_id =
                packet->responders[index].responder_id;
            if (responder_id == 0U ||
                responder_id == packet->initiator_id) {
                return false;
            }
            for (uint8_t previous = 0U; previous < index; ++previous) {
                if (packet->responders[previous].responder_id ==
                    responder_id) {
                    return false;
                }
            }
            payload[offset] = responder_id;
            put_ts40(payload, offset + 1U,
                     packet->responders[index].initiator_response_rx);
        }
    }
    put_u16(payload, length - PASSIVE_DS_CRC_SIZE,
            crc16_ccitt_false(payload, length - PASSIVE_DS_CRC_SIZE));
    *payload_len = length;
    return true;
}

enum uwb_passive_ds_decode_result uwb_passive_ds_protocol_decode(
    const uint8_t *payload, size_t payload_len,
    struct uwb_passive_ds_packet *packet)
{
    if (payload == NULL || packet == NULL ||
        payload_len < PASSIVE_DS_HEADER_SIZE + PASSIVE_DS_CRC_SIZE ||
        memcmp(payload, s_magic, sizeof(s_magic)) != 0 ||
        payload[4] != UWB_PASSIVE_DS_PROTOCOL_VERSION) {
        return UWB_PASSIVE_DS_DECODE_INVALID;
    }
    const enum uwb_passive_ds_message_type type =
        (enum uwb_passive_ds_message_type)payload[5];
    const uint8_t responder_count =
        type == UWB_PASSIVE_DS_MESSAGE_FINAL && payload_len > 16U
            ? payload[16]
            : 0U;
    const size_t expected =
        uwb_passive_ds_protocol_packet_size(type, responder_count);
    if (expected == 0U || payload_len != expected) {
        return UWB_PASSIVE_DS_DECODE_INVALID;
    }
    if (get_u16(payload, expected - PASSIVE_DS_CRC_SIZE) !=
        crc16_ccitt_false(payload, expected - PASSIVE_DS_CRC_SIZE)) {
        return UWB_PASSIVE_DS_DECODE_CRC_ERROR;
    }

    memset(packet, 0, sizeof(*packet));
    packet->type = type;
    packet->session_id = get_u32(payload, 6U);
    packet->frame_id = get_u32(payload, 10U);
    packet->initiator_id = payload[14];
    packet->anchor_count = payload[15];
    if (!common_fields_valid(packet)) {
        return UWB_PASSIVE_DS_DECODE_INVALID;
    }
    if (type == UWB_PASSIVE_DS_MESSAGE_POLL) {
        if (!decode_completed_exchanges(payload, 16U, packet)) {
            return UWB_PASSIVE_DS_DECODE_INVALID;
        }
        if (!decode_position(payload,
                             expected - PASSIVE_DS_CRC_SIZE -
                                 PASSIVE_DS_POSITION_SIZE,
                             &packet->sender_position)) {
            return UWB_PASSIVE_DS_DECODE_INVALID;
        }
    } else if (type == UWB_PASSIVE_DS_MESSAGE_RESPONSE) {
        packet->responder_index = payload[16];
        packet->responder_reply_dtu = get_u32(payload, 17U);
        if (packet->responder_index >= packet->anchor_count - 1U ||
            packet->responder_reply_dtu == 0U ||
            !decode_completed_exchanges(payload, 21U, packet)) {
            return UWB_PASSIVE_DS_DECODE_INVALID;
        }
        if (!decode_position(payload,
                             expected - PASSIVE_DS_CRC_SIZE -
                                 PASSIVE_DS_POSITION_SIZE,
                             &packet->sender_position)) {
            return UWB_PASSIVE_DS_DECODE_INVALID;
        }
    } else if (type == UWB_PASSIVE_DS_MESSAGE_FINAL) {
        packet->responder_count = responder_count;
        if (responder_count > packet->anchor_count - 1U) {
            return UWB_PASSIVE_DS_DECODE_INVALID;
        }
        packet->initiator_poll_tx = get_ts40(payload, 17U);
        packet->initiator_final_tx = get_ts40(payload, 22U);
        for (uint8_t index = 0U; index < responder_count; ++index) {
            const size_t offset = 27U +
                                  (size_t)index *
                                      PASSIVE_DS_FINAL_ENTRY_SIZE;
            const uint8_t responder_id = payload[offset];
            if (responder_id == 0U ||
                responder_id == packet->initiator_id) {
                return UWB_PASSIVE_DS_DECODE_INVALID;
            }
            for (uint8_t previous = 0U; previous < index; ++previous) {
                if (packet->responders[previous].responder_id ==
                    responder_id) {
                    return UWB_PASSIVE_DS_DECODE_INVALID;
                }
            }
            packet->responders[index].responder_id = responder_id;
            packet->responders[index].initiator_response_rx =
                get_ts40(payload, offset + 1U);
        }
    }
    return UWB_PASSIVE_DS_DECODE_OK;
}

bool uwb_passive_ds_build_plan(
    const uint8_t *anchor_ids, size_t anchor_count, uint32_t frame_id,
    struct uwb_passive_ds_plan *plan)
{
    if (anchor_ids == NULL || plan == NULL || anchor_count < 3U ||
        anchor_count > UWB_PASSIVE_DS_MAX_ANCHORS) {
        return false;
    }
    for (size_t first = 0U; first < anchor_count; ++first) {
        if (anchor_ids[first] == 0U) {
            return false;
        }
        for (size_t second = first + 1U; second < anchor_count;
             ++second) {
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
        plan->responder_ids[plan->responder_count++] =
            anchor_ids[(initiator_index + offset) % anchor_count];
    }
    return true;
}

int uwb_passive_ds_responder_index(
    const struct uwb_passive_ds_plan *plan, uint8_t responder_id)
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

uint32_t uwb_passive_ds_response_delay_us(
    uint32_t first_response_delay_us, uint32_t response_spacing_us,
    uint8_t responder_index)
{
    const uint64_t delay = (uint64_t)first_response_delay_us +
                           (uint64_t)response_spacing_us *
                               responder_index;
    return delay <= UINT32_MAX ? (uint32_t)delay : 0U;
}

uint32_t uwb_passive_ds_final_delay_from_poll_us(
    uint32_t first_response_delay_us, uint32_t response_spacing_us,
    uint8_t responder_count, uint32_t final_delay_us)
{
    const uint64_t response_train = responder_count > 0U
        ? (uint64_t)response_spacing_us * (responder_count - 1U)
        : 0U;
    const uint64_t delay = (uint64_t)first_response_delay_us +
                           response_train + final_delay_us;
    return delay <= UINT32_MAX ? (uint32_t)delay : 0U;
}

uint64_t uwb_passive_ds_timestamp_delta(uint64_t later,
                                        uint64_t earlier)
{
    return (later - earlier) & UWB_PASSIVE_DS_TIMESTAMP_MASK;
}

bool uwb_passive_ds_calculate_anchor_distance(
    uint64_t poll_tx, uint64_t poll_rx, uint64_t response_tx,
    uint64_t response_rx, uint64_t final_tx, uint64_t final_rx,
    double *distance_m)
{
    if (distance_m == NULL) {
        return false;
    }
    const double round_a = (double)uwb_passive_ds_timestamp_delta(
        response_rx, poll_tx);
    const double round_b = (double)uwb_passive_ds_timestamp_delta(
        final_rx, response_tx);
    const double reply_a = (double)uwb_passive_ds_timestamp_delta(
        final_tx, response_rx);
    const double reply_b = (double)uwb_passive_ds_timestamp_delta(
        response_tx, poll_rx);
    const double denominator = round_a + round_b + reply_a + reply_b;
    if (denominator <= 0.0) {
        return false;
    }
    const double tof_dtu =
        (round_a * round_b - reply_a * reply_b) / denominator;
    const double result = tof_dtu * PASSIVE_DS_TIME_UNIT_SECONDS *
                          PASSIVE_DS_SPEED_OF_LIGHT_MPS;
    if (!isfinite(result) || result <= 0.0) {
        return false;
    }
    *distance_m = result;
    return true;
}
