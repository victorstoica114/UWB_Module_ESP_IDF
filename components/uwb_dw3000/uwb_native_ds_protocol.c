#include "uwb_native_ds_protocol.h"

#include <math.h>
#include <string.h>

#define NATIVE_DS_HEADER_SIZE 18U
#define NATIVE_DS_CRC_SIZE 2U
#define NATIVE_DS_BASIC_SIZE (NATIVE_DS_HEADER_SIZE + NATIVE_DS_CRC_SIZE)
#define NATIVE_DS_FINAL_SIZE \
    (NATIVE_DS_HEADER_SIZE + 15U + NATIVE_DS_CRC_SIZE)
#define NATIVE_DS_RESULT_SIZE \
    (NATIVE_DS_HEADER_SIZE + 4U + NATIVE_DS_CRC_SIZE)
#define NATIVE_DS_TIME_UNIT_SECONDS 15.650040064102564e-12
#define NATIVE_DS_SPEED_OF_LIGHT_MPS 299702547.0

static const uint8_t s_magic[4] = {'N', 'D', 'S', '4'};

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

static void put_u16(uint8_t *payload, size_t offset, uint16_t value)
{
    payload[offset] = (uint8_t)(value & 0xffU);
    payload[offset + 1U] = (uint8_t)(value >> 8U);
}

static uint16_t get_u16(const uint8_t *payload, size_t offset)
{
    return (uint16_t)payload[offset] |
           (uint16_t)((uint16_t)payload[offset + 1U] << 8U);
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

static void put_ts40(uint8_t *payload, size_t offset, uint64_t timestamp)
{
    timestamp &= UWB_NATIVE_DS_TIMESTAMP_MASK;
    for (size_t index = 0U; index < 5U; ++index) {
        payload[offset + index] =
            (uint8_t)((timestamp >> (8U * index)) & 0xffU);
    }
}

static uint64_t get_ts40(const uint8_t *payload, size_t offset)
{
    uint64_t timestamp = 0U;
    for (size_t index = 0U; index < 5U; ++index) {
        timestamp |= (uint64_t)payload[offset + index] << (8U * index);
    }
    return timestamp & UWB_NATIVE_DS_TIMESTAMP_MASK;
}

size_t uwb_native_ds_protocol_packet_size(
    enum uwb_native_ds_message_type type)
{
    switch (type) {
    case UWB_NATIVE_DS_MESSAGE_POLL:
    case UWB_NATIVE_DS_MESSAGE_RESPONSE:
        return NATIVE_DS_BASIC_SIZE;
    case UWB_NATIVE_DS_MESSAGE_FINAL:
        return NATIVE_DS_FINAL_SIZE;
    case UWB_NATIVE_DS_MESSAGE_RESULT:
        return NATIVE_DS_RESULT_SIZE;
    default:
        return 0U;
    }
}

static bool packet_header_valid(const struct uwb_native_ds_packet *packet)
{
    return packet != NULL &&
           uwb_native_ds_protocol_packet_size(packet->type) != 0U &&
           packet->exchange_kind == UWB_NATIVE_DS_EXCHANGE_TAG_RANGE &&
           packet->source_id != 0U && packet->destination_id != 0U &&
           packet->source_id != packet->destination_id &&
           packet->session_id != 0U;
}

bool uwb_native_ds_protocol_encode(
    const struct uwb_native_ds_packet *packet, uint8_t *payload,
    size_t capacity, size_t *payload_len)
{
    if (!packet_header_valid(packet) || payload == NULL ||
        payload_len == NULL) {
        return false;
    }
    const size_t size = uwb_native_ds_protocol_packet_size(packet->type);
    if (capacity < size) {
        return false;
    }
    memset(payload, 0, size);
    memcpy(payload, s_magic, sizeof(s_magic));
    payload[4] = UWB_NATIVE_DS_PROTOCOL_VERSION;
    payload[5] = (uint8_t)packet->type;
    payload[6] = (uint8_t)packet->exchange_kind;
    payload[7] = packet->source_id;
    payload[8] = packet->destination_id;
    put_u32(payload, 10U, packet->session_id);
    put_u32(payload, 14U, packet->frame_id);
    if (packet->type == UWB_NATIVE_DS_MESSAGE_FINAL) {
        put_ts40(payload, NATIVE_DS_HEADER_SIZE, packet->poll_tx_timestamp);
        put_ts40(payload, NATIVE_DS_HEADER_SIZE + 5U,
                 packet->response_rx_timestamp);
        put_ts40(payload, NATIVE_DS_HEADER_SIZE + 10U,
                 packet->final_tx_timestamp);
    } else if (packet->type == UWB_NATIVE_DS_MESSAGE_RESULT) {
        put_u32(payload, NATIVE_DS_HEADER_SIZE, packet->distance_mm);
    }
    put_u16(payload, size - NATIVE_DS_CRC_SIZE,
            crc16_ccitt_false(payload, size - NATIVE_DS_CRC_SIZE));
    *payload_len = size;
    return true;
}

enum uwb_native_ds_decode_result uwb_native_ds_protocol_decode_ex(
    const uint8_t *payload, size_t payload_len,
    struct uwb_native_ds_packet *packet)
{
    if (payload == NULL || packet == NULL ||
        payload_len < NATIVE_DS_HEADER_SIZE ||
        memcmp(payload, s_magic, sizeof(s_magic)) != 0 ||
        payload[4] != UWB_NATIVE_DS_PROTOCOL_VERSION) {
        return UWB_NATIVE_DS_DECODE_INVALID;
    }
    const enum uwb_native_ds_message_type type =
        (enum uwb_native_ds_message_type)payload[5];
    const size_t expected_size = uwb_native_ds_protocol_packet_size(type);
    if (payload_len != expected_size) {
        return UWB_NATIVE_DS_DECODE_INVALID;
    }
    if (get_u16(payload, expected_size - NATIVE_DS_CRC_SIZE) !=
        crc16_ccitt_false(payload, expected_size - NATIVE_DS_CRC_SIZE)) {
        return UWB_NATIVE_DS_DECODE_CRC_ERROR;
    }
    memset(packet, 0, sizeof(*packet));
    packet->type = type;
    packet->exchange_kind =
        (enum uwb_native_ds_exchange_kind)payload[6];
    packet->source_id = payload[7];
    packet->destination_id = payload[8];
    packet->session_id = get_u32(payload, 10U);
    packet->frame_id = get_u32(payload, 14U);
    if (!packet_header_valid(packet)) {
        return UWB_NATIVE_DS_DECODE_INVALID;
    }
    if (type == UWB_NATIVE_DS_MESSAGE_FINAL) {
        packet->poll_tx_timestamp = get_ts40(payload, NATIVE_DS_HEADER_SIZE);
        packet->response_rx_timestamp =
            get_ts40(payload, NATIVE_DS_HEADER_SIZE + 5U);
        packet->final_tx_timestamp =
            get_ts40(payload, NATIVE_DS_HEADER_SIZE + 10U);
    } else if (type == UWB_NATIVE_DS_MESSAGE_RESULT) {
        packet->distance_mm = get_u32(payload, NATIVE_DS_HEADER_SIZE);
    }
    return UWB_NATIVE_DS_DECODE_OK;
}

bool uwb_native_ds_protocol_decode(
    const uint8_t *payload, size_t payload_len,
    struct uwb_native_ds_packet *packet)
{
    return uwb_native_ds_protocol_decode_ex(payload, payload_len, packet) ==
           UWB_NATIVE_DS_DECODE_OK;
}

uint64_t uwb_native_ds_protocol_timestamp_delta(uint64_t later,
                                                uint64_t earlier)
{
    return (later - earlier) & UWB_NATIVE_DS_TIMESTAMP_MASK;
}

bool uwb_native_ds_protocol_calculate_distance(
    uint64_t poll_tx, uint64_t poll_rx, uint64_t response_tx,
    uint64_t response_rx, uint64_t final_tx, uint64_t final_rx,
    double *distance_m)
{
    if (distance_m == NULL) {
        return false;
    }
    const double round_a = (double)uwb_native_ds_protocol_timestamp_delta(
        response_rx, poll_tx);
    const double round_b = (double)uwb_native_ds_protocol_timestamp_delta(
        final_rx, response_tx);
    const double reply_a = (double)uwb_native_ds_protocol_timestamp_delta(
        final_tx, response_rx);
    const double reply_b = (double)uwb_native_ds_protocol_timestamp_delta(
        response_tx, poll_rx);
    const double denominator = round_a + round_b + reply_a + reply_b;
    if (denominator <= 0.0) {
        return false;
    }
    const double tof_dtu =
        (round_a * round_b - reply_a * reply_b) / denominator;
    const double result = tof_dtu * NATIVE_DS_TIME_UNIT_SECONDS *
                          NATIVE_DS_SPEED_OF_LIGHT_MPS;
    if (!isfinite(result) || result <= 0.0) {
        return false;
    }
    *distance_m = result;
    return true;
}
