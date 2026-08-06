#ifndef UWB_NATIVE_DS_PROTOCOL_H
#define UWB_NATIVE_DS_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "uwb_mobile_geometry.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UWB_NATIVE_DS_PROTOCOL_MAX_PACKET_SIZE 40U
#define UWB_NATIVE_DS_PROTOCOL_VERSION 2U
#define UWB_NATIVE_DS_TIMESTAMP_MASK ((1ULL << 40U) - 1ULL)

enum uwb_native_ds_message_type {
    UWB_NATIVE_DS_MESSAGE_POLL = 1,
    UWB_NATIVE_DS_MESSAGE_RESPONSE = 2,
    UWB_NATIVE_DS_MESSAGE_FINAL = 3,
    UWB_NATIVE_DS_MESSAGE_RESULT = 4,
};

enum uwb_native_ds_exchange_kind {
    UWB_NATIVE_DS_EXCHANGE_TAG_RANGE = 1,
};

enum uwb_native_ds_decode_result {
    UWB_NATIVE_DS_DECODE_INVALID = 0,
    UWB_NATIVE_DS_DECODE_OK = 1,
    UWB_NATIVE_DS_DECODE_CRC_ERROR = 2,
};

struct uwb_native_ds_packet {
    enum uwb_native_ds_message_type type;
    enum uwb_native_ds_exchange_kind exchange_kind;
    uint8_t source_id;
    uint8_t destination_id;
    uint32_t session_id;
    uint32_t frame_id;
    uint64_t poll_tx_timestamp;
    uint64_t response_rx_timestamp;
    uint64_t final_tx_timestamp;
    uint32_t distance_mm;
    struct uwb_mobile_position sender_position;
};

size_t uwb_native_ds_protocol_packet_size(
    enum uwb_native_ds_message_type type);

bool uwb_native_ds_protocol_encode(
    const struct uwb_native_ds_packet *packet, uint8_t *payload,
    size_t capacity, size_t *payload_len);

bool uwb_native_ds_protocol_decode(
    const uint8_t *payload, size_t payload_len,
    struct uwb_native_ds_packet *packet);

enum uwb_native_ds_decode_result uwb_native_ds_protocol_decode_ex(
    const uint8_t *payload, size_t payload_len,
    struct uwb_native_ds_packet *packet);

uint64_t uwb_native_ds_protocol_timestamp_delta(uint64_t later,
                                                uint64_t earlier);

bool uwb_native_ds_protocol_calculate_distance(
    uint64_t poll_tx, uint64_t poll_rx, uint64_t response_tx,
    uint64_t response_rx, uint64_t final_tx, uint64_t final_rx,
    double *distance_m);

#ifdef __cplusplus
}
#endif

#endif
