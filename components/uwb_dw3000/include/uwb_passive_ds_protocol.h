#ifndef UWB_PASSIVE_DS_PROTOCOL_H
#define UWB_PASSIVE_DS_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UWB_PASSIVE_DS_PROTOCOL_VERSION 2U
#define UWB_PASSIVE_DS_MAX_ANCHORS 4U
#define UWB_PASSIVE_DS_MAX_RESPONDERS (UWB_PASSIVE_DS_MAX_ANCHORS - 1U)
#define UWB_PASSIVE_DS_EXCHANGE_HISTORY 2U
#define UWB_PASSIVE_DS_MAX_PACKET_SIZE 50U
#define UWB_PASSIVE_DS_TIMESTAMP_MASK ((1ULL << 40U) - 1ULL)

enum uwb_passive_ds_message_type {
    UWB_PASSIVE_DS_MESSAGE_POLL = 1,
    UWB_PASSIVE_DS_MESSAGE_RESPONSE = 2,
    UWB_PASSIVE_DS_MESSAGE_FINAL = 3,
};

enum uwb_passive_ds_decode_result {
    UWB_PASSIVE_DS_DECODE_INVALID = 0,
    UWB_PASSIVE_DS_DECODE_OK,
    UWB_PASSIVE_DS_DECODE_CRC_ERROR,
};

struct uwb_passive_ds_final_entry {
    uint8_t responder_id;
    uint64_t initiator_response_rx;
};

struct uwb_passive_ds_exchange_reference {
    uint32_t session_id;
    uint32_t frame_id;
    uint8_t initiator_id;
    uint32_t responder_exchange_dtu;
};

struct uwb_passive_ds_packet {
    enum uwb_passive_ds_message_type type;
    uint32_t session_id;
    uint32_t frame_id;
    uint8_t initiator_id;
    uint8_t anchor_count;

    /* POLL and RESPONSE completed-exchange references. */
    uint8_t completed_exchange_count;
    struct uwb_passive_ds_exchange_reference completed_exchanges[
        UWB_PASSIVE_DS_EXCHANGE_HISTORY];

    /* RESPONSE fields. */
    uint8_t responder_index;
    uint32_t responder_reply_dtu;

    /* FINAL fields. */
    uint64_t initiator_poll_tx;
    uint64_t initiator_final_tx;
    uint8_t responder_count;
    struct uwb_passive_ds_final_entry
        responders[UWB_PASSIVE_DS_MAX_RESPONDERS];
};

struct uwb_passive_ds_plan {
    uint32_t frame_id;
    uint8_t initiator_id;
    uint8_t responder_count;
    uint8_t responder_ids[UWB_PASSIVE_DS_MAX_RESPONDERS];
};

size_t uwb_passive_ds_protocol_packet_size(
    enum uwb_passive_ds_message_type type, uint8_t responder_count);
bool uwb_passive_ds_protocol_encode(
    const struct uwb_passive_ds_packet *packet, uint8_t *payload,
    size_t capacity, size_t *payload_len);
enum uwb_passive_ds_decode_result uwb_passive_ds_protocol_decode(
    const uint8_t *payload, size_t payload_len,
    struct uwb_passive_ds_packet *packet);

bool uwb_passive_ds_build_plan(
    const uint8_t *anchor_ids, size_t anchor_count, uint32_t frame_id,
    struct uwb_passive_ds_plan *plan);
int uwb_passive_ds_responder_index(
    const struct uwb_passive_ds_plan *plan, uint8_t responder_id);
uint32_t uwb_passive_ds_response_delay_us(
    uint32_t first_response_delay_us, uint32_t response_spacing_us,
    uint8_t responder_index);
uint32_t uwb_passive_ds_final_delay_from_poll_us(
    uint32_t first_response_delay_us, uint32_t response_spacing_us,
    uint8_t responder_count, uint32_t final_delay_us);

uint64_t uwb_passive_ds_timestamp_delta(uint64_t later,
                                        uint64_t earlier);
bool uwb_passive_ds_calculate_anchor_distance(
    uint64_t poll_tx, uint64_t poll_rx, uint64_t response_tx,
    uint64_t response_rx, uint64_t final_tx, uint64_t final_rx,
    double *distance_m);

#ifdef __cplusplus
}
#endif

#endif
