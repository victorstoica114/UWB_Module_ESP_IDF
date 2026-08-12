#ifndef UWB_PASSIVE_DS_PROTOCOL_H
#define UWB_PASSIVE_DS_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UWB_PASSIVE_DS_PROTOCOL_VERSION 3U
#define UWB_PASSIVE_DS_MAX_ANCHORS 4U
#define UWB_PASSIVE_DS_MAX_RESPONDERS (UWB_PASSIVE_DS_MAX_ANCHORS - 1U)
#define UWB_PASSIVE_DS_EXCHANGE_HISTORY 2U
#define UWB_PASSIVE_DS_MAX_PACKET_SIZE 65U
#define UWB_PASSIVE_DS_TIMESTAMP_MASK ((1ULL << 40U) - 1ULL)

#define UWB_PASSIVE_DS_POSITION_VALID (1U << 0U)
#define UWB_PASSIVE_DS_POSITION_RTK_FIXED (1U << 1U)
#define UWB_PASSIVE_DS_POSITION_VELOCITY_VALID (1U << 2U)
#define UWB_PASSIVE_DS_POSITION_KNOWN_FLAGS                         \
    (UWB_PASSIVE_DS_POSITION_VALID |                               \
     UWB_PASSIVE_DS_POSITION_RTK_FIXED |                           \
     UWB_PASSIVE_DS_POSITION_VELOCITY_VALID)

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

/*
 * Position of the packet transmitter at its most recent GNSS epoch.
 * latitude/longitude use 1e-7 degree units (about 1.1 cm north/south),
 * velocity uses mm/s in the geographic east/north axes and age_ms advances
 * the GNSS epoch to the UWB transmission.  Invalid fixes are encoded as an
 * all-zero structure and keep the fixed surveyed-geometry fallback intact.
 */
struct uwb_passive_ds_anchor_position {
    uint8_t flags;
    uint16_t age_ms;
    int32_t latitude_e7;
    int32_t longitude_e7;
    int16_t velocity_east_mmps;
    int16_t velocity_north_mmps;
};

struct uwb_passive_ds_packet {
    enum uwb_passive_ds_message_type type;
    uint32_t session_id;
    uint32_t frame_id;
    uint8_t initiator_id;
    uint8_t anchor_count;

    /* Present on POLL/RESPONSE; FINAL has no sender-position field. */
    struct uwb_passive_ds_anchor_position sender_position;

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
bool uwb_passive_ds_next_owned_frame(
    const uint8_t *anchor_ids, size_t anchor_count,
    uint32_t after_frame_id, uint8_t local_anchor_id,
    uint32_t *next_frame_id, uint8_t *frame_offset);
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
