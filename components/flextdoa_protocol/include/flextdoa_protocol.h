#ifndef FLEXTDOA_PROTOCOL_H
#define FLEXTDOA_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FLEXTDOA_MAX_ANCHORS 16U
#define FLEXTDOA_MAX_RESPONDERS (FLEXTDOA_MAX_ANCHORS - 1U)
#define FLEXTDOA_TIMESTAMP_BITS 40U
#define FLEXTDOA_PACKET_FIXED_SIZE 18U

enum flextdoa_message_type {
    FLEXTDOA_MESSAGE_REQUEST = 1,
    FLEXTDOA_MESSAGE_RESPONSE = 2,
};

struct flextdoa_timing {
    uint32_t guard_us;
    uint32_t request_subslot_us;
    uint32_t request_process_us;
    uint32_t response_subslot_us;
    uint32_t response_process_us;
};

extern const struct flextdoa_timing FLEXTDOA_PAPER_TIMING;

struct flextdoa_slot_plan {
    uint32_t slot_id;
    uint8_t slot_index;
    uint16_t initiator_id;
    uint8_t responder_count;
    uint8_t responder_ids[FLEXTDOA_MAX_RESPONDERS];
};

struct flextdoa_packet {
    enum flextdoa_message_type type;
    uint32_t slot_id;
    uint16_t source_id;
    uint8_t destination_count;
    uint8_t destination_ids[FLEXTDOA_MAX_RESPONDERS];
    uint32_t processing_time_dtu;
    uint16_t previous_twr_responder_id;
    uint16_t previous_twr_mm;
    uint16_t previous_slot_id;
};

struct flextdoa_observation_input {
    uint64_t request_rx_tag_dtu;
    uint64_t response_rx_tag_dtu;
    uint32_t responder_processing_dtu;
    /* DW3000 additive CFO convention used by Eq. (12), as a ratio. */
    double responder_to_tag_cfo_fraction;
    double initiator_responder_tof_dtu;
    double dtu_seconds;
    double speed_of_light_mps;
};

uint32_t flextdoa_slot_duration_us(
    const struct flextdoa_timing *timing, uint8_t responder_count);
uint32_t flextdoa_response_collection_us(
    const struct flextdoa_timing *timing, uint8_t responder_count);
uint32_t flextdoa_frame_duration_us(
    const struct flextdoa_timing *timing, uint8_t responder_count,
    uint8_t slot_count);

bool flextdoa_build_ci_cr_slot(
    const uint16_t *anchor_ids, size_t anchor_count,
    const uint16_t *slot_initiator_ids, uint8_t slot_count,
    const uint16_t *slot_responder_masks, uint8_t responder_count,
    uint32_t slot_id,
    struct flextdoa_slot_plan *plan);
int flextdoa_responder_index(
    const struct flextdoa_slot_plan *plan, uint16_t responder_id);

size_t flextdoa_packet_size(uint8_t destination_count);
size_t flextdoa_encode_packet(
    const struct flextdoa_packet *packet, uint8_t *payload,
    size_t capacity);
bool flextdoa_decode_packet(
    const uint8_t *payload, size_t payload_len,
    struct flextdoa_packet *packet);

uint64_t flextdoa_timestamp_delta(uint64_t later, uint64_t earlier);
/*
 * Convert DW3000 CIA_DIAG_0 COE_PPM into k_local / k_remote - 1.
 * Qorvo encodes epsilon as raw / 2^26 and Eq. (12) uses
 * k_local / k_remote = 1 - epsilon.
 */
double flextdoa_dw3000_cia_scale_delta(int16_t clock_offset_raw);
bool flextdoa_compute_range_difference_m(
    const struct flextdoa_observation_input *input,
    double *range_difference_m);

#endif
