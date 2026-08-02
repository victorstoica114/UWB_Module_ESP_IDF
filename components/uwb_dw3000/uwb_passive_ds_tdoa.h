#ifndef UWB_PASSIVE_DS_TDOA_H
#define UWB_PASSIVE_DS_TDOA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define UWB_PASSIVE_DS_TDOA_MAX_PENDING 16U

/*
 * One coherent passive observation of a three-packet DS-TWR exchange.
 *
 * A is the initiator, B is the responder and L is the receive-only tag.
 * All timestamp intervals remain in the clock domain in which they were
 * measured.  The double-sided equations convert A and B intervals into L's
 * clock domain without using a carrier-frequency-offset estimate.
 */
struct uwb_passive_ds_tdoa_input {
    uint64_t listener_poll_rx;
    uint64_t listener_response_rx;
    uint64_t listener_final_rx;

    uint64_t initiator_poll_tx;
    uint64_t initiator_response_rx;
    uint64_t initiator_final_tx;

    uint32_t responder_reply_dtu;
    uint32_t responder_exchange_dtu;
};

struct uwb_passive_ds_tdoa_result {
    /* Solver convention: distance(tag, responder) - distance(tag, initiator). */
    double difference_m;
    double listener_to_initiator_clock_ratio;
    double listener_to_responder_clock_ratio;
    double responder_delay_ratio;
    uint32_t responder_reply_dtu;
};

enum uwb_passive_ds_tdoa_status {
    UWB_PASSIVE_DS_TDOA_INCOMPLETE = 0,
    UWB_PASSIVE_DS_TDOA_READY,
    UWB_PASSIVE_DS_TDOA_REJECTED,
};

struct uwb_passive_ds_tdoa_pending {
    bool in_use;
    bool have_poll;
    bool have_response;
    bool have_final;
    bool have_responder_exchange;
    uint8_t initiator_id;
    uint8_t responder_id;
    uint16_t sequence;
    uint32_t slot_id;
    uint32_t generation;
    struct uwb_passive_ds_tdoa_input input;
};

struct uwb_passive_ds_tdoa_context {
    uint32_t generation;
    uint32_t ready_count;
    uint32_t calculation_rejected_count;
    uint32_t missing_final_context_count;
    uint32_t missing_exchange_context_count;
    uint32_t pending_replacement_count;
    struct uwb_passive_ds_tdoa_pending
        pending[UWB_PASSIVE_DS_TDOA_MAX_PENDING];
};

bool uwb_passive_ds_tdoa_calculate(
    const struct uwb_passive_ds_tdoa_input *input,
    struct uwb_passive_ds_tdoa_result *result);

void uwb_passive_ds_tdoa_init(
    struct uwb_passive_ds_tdoa_context *context);

enum uwb_passive_ds_tdoa_status uwb_passive_ds_tdoa_record_poll(
    struct uwb_passive_ds_tdoa_context *context, uint8_t initiator_id,
    uint8_t responder_id, uint16_t sequence, uint32_t slot_id,
    uint64_t listener_poll_rx,
    struct uwb_passive_ds_tdoa_result *result);

enum uwb_passive_ds_tdoa_status uwb_passive_ds_tdoa_record_response(
    struct uwb_passive_ds_tdoa_context *context, uint8_t initiator_id,
    uint8_t responder_id, uint16_t sequence, uint32_t slot_id,
    uint64_t listener_response_rx, uint32_t responder_reply_dtu,
    struct uwb_passive_ds_tdoa_result *result);

enum uwb_passive_ds_tdoa_status uwb_passive_ds_tdoa_record_final(
    struct uwb_passive_ds_tdoa_context *context, uint8_t initiator_id,
    uint8_t responder_id, uint16_t sequence, uint32_t slot_id,
    uint64_t listener_final_rx, uint64_t initiator_poll_tx,
    uint64_t initiator_response_rx, uint64_t initiator_final_tx,
    struct uwb_passive_ds_tdoa_result *result);

enum uwb_passive_ds_tdoa_status
uwb_passive_ds_tdoa_record_responder_exchange(
    struct uwb_passive_ds_tdoa_context *context, uint8_t initiator_id,
    uint8_t responder_id, uint16_t sequence, uint32_t slot_id,
    uint32_t responder_exchange_dtu,
    struct uwb_passive_ds_tdoa_result *result);

#endif
