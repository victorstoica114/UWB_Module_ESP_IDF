#include "uwb_passive_ds_tdoa.h"

#include <math.h>
#include <string.h>

#define PASSIVE_DS_TIMESTAMP_MASK ((1ULL << 40U) - 1ULL)
#define PASSIVE_DS_TIME_UNIT_SECONDS 15.650040064102564e-12
#define PASSIVE_DS_SPEED_OF_LIGHT_MPS 299702547.0
#define PASSIVE_DS_MIN_CLOCK_RATIO 0.9995
#define PASSIVE_DS_MAX_CLOCK_RATIO 1.0005
#define PASSIVE_DS_MAX_ABS_DIFFERENCE_M 100.0

static uint64_t timestamp_delta(uint64_t later, uint64_t earlier)
{
    return (later - earlier) & PASSIVE_DS_TIMESTAMP_MASK;
}

bool uwb_passive_ds_tdoa_calculate(
    const struct uwb_passive_ds_tdoa_input *input,
    struct uwb_passive_ds_tdoa_result *result)
{
    if (input == NULL || result == NULL ||
        input->responder_reply_dtu == 0U ||
        input->responder_exchange_dtu <= input->responder_reply_dtu) {
        return false;
    }

    const double listener_first = (double)timestamp_delta(
        input->listener_response_rx, input->listener_poll_rx);
    const double listener_second = (double)timestamp_delta(
        input->listener_final_rx, input->listener_response_rx);
    const double listener_exchange = listener_first + listener_second;

    const double initiator_round = (double)timestamp_delta(
        input->initiator_response_rx, input->initiator_poll_tx);
    const double initiator_reply = (double)timestamp_delta(
        input->initiator_final_tx, input->initiator_response_rx);
    const double initiator_exchange = initiator_round + initiator_reply;
    const double responder_reply = (double)input->responder_reply_dtu;
    const double responder_exchange =
        (double)input->responder_exchange_dtu;

    if (listener_first <= 0.0 || listener_second <= 0.0 ||
        initiator_round <= 0.0 || initiator_reply <= 0.0 ||
        listener_exchange <= 0.0 || initiator_exchange <= 0.0 ||
        responder_exchange <= 0.0) {
        return false;
    }

    const double listener_to_initiator =
        listener_exchange / initiator_exchange;
    const double listener_to_responder =
        listener_exchange / responder_exchange;
    if (!isfinite(listener_to_initiator) ||
        !isfinite(listener_to_responder) ||
        listener_to_initiator < PASSIVE_DS_MIN_CLOCK_RATIO ||
        listener_to_initiator > PASSIVE_DS_MAX_CLOCK_RATIO ||
        listener_to_responder < PASSIVE_DS_MIN_CLOCK_RATIO ||
        listener_to_responder > PASSIVE_DS_MAX_CLOCK_RATIO) {
        return false;
    }

    /*
     * Rathje & Landsiedel, Eq. (19), with the sign inverted to match the
     * existing solver convention (responder range minus initiator range):
     *
     *   d(B,L)-d(A,L) = M_L
     *                   - 0.5*(k_L/k_A)*R_A
     *                   - 0.5*(k_L/k_B)*D_B
     */
    const double difference_dtu =
        listener_first -
        0.5 * listener_to_initiator * initiator_round -
        0.5 * listener_to_responder * responder_reply;
    const double difference_m =
        difference_dtu * PASSIVE_DS_TIME_UNIT_SECONDS *
        PASSIVE_DS_SPEED_OF_LIGHT_MPS;
    if (!isfinite(difference_m) ||
        fabs(difference_m) > PASSIVE_DS_MAX_ABS_DIFFERENCE_M) {
        return false;
    }

    result->difference_m = difference_m;
    result->listener_to_initiator_clock_ratio = listener_to_initiator;
    result->listener_to_responder_clock_ratio = listener_to_responder;
    result->responder_reply_dtu = input->responder_reply_dtu;
    return true;
}

void uwb_passive_ds_tdoa_init(
    struct uwb_passive_ds_tdoa_context *context)
{
    if (context != NULL) {
        memset(context, 0, sizeof(*context));
    }
}

static struct uwb_passive_ds_tdoa_pending *find_pending(
    struct uwb_passive_ds_tdoa_context *context, uint8_t initiator_id,
    uint8_t responder_id, uint16_t sequence, uint32_t slot_id,
    bool create)
{
    if (context == NULL || initiator_id == 0U || responder_id == 0U ||
        initiator_id == responder_id) {
        return NULL;
    }

    struct uwb_passive_ds_tdoa_pending *replacement = NULL;
    for (size_t index = 0U; index < UWB_PASSIVE_DS_TDOA_MAX_PENDING;
         ++index) {
        struct uwb_passive_ds_tdoa_pending *item =
            &context->pending[index];
        if (item->in_use && item->initiator_id == initiator_id &&
            item->responder_id == responder_id &&
            item->sequence == sequence && item->slot_id == slot_id) {
            item->generation = ++context->generation;
            return item;
        }
        if (!item->in_use) {
            replacement = item;
            break;
        }
        if (replacement == NULL ||
            item->generation < replacement->generation) {
            replacement = item;
        }
    }
    if (!create || replacement == NULL) {
        return NULL;
    }

    memset(replacement, 0, sizeof(*replacement));
    replacement->in_use = true;
    replacement->initiator_id = initiator_id;
    replacement->responder_id = responder_id;
    replacement->sequence = sequence;
    replacement->slot_id = slot_id;
    replacement->generation = ++context->generation;
    return replacement;
}

static enum uwb_passive_ds_tdoa_status try_complete(
    struct uwb_passive_ds_tdoa_pending *pending,
    struct uwb_passive_ds_tdoa_result *result)
{
    if (pending == NULL || !pending->have_poll ||
        !pending->have_response || !pending->have_final ||
        !pending->have_responder_exchange) {
        return UWB_PASSIVE_DS_TDOA_INCOMPLETE;
    }
    const bool valid =
        uwb_passive_ds_tdoa_calculate(&pending->input, result);
    memset(pending, 0, sizeof(*pending));
    return valid ? UWB_PASSIVE_DS_TDOA_READY
                 : UWB_PASSIVE_DS_TDOA_REJECTED;
}

enum uwb_passive_ds_tdoa_status uwb_passive_ds_tdoa_record_poll(
    struct uwb_passive_ds_tdoa_context *context, uint8_t initiator_id,
    uint8_t responder_id, uint16_t sequence, uint32_t slot_id,
    uint64_t listener_poll_rx,
    struct uwb_passive_ds_tdoa_result *result)
{
    struct uwb_passive_ds_tdoa_pending *pending = find_pending(
        context, initiator_id, responder_id, sequence, slot_id, true);
    if (pending == NULL || result == NULL) {
        return UWB_PASSIVE_DS_TDOA_REJECTED;
    }
    pending->input.listener_poll_rx = listener_poll_rx;
    pending->have_poll = true;
    return try_complete(pending, result);
}

enum uwb_passive_ds_tdoa_status uwb_passive_ds_tdoa_record_response(
    struct uwb_passive_ds_tdoa_context *context, uint8_t initiator_id,
    uint8_t responder_id, uint16_t sequence, uint32_t slot_id,
    uint64_t listener_response_rx, uint32_t responder_reply_dtu,
    struct uwb_passive_ds_tdoa_result *result)
{
    struct uwb_passive_ds_tdoa_pending *pending = find_pending(
        context, initiator_id, responder_id, sequence, slot_id, true);
    if (pending == NULL || result == NULL || responder_reply_dtu == 0U) {
        return UWB_PASSIVE_DS_TDOA_REJECTED;
    }
    pending->input.listener_response_rx = listener_response_rx;
    pending->input.responder_reply_dtu = responder_reply_dtu;
    pending->have_response = true;
    return try_complete(pending, result);
}

enum uwb_passive_ds_tdoa_status uwb_passive_ds_tdoa_record_final(
    struct uwb_passive_ds_tdoa_context *context, uint8_t initiator_id,
    uint8_t responder_id, uint16_t sequence, uint32_t slot_id,
    uint64_t listener_final_rx, uint64_t initiator_poll_tx,
    uint64_t initiator_response_rx, uint64_t initiator_final_tx,
    struct uwb_passive_ds_tdoa_result *result)
{
    struct uwb_passive_ds_tdoa_pending *pending = find_pending(
        context, initiator_id, responder_id, sequence, slot_id, false);
    if (pending == NULL || result == NULL) {
        return UWB_PASSIVE_DS_TDOA_REJECTED;
    }
    pending->input.listener_final_rx = listener_final_rx;
    pending->input.initiator_poll_tx = initiator_poll_tx;
    pending->input.initiator_response_rx = initiator_response_rx;
    pending->input.initiator_final_tx = initiator_final_tx;
    pending->have_final = true;
    return try_complete(pending, result);
}

enum uwb_passive_ds_tdoa_status
uwb_passive_ds_tdoa_record_responder_exchange(
    struct uwb_passive_ds_tdoa_context *context, uint8_t initiator_id,
    uint8_t responder_id, uint16_t sequence, uint32_t slot_id,
    uint32_t responder_exchange_dtu,
    struct uwb_passive_ds_tdoa_result *result)
{
    struct uwb_passive_ds_tdoa_pending *pending = find_pending(
        context, initiator_id, responder_id, sequence, slot_id, false);
    if (pending == NULL || result == NULL || responder_exchange_dtu == 0U) {
        return UWB_PASSIVE_DS_TDOA_REJECTED;
    }
    pending->input.responder_exchange_dtu = responder_exchange_dtu;
    pending->have_responder_exchange = true;
    return try_complete(pending, result);
}
