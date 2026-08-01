#include "uwb_passive_ds_multi.h"
#include "uwb_passive_ds_tdoa.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TS_MASK ((1ULL << 40U) - 1ULL)
#define DTU_SECONDS 15.650040064102564e-12
#define C_MPS 299702547.0

struct clock_model {
    double scale;
    double offset;
};

static uint64_t clock_ts(struct clock_model clock, double physical_dtu)
{
    return (uint64_t)llround(
               physical_dtu * clock.scale + clock.offset) & TS_MASK;
}

static double tof_dtu(double first_x, double first_y,
                      double second_x, double second_y)
{
    const double distance = hypot(first_x - second_x,
                                  first_y - second_y);
    return distance / (C_MPS * DTU_SECONDS);
}

static void test_plan_and_codec(void)
{
    const uint8_t anchors[] = {2U, 3U, 4U, 5U};
    struct uwb_passive_ds_multi_plan plan = {0};
    assert(uwb_passive_ds_multi_build_plan(
        anchors, 4U, 5U, &plan));
    assert(plan.initiator_id == 3U);
    assert(plan.responder_count == 3U);
    assert(plan.responder_ids[0] == 4U);
    assert(plan.responder_ids[1] == 5U);
    assert(plan.responder_ids[2] == 2U);
    assert(uwb_passive_ds_multi_responder_index(&plan, 5U) == 1);
    assert(uwb_passive_ds_multi_response_delay_us(1500U, 0U) == 1500U);
    assert(uwb_passive_ds_multi_response_delay_us(1500U, 1U) == 2250U);
    assert(uwb_passive_ds_multi_response_delay_us(1500U, 2U) == 3000U);
    assert(uwb_passive_ds_multi_final_delay_from_poll_us(
               1500U, 3U, 1500U) == 4500U);

    uint8_t payload[64] = {0};
    assert(uwb_passive_ds_multi_encode_poll(
               payload, sizeof(payload), 10U, 1234U) == 14U);
    uint32_t frame_id = 0U;
    assert(uwb_passive_ds_multi_decode_poll(
        payload, 14U, 10U, &frame_id));
    assert(frame_id == 1234U);

    memset(payload, 0, sizeof(payload));
    assert(uwb_passive_ds_multi_encode_response(
               payload, sizeof(payload), 10U, 1234U, 2U,
               63897764U) == 19U);
    uint8_t responder_index = 0U;
    uint32_t reply_dtu = 0U;
    assert(uwb_passive_ds_multi_decode_response(
        payload, 19U, 10U, &frame_id, &responder_index, &reply_dtu));
    assert(frame_id == 1234U);
    assert(responder_index == 2U);
    assert(reply_dtu == 63897764U);

    const struct uwb_passive_ds_multi_final encoded = {
        .frame_id = 1234U,
        .initiator_poll_tx = 0x0102030405ULL,
        .initiator_final_tx = 0x1020304050ULL,
        .responder_count = 3U,
        .responders = {
            {.responder_id = 4U,
             .initiator_response_rx = 0x1112131415ULL},
            {.responder_id = 5U,
             .initiator_response_rx = 0x2122232425ULL},
            {.responder_id = 2U,
             .initiator_response_rx = 0x3132333435ULL},
        },
    };
    memset(payload, 0, sizeof(payload));
    const size_t length = uwb_passive_ds_multi_encode_final(
        payload, sizeof(payload), 10U, &encoded);
    assert(length == 43U);
    struct uwb_passive_ds_multi_final decoded = {0};
    assert(uwb_passive_ds_multi_decode_final(
        payload, length, 10U, &decoded));
    assert(decoded.frame_id == encoded.frame_id);
    assert(decoded.initiator_poll_tx == encoded.initiator_poll_tx);
    assert(decoded.initiator_final_tx == encoded.initiator_final_tx);
    assert(decoded.responders[2].responder_id == 2U);
    assert(decoded.responders[2].initiator_response_rx ==
           encoded.responders[2].initiator_response_rx);
}

static void test_full_ds_multi_math(void)
{
    const double ax = 0.0;
    const double ay = 0.0;
    const double lx = 1.3;
    const double ly = 1.7;
    const double bx[] = {3.0, 3.0, 0.0};
    const double by[] = {0.0, 3.0, 3.0};
    const struct clock_model initiator = {1.000012, 210000.0};
    const struct clock_model listener = {0.999991, 730000.0};
    const struct clock_model responders[] = {
        {1.000004, 310000.0},
        {0.999986, 510000.0},
        {1.000019, 910000.0},
    };

    const double poll_tx_physical = 1000000000.0;
    const double listener_poll_rx_physical = poll_tx_physical +
        tof_dtu(ax, ay, lx, ly);
    double response_tx_physical[3] = {0};
    double response_rx_a_physical[3] = {0};
    double poll_rx_b_physical[3] = {0};
    uint32_t reply_dtu[3] = {0};
    for (size_t index = 0U; index < 3U; ++index) {
        poll_rx_b_physical[index] = poll_tx_physical +
            tof_dtu(ax, ay, bx[index], by[index]);
        response_tx_physical[index] = poll_rx_b_physical[index] +
            (double)(index + 1U) * 64000000.0;
        const uint64_t poll_rx_b = clock_ts(
            responders[index], poll_rx_b_physical[index]);
        const uint64_t response_tx_b = clock_ts(
            responders[index], response_tx_physical[index]);
        reply_dtu[index] = (uint32_t)((response_tx_b - poll_rx_b) & TS_MASK);
        response_rx_a_physical[index] = response_tx_physical[index] +
            tof_dtu(ax, ay, bx[index], by[index]);
    }

    const double final_tx_physical =
        response_rx_a_physical[2] + 64000000.0;
    const double listener_final_rx_physical = final_tx_physical +
        tof_dtu(ax, ay, lx, ly);
    for (size_t index = 0U; index < 3U; ++index) {
        const double listener_response_rx_physical =
            response_tx_physical[index] +
            tof_dtu(bx[index], by[index], lx, ly);
        const double responder_final_rx_physical = final_tx_physical +
            tof_dtu(ax, ay, bx[index], by[index]);
        const uint64_t responder_poll_rx = clock_ts(
            responders[index], poll_rx_b_physical[index]);
        const uint64_t responder_final_rx = clock_ts(
            responders[index], responder_final_rx_physical);
        const struct uwb_passive_ds_tdoa_input input = {
            .listener_poll_rx = clock_ts(
                listener, listener_poll_rx_physical),
            .listener_response_rx = clock_ts(
                listener, listener_response_rx_physical),
            .listener_final_rx = clock_ts(
                listener, listener_final_rx_physical),
            .initiator_poll_tx = clock_ts(initiator, poll_tx_physical),
            .initiator_response_rx = clock_ts(
                initiator, response_rx_a_physical[index]),
            .initiator_final_tx = clock_ts(initiator, final_tx_physical),
            .responder_reply_dtu = reply_dtu[index],
            .responder_exchange_dtu = (uint32_t)(
                (responder_final_rx - responder_poll_rx) & TS_MASK),
        };
        struct uwb_passive_ds_tdoa_result result = {0};
        assert(uwb_passive_ds_tdoa_calculate(&input, &result));
        assert(result.responder_delay_ratio > 0.0);
        assert(result.responder_delay_ratio < 1.0);
        const double expected =
            hypot(lx - bx[index], ly - by[index]) -
            hypot(lx - ax, ly - ay);
        assert(fabs(result.difference_m - expected) < 0.005);
    }
}

static enum uwb_passive_ds_tdoa_status complete_pending_observation(
    struct uwb_passive_ds_tdoa_context *context, uint8_t initiator_id,
    uint8_t responder_id, uint16_t sequence, uint32_t frame_id)
{
    struct uwb_passive_ds_tdoa_result result = {0};
    assert(uwb_passive_ds_tdoa_record_response(
               context, initiator_id, responder_id, sequence, frame_id,
               101000U, 100000U, &result) ==
           UWB_PASSIVE_DS_TDOA_INCOMPLETE);
    assert(uwb_passive_ds_tdoa_record_final(
               context, initiator_id, responder_id, sequence, frame_id,
               202000U, 100U, 100100U, 201100U, &result) ==
           UWB_PASSIVE_DS_TDOA_INCOMPLETE);
    return uwb_passive_ds_tdoa_record_responder_exchange(
        context, initiator_id, responder_id, sequence, frame_id,
        201000U, &result);
}

static void test_pending_lookup_across_free_slot(void)
{
    struct uwb_passive_ds_tdoa_context context = {0};
    struct uwb_passive_ds_tdoa_result result = {0};
    uwb_passive_ds_tdoa_init(&context);

    assert(uwb_passive_ds_tdoa_record_poll(
               &context, 2U, 3U, 10U, 10U, 1000U, &result) ==
           UWB_PASSIVE_DS_TDOA_INCOMPLETE);
    assert(uwb_passive_ds_tdoa_record_poll(
               &context, 2U, 4U, 11U, 11U, 1000U, &result) ==
           UWB_PASSIVE_DS_TDOA_INCOMPLETE);
    assert(complete_pending_observation(&context, 2U, 3U, 10U, 10U) ==
           UWB_PASSIVE_DS_TDOA_READY);

    /* Completing the first key leaves a hole before the second key. */
    assert(!context.pending[0].in_use);
    assert(context.pending[1].in_use);
    assert(complete_pending_observation(&context, 2U, 4U, 11U, 11U) ==
           UWB_PASSIVE_DS_TDOA_READY);
    assert(context.ready_count == 2U);
    assert(context.pending_replacement_count == 0U);
}

int main(void)
{
    test_plan_and_codec();
    test_full_ds_multi_math();
    test_pending_lookup_across_free_slot();
    puts("passive DS multipoint N+2 protocol: OK");
    return 0;
}
