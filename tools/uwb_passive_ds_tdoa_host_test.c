#include "uwb_passive_ds_tdoa.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define TEST_DTU_SECONDS 15.650040064102564e-12
#define TEST_SPEED_OF_LIGHT_MPS 299702547.0
#define TEST_TIMESTAMP_MASK ((1ULL << 40U) - 1ULL)

static uint64_t local_timestamp(double physical_seconds,
                                double clock_scale,
                                uint64_t origin_dtu)
{
    const uint64_t elapsed = (uint64_t)llround(
        physical_seconds * clock_scale / TEST_DTU_SECONDS);
    return (origin_dtu + elapsed) & TEST_TIMESTAMP_MASK;
}

static uint32_t local_interval(double physical_seconds,
                               double clock_scale)
{
    return (uint32_t)llround(
        physical_seconds * clock_scale / TEST_DTU_SECONDS);
}

static void test_double_sided_solution(bool wrap_timestamps)
{
    const double distance_ab_m = 3.0;
    const double distance_al_m = 1.0;
    const double distance_bl_m = 2.0;
    const double tof_ab = distance_ab_m / TEST_SPEED_OF_LIGHT_MPS;
    const double tof_al = distance_al_m / TEST_SPEED_OF_LIGHT_MPS;
    const double tof_bl = distance_bl_m / TEST_SPEED_OF_LIGHT_MPS;
    const double responder_delay = 0.002;
    const double initiator_delay = 0.0015;
    const double initiator_clock = 1.0 + 10.0e-6;
    const double responder_clock = 1.0 - 8.0e-6;
    const double listener_clock = 1.0 + 5.0e-6;
    const uint64_t origin = wrap_timestamps
        ? TEST_TIMESTAMP_MASK - 50000000ULL
        : 100000000ULL;

    const double poll_a_tx = 0.0;
    const double poll_b_rx = tof_ab;
    const double poll_l_rx = tof_al;
    const double response_b_tx = tof_ab + responder_delay;
    const double response_a_rx = 2.0 * tof_ab + responder_delay;
    const double response_l_rx = tof_ab + responder_delay + tof_bl;
    const double final_a_tx = response_a_rx + initiator_delay;
    const double final_b_rx = final_a_tx + tof_ab;
    const double final_l_rx = final_a_tx + tof_al;

    struct uwb_passive_ds_tdoa_input input = {
        .listener_poll_rx = local_timestamp(
            poll_l_rx, listener_clock, origin),
        .listener_response_rx = local_timestamp(
            response_l_rx, listener_clock, origin),
        .listener_final_rx = local_timestamp(
            final_l_rx, listener_clock, origin),
        .initiator_poll_tx = local_timestamp(
            poll_a_tx, initiator_clock, origin),
        .initiator_response_rx = local_timestamp(
            response_a_rx, initiator_clock, origin),
        .initiator_final_tx = local_timestamp(
            final_a_tx, initiator_clock, origin),
        .responder_reply_dtu = local_interval(
            response_b_tx - poll_b_rx, responder_clock),
        .responder_exchange_dtu = local_interval(
            final_b_rx - poll_b_rx, responder_clock),
    };
    struct uwb_passive_ds_tdoa_result result = {0};

    assert(uwb_passive_ds_tdoa_calculate(&input, &result));
    const double expected_difference_m =
        (distance_bl_m - distance_al_m) * listener_clock;
    assert(fabs(result.difference_m - expected_difference_m) < 0.001);
    assert(fabs(result.listener_to_initiator_clock_ratio -
                listener_clock / initiator_clock) < 1.0e-7);
    assert(fabs(result.listener_to_responder_clock_ratio -
                listener_clock / responder_clock) < 1.0e-7);
}

static void test_coherent_assembly(void)
{
    struct uwb_passive_ds_tdoa_context context;
    struct uwb_passive_ds_tdoa_result result = {0};
    uwb_passive_ds_tdoa_init(&context);

    const double tof_ab = 3.0 / TEST_SPEED_OF_LIGHT_MPS;
    const double tof_al = 1.0 / TEST_SPEED_OF_LIGHT_MPS;
    const double tof_bl = 2.0 / TEST_SPEED_OF_LIGHT_MPS;
    const double responder_delay = 0.002;
    const double initiator_delay = 0.0015;
    const double response_a_rx = 2.0 * tof_ab + responder_delay;
    const double final_a_tx = response_a_rx + initiator_delay;
    const double final_b_rx = final_a_tx + tof_ab;
    const double initiator_clock = 1.0 + 10.0e-6;
    const double responder_clock = 1.0 - 8.0e-6;
    const double listener_clock = 1.0 + 5.0e-6;
    const uint64_t origin = 100000000ULL;
    const uint8_t initiator = 2U;
    const uint8_t responder = 3U;
    const uint16_t sequence = 42U;
    const uint32_t slot_id = 0x1234002aU;
    const uint64_t listener_poll = local_timestamp(
        tof_al, listener_clock, origin);
    const uint64_t listener_response = local_timestamp(
        tof_ab + responder_delay + tof_bl, listener_clock, origin);
    const uint64_t listener_final = local_timestamp(
        final_a_tx + tof_al, listener_clock, origin);
    const uint64_t initiator_poll = local_timestamp(
        0.0, initiator_clock, origin);
    const uint64_t initiator_response = local_timestamp(
        response_a_rx, initiator_clock, origin);
    const uint64_t initiator_final = local_timestamp(
        final_a_tx, initiator_clock, origin);
    const uint32_t responder_reply = local_interval(
        responder_delay, responder_clock);
    const uint32_t responder_exchange = local_interval(
        final_b_rx - tof_ab, responder_clock);

    assert(uwb_passive_ds_tdoa_record_poll(
               &context, initiator, responder, sequence, slot_id,
               listener_poll, &result) == UWB_PASSIVE_DS_TDOA_INCOMPLETE);
    assert(uwb_passive_ds_tdoa_record_response(
               &context, initiator, responder, sequence, slot_id,
               listener_response, responder_reply,
               &result) == UWB_PASSIVE_DS_TDOA_INCOMPLETE);
    assert(uwb_passive_ds_tdoa_record_final(
               &context, initiator, responder, sequence, slot_id,
               listener_final, initiator_poll, initiator_response,
               initiator_final,
               &result) == UWB_PASSIVE_DS_TDOA_INCOMPLETE);
    assert(uwb_passive_ds_tdoa_record_responder_exchange(
               &context, initiator, responder, sequence, slot_id,
               responder_exchange,
               &result) == UWB_PASSIVE_DS_TDOA_READY);
    assert(fabs(result.difference_m - listener_clock) < 0.001);
}

int main(void)
{
    test_double_sided_solution(false);
    test_double_sided_solution(true);
    test_coherent_assembly();
    puts("passive DS-TDoA host tests passed");
    return 0;
}
