#include "uwb_passive_ds_observation.h"
#include "uwb_passive_ds_protocol.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEST_DTU_SECONDS 15.650040064102564e-12
#define TEST_C_MPS 299702547.0

static void assert_near(double actual, double expected, double tolerance)
{
    assert(fabs(actual - expected) <= tolerance);
}

static void test_rotating_plan_and_timing(void)
{
    const uint8_t anchors[] = {2U, 3U, 4U, 5U};
    struct uwb_passive_ds_plan plan = {0};
    assert(uwb_passive_ds_build_plan(anchors, 4U, 5U, &plan));
    assert(plan.initiator_id == 3U);
    assert(plan.responder_count == 3U);
    assert(plan.responder_ids[0] == 4U);
    assert(plan.responder_ids[1] == 5U);
    assert(plan.responder_ids[2] == 2U);
    assert(uwb_passive_ds_responder_index(&plan, 5U) == 1);
    assert(uwb_passive_ds_response_delay_us(1500U, 750U, 2U) ==
           3000U);
    assert(uwb_passive_ds_final_delay_from_poll_us(
               1500U, 750U, 3U, 1500U) == 4500U);
}

static void round_trip(const struct uwb_passive_ds_packet *packet)
{
    uint8_t payload[UWB_PASSIVE_DS_MAX_PACKET_SIZE] = {0};
    size_t payload_len = 0U;
    assert(uwb_passive_ds_protocol_encode(
        packet, payload, sizeof(payload), &payload_len));
    assert(payload_len == uwb_passive_ds_protocol_packet_size(
                              packet->type, packet->responder_count));
    struct uwb_passive_ds_packet decoded = {0};
    assert(uwb_passive_ds_protocol_decode(
               payload, payload_len, &decoded) ==
           UWB_PASSIVE_DS_DECODE_OK);
    assert(decoded.type == packet->type);
    assert(decoded.session_id == packet->session_id);
    assert(decoded.frame_id == packet->frame_id);
    assert(decoded.initiator_id == packet->initiator_id);
    assert(decoded.anchor_count == packet->anchor_count);

    for (size_t byte = 0U; byte < payload_len; ++byte) {
        uint8_t damaged[UWB_PASSIVE_DS_MAX_PACKET_SIZE] = {0};
        memcpy(damaged, payload, payload_len);
        damaged[byte] ^= 0x40U;
        assert(uwb_passive_ds_protocol_decode(
                   damaged, payload_len, &decoded) !=
               UWB_PASSIVE_DS_DECODE_OK);
    }
    assert(uwb_passive_ds_protocol_decode(
               payload, payload_len - 1U, &decoded) ==
           UWB_PASSIVE_DS_DECODE_INVALID);
}

static void test_codec_and_crc(void)
{
    const struct uwb_passive_ds_packet poll = {
        .type = UWB_PASSIVE_DS_MESSAGE_POLL,
        .session_id = 0x12345678U,
        .frame_id = 42U,
        .initiator_id = 4U,
        .anchor_count = 4U,
    };
    round_trip(&poll);

    const struct uwb_passive_ds_packet response = {
        .type = UWB_PASSIVE_DS_MESSAGE_RESPONSE,
        .session_id = 0x12345678U,
        .frame_id = 42U,
        .initiator_id = 4U,
        .anchor_count = 4U,
        .responder_index = 2U,
        .responder_reply_dtu = 95846644U,
    };
    round_trip(&response);

    const struct uwb_passive_ds_packet final = {
        .type = UWB_PASSIVE_DS_MESSAGE_FINAL,
        .session_id = 0x12345678U,
        .frame_id = 42U,
        .initiator_id = 4U,
        .anchor_count = 4U,
        .initiator_poll_tx = 0x0102030405ULL,
        .initiator_final_tx = 0x1020304050ULL,
        .responder_count = 3U,
        .responders = {
            {3U, 0x1112131415ULL},
            {5U, 0x2122232425ULL},
            {2U, 0x3132333435ULL},
        },
    };
    round_trip(&final);
}

static void test_passive_observation_with_cfo_and_wrap(void)
{
    const uint64_t mask = UWB_PASSIVE_DS_TIMESTAMP_MASK;
    const uint64_t poll_rx = mask - 100000U;
    const uint32_t reply_dtu = 95846644U;
    const double cfo = 18.0e-6;
    const double baseline_m = 4.85;
    const double expected_difference_m = -1.275;
    const double baseline_dtu = baseline_m /
        (TEST_DTU_SECONDS * TEST_C_MPS);
    const double difference_dtu = expected_difference_m /
        (TEST_DTU_SECONDS * TEST_C_MPS);
    const uint64_t listener_interval = (uint64_t)llround(
        reply_dtu * (1.0 - cfo) + baseline_dtu + difference_dtu);
    const uint64_t response_rx = (poll_rx + listener_interval) & mask;
    const struct uwb_passive_ds_observation_input input = {
        .listener_poll_rx = poll_rx,
        .listener_response_rx = response_rx,
        .responder_reply_dtu = reply_dtu,
        .responder_to_listener_cfo_fraction = cfo,
        .initiator_responder_distance_m = baseline_m,
    };
    struct uwb_passive_ds_observation_result result = {0};
    assert(uwb_passive_ds_compute_observation(&input, &result));
    assert_near(result.difference_m, expected_difference_m, 0.003);
    assert(fabs(result.raw_difference_m - expected_difference_m) > 7.0);
    assert_near(result.cfo_correction_m,
                result.difference_m - result.raw_difference_m, 1.0e-12);
}

static void test_anchor_full_ds_math(void)
{
    const uint64_t poll_tx = 100000000ULL;
    const uint64_t tof = 900U;
    const uint64_t responder_delay = 64000000ULL;
    const uint64_t initiator_delay = 48000000ULL;
    const uint64_t poll_rx = poll_tx + tof;
    const uint64_t response_tx = poll_rx + responder_delay;
    const uint64_t response_rx = response_tx + tof;
    const uint64_t final_tx = response_rx + initiator_delay;
    const uint64_t final_rx = final_tx + tof;
    double distance_m = 0.0;
    assert(uwb_passive_ds_calculate_anchor_distance(
        poll_tx, poll_rx, response_tx, response_rx,
        final_tx, final_rx, &distance_m));
    assert_near(distance_m,
                tof * TEST_DTU_SECONDS * TEST_C_MPS, 1.0e-9);
}

int main(void)
{
    test_rotating_plan_and_timing();
    test_codec_and_crc();
    test_passive_observation_with_cfo_and_wrap();
    test_anchor_full_ds_math();
    puts("uwb_passive_ds_protocol_test: PASS");
    return 0;
}
