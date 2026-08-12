#include "uwb_passive_ds_observation.h"
#include "uwb_passive_ds_protocol.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEST_DTU_SECONDS 15.650040064102564e-12
#define TEST_C_MPS 299702547.0
#define TEST_TS_MASK ((1ULL << 40U) - 1ULL)

struct test_clock {
    double scale;
    double offset;
};

static uint64_t test_clock_timestamp(struct test_clock clock,
                                     double physical_dtu)
{
    return (uint64_t)llround(
               physical_dtu * clock.scale + clock.offset) &
           TEST_TS_MASK;
}

static double test_tof_dtu(double first_x, double first_y,
                           double second_x, double second_y)
{
    return hypot(first_x - second_x, first_y - second_y) /
           (TEST_C_MPS * TEST_DTU_SECONDS);
}

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
    assert(uwb_passive_ds_response_delay_us(1750U, 750U, 2U) ==
           3250U);
    assert(uwb_passive_ds_final_delay_from_poll_us(
               1750U, 750U, 3U, 1750U) == 5000U);
    assert(uwb_passive_ds_protocol_packet_size(
               UWB_PASSIVE_DS_MESSAGE_POLL, 0U) == 60U);
    assert(uwb_passive_ds_protocol_packet_size(
               UWB_PASSIVE_DS_MESSAGE_RESPONSE, 0U) == 65U);
    assert(uwb_passive_ds_protocol_packet_size(
               UWB_PASSIVE_DS_MESSAGE_FINAL, 3U) == 47U);

    uint32_t next_frame_id = 0U;
    uint8_t frame_offset = 0U;
    assert(uwb_passive_ds_next_owned_frame(
        anchors, 4U, 5U, 2U, &next_frame_id, &frame_offset));
    assert(next_frame_id == 8U);
    assert(frame_offset == 3U);
    assert(uwb_passive_ds_next_owned_frame(
        anchors, 4U, 5U, 4U, &next_frame_id, &frame_offset));
    assert(next_frame_id == 6U);
    assert(frame_offset == 1U);

    /* uint32_t frame wrap keeps the modulo rotation deterministic. */
    assert(uwb_passive_ds_next_owned_frame(
        anchors, 4U, UINT32_MAX - 1U, 2U,
        &next_frame_id, &frame_offset));
    assert(next_frame_id == 0U);
    assert(frame_offset == 2U);
    assert(!uwb_passive_ds_next_owned_frame(
        anchors, 4U, 5U, 9U, &next_frame_id, &frame_offset));

    const uint32_t boundary_frames[] = {
        0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
        UINT32_MAX - 3U, UINT32_MAX - 2U,
        UINT32_MAX - 1U, UINT32_MAX,
    };
    for (size_t frame_index = 0U;
         frame_index < sizeof(boundary_frames) /
                           sizeof(boundary_frames[0]);
         ++frame_index) {
        const uint32_t after = boundary_frames[frame_index];
        for (size_t local_index = 0U; local_index < 4U;
             ++local_index) {
            assert(uwb_passive_ds_next_owned_frame(
                anchors, 4U, after, anchors[local_index],
                &next_frame_id, &frame_offset));
            assert(frame_offset >= 1U && frame_offset <= 4U);
            assert(next_frame_id == after + frame_offset);
            struct uwb_passive_ds_plan owned = {0};
            assert(uwb_passive_ds_build_plan(
                anchors, 4U, next_frame_id, &owned));
            assert(owned.initiator_id == anchors[local_index]);
            for (uint8_t earlier = 1U; earlier < frame_offset;
                 ++earlier) {
                struct uwb_passive_ds_plan skipped = {0};
                assert(uwb_passive_ds_build_plan(
                    anchors, 4U, after + earlier, &skipped));
                assert(skipped.initiator_id != anchors[local_index]);
            }
        }
    }

    /* If frame 101 is lost, A4 still keeps frame 102 scheduled from
     * frame 100; the rotation no longer depends on a one-hop daisy chain. */
    assert(uwb_passive_ds_next_owned_frame(
        anchors, 4U, 100U, 4U, &next_frame_id, &frame_offset));
    assert(next_frame_id == 102U);
    assert(frame_offset == 2U);

    assert(!uwb_passive_ds_next_owned_frame(
        anchors, 2U, 5U, 2U, &next_frame_id, &frame_offset));
    assert(!uwb_passive_ds_next_owned_frame(
        anchors, 4U, 5U, 2U, NULL, &frame_offset));
    assert(!uwb_passive_ds_next_owned_frame(
        anchors, 4U, 5U, 2U, &next_frame_id, NULL));
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
    if (packet->type == UWB_PASSIVE_DS_MESSAGE_POLL ||
        packet->type == UWB_PASSIVE_DS_MESSAGE_RESPONSE) {
        assert(decoded.sender_position.flags ==
               packet->sender_position.flags);
        assert(decoded.sender_position.age_ms ==
               packet->sender_position.age_ms);
        assert(decoded.sender_position.latitude_e7 ==
               packet->sender_position.latitude_e7);
        assert(decoded.sender_position.longitude_e7 ==
               packet->sender_position.longitude_e7);
        assert(decoded.sender_position.velocity_east_mmps ==
               packet->sender_position.velocity_east_mmps);
        assert(decoded.sender_position.velocity_north_mmps ==
               packet->sender_position.velocity_north_mmps);
    }
    if (packet->type == UWB_PASSIVE_DS_MESSAGE_POLL ||
        packet->type == UWB_PASSIVE_DS_MESSAGE_RESPONSE) {
        if (packet->type == UWB_PASSIVE_DS_MESSAGE_RESPONSE) {
            assert(decoded.responder_index == packet->responder_index);
            assert(decoded.responder_reply_dtu ==
                   packet->responder_reply_dtu);
        }
        assert(decoded.completed_exchange_count ==
               packet->completed_exchange_count);
        for (uint8_t index = 0U;
             index < packet->completed_exchange_count; ++index) {
            assert(decoded.completed_exchanges[index].session_id ==
                   packet->completed_exchanges[index].session_id);
            assert(decoded.completed_exchanges[index].frame_id ==
                   packet->completed_exchanges[index].frame_id);
            assert(decoded.completed_exchanges[index].initiator_id ==
                   packet->completed_exchanges[index].initiator_id);
            assert(decoded.completed_exchanges[index]
                       .responder_exchange_dtu ==
                   packet->completed_exchanges[index]
                       .responder_exchange_dtu);
        }
    }

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
        .sender_position = {
            .flags = UWB_PASSIVE_DS_POSITION_VALID |
                     UWB_PASSIVE_DS_POSITION_RTK_FIXED |
                     UWB_PASSIVE_DS_POSITION_VELOCITY_VALID,
            .age_ms = 37U,
            .latitude_e7 = 444355123,
            .longitude_e7 = 260973456,
            .velocity_east_mmps = 1250,
            .velocity_north_mmps = -340,
        },
        .completed_exchange_count = 1U,
        .completed_exchanges = {
            {0x12345678U, 41U, 3U, 287538800U},
        },
    };
    round_trip(&poll);

    const struct uwb_passive_ds_packet response = {
        .type = UWB_PASSIVE_DS_MESSAGE_RESPONSE,
        .session_id = 0x12345678U,
        .frame_id = 42U,
        .initiator_id = 4U,
        .anchor_count = 4U,
        .sender_position = {
            /* A valid non-fixed sample must remain encodable after mobile
             * geometry activation, preserving RTK Float/SPS continuity. */
            .flags = UWB_PASSIVE_DS_POSITION_VALID,
            .age_ms = 81U,
            .latitude_e7 = 444355456,
            .longitude_e7 = 260973987,
        },
        .responder_index = 2U,
        .responder_reply_dtu = 95846644U,
        .completed_exchange_count = 2U,
        .completed_exchanges = {
            {0x12345678U, 40U, 2U, 287539000U},
            {0x12345678U, 39U, 5U, 287538900U},
        },
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

static void test_passive_double_sided_observation_with_wrap(void)
{
    const double initiator_x = 0.0;
    const double initiator_y = 0.0;
    const double responder_x = 4.8;
    const double responder_y = 0.0;
    const double listener_x = 1.2;
    const double listener_y = 2.1;
    const double poll_tx_physical = 1000000000.0;
    const double anchor_tof = test_tof_dtu(
        initiator_x, initiator_y, responder_x, responder_y);
    const double initiator_listener_tof = test_tof_dtu(
        initiator_x, initiator_y, listener_x, listener_y);
    const double responder_listener_tof = test_tof_dtu(
        responder_x, responder_y, listener_x, listener_y);
    const double responder_poll_rx_physical =
        poll_tx_physical + anchor_tof;
    const double responder_reply_physical = 95846644.0;
    const double responder_tx_physical =
        responder_poll_rx_physical + responder_reply_physical;
    const double initiator_response_rx_physical =
        responder_tx_physical + anchor_tof;
    const double initiator_reply_physical = 95846644.0;
    const double initiator_final_tx_physical =
        initiator_response_rx_physical + initiator_reply_physical;
    const double responder_final_rx_physical =
        initiator_final_tx_physical + anchor_tof;

    /* Put each clock's POLL timestamp just before its independent 40-bit
     * wrap. RESPONSE and FINAL must therefore exercise modulo deltas. */
    const double wrap_margin_dtu = 50000000.0;
    const struct test_clock initiator = {
        1.000012,
        (double)TEST_TS_MASK - wrap_margin_dtu -
            poll_tx_physical * 1.000012};
    const struct test_clock responder = {
        0.999986,
        (double)TEST_TS_MASK - wrap_margin_dtu -
            responder_poll_rx_physical * 0.999986};
    const struct test_clock listener = {
        0.999991,
        (double)TEST_TS_MASK - wrap_margin_dtu -
            (poll_tx_physical + initiator_listener_tof) * 0.999991};

    const uint64_t responder_poll_rx = test_clock_timestamp(
        responder, responder_poll_rx_physical);
    const uint64_t responder_tx = test_clock_timestamp(
        responder, responder_tx_physical);
    const uint64_t responder_final_rx = test_clock_timestamp(
        responder, responder_final_rx_physical);
    const uint64_t initiator_poll_tx = test_clock_timestamp(
        initiator, poll_tx_physical);
    const uint64_t initiator_response_rx = test_clock_timestamp(
        initiator, initiator_response_rx_physical);
    const uint64_t listener_poll_rx = test_clock_timestamp(
        listener, poll_tx_physical + initiator_listener_tof);
    const uint64_t listener_response_rx = test_clock_timestamp(
        listener, responder_tx_physical + responder_listener_tof);
    assert(responder_tx < responder_poll_rx);
    assert(initiator_response_rx < initiator_poll_tx);
    assert(listener_response_rx < listener_poll_rx);
    const struct uwb_passive_ds_observation_input input = {
        .listener_poll_rx = listener_poll_rx,
        .listener_response_rx = listener_response_rx,
        .listener_final_rx = test_clock_timestamp(
            listener,
            initiator_final_tx_physical + initiator_listener_tof),
        .initiator_poll_tx = initiator_poll_tx,
        .initiator_response_rx = initiator_response_rx,
        .initiator_final_tx = test_clock_timestamp(
            initiator, initiator_final_tx_physical),
        .responder_reply_dtu = (uint32_t)(
            (responder_tx - responder_poll_rx) & TEST_TS_MASK),
        .responder_exchange_dtu = (uint32_t)(
            (responder_final_rx - responder_poll_rx) & TEST_TS_MASK),
    };
    struct uwb_passive_ds_observation_result result = {0};
    assert(uwb_passive_ds_compute_observation(&input, &result));
    const double expected_difference_m =
        hypot(listener_x - responder_x, listener_y - responder_y) -
        hypot(listener_x - initiator_x, listener_y - initiator_y);
    assert_near(result.difference_m, expected_difference_m, 0.005);
    assert_near(result.listener_to_initiator_clock_ratio,
                listener.scale / initiator.scale, 1.0e-8);
    assert_near(result.listener_to_responder_clock_ratio,
                listener.scale / responder.scale, 1.0e-8);
    assert(result.responder_delay_ratio > 0.0);
    assert(result.responder_delay_ratio < 1.0);
    const double expected_interpolation_ratio = 0.5 * (
        ((double)uwb_passive_ds_timestamp_delta(
             input.initiator_response_rx, input.initiator_poll_tx) /
         (double)uwb_passive_ds_timestamp_delta(
             input.initiator_final_tx, input.initiator_poll_tx)) +
        ((double)input.responder_reply_dtu /
         (double)input.responder_exchange_dtu));
    assert_near(result.listener_interpolation_ratio,
                expected_interpolation_ratio, 1.0e-12);
    assert(result.listener_interpolation_ratio > 0.0);
    assert(result.listener_interpolation_ratio < 1.0);
}

static void test_cfo_observation_is_diagnostic_only(void)
{
    const uint64_t poll_rx = TEST_TS_MASK - 100000U;
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
    const struct uwb_passive_ds_cfo_observation_input input = {
        .listener_poll_rx = poll_rx,
        .listener_response_rx =
            (poll_rx + listener_interval) & TEST_TS_MASK,
        .responder_reply_dtu = reply_dtu,
        .responder_to_listener_cfo_fraction = cfo,
        .initiator_responder_distance_m = baseline_m,
    };
    struct uwb_passive_ds_cfo_observation_result result = {0};
    assert(uwb_passive_ds_compute_cfo_observation(&input, &result));
    assert_near(result.difference_m, expected_difference_m, 0.003);
    assert(fabs(result.raw_difference_m - expected_difference_m) > 7.0);
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
    test_passive_double_sided_observation_with_wrap();
    test_cfo_observation_is_diagnostic_only();
    test_anchor_full_ds_math();
    puts("uwb_passive_ds_protocol_test: PASS");
    return 0;
}
