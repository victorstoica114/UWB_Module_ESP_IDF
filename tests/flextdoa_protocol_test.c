#include "flextdoa_protocol.h"
#include "flextdoa_collector.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define TEST_DTU_SECONDS 15.650040064102564e-12
#define TEST_C_MPS 299702547.0

static void assert_near(double actual, double expected, double tolerance)
{
    assert(fabs(actual - expected) <= tolerance);
}

static void test_paper_timing(void)
{
    assert(flextdoa_slot_duration_us(&FLEXTDOA_PAPER_TIMING, 3U) ==
           5050U);
    assert(flextdoa_frame_duration_us(&FLEXTDOA_PAPER_TIMING, 3U, 4U) ==
           20200U);
}

static void test_ci_cr_schedule(void)
{
    const uint16_t anchors[] = {2U, 3U, 4U, 5U};
    const uint16_t slots[] = {2U, 3U, 4U, 5U};
    const uint16_t masks[] = {0x0EU, 0x0DU, 0x0BU, 0x07U};
    const uint8_t expected[][3] = {
        {3U, 4U, 5U},
        {4U, 5U, 2U},
        {5U, 2U, 3U},
        {2U, 3U, 4U},
    };

    for (uint32_t slot_id = 0U; slot_id < 4U; ++slot_id) {
        struct flextdoa_slot_plan plan = {0};
        assert(flextdoa_build_ci_cr_slot(
            anchors, 4U, slots, 4U, masks, 3U, slot_id, &plan));
        assert(plan.slot_id == slot_id);
        assert(plan.slot_index == slot_id);
        assert(plan.initiator_id == anchors[slot_id]);
        assert(plan.responder_count == 3U);
        assert(memcmp(plan.responder_ids, expected[slot_id], 3U) == 0);
        assert(flextdoa_responder_index(
                   &plan, expected[slot_id][1]) == 1);
        assert(flextdoa_responder_index(&plan, 99U) == -1);
    }
}

static void test_slot_collector(void)
{
    const struct flextdoa_packet request = {
        .type = FLEXTDOA_MESSAGE_REQUEST,
        .slot_id = 42U,
        .source_id = 2U,
        .destination_count = 3U,
        .destination_ids = {3U, 4U, 5U},
    };
    struct flextdoa_slot_collection collection = {0};
    assert(flextdoa_collector_ingest(
               &collection, &request, 1000U, false, 0.0) ==
           FLEXTDOA_COLLECT_REQUEST);
    assert(!flextdoa_collector_complete(&collection));

    for (uint8_t index = 0U; index < 3U; ++index) {
        const struct flextdoa_packet response = {
            .type = FLEXTDOA_MESSAGE_RESPONSE,
            .slot_id = 42U,
            .source_id = (uint16_t)(3U + index),
            .processing_time_dtu = (uint32_t)(100U + index),
        };
        const enum flextdoa_collect_result result =
            flextdoa_collector_ingest(
                &collection, &response, 1200U + index, true, 10.0e-6);
        assert(result == (index == 2U ? FLEXTDOA_COLLECT_COMPLETE
                                     : FLEXTDOA_COLLECT_RESPONSE));
    }
    assert(flextdoa_collector_complete(&collection));
    assert(collection.received_mask == 0x07U);

    const struct flextdoa_packet stale_response = {
        .type = FLEXTDOA_MESSAGE_RESPONSE,
        .slot_id = 41U,
        .source_id = 3U,
        .processing_time_dtu = 100U,
    };
    assert(flextdoa_collector_ingest(
               &collection, &stale_response, 1300U, true, 0.0) ==
           FLEXTDOA_COLLECT_REJECTED);
}

static void test_packet_codec(void)
{
    const struct flextdoa_packet request = {
        .type = FLEXTDOA_MESSAGE_REQUEST,
        .slot_id = 0x10203040U,
        .source_id = 2U,
        .destination_count = 3U,
        .destination_ids = {3U, 4U, 5U},
        .processing_time_dtu = 63897764U,
        .previous_twr_responder_id = 5U,
        .previous_twr_mm = 4321U,
        .previous_slot_id = 0x1234U,
    };
    uint8_t payload[64] = {0};
    const size_t encoded_size =
        flextdoa_encode_packet(&request, payload, sizeof(payload));
    assert(encoded_size == 21U);
    assert(payload[0] == FLEXTDOA_MESSAGE_REQUEST);
    assert(payload[1] == 0x40U && payload[4] == 0x10U);
    assert(payload[7] == 3U);

    struct flextdoa_packet decoded = {0};
    assert(flextdoa_decode_packet(payload, encoded_size, &decoded));
    assert(decoded.type == request.type);
    assert(decoded.slot_id == request.slot_id);
    assert(decoded.source_id == request.source_id);
    assert(decoded.destination_count == request.destination_count);
    assert(memcmp(decoded.destination_ids, request.destination_ids, 3U) ==
           0);
    assert(decoded.processing_time_dtu == request.processing_time_dtu);
    assert(decoded.previous_twr_responder_id ==
           request.previous_twr_responder_id);
    assert(decoded.previous_twr_mm == request.previous_twr_mm);
    assert(decoded.previous_slot_id == request.previous_slot_id);
    assert(!flextdoa_decode_packet(payload, encoded_size - 1U, &decoded));

    const struct flextdoa_packet response = {
        .type = FLEXTDOA_MESSAGE_RESPONSE,
        .slot_id = 7U,
        .source_id = 4U,
        .processing_time_dtu = 424242U,
    };
    assert(flextdoa_encode_packet(
               &response, payload, sizeof(payload)) == 18U);
    assert(flextdoa_decode_packet(payload, 18U, &decoded));
    assert(decoded.type == FLEXTDOA_MESSAGE_RESPONSE);
    assert(decoded.destination_count == 0U);
}

static void test_observation_math_and_wrap(void)
{
    const uint64_t timestamp_mask = (1ULL << 40U) - 1ULL;
    assert(flextdoa_timestamp_delta(25U, timestamp_mask - 74U) == 100U);

    const double cfo = 20.0e-6;
    const uint32_t processing_dtu = 150000U;
    const double anchor_tof_dtu = 10000.0;
    const double expected_tdoa_dtu = 40003.0;
    const struct flextdoa_observation_input input = {
        .request_rx_tag_dtu = timestamp_mask - 99999U,
        .response_rx_tag_dtu = 100000U,
        .responder_processing_dtu = processing_dtu,
        .responder_to_tag_cfo_fraction = cfo,
        .initiator_responder_tof_dtu = anchor_tof_dtu,
        .dtu_seconds = TEST_DTU_SECONDS,
        .speed_of_light_mps = TEST_C_MPS,
    };
    double range_difference_m = 0.0;
    assert(flextdoa_compute_range_difference_m(
        &input, &range_difference_m));
    assert_near(
        range_difference_m,
        expected_tdoa_dtu * TEST_DTU_SECONDS * TEST_C_MPS,
        1.0e-9);
}

int main(void)
{
    test_paper_timing();
    test_ci_cr_schedule();
    test_packet_codec();
    test_slot_collector();
    test_observation_math_and_wrap();
    puts("flextdoa_protocol_test: PASS");
    return 0;
}
