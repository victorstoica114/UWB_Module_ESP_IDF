#include "flextdoa_algmin.h"
#include "flextdoa_frame_aggregator.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static double distance(double first_x, double first_y,
                       double second_x, double second_y)
{
    return hypot(first_x - second_x, first_y - second_y);
}

static struct flextdoa_range_difference make_difference(
    uint16_t initiator_id, uint16_t responder_id, double value_m)
{
    return (struct flextdoa_range_difference){
        .initiator_id = initiator_id,
        .responder_id = responder_id,
        .range_difference_m = value_m,
    };
}

static size_t build_complete_observations(
    const struct flextdoa_anchor_position *anchors,
    size_t anchor_count, double tag_x, double tag_y,
    struct flextdoa_range_difference *observations, size_t capacity)
{
    size_t count = 0U;
    for (size_t initiator = 0U; initiator < anchor_count; ++initiator) {
        for (size_t responder = 0U; responder < anchor_count; ++responder) {
            if (responder == initiator) {
                continue;
            }
            assert(count < capacity);
            observations[count++] = make_difference(
                anchors[initiator].anchor_id,
                anchors[responder].anchor_id,
                distance(tag_x, tag_y, anchors[responder].x_m,
                         anchors[responder].y_m) -
                    distance(tag_x, tag_y, anchors[initiator].x_m,
                             anchors[initiator].y_m));
        }
    }
    return count;
}

static void test_frame_boundaries_duplicates_and_threshold(void)
{
    uint32_t slot_ids[12] = {0};
    struct flextdoa_range_difference observations[12] = {0};
    struct flextdoa_frame_aggregator frame = {0};
    flextdoa_frame_aggregator_init(
        &frame, slot_ids, observations, 12U);

    const struct flextdoa_range_difference first =
        make_difference(2U, 3U, 0.10);
    assert(flextdoa_frame_aggregator_ingest(
               &frame, 8U, 4U, 3U, 0U, &first) ==
           FLEXTDOA_FRAME_REJECTED);
    assert(!frame.active);
    assert(flextdoa_frame_aggregator_ingest(
               &frame, 8U, 4U, 3U, 4U, &first) ==
           FLEXTDOA_FRAME_ACCEPTED);
    assert(frame.active && frame.frame_id == 2U);
    assert(frame.observation_count == 1U);

    const struct flextdoa_range_difference conflicting_duplicate =
        make_difference(2U, 3U, 0.90);
    assert(flextdoa_frame_aggregator_ingest(
               &frame, 8U, 4U, 3U, 4U,
               &conflicting_duplicate) == FLEXTDOA_FRAME_DUPLICATE);
    assert(frame.observation_count == 1U);
    assert(frame.observations[0].range_difference_m == 0.10);

    const struct flextdoa_range_difference second =
        make_difference(2U, 4U, -0.20);
    const struct flextdoa_range_difference third =
        make_difference(2U, 5U, 0.30);
    const struct flextdoa_range_difference fourth =
        make_difference(3U, 2U, -0.10);
    assert(flextdoa_frame_aggregator_ingest(
               &frame, 8U, 4U, 3U, 4U, &second) ==
           FLEXTDOA_FRAME_ACCEPTED);
    assert(flextdoa_frame_aggregator_ingest(
               &frame, 8U, 4U, 3U, 4U, &third) ==
           FLEXTDOA_FRAME_ACCEPTED);
    assert(!flextdoa_frame_aggregator_solvable(&frame));
    assert(flextdoa_frame_aggregator_ingest(
               &frame, 9U, 4U, 3U, 4U, &fourth) ==
           FLEXTDOA_FRAME_ACCEPTED);
    assert(flextdoa_frame_aggregator_solvable(&frame));
    assert(!flextdoa_frame_aggregator_complete(&frame));

    struct flextdoa_range_difference corrected[12] = {0};
    assert(flextdoa_frame_aggregator_copy_corrected(
               &frame, NULL, 0U, corrected, 12U) == 4U);
    assert(corrected[0].range_difference_m == 0.10);
    /* These test values are already signed corrections, not measured raw
     * offsets. A measured +0.10 m offset would be configured as -0.10 m. */
    const struct flextdoa_anchor_bias biases[] = {
        {.anchor_id = 2U, .bias_m = 0.10},
        {.anchor_id = 3U, .bias_m = 0.25},
    };
    assert(flextdoa_frame_aggregator_copy_corrected(
               &frame, biases, 2U, corrected, 12U) == 4U);
    assert(fabs(corrected[0].range_difference_m - 0.25) < 1.0e-12);
    assert(fabs(corrected[3].range_difference_m + 0.25) < 1.0e-12);
    assert(frame.observations[0].range_difference_m == 0.10);

    const struct flextdoa_range_difference next_frame =
        make_difference(2U, 3U, 0.11);
    assert(flextdoa_frame_aggregator_ingest(
               &frame, 12U, 4U, 3U, 4U, &next_frame) ==
           FLEXTDOA_FRAME_BOUNDARY);
    assert(frame.frame_id == 2U && frame.observation_count == 4U);

    flextdoa_frame_aggregator_reset(&frame);
    assert(flextdoa_frame_aggregator_ingest(
               &frame, 12U, 4U, 3U, 4U, &next_frame) ==
           FLEXTDOA_FRAME_ACCEPTED);
    assert(frame.frame_id == 3U && frame.observation_count == 1U);
    assert(flextdoa_frame_aggregator_ingest(
               &frame, 11U, 4U, 3U, 4U, &next_frame) ==
           FLEXTDOA_FRAME_REJECTED);
}

static void test_complete_frame_produces_one_solution(
    const struct flextdoa_anchor_position *anchors,
    size_t anchor_count, double tag_x, double tag_y)
{
    uint32_t slot_ids[12] = {0};
    struct flextdoa_range_difference observations[12] = {0};
    struct flextdoa_frame_aggregator frame = {0};
    flextdoa_frame_aggregator_init(
        &frame, slot_ids, observations, 12U);

    unsigned solution_count = 0U;
    for (uint32_t slot_id = 0U; slot_id < 4U; ++slot_id) {
        const size_t initiator_index = slot_id;
        for (size_t responder_index = 0U;
             responder_index < anchor_count; ++responder_index) {
            if (responder_index == initiator_index) {
                continue;
            }
            const double measured =
                distance(tag_x, tag_y,
                         anchors[responder_index].x_m,
                         anchors[responder_index].y_m) -
                distance(tag_x, tag_y,
                         anchors[initiator_index].x_m,
                         anchors[initiator_index].y_m);
            const struct flextdoa_range_difference observation =
                make_difference(
                    anchors[initiator_index].anchor_id,
                    anchors[responder_index].anchor_id, measured);
            const enum flextdoa_frame_ingest_result ingest =
                flextdoa_frame_aggregator_ingest(
                    &frame, slot_id, 4U, 3U, 12U, &observation);
            if (ingest == FLEXTDOA_FRAME_COMPLETE) {
                solution_count++;
                struct flextdoa_algmin_result result = {0};
                assert(flextdoa_algmin_solve_2d(
                    anchors, anchor_count, frame.observations,
                    frame.observation_count, NULL, &result));
                assert(result.valid);
                assert(fabs(result.x_m - tag_x) < 1.0e-5);
                assert(fabs(result.y_m - tag_y) < 1.0e-5);
            } else {
                assert(ingest == FLEXTDOA_FRAME_ACCEPTED);
            }
        }
    }
    assert(frame.observation_count == 12U);
    assert(flextdoa_frame_aggregator_complete(&frame));
    assert(solution_count == 1U);

    const struct flextdoa_range_difference duplicate =
        frame.observations[0];
    assert(flextdoa_frame_aggregator_ingest(
               &frame, 0U, 4U, 3U, 12U, &duplicate) ==
           FLEXTDOA_FRAME_DUPLICATE);
    assert(solution_count == 1U);
}

static void test_conservative_partial_frame_and_geometry_gate(
    const struct flextdoa_anchor_position *anchors,
    size_t anchor_count, double tag_x, double tag_y)
{
    struct flextdoa_range_difference complete[12] = {0};
    assert(build_complete_observations(
               anchors, anchor_count, tag_x, tag_y, complete, 12U) ==
           12U);

    uint32_t slot_ids[12] = {0};
    struct flextdoa_range_difference stored[12] = {0};
    struct flextdoa_frame_aggregator frame = {0};
    flextdoa_frame_aggregator_init(&frame, slot_ids, stored, 12U);
    for (size_t index = 0U; index < 12U; ++index) {
        /* Lose one response in each of two slots, retaining all four
         * initiators: this is the runtime 10/12 acceptance boundary. */
        if (index == 1U || index == 4U) {
            continue;
        }
        const enum flextdoa_frame_ingest_result ingest =
            flextdoa_frame_aggregator_ingest(
                &frame, (uint32_t)(index / 3U), 4U, 3U, 10U,
                &complete[index]);
        assert(ingest == FLEXTDOA_FRAME_ACCEPTED);
    }
    assert(frame.observation_count == 10U);
    assert(flextdoa_frame_aggregator_solvable(&frame));
    assert(!flextdoa_frame_aggregator_complete(&frame));

    struct flextdoa_algmin_result result = {0};
    assert(flextdoa_algmin_solve_2d(
        anchors, anchor_count, frame.observations,
        frame.observation_count, NULL, &result));
    assert(fabs(result.x_m - tag_x) < 1.0e-5);
    assert(fabs(result.y_m - tag_y) < 1.0e-5);
    struct flextdoa_solution_gate_metrics metrics = {0};
    assert(flextdoa_algmin_gate_solution_2d(
               anchors, anchor_count, frame.observations,
               frame.observation_count, result.x_m, result.y_m, 4U,
               10.0, &metrics) == FLEXTDOA_SOLUTION_GATE_OK);
    assert(metrics.initiator_count == 4U);
    assert(metrics.connected_anchor_count == 4U);
    assert(metrics.normal_condition_number < 10.0);

    const struct flextdoa_range_difference next = complete[0];
    assert(flextdoa_frame_aggregator_ingest(
               &frame, 4U, 4U, 3U, 10U, &next) ==
           FLEXTDOA_FRAME_BOUNDARY);

    const struct flextdoa_range_difference disconnected[] = {
        make_difference(2U, 3U, 0.0),
        make_difference(3U, 2U, 0.0),
        make_difference(4U, 5U, 0.0),
        make_difference(5U, 4U, 0.0),
    };
    assert(flextdoa_algmin_gate_solution_2d(
               anchors, anchor_count, disconnected, 4U, tag_x, tag_y,
               4U, 10.0, &metrics) ==
           FLEXTDOA_SOLUTION_GATE_DISCONNECTED);

    assert(flextdoa_algmin_gate_solution_2d(
               anchors, anchor_count, complete, 3U, tag_x, tag_y, 4U,
               10.0, &metrics) ==
           FLEXTDOA_SOLUTION_GATE_TOO_FEW_INITIATORS);

    const struct flextdoa_anchor_position collinear[] = {
        {.anchor_id = 2U, .x_m = 0.0, .y_m = 0.0},
        {.anchor_id = 3U, .x_m = 1.0, .y_m = 0.0},
        {.anchor_id = 4U, .x_m = 2.0, .y_m = 0.0},
        {.anchor_id = 5U, .x_m = 3.0, .y_m = 0.0},
    };
    struct flextdoa_range_difference collinear_observations[12] = {0};
    assert(build_complete_observations(
               collinear, 4U, 1.5, 0.0, collinear_observations, 12U) ==
           12U);
    assert(flextdoa_algmin_gate_solution_2d(
               collinear, 4U, collinear_observations, 12U, 1.5, 0.0,
               4U, 10.0, &metrics) ==
           FLEXTDOA_SOLUTION_GATE_RANK_DEFICIENT);
}

int main(void)
{
    const struct flextdoa_anchor_position anchors[] = {
        {.anchor_id = 2U, .x_m = 0.0, .y_m = 0.0},
        {.anchor_id = 3U, .x_m = 5.0, .y_m = 0.0},
        {.anchor_id = 4U, .x_m = 5.0, .y_m = 4.0},
        {.anchor_id = 5U, .x_m = 0.0, .y_m = 4.0},
    };
    const double tag_x = 1.2;
    const double tag_y = 2.3;
    struct flextdoa_range_difference observations[3] = {0};
    for (size_t index = 0U; index < 3U; ++index) {
        observations[index].initiator_id = 2U;
        observations[index].responder_id = (uint16_t)(3U + index);
        observations[index].range_difference_m =
            distance(tag_x, tag_y, anchors[index + 1U].x_m,
                     anchors[index + 1U].y_m) -
            distance(tag_x, tag_y, anchors[0].x_m, anchors[0].y_m);
    }

    struct flextdoa_algmin_result result = {0};
    assert(flextdoa_algmin_solve_2d(
        anchors, 4U, observations, 3U, NULL, &result));
    assert(result.valid);
    assert(fabs(result.x_m - tag_x) < 1.0e-5);
    assert(fabs(result.y_m - tag_y) < 1.0e-5);
    assert(result.residual_rms_m < 1.0e-6);

    const struct flextdoa_algmin_seed previous = {
        .valid = true,
        .x_m = 1.19,
        .y_m = 2.31,
    };
    assert(flextdoa_algmin_solve_2d(
        anchors, 4U, observations, 3U, &previous, &result));
    assert(result.valid);
    assert(fabs(result.x_m - tag_x) < 1.0e-5);
    assert(fabs(result.y_m - tag_y) < 1.0e-5);

    test_frame_boundaries_duplicates_and_threshold();
    test_complete_frame_produces_one_solution(
        anchors, 4U, tag_x, tag_y);
    test_conservative_partial_frame_and_geometry_gate(
        anchors, 4U, tag_x, tag_y);

    puts("flextdoa_algmin_test: PASS");
    return 0;
}
