#include "uwb_native_ds_position_solver.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void assert_near(float actual, float expected, float tolerance)
{
    assert(fabsf(actual - expected) <= tolerance);
}

static float range_to(float tag_x, float tag_y, float anchor_x,
                      float anchor_y)
{
    return hypotf(tag_x - anchor_x, tag_y - anchor_y);
}

static void submit_complete_frame(
    struct uwb_native_ds_position_solver *solver, const uint8_t *anchor_ids,
    const float *anchor_x, const float *anchor_y, size_t anchor_count,
    uint32_t frame_id, float tag_x, float tag_y,
    struct uwb_native_ds_position_output *output)
{
    for (size_t index = 0U; index < anchor_count; ++index) {
        assert(uwb_native_ds_position_solver_submit_tag_range(
            solver, 1U, anchor_ids[index], frame_id,
            range_to(tag_x, tag_y, anchor_x[index], anchor_y[index]),
            output));
        assert(output->position_valid == (index + 1U == anchor_count));
    }
}

int main(void)
{
    const uint8_t anchors[] = {2U, 3U, 4U, 5U};
    const float anchor_x[] = {0.000f, 3.844f, -3.120f, 1.055f};
    const float anchor_y[] = {0.000f, 2.923f, 3.899f, 6.836f};
    const size_t anchor_count = sizeof(anchors) / sizeof(anchors[0]);
    struct uwb_native_ds_position_solver solver;
    assert(uwb_native_ds_position_solver_init(
        &solver, 1U, anchors, anchor_x, anchor_y, anchor_count, 29U));

    struct uwb_native_ds_position_output output = {0};
    assert(uwb_native_ds_position_solver_geometry(&solver, &output));
    assert(output.geometry_updated);
    assert(output.geometry_version == 29U);
    assert_near(output.anchor_x_m[2], -3.120f, 0.0001f);
    assert_near(output.anchor_y_m[3], 6.836f, 0.0001f);

    /* A coherent 3/4 frame is finalized at the next frame boundary. */
    const float tag_x = 0.332f;
    const float tag_y = 3.392f;
    for (size_t index = 0U; index < anchor_count - 1U; ++index) {
        assert(uwb_native_ds_position_solver_submit_tag_range(
            &solver, 1U, anchors[index], 100U,
            range_to(tag_x, tag_y, anchor_x[index], anchor_y[index]),
            &output));
        assert(!output.position_valid);
    }
    assert(uwb_native_ds_position_solver_submit_tag_range(
        &solver, 1U, anchors[3], 101U,
        range_to(tag_x, tag_y, anchor_x[3], anchor_y[3]), &output));
    assert(output.position_valid);
    assert(output.frame_id == 100U);
    assert(output.observation_count == 3U);
    assert_near(output.x_m, tag_x, 0.001f);
    assert_near(output.y_m, tag_y, 0.001f);

    /* A late range cannot alter or re-emit the finalized older frame. */
    assert(uwb_native_ds_position_solver_submit_tag_range(
        &solver, 1U, anchors[3], 100U,
        range_to(tag_x, tag_y, anchor_x[3], anchor_y[3]), &output));
    assert(!output.position_valid);
    for (size_t index = 0U; index < anchor_count - 1U; ++index) {
        assert(uwb_native_ds_position_solver_submit_tag_range(
            &solver, 1U, anchors[index], 101U,
            range_to(tag_x, tag_y, anchor_x[index], anchor_y[index]),
            &output));
    }
    assert(output.position_valid);
    assert(output.frame_id == 101U);
    assert(output.observation_count == 4U);
    assert_near(output.x_m, tag_x, 0.001f);
    assert_near(output.y_m, tag_y, 0.001f);
    assert(output.rms_m < 0.001f);

    /* Every frame is solved from its own raw ranges, not the prior solution. */
    submit_complete_frame(&solver, anchors, anchor_x, anchor_y, anchor_count,
                          102U, -0.650f, 1.175f, &output);
    assert_near(output.x_m, -0.650f, 0.001f);
    assert_near(output.y_m, 1.175f, 0.001f);
    assert(output.observation_count == 4U);

    /* A 2/4 frame is discarded, while the first range of the next frame is
     * retained and can still complete a full solution. */
    for (size_t index = 0U; index < 2U; ++index) {
        assert(uwb_native_ds_position_solver_submit_tag_range(
            &solver, 1U, anchors[index], 103U,
            range_to(tag_x, tag_y, anchor_x[index], anchor_y[index]),
            &output));
        assert(!output.position_valid);
    }
    assert(uwb_native_ds_position_solver_submit_tag_range(
        &solver, 1U, anchors[2], 104U,
        range_to(tag_x, tag_y, anchor_x[2], anchor_y[2]), &output));
    assert(!output.position_valid);
    for (size_t index = 0U; index < anchor_count; ++index) {
        if (index == 2U) {
            continue;
        }
        assert(uwb_native_ds_position_solver_submit_tag_range(
            &solver, 1U, anchors[index], 104U,
            range_to(tag_x, tag_y, anchor_x[index], anchor_y[index]),
            &output));
    }
    assert(output.position_valid);
    assert(output.frame_id == 104U);
    assert(output.observation_count == 4U);

    /* Every possible coherent 3-anchor subset produces one raw solution. */
    for (size_t missing = 0U; missing < anchor_count; ++missing) {
        struct uwb_native_ds_position_solver subset_solver;
        assert(uwb_native_ds_position_solver_init(
            &subset_solver, 1U, anchors, anchor_x, anchor_y,
            anchor_count, 29U));
        const uint32_t partial_frame = (uint32_t)(200U + missing * 2U);
        for (size_t index = 0U; index < anchor_count; ++index) {
            if (index == missing) {
                continue;
            }
            assert(uwb_native_ds_position_solver_submit_tag_range(
                &subset_solver, 1U, anchors[index], partial_frame,
                range_to(tag_x, tag_y, anchor_x[index], anchor_y[index]),
                &output));
            assert(!output.position_valid);
        }
        assert(uwb_native_ds_position_solver_submit_tag_range(
            &subset_solver, 1U, anchors[0], partial_frame + 1U,
            range_to(tag_x, tag_y, anchor_x[0], anchor_y[0]),
            &output));
        assert(output.position_valid);
        assert(output.frame_id == partial_frame);
        assert(output.observation_count == 3U);
        assert_near(output.x_m, tag_x, 0.001f);
        assert_near(output.y_m, tag_y, 0.001f);
    }

    /* Physically incoherent 3/4 ranges are rejected without losing the
     * first range of the following coherent frame. */
    struct uwb_native_ds_position_solver incoherent_solver;
    assert(uwb_native_ds_position_solver_init(
        &incoherent_solver, 1U, anchors, anchor_x, anchor_y,
        anchor_count, 29U));
    for (size_t index = 0U; index < 3U; ++index) {
        assert(uwb_native_ds_position_solver_submit_tag_range(
            &incoherent_solver, 1U, anchors[index], 300U, 0.100f,
            &output));
        assert(!output.position_valid);
    }
    assert(uwb_native_ds_position_solver_submit_tag_range(
        &incoherent_solver, 1U, anchors[0], 301U,
        range_to(tag_x, tag_y, anchor_x[0], anchor_y[0]), &output));
    assert(!output.position_valid);
    for (size_t index = 1U; index < anchor_count; ++index) {
        assert(uwb_native_ds_position_solver_submit_tag_range(
            &incoherent_solver, 1U, anchors[index], 301U,
            range_to(tag_x, tag_y, anchor_x[index], anchor_y[index]),
            &output));
    }
    assert(output.position_valid);
    assert(output.frame_id == 301U);
    assert(output.observation_count == 4U);

    /* Anchor-to-anchor data only diagnoses the immutable RTK geometry. */
    for (size_t first = 0U; first < anchor_count; ++first) {
        for (size_t second = first + 1U; second < anchor_count; ++second) {
            assert(uwb_native_ds_position_solver_submit_anchor_range(
                &solver, anchors[first], anchors[second],
                (uint32_t)(200U + first * anchor_count + second),
                hypotf(anchor_x[first] - anchor_x[second],
                       anchor_y[first] - anchor_y[second]),
                &output));
        }
    }
    assert(output.geometry_updated);
    assert(output.geometry_version == 29U);
    assert(output.geometry_fit_rms_m < 0.001f);

    puts("uwb_native_ds_position_solver_test: PASS");
    return 0;
}
