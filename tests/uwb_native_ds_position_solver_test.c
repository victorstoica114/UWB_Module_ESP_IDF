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

    /* A partial older frame must never be combined with a newer frame. */
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
    assert(!output.position_valid);
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
    assert_near(output.x_m, tag_x, 0.001f);
    assert_near(output.y_m, tag_y, 0.001f);
    assert(output.rms_m < 0.001f);

    /* Every frame is solved from its own raw ranges, not the prior solution. */
    submit_complete_frame(&solver, anchors, anchor_x, anchor_y, anchor_count,
                          102U, -0.650f, 1.175f, &output);
    assert_near(output.x_m, -0.650f, 0.001f);
    assert_near(output.y_m, 1.175f, 0.001f);

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

    /* Packet-time mobile coordinates replace the surveyed solver geometry. */
    const float moved_x[] = {0.100f, 3.944f, -3.020f, 1.155f};
    const float moved_y[] = {-0.050f, 2.873f, 3.849f, 6.786f};
    assert(uwb_native_ds_position_solver_update_geometry(
        &solver, moved_x, moved_y, 0x80000067U));
    assert(uwb_native_ds_position_solver_geometry(&solver, &output));
    assert(output.geometry_version == 0x80000067U);
    assert_near(output.anchor_x_m[0], 0.100f, 0.0001f);
    submit_complete_frame(&solver, anchors, moved_x, moved_y, anchor_count,
                          103U, -0.450f, 1.125f, &output);
    assert_near(output.x_m, -0.450f, 0.001f);
    assert_near(output.y_m, 1.125f, 0.001f);

    puts("uwb_native_ds_position_solver_test: PASS");
    return 0;
}
