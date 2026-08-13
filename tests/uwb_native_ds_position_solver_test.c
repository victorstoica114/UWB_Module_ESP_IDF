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

    /* A complete mask must not bypass the range/residual quality gate. This
     * models a syntactically complete radio frame with one corrupt range. */
    struct uwb_native_ds_position_solver corrupt_complete_solver;
    assert(uwb_native_ds_position_solver_init(
        &corrupt_complete_solver, 1U, anchors, anchor_x, anchor_y,
        anchor_count, 29U));
    for (size_t index = 0U; index < anchor_count; ++index) {
        const float distance = index == 3U
            ? 0.100f
            : range_to(tag_x, tag_y, anchor_x[index], anchor_y[index]);
        assert(uwb_native_ds_position_solver_submit_tag_range(
            &corrupt_complete_solver, 1U, anchors[index], 302U,
            distance, &output));
    }
    assert(!output.position_valid);

    /* Rejection is frame-local; the following coherent frame still solves. */
    submit_complete_frame(
        &corrupt_complete_solver, anchors, anchor_x, anchor_y,
        anchor_count, 303U, tag_x, tag_y, &output);
    assert(output.position_valid);
    assert(output.frame_id == 303U);
    assert(output.observation_count == 4U);
    assert_near(output.x_m, tag_x, 0.001f);
    assert_near(output.y_m, tag_y, 0.001f);

    /* The ID-keyed UWB geometry and live field ranges can contain a bounded
     * systematic mismatch after an anchor layout change.  It remains a
     * stable, useful raw solution and must not require GPS/RTK to publish. */
    const float field_anchor_x[] = {0.000f, 0.000f, 7.036f, 7.410f};
    const float field_anchor_y[] = {0.000f, 6.838f, 5.980f, 0.221f};
    const float field_ranges[] = {4.960f, 4.497f, 3.929f, 4.686f};
    struct uwb_native_ds_position_solver field_solver;
    assert(uwb_native_ds_position_solver_init(
        &field_solver, 1U, anchors, field_anchor_x, field_anchor_y,
        anchor_count, 104U));
    for (size_t index = 0U; index < anchor_count; ++index) {
        assert(uwb_native_ds_position_solver_submit_tag_range(
            &field_solver, 1U, anchors[index], 500U,
            field_ranges[index], &output));
    }
    assert(output.position_valid);
    assert(output.geometry_version == 104U);
    assert(output.observation_count == 4U);
    assert_near(output.x_m, 3.667f, 0.050f);
    assert_near(output.y_m, 3.606f, 0.050f);
    assert_near(output.rms_m, 0.297f, 0.050f);

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
                          400U, -0.450f, 1.125f, &output);
    assert_near(output.x_m, -0.450f, 0.001f);
    assert_near(output.y_m, 1.125f, 0.001f);

    puts("uwb_native_ds_position_solver_test: PASS");
    return 0;
}
