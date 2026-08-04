#include "uwb_native_ds_position_solver.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void assert_near(float actual, float expected, float tolerance)
{
    assert(fabsf(actual - expected) <= tolerance);
}

int main(void)
{
    const uint8_t anchors[] = {2, 3, 4, 5};
    struct uwb_native_ds_position_solver solver;
    assert(uwb_native_ds_position_solver_init(
        &solver, 1, anchors, sizeof(anchors)));

    const struct {
        uint8_t first;
        uint8_t second;
        float distance_m;
    } pairs[] = {
        {2, 3, 3.0f},
        {2, 4, 3.0f},
        {2, 5, 4.2426407f},
        {3, 4, 4.2426407f},
        {3, 5, 3.0f},
        {4, 5, 3.0f},
    };
    struct uwb_native_ds_position_output output;
    for (size_t index = 0; index < sizeof(pairs) / sizeof(pairs[0]); ++index) {
        assert(uwb_native_ds_position_solver_submit_anchor_range(
            &solver, pairs[index].first, pairs[index].second,
            (uint16_t)(100 + index), pairs[index].distance_m, &output));
    }
    assert(output.geometry_updated);
    assert(output.geometry_version == 1U);
    assert(output.geometry_fit_rms_m < 0.001f);

    const float centered_range = sqrtf(4.5f);
    for (size_t index = 0; index + 1U < sizeof(anchors); ++index) {
        assert(uwb_native_ds_position_solver_submit_tag_range(
            &solver, 1, anchors[index], 499, centered_range, &output));
        assert(!output.position_valid);
    }
    assert(uwb_native_ds_position_solver_submit_tag_range(
        &solver, 1, anchors[3], 500, centered_range, &output));
    assert(!output.position_valid);

    for (size_t index = 0; index + 1U < sizeof(anchors); ++index) {
        assert(uwb_native_ds_position_solver_submit_tag_range(
            &solver, 1, anchors[index], 500, centered_range, &output));
    }
    assert(output.position_valid);
    assert(output.observation_count == 4U);
    assert_near(output.x_m, 1.5f, 0.001f);
    assert_near(output.y_m, 1.5f, 0.001f);
    assert(output.rms_m < 0.001f);

    for (size_t index = 0; index < sizeof(anchors); ++index) {
        assert(uwb_native_ds_position_solver_submit_tag_range(
            &solver, 1, anchors[index], 501, centered_range, &output));
    }
    assert(output.position_valid);
    assert(output.frame_id == 501U);
    assert(output.iteration_count <= 2U);

    puts("uwb_native_ds_position_solver_test: PASS");
    return 0;
}
