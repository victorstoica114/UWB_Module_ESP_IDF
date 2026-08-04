#include "flextdoa_algmin.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static double distance(double first_x, double first_y,
                       double second_x, double second_y)
{
    return hypot(first_x - second_x, first_y - second_y);
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

    puts("flextdoa_algmin_test: PASS");
    return 0;
}
