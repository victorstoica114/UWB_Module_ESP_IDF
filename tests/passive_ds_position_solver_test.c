#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "passive_ds_position_solver.h"

static double distance_to(
    const struct passive_ds_position_anchor *anchor,
    double x_m,
    double y_m)
{
    return hypot(x_m - anchor->x_m, y_m - anchor->y_m);
}

static void solve_known_point(double expected_x_m, double expected_y_m)
{
    const struct passive_ds_position_anchor anchors[] = {
        {.id = 2, .x_m = 0.0, .y_m = 0.0},
        {.id = 3, .x_m = 0.0, .y_m = 3.0},
        {.id = 4, .x_m = 3.0, .y_m = 0.0},
        {.id = 5, .x_m = 3.0, .y_m = 3.0},
    };
    struct passive_ds_position_observation observations[3];
    for (size_t index = 0U; index < 3U; ++index) {
        observations[index] = (struct passive_ds_position_observation){
            .initiator_id = anchors[0].id,
            .responder_id = anchors[index + 1U].id,
            .difference_m =
                distance_to(&anchors[index + 1U], expected_x_m, expected_y_m) -
                distance_to(&anchors[0], expected_x_m, expected_y_m),
        };
    }

    struct passive_ds_position_result result = {0};
    assert(passive_ds_position_solve(
        anchors, 4U, observations, 3U, false, 0.0, 0.0, &result));
    assert(fabs(result.x_m - expected_x_m) < 1e-5);
    assert(fabs(result.y_m - expected_y_m) < 1e-5);
    assert(result.rms_m < 1e-6);
    assert(result.observation_count == 3U);
}

int main(void)
{
    solve_known_point(1.5, 1.5);
    solve_known_point(1.1, 1.8);
    solve_known_point(2.4, 0.7);
    puts("passive DS raw position solver: OK");
    return 0;
}
