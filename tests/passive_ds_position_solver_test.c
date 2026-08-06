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
            .delay_ratio = (double)(index + 1U) / 4.0,
        };
    }

    struct passive_ds_position_result result = {0};
    assert(passive_ds_position_solve(
        anchors, 4U, observations, 3U, false, 0.0, 0.0, &result));
    assert(fabs(result.x_m - expected_x_m) < 1e-5);
    assert(fabs(result.y_m - expected_y_m) < 1e-5);
    assert(result.rms_m < 1e-6);
    assert(result.observation_count == 3U);

    result = (struct passive_ds_position_result){0};
    assert(passive_ds_position_solve_correlated(
        anchors, 4U, observations, 3U, 0.5,
        false, 0.0, 0.0, &result));
    assert(fabs(result.x_m - expected_x_m) < 1e-5);
    assert(fabs(result.y_m - expected_y_m) < 1e-5);
    assert(result.rms_m < 1e-6);

    result = (struct passive_ds_position_result){0};
    assert(passive_ds_position_solve_timing_covariance(
        anchors, 4U, observations, 3U,
        false, 0.0, 0.0, &result));
    assert(fabs(result.x_m - expected_x_m) < 1e-5);
    assert(fabs(result.y_m - expected_y_m) < 1e-5);
    assert(result.rms_m < 1e-6);
}

static void solve_known_point_with_three_anchors(
    double expected_x_m, double expected_y_m)
{
    const struct passive_ds_position_anchor anchors[] = {
        {.id = 2, .x_m = 0.0, .y_m = 0.0},
        {.id = 3, .x_m = 0.0, .y_m = 3.0},
        {.id = 4, .x_m = 3.0, .y_m = 0.0},
    };
    struct passive_ds_position_observation observations[2];
    for (size_t index = 0U; index < 2U; ++index) {
        observations[index] = (struct passive_ds_position_observation){
            .initiator_id = anchors[0].id,
            .responder_id = anchors[index + 1U].id,
            .difference_m =
                distance_to(&anchors[index + 1U], expected_x_m, expected_y_m) -
                distance_to(&anchors[0], expected_x_m, expected_y_m),
            .delay_ratio = (double)(index + 1U) / 3.0,
        };
    }

    struct passive_ds_position_result result = {0};
    assert(passive_ds_position_solve(
        anchors, 3U, observations, 2U, false, 0.0, 0.0, &result));
    assert(fabs(result.x_m - expected_x_m) < 1e-5);
    assert(fabs(result.y_m - expected_y_m) < 1e-5);
    assert(result.rms_m < 1e-6);
    assert(result.observation_count == 2U);

    result = (struct passive_ds_position_result){0};
    assert(passive_ds_position_solve_correlated(
        anchors, 3U, observations, 2U, 0.5,
        false, 0.0, 0.0, &result));
    assert(fabs(result.x_m - expected_x_m) < 1e-5);
    assert(fabs(result.y_m - expected_y_m) < 1e-5);
    assert(result.rms_m < 1e-6);
}

static void solve_known_point_with_moving_anchors(void)
{
    const struct passive_ds_position_anchor anchors[] = {
        {.id = 2, .x_m = 0.0, .y_m = 0.0},
        {.id = 3, .x_m = 0.0, .y_m = 3.0},
        {.id = 4, .x_m = 3.0, .y_m = 0.0},
        {.id = 5, .x_m = 3.0, .y_m = 3.0},
    };
    const double expected_x_m = 1.35;
    const double expected_y_m = 1.65;
    struct passive_ds_position_observation observations[3] = {0};
    for (size_t index = 0U; index < 3U; ++index) {
        const double initiator_x = 0.10 * (double)(index + 1U);
        const double initiator_y = -0.04 * (double)index;
        const double responder_x = anchors[index + 1U].x_m +
                                   0.15 * (double)(index + 1U);
        const double responder_y = anchors[index + 1U].y_m -
                                   0.08 * (double)(index + 1U);
        observations[index] = (struct passive_ds_position_observation){
            .initiator_id = 2U,
            .responder_id = anchors[index + 1U].id,
            .difference_m =
                hypot(expected_x_m - responder_x,
                      expected_y_m - responder_y) -
                hypot(expected_x_m - initiator_x,
                      expected_y_m - initiator_y),
            .delay_ratio = (double)(index + 1U) / 4.0,
            .dynamic_geometry = true,
            .initiator_x_m = initiator_x,
            .initiator_y_m = initiator_y,
            .responder_x_m = responder_x,
            .responder_y_m = responder_y,
        };
    }

    struct passive_ds_position_result result = {0};
    assert(passive_ds_position_solve(
        anchors, 4U, observations, 3U, false, 0.0, 0.0, &result));
    assert(fabs(result.x_m - expected_x_m) < 1e-5);
    assert(fabs(result.y_m - expected_y_m) < 1e-5);
    assert(result.rms_m < 1e-6);
}

int main(void)
{
    solve_known_point(1.5, 1.5);
    solve_known_point(1.1, 1.8);
    solve_known_point(2.4, 0.7);
    solve_known_point_with_three_anchors(1.1, 1.2);
    solve_known_point_with_three_anchors(2.1, 0.8);
    solve_known_point_with_moving_anchors();
    puts("passive DS raw position solver: OK");
    return 0;
}
