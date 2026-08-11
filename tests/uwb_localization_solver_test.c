#include "uwb_localization_solver.h"
#include "uwb_localization_protocol_models.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static double range_to(double x_m, double y_m, double anchor_x_m,
                       double anchor_y_m)
{
    return hypot(x_m - anchor_x_m, y_m - anchor_y_m);
}

static void test_direct_ranges(void)
{
    const double tag_x_m = 1.2;
    const double tag_y_m = 0.8;
    const double anchors[][2] = {
        {0.0, 0.0}, {3.0, 0.0}, {3.0, 3.0}, {0.0, 3.0},
    };
    struct uwb_localization_observation observations[4] = {0};
    for (size_t index = 0U; index < 4U; ++index) {
        observations[index] = (struct uwb_localization_observation){
            .type = UWB_LOCALIZATION_MEASUREMENT_RANGE,
            .first_x_m = anchors[index][0],
            .first_y_m = anchors[index][1],
            .measured_m = range_to(tag_x_m, tag_y_m,
                                   anchors[index][0], anchors[index][1]),
            .base_weight = 1.0,
        };
    }
    const struct uwb_localization_seed seed = {
        .valid = true, .x_m = 1.5, .y_m = 1.5,
    };
    const struct uwb_localization_solver_config config =
        uwb_localization_solver_default_config();
    struct uwb_localization_result result = {0};
    assert(uwb_localization_solve_2d(
        observations, 4U, &seed, &config, &result));
    assert(result.valid);
    assert(fabs(result.x_m - tag_x_m) < 1.0e-5);
    assert(fabs(result.y_m - tag_y_m) < 1.0e-5);
    assert(result.residual_rms_m < 1.0e-5);
    assert(result.normal_condition_number < 3.0);
}

static void test_range_differences(void)
{
    const double tag_x_m = 1.1;
    const double tag_y_m = 1.7;
    const double anchors[][2] = {
        {0.0, 0.0}, {3.0, 0.0}, {3.0, 3.0}, {0.0, 3.0},
    };
    struct uwb_localization_observation observations[6] = {0};
    size_t count = 0U;
    for (size_t first = 0U; first < 4U; ++first) {
        for (size_t second = first + 1U; second < 4U; ++second) {
            observations[count++] =
                (struct uwb_localization_observation){
                    .type = UWB_LOCALIZATION_MEASUREMENT_RANGE_DIFFERENCE,
                    .first_x_m = anchors[first][0],
                    .first_y_m = anchors[first][1],
                    .second_x_m = anchors[second][0],
                    .second_y_m = anchors[second][1],
                    .measured_m =
                        range_to(tag_x_m, tag_y_m, anchors[second][0],
                                 anchors[second][1]) -
                        range_to(tag_x_m, tag_y_m, anchors[first][0],
                                 anchors[first][1]),
                    .base_weight = 1.0,
                };
        }
    }
    const struct uwb_localization_seed seed = {
        .valid = true, .x_m = 1.5, .y_m = 1.5,
    };
    const struct uwb_localization_solver_config config =
        uwb_localization_solver_default_config();
    struct uwb_localization_result result = {0};
    assert(uwb_localization_solve_2d(
        observations, count, &seed, &config, &result));
    assert(fabs(result.x_m - tag_x_m) < 1.0e-5);
    assert(fabs(result.y_m - tag_y_m) < 1.0e-5);
}

static void test_huber_limits_one_corrupt_range(void)
{
    const double tag_x_m = 1.0;
    const double tag_y_m = 1.2;
    const double anchors[][2] = {
        {-2.0, -2.0}, {2.0, -2.0}, {4.0, 0.0},
        {2.0, 4.0}, {-2.0, 4.0}, {-4.0, 0.0},
    };
    struct uwb_localization_observation observations[6] = {0};
    for (size_t index = 0U; index < 6U; ++index) {
        observations[index] = (struct uwb_localization_observation){
            .type = UWB_LOCALIZATION_MEASUREMENT_RANGE,
            .first_x_m = anchors[index][0],
            .first_y_m = anchors[index][1],
            .measured_m = range_to(tag_x_m, tag_y_m,
                                   anchors[index][0], anchors[index][1]),
            .base_weight = 1.0,
        };
    }
    observations[2].measured_m += 1.5;
    const struct uwb_localization_seed seed = {
        .valid = true, .x_m = 0.0, .y_m = 0.0,
    };
    struct uwb_localization_solver_config least_squares =
        uwb_localization_solver_default_config();
    least_squares.loss = UWB_LOCALIZATION_LOSS_NONE;
    struct uwb_localization_solver_config huber = least_squares;
    huber.loss = UWB_LOCALIZATION_LOSS_HUBER;
    huber.loss_scale_m = 0.08;
    struct uwb_localization_result plain = {0};
    struct uwb_localization_result robust = {0};
    struct uwb_localization_result correlated_robust = {0};
    assert(uwb_localization_solve_2d(
        observations, 6U, &seed, &least_squares, &plain));
    assert(uwb_localization_solve_2d(
        observations, 6U, &seed, &huber, &robust));
    double identity[6][6] = {{0}};
    for (size_t index = 0U; index < 6U; ++index) {
        identity[index][index] = 1.0;
    }
    assert(uwb_localization_solve_2d_correlated(
        observations, 6U, &identity[0][0], 6U, &seed, &huber,
        &correlated_robust));
    const double plain_error = hypot(plain.x_m - tag_x_m,
                                     plain.y_m - tag_y_m);
    const double robust_error = hypot(robust.x_m - tag_x_m,
                                      robust.y_m - tag_y_m);
    assert(robust_error < plain_error * 0.35);
    assert(robust_error < 0.10);
    assert(robust.downweighted_count >= 1U);
    assert(hypot(correlated_robust.x_m - tag_x_m,
                 correlated_robust.y_m - tag_y_m) < 0.10);
    assert(correlated_robust.downweighted_count >= 1U);
}

static void test_rejects_rank_deficient_geometry(void)
{
    struct uwb_localization_observation observations[3] = {
        {.type = UWB_LOCALIZATION_MEASUREMENT_RANGE,
         .first_x_m = 0.0, .first_y_m = 0.0,
         .measured_m = 3.0, .base_weight = 1.0},
        {.type = UWB_LOCALIZATION_MEASUREMENT_RANGE,
         .first_x_m = 1.0, .first_y_m = 0.0,
         .measured_m = 2.0, .base_weight = 1.0},
        {.type = UWB_LOCALIZATION_MEASUREMENT_RANGE,
         .first_x_m = 2.0, .first_y_m = 0.0,
         .measured_m = 1.0, .base_weight = 1.0},
    };
    const struct uwb_localization_seed seed = {
        .valid = true, .x_m = 3.0, .y_m = 0.0,
    };
    const struct uwb_localization_solver_config config =
        uwb_localization_solver_default_config();
    struct uwb_localization_result result = {0};
    assert(!uwb_localization_solve_2d(
        observations, 3U, &seed, &config, &result));
}

static void test_range_ignores_unused_second_endpoint(void)
{
    struct uwb_localization_observation observations[4] = {
        {.type = UWB_LOCALIZATION_MEASUREMENT_RANGE,
         .first_x_m = 0.0, .first_y_m = 0.0,
         .second_x_m = NAN, .second_y_m = NAN,
         .measured_m = sqrt(2.0), .base_weight = 1.0},
        {.type = UWB_LOCALIZATION_MEASUREMENT_RANGE,
         .first_x_m = 2.0, .first_y_m = 0.0,
         .second_x_m = NAN, .second_y_m = NAN,
         .measured_m = sqrt(2.0), .base_weight = 1.0},
        {.type = UWB_LOCALIZATION_MEASUREMENT_RANGE,
         .first_x_m = 2.0, .first_y_m = 2.0,
         .second_x_m = NAN, .second_y_m = NAN,
         .measured_m = sqrt(2.0), .base_weight = 1.0},
        {.type = UWB_LOCALIZATION_MEASUREMENT_RANGE,
         .first_x_m = 0.0, .first_y_m = 2.0,
         .second_x_m = NAN, .second_y_m = NAN,
         .measured_m = sqrt(2.0), .base_weight = 1.0},
    };
    const struct uwb_localization_solver_config config =
        uwb_localization_solver_default_config();
    struct uwb_localization_result result = {0};
    assert(uwb_localization_solve_2d(
        observations, 4U, NULL, &config, &result));
    assert(fabs(result.x_m - 1.0) < 1.0e-5);
    assert(fabs(result.y_m - 1.0) < 1.0e-5);
}

static void test_protocol_models(void)
{
    const struct uwb_localization_model_anchor anchors[] = {
        {.id = 2U, .x_m = 0.0, .y_m = 0.0},
        {.id = 3U, .x_m = 4.0, .y_m = 0.0},
        {.id = 4U, .x_m = 4.0, .y_m = 3.0},
        {.id = 5U, .x_m = 0.0, .y_m = 3.0},
    };
    const double tag_x_m = 1.3;
    const double tag_y_m = 1.8;
    struct uwb_localization_native_ds_range ranges[4] = {0};
    for (size_t index = 0U; index < 4U; ++index) {
        ranges[index] = (struct uwb_localization_native_ds_range){
            .anchor_id = anchors[index].id,
            .range_m = range_to(tag_x_m, tag_y_m,
                                anchors[index].x_m,
                                anchors[index].y_m),
            .base_weight = 1.0,
        };
    }
    struct uwb_localization_solver_config config =
        uwb_localization_solver_default_config();
    struct uwb_localization_result result = {0};
    assert(uwb_localization_solve_native_ds_2d(
        anchors, 4U, ranges, 4U, NULL, &config, &result));
    assert(hypot(result.x_m - tag_x_m,
                 result.y_m - tag_y_m) < 1.0e-5);

    struct uwb_localization_tdoa_difference differences[3] = {0};
    for (size_t index = 0U; index < 3U; ++index) {
        differences[index] =
            (struct uwb_localization_tdoa_difference){
                .initiator_id = anchors[0].id,
                .responder_id = anchors[index + 1U].id,
                .difference_m =
                    range_to(tag_x_m, tag_y_m,
                             anchors[index + 1U].x_m,
                             anchors[index + 1U].y_m) -
                    range_to(tag_x_m, tag_y_m,
                             anchors[0].x_m, anchors[0].y_m),
                .base_weight = 1.0,
            };
    }
    result = (struct uwb_localization_result){0};
    assert(uwb_localization_solve_flextdoa_2d(
        anchors, 4U, differences, 3U, NULL, &config, &result));
    assert(hypot(result.x_m - tag_x_m,
                 result.y_m - tag_y_m) < 1.0e-5);

    const double timing_precision[2][2] = {
        {1.2, -0.2}, {-0.2, 1.2},
    };
    result = (struct uwb_localization_result){0};
    assert(uwb_localization_solve_passive_ds_2d(
        anchors, 3U, differences, 2U, &timing_precision[0][0], 2U,
        NULL, &config, &result));
    assert(hypot(result.x_m - tag_x_m,
                 result.y_m - tag_y_m) < 1.0e-5);
}

int main(void)
{
    test_direct_ranges();
    test_range_differences();
    test_huber_limits_one_corrupt_range();
    test_rejects_rank_deficient_geometry();
    test_range_ignores_unused_second_endpoint();
    test_protocol_models();
    puts("uwb_localization_solver_test: PASS");
    return 0;
}
