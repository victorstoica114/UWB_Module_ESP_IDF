#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "passive_ds_batch_policy.h"
#include "passive_ds_position_solver.h"

static void test_frame_order_wrap_safe(void)
{
    assert(passive_ds_batch_frame_after(11U, 10U));
    assert(!passive_ds_batch_frame_after(10U, 10U));
    assert(!passive_ds_batch_frame_after(9U, 10U));
    assert(passive_ds_batch_frame_after(0U, UINT32_MAX));
    assert(passive_ds_batch_frame_after(1U, UINT32_MAX));
    assert(!passive_ds_batch_frame_after(UINT32_MAX, 1U));
}

static void assert_selected(
    const struct passive_ds_batch_star_ref *stars,
    const size_t selected[PASSIVE_DS_BATCH_STARS_PER_POSITION],
    uint32_t first,
    uint32_t second,
    uint32_t third)
{
    assert(stars[selected[0]].frame_id == first);
    assert(stars[selected[1]].frame_id == second);
    assert(stars[selected[2]].frame_id == third);
    assert(stars[selected[0]].initiator_id !=
           stars[selected[1]].initiator_id);
    assert(stars[selected[0]].initiator_id !=
           stars[selected[2]].initiator_id);
    assert(stars[selected[1]].initiator_id !=
           stars[selected[2]].initiator_id);
}

static void test_recent_unique_selection(void)
{
    const struct passive_ds_batch_star_ref stars[] = {
        {.valid = true, .initiator_id = 5U, .frame_id = 97U},
        {.valid = true, .initiator_id = 2U, .frame_id = 100U},
        {.valid = false, .initiator_id = 8U, .frame_id = 102U},
        {.valid = true, .initiator_id = 3U, .frame_id = 99U},
        {.valid = true, .initiator_id = 4U, .frame_id = 90U},
    };
    size_t selected[PASSIVE_DS_BATCH_STARS_PER_POSITION] = {0};
    uint32_t newest = 0U;
    uint32_t span = 0U;
    assert(passive_ds_batch_select_recent_unique(
               stars, sizeof(stars) / sizeof(stars[0]), 3U,
               selected, &newest, &span) == 3U);
    assert_selected(stars, selected, 100U, 99U, 97U);
    assert(newest == 100U);
    assert(span == 3U);

    assert(passive_ds_batch_select_recent_unique(
               stars, sizeof(stars) / sizeof(stars[0]), 2U,
               selected, &newest, &span) == 0U);
}

static void test_duplicate_initiator_and_nonreuse(void)
{
    struct passive_ds_batch_star_ref stars[] = {
        {.valid = true, .initiator_id = 2U, .frame_id = 100U},
        {.valid = true, .initiator_id = 2U, .frame_id = 99U},
        {.valid = true, .initiator_id = 3U, .frame_id = 98U},
        {.valid = true, .initiator_id = 4U, .frame_id = 97U},
    };
    size_t selected[PASSIVE_DS_BATCH_STARS_PER_POSITION] = {0};
    uint32_t newest = 0U;
    uint32_t span = 0U;
    assert(passive_ds_batch_select_recent_unique(
               stars, sizeof(stars) / sizeof(stars[0]), 3U,
               selected, &newest, &span) == 3U);
    assert_selected(stars, selected, 100U, 98U, 97U);

    for (size_t index = 0U;
         index < PASSIVE_DS_BATCH_STARS_PER_POSITION; ++index) {
        stars[selected[index]].valid = false;
    }
    assert(passive_ds_batch_select_recent_unique(
               stars, sizeof(stars) / sizeof(stars[0]), 3U,
               selected, &newest, &span) == 0U);
}

static void test_wrap_selection(void)
{
    const struct passive_ds_batch_star_ref stars[] = {
        {.valid = true, .initiator_id = 2U, .frame_id = UINT32_MAX},
        {.valid = true, .initiator_id = 3U, .frame_id = 0U},
        {.valid = true, .initiator_id = 4U, .frame_id = 1U},
    };
    size_t selected[PASSIVE_DS_BATCH_STARS_PER_POSITION] = {0};
    uint32_t newest = 0U;
    uint32_t span = 0U;
    assert(passive_ds_batch_select_recent_unique(
               stars, sizeof(stars) / sizeof(stars[0]), 2U,
               selected, &newest, &span) == 3U);
    assert_selected(stars, selected, 1U, 0U, UINT32_MAX);
    assert(newest == 1U);
    assert(span == 2U);
}

static void test_raw_residual_gate(void)
{
    assert(passive_ds_batch_residuals_valid(0.25, 0.50));
    assert(passive_ds_batch_residuals_valid(0.0, 0.0));
    assert(!passive_ds_batch_residuals_valid(0.251, 0.50));
    assert(!passive_ds_batch_residuals_valid(0.25, 0.501));
    assert(!passive_ds_batch_residuals_valid(-0.1, 0.1));
    assert(!passive_ds_batch_residuals_valid(NAN, 0.1));
    assert(!passive_ds_batch_residuals_valid(0.1, INFINITY));
}

static void test_partial_star_policy(void)
{
    assert(!passive_ds_batch_star_observation_count_valid(2U, 2U));
    assert(passive_ds_batch_star_observation_count_valid(3U, 2U));
    assert(!passive_ds_batch_star_observation_count_valid(3U, 1U));
    assert(!passive_ds_batch_star_observation_count_valid(3U, 3U));
    assert(passive_ds_batch_star_observation_count_valid(4U, 2U));
    assert(passive_ds_batch_star_observation_count_valid(4U, 3U));
    assert(!passive_ds_batch_star_observation_count_valid(4U, 1U));
    assert(!passive_ds_batch_star_observation_count_valid(4U, 4U));
}

static double distance_to(
    const struct passive_ds_position_anchor *anchor,
    double x_m,
    double y_m)
{
    return hypot(x_m - anchor->x_m, y_m - anchor->y_m);
}

static void test_raw_three_star_correlated_solve(void)
{
    const struct passive_ds_position_anchor anchors[] = {
        {.id = 2U, .x_m = 0.0, .y_m = 0.0},
        {.id = 3U, .x_m = 0.0, .y_m = 4.9},
        {.id = 4U, .x_m = 5.0, .y_m = 0.0},
        {.id = 5U, .x_m = 5.0, .y_m = 4.9},
    };
    const double expected_x_m = 2.1;
    const double expected_y_m = 3.2;
    struct passive_ds_position_observation observations[9] = {0};
    const uint16_t response_ratio_q15[] = {10923U, 16384U, 21845U};
    size_t output = 0U;

    for (size_t star = 0U; star < 3U; ++star) {
        const size_t initiator = star;
        for (size_t response_slot = 0U; response_slot < 3U;
             ++response_slot) {
            const size_t responder =
                (initiator + response_slot + 1U) % 4U;
            const uint16_t ratio_q15 =
                response_ratio_q15[response_slot];
            observations[output++] =
                (struct passive_ds_position_observation){
                    .initiator_id = anchors[initiator].id,
                    .responder_id = anchors[responder].id,
                    .difference_m =
                        distance_to(
                            &anchors[responder], expected_x_m,
                            expected_y_m) -
                        distance_to(
                            &anchors[initiator], expected_x_m,
                            expected_y_m),
                    .delay_ratio = (double)ratio_q15 / 32768.0,
                };
        }
    }
    assert(output == 9U);

    struct passive_ds_position_result result = {0};
    assert(passive_ds_position_solve_correlated(
        anchors, 4U, observations, output, 0.25,
        false, 0.0, 0.0, &result));
    assert(result.observation_count == 9U);
    assert(fabs(result.x_m - expected_x_m) < 1e-5);
    assert(fabs(result.y_m - expected_y_m) < 1e-5);
    assert(result.rms_m < 1e-6);
}

static void test_raw_three_partial_star_correlated_solve(void)
{
    const struct passive_ds_position_anchor anchors[] = {
        {.id = 2U, .x_m = 0.0, .y_m = 0.0},
        {.id = 3U, .x_m = 0.0, .y_m = 4.9},
        {.id = 4U, .x_m = 5.0, .y_m = 0.0},
        {.id = 5U, .x_m = 5.0, .y_m = 4.9},
    };
    const double expected_x_m = 2.1;
    const double expected_y_m = 3.2;
    struct passive_ds_position_observation observations[6] = {0};
    size_t output = 0U;

    /* Simulate A4 being shadowed: the other three initiators retain the two
     * equations that do not involve A4. */
    const size_t initiators[] = {0U, 1U, 3U};
    for (size_t star = 0U; star < 3U; ++star) {
        const size_t initiator = initiators[star];
        for (size_t responder = 0U; responder < 4U; ++responder) {
            if (responder == initiator || responder == 2U) {
                continue;
            }
            observations[output++] =
                (struct passive_ds_position_observation){
                    .initiator_id = anchors[initiator].id,
                    .responder_id = anchors[responder].id,
                    .difference_m =
                        distance_to(&anchors[responder], expected_x_m,
                                    expected_y_m) -
                        distance_to(&anchors[initiator], expected_x_m,
                                    expected_y_m),
                    .delay_ratio = 0.5,
                };
        }
    }
    assert(output == 6U);

    struct passive_ds_position_result result = {0};
    assert(passive_ds_position_solve_correlated(
        anchors, 4U, observations, output, 0.25,
        false, 0.0, 0.0, &result));
    assert(result.observation_count == 6U);
    assert(fabs(result.x_m - expected_x_m) < 1e-5);
    assert(fabs(result.y_m - expected_y_m) < 1e-5);
    assert(result.rms_m < 1e-6);
}

int main(void)
{
    test_frame_order_wrap_safe();
    test_recent_unique_selection();
    test_duplicate_initiator_and_nonreuse();
    test_wrap_selection();
    test_raw_residual_gate();
    test_partial_star_policy();
    test_raw_three_star_correlated_solve();
    test_raw_three_partial_star_correlated_solve();
    puts("passive DS raw three-star policy and solver: OK");
    return 0;
}
