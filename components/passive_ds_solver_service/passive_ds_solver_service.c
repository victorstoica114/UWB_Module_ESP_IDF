#include "passive_ds_solver_service.h"

#include <math.h>
#include <string.h>

#include "app_runtime_config.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "passive_ds_position_solver.h"
#include "uwb_config.h"
#include "wireless_log_service.h"
#include "wireless_telemetry_service.h"

static const char *TAG = "passive_ds_solver";

enum {
    /* Absorb short Wi-Fi/telemetry bursts without dropping radio events. */
    PASSIVE_DS_SOLVER_QUEUE_LEN = 512,
    PASSIVE_DS_SOLVER_TASK_STACK_BYTES = 40960,
    /* Wi-Fi stays above us (6); log/telemetry delivery stays below us (4). */
    PASSIVE_DS_SOLVER_TASK_PRIORITY = 5,
    PASSIVE_DS_SOLVER_FRAME_BUCKETS = 32,
    PASSIVE_DS_SOLVER_RANGE_MAX_AGE_MS = 2000,
    PASSIVE_DS_SOLVER_FRAME_MAX_AGE_MS = 250,
    PASSIVE_DS_SOLVER_IDLE_WINDOW_MS = 500,
    PASSIVE_DS_SOLVER_GEOMETRY_SAMPLES_PER_PAIR = 8,
    PASSIVE_DS_SOLVER_MULTIPOINT_STARS_PER_POSITION = 3,
    /*
     * Preserve a non-overlapping three-star precision stream, then publish
     * two additional raw three-star windows in each six-star cycle.  The
     * supplemental results reuse complete radio stars and are therefore
     * marked as non-independent; they are neither averaged nor temporally
     * filtered.
     */
    PASSIVE_DS_SOLVER_SUPPLEMENTAL_PERIOD_STARS = 6,
    PASSIVE_DS_SOLVER_SUPPLEMENTAL_A_FIRST_PHASE = 1,
    PASSIVE_DS_SOLVER_SUPPLEMENTAL_B_FIRST_PHASE = 2,
    PASSIVE_DS_SOLVER_MAX_OBSERVATIONS =
        PASSIVE_DS_SOLVER_MULTIPOINT_STARS_PER_POSITION *
        (APP_RUNTIME_CONFIG_MAX_ANCHORS - 1),
    PASSIVE_DS_SOLVER_MAX_VARIABLES =
        2 * APP_RUNTIME_CONFIG_MAX_ANCHORS - 3,
};

/*
 * The exact listener-timestamp covariance for the current 1.5/0.75/1.5 ms
 * star has normalized off-diagonal terms close to 0.25.  This compact GLS
 * form preserves the physically justified same-star correlation without
 * repeatedly inverting matrices on the receive-only tag.  It is spatial
 * weighting only; it does not average or filter positions over time.
 */
static const double PASSIVE_DS_SOLVER_TIMING_COMMON_CORRELATION = 0.25;

/* Keep supplemental frame keys disjoint from the baseline frame counter. */
static const uint32_t PASSIVE_DS_SOLVER_SUPPLEMENTAL_A_FRAME_NAMESPACE =
    UINT32_C(0x80000000);
static const uint32_t PASSIVE_DS_SOLVER_SUPPLEMENTAL_B_FRAME_NAMESPACE =
    UINT32_C(0xc0000000);

static size_t multipoint_stars_per_position(
    const app_runtime_config_t *config)
{
    return config != NULL &&
                   config->passive_ds_solve_mode ==
                       APP_RUNTIME_PASSIVE_DS_SOLVE_SINGLE_STAR
        ? 1U
        : PASSIVE_DS_SOLVER_MULTIPOINT_STARS_PER_POSITION;
}

static uint32_t multipoint_period_us(
    const app_runtime_config_t *config)
{
    if (config == NULL) {
        return 0U;
    }
    return config->passive_ds_slot_ms * 1000U +
           (config->passive_ds_solve_mode ==
                    APP_RUNTIME_PASSIVE_DS_SOLVE_SINGLE_STAR
                ? APP_UWB_PASSIVE_DS_DYNAMIC_GUARD_US
                : config->passive_ds_round_gap_ms * 1000U);
}

enum passive_ds_solver_item_type {
    PASSIVE_DS_SOLVER_ITEM_RANGE,
    PASSIVE_DS_SOLVER_ITEM_OBSERVATION,
    PASSIVE_DS_SOLVER_ITEM_RESET,
};

struct passive_ds_solver_item {
    enum passive_ds_solver_item_type type;
    uint8_t tag_id;
    uint8_t first_id;
    uint8_t second_id;
    uint32_t slot_id;
    int32_t value_mm;
    uint16_t delay_ratio_q15;
};

struct passive_ds_solver_range {
    bool valid;
    double value_m;
    uint32_t slot_id;
    TickType_t updated_tick;
};

struct passive_ds_solver_frame_item {
    bool valid;
    uint8_t initiator_id;
    uint8_t responder_id;
    double difference_m;
    double delay_ratio;
    uint32_t slot_id;
    TickType_t updated_tick;
};

struct passive_ds_solver_frame {
    bool pending;
    bool independent_frame;
    uint8_t tag_id;
    uint8_t observation_count;
    uint8_t initiator_ids[PASSIVE_DS_SOLVER_MULTIPOINT_STARS_PER_POSITION];
    uint16_t observation_mask;
    uint32_t frame_id;
    uint32_t last_slot_id;
    TickType_t updated_tick;
    struct passive_ds_solver_frame_item items[PASSIVE_DS_SOLVER_MAX_OBSERVATIONS];
};

struct passive_ds_solver_state {
    uint8_t anchor_count;
    uint8_t anchor_ids[APP_RUNTIME_CONFIG_MAX_ANCHORS];
    struct passive_ds_solver_range
        ranges[APP_RUNTIME_CONFIG_MAX_ANCHORS]
              [APP_RUNTIME_CONFIG_MAX_ANCHORS];
    double range_batch_sum_m[APP_RUNTIME_CONFIG_MAX_ANCHORS]
                            [APP_RUNTIME_CONFIG_MAX_ANCHORS];
    uint8_t range_batch_count[APP_RUNTIME_CONFIG_MAX_ANCHORS]
                             [APP_RUNTIME_CONFIG_MAX_ANCHORS];
    uint32_t range_batch_slot_id[APP_RUNTIME_CONFIG_MAX_ANCHORS]
                                [APP_RUNTIME_CONFIG_MAX_ANCHORS];
    TickType_t range_batch_updated_tick[APP_RUNTIME_CONFIG_MAX_ANCHORS]
                                           [APP_RUNTIME_CONFIG_MAX_ANCHORS];
    struct passive_ds_position_anchor
        anchors[APP_RUNTIME_CONFIG_MAX_ANCHORS];
    struct passive_ds_solver_frame
        frames[PASSIVE_DS_SOLVER_FRAME_BUCKETS];
    uint32_t range_update_mask;
    uint32_t geometry_version;
    double geometry_fit_rms_m;
    bool geometry_ready;
    bool previous_position_valid;
    double previous_x_m;
    double previous_y_m;
    uint32_t position_accepted;
    uint32_t position_independent_accepted;
    uint32_t position_rejected;
    uint32_t frame_rejected;
    uint32_t geometry_accepted;
    uint32_t geometry_rejected;
    uint32_t previous_position_accepted;
    uint32_t previous_position_rejected;
    uint32_t previous_frame_rejected;
    uint32_t previous_geometry_accepted;
    uint32_t previous_geometry_rejected;
    uint32_t observation_count[APP_RUNTIME_CONFIG_MAX_ANCHORS]
                              [APP_RUNTIME_CONFIG_MAX_ANCHORS];
    double observation_mean_m[APP_RUNTIME_CONFIG_MAX_ANCHORS]
                             [APP_RUNTIME_CONFIG_MAX_ANCHORS];
    double observation_m2_m2[APP_RUNTIME_CONFIG_MAX_ANCHORS]
                            [APP_RUNTIME_CONFIG_MAX_ANCHORS];
    TickType_t summary_tick;
    TickType_t observation_summary_tick;
};

static QueueHandle_t s_queue;
static bool s_started;
static uint32_t s_dropped;

static int anchor_index(
    const struct passive_ds_solver_state *state, uint8_t id)
{
    for (size_t index = 0U; index < state->anchor_count; ++index) {
        if (state->anchor_ids[index] == id) {
            return (int)index;
        }
    }
    return -1;
}

static size_t pair_index(size_t first, size_t second, size_t count)
{
    if (first == second || first >= count || second >= count) {
        return SIZE_MAX;
    }
    if (first > second) {
        const size_t temporary = first;
        first = second;
        second = temporary;
    }
    size_t index = 0U;
    for (size_t a = 0U; a < count; ++a) {
        for (size_t b = a + 1U; b < count; ++b) {
            if (a == first && b == second) {
                return index;
            }
            index++;
        }
    }
    return SIZE_MAX;
}

static uint32_t expected_pair_mask(size_t anchor_count)
{
    const size_t pair_count = anchor_count * (anchor_count - 1U) / 2U;
    return pair_count >= 32U
        ? UINT32_MAX
        : (uint32_t)((1ULL << pair_count) - 1ULL);
}

static int geometry_variable_index(size_t anchor, bool y_axis)
{
    if (anchor == 0U || (anchor == 1U && !y_axis)) {
        return -1;
    }
    if (anchor == 1U) {
        return 0;
    }
    return 1 + (int)(2U * (anchor - 2U)) + (y_axis ? 1 : 0);
}

static bool solve_linear_system(
    double matrix[PASSIVE_DS_SOLVER_MAX_VARIABLES]
                 [PASSIVE_DS_SOLVER_MAX_VARIABLES + 1U],
    size_t count,
    double *solution)
{
    for (size_t column = 0U; column < count; ++column) {
        size_t pivot = column;
        double pivot_abs = fabs(matrix[pivot][column]);
        for (size_t row = column + 1U; row < count; ++row) {
            const double candidate = fabs(matrix[row][column]);
            if (candidate > pivot_abs) {
                pivot = row;
                pivot_abs = candidate;
            }
        }
        if (pivot_abs < 1e-12) {
            return false;
        }
        if (pivot != column) {
            for (size_t item = column; item <= count; ++item) {
                const double temporary = matrix[column][item];
                matrix[column][item] = matrix[pivot][item];
                matrix[pivot][item] = temporary;
            }
        }
        const double divisor = matrix[column][column];
        for (size_t item = column; item <= count; ++item) {
            matrix[column][item] /= divisor;
        }
        for (size_t row = 0U; row < count; ++row) {
            if (row == column) {
                continue;
            }
            const double factor = matrix[row][column];
            for (size_t item = column; item <= count; ++item) {
                matrix[row][item] -= factor * matrix[column][item];
            }
        }
    }
    for (size_t index = 0U; index < count; ++index) {
        solution[index] = matrix[index][count];
    }
    return true;
}

static bool ranges_complete_and_fresh(
    const struct passive_ds_solver_state *state, TickType_t now)
{
    const TickType_t max_age =
        pdMS_TO_TICKS(PASSIVE_DS_SOLVER_RANGE_MAX_AGE_MS);
    for (size_t first = 0U; first < state->anchor_count; ++first) {
        for (size_t second = first + 1U;
             second < state->anchor_count; ++second) {
            const struct passive_ds_solver_range *range =
                &state->ranges[first][second];
            if (!range->valid || range->value_m <= 0.05 ||
                now - range->updated_tick > max_age) {
                return false;
            }
        }
    }
    return true;
}

static bool initialize_geometry(
    struct passive_ds_solver_state *state,
    double *x_m,
    double *y_m)
{
    const double baseline_m = state->ranges[0][1].value_m;
    if (!isfinite(baseline_m) || baseline_m <= 0.05) {
        return false;
    }
    x_m[0] = 0.0;
    y_m[0] = 0.0;
    x_m[1] = 0.0;
    y_m[1] = baseline_m;

    for (size_t anchor = 2U; anchor < state->anchor_count; ++anchor) {
        const double d0_m = state->ranges[0][anchor].value_m;
        const double d1_m = state->ranges[1][anchor].value_m;
        const double y =
            (d0_m * d0_m + baseline_m * baseline_m - d1_m * d1_m) /
            (2.0 * baseline_m);
        const double x_square = d0_m * d0_m - y * y;
        if (!isfinite(y) || x_square < -0.02) {
            return false;
        }
        const double x = sqrt(fmax(0.0, x_square));
        x_m[anchor] = x;
        y_m[anchor] = y;
        if (anchor > 2U) {
            double positive_error = 0.0;
            double negative_error = 0.0;
            for (size_t known = 2U; known < anchor; ++known) {
                const double measured_m =
                    state->ranges[known][anchor].value_m;
                positive_error += fabs(
                    hypot(x - x_m[known], y - y_m[known]) - measured_m);
                negative_error += fabs(
                    hypot(-x - x_m[known], y - y_m[known]) - measured_m);
            }
            if (negative_error < positive_error) {
                x_m[anchor] = -x;
            }
        }
    }
    return true;
}

static bool reconstruct_geometry(
    struct passive_ds_solver_state *state,
    double *fit_rms_m)
{
    double x_m[APP_RUNTIME_CONFIG_MAX_ANCHORS] = {0};
    double y_m[APP_RUNTIME_CONFIG_MAX_ANCHORS] = {0};
    if (!initialize_geometry(state, x_m, y_m)) {
        return false;
    }

    const size_t variable_count = 2U * state->anchor_count - 3U;
    for (size_t iteration = 0U; iteration < 8U; ++iteration) {
        double normal[PASSIVE_DS_SOLVER_MAX_VARIABLES]
                     [PASSIVE_DS_SOLVER_MAX_VARIABLES + 1U] = {{0}};
        size_t measurement_count = 0U;
        for (size_t first = 0U; first < state->anchor_count; ++first) {
            for (size_t second = first + 1U;
                 second < state->anchor_count; ++second) {
                const double dx = x_m[first] - x_m[second];
                const double dy = y_m[first] - y_m[second];
                const double predicted_m = hypot(dx, dy);
                if (predicted_m < 0.02) {
                    continue;
                }
                const double residual_m =
                    state->ranges[first][second].value_m - predicted_m;
                double jacobian[PASSIVE_DS_SOLVER_MAX_VARIABLES] = {0};
                const int first_x = geometry_variable_index(first, false);
                const int first_y = geometry_variable_index(first, true);
                const int second_x = geometry_variable_index(second, false);
                const int second_y = geometry_variable_index(second, true);
                if (first_x >= 0) jacobian[first_x] = dx / predicted_m;
                if (first_y >= 0) jacobian[first_y] = dy / predicted_m;
                if (second_x >= 0) jacobian[second_x] = -dx / predicted_m;
                if (second_y >= 0) jacobian[second_y] = -dy / predicted_m;
                for (size_t row = 0U; row < variable_count; ++row) {
                    normal[row][variable_count] +=
                        jacobian[row] * residual_m;
                    for (size_t column = 0U;
                         column < variable_count; ++column) {
                        normal[row][column] +=
                            jacobian[row] * jacobian[column];
                    }
                }
                measurement_count++;
            }
        }
        if (measurement_count < variable_count) {
            return false;
        }
        for (size_t index = 0U; index < variable_count; ++index) {
            normal[index][index] += 1e-8;
        }
        double delta[PASSIVE_DS_SOLVER_MAX_VARIABLES] = {0};
        if (!solve_linear_system(normal, variable_count, delta)) {
            return false;
        }
        double max_delta_m = 0.0;
        for (size_t anchor = 1U; anchor < state->anchor_count; ++anchor) {
            const int x_index = geometry_variable_index(anchor, false);
            const int y_index = geometry_variable_index(anchor, true);
            if (x_index >= 0) {
                x_m[anchor] += delta[x_index];
                max_delta_m = fmax(max_delta_m, fabs(delta[x_index]));
            }
            if (y_index >= 0) {
                y_m[anchor] += delta[y_index];
                max_delta_m = fmax(max_delta_m, fabs(delta[y_index]));
            }
        }
        if (max_delta_m < 0.00001) {
            break;
        }
    }

    if (state->anchor_count > 2U && x_m[2] < 0.0) {
        for (size_t anchor = 2U; anchor < state->anchor_count; ++anchor) {
            x_m[anchor] = -x_m[anchor];
        }
    }
    double sse = 0.0;
    size_t count = 0U;
    for (size_t first = 0U; first < state->anchor_count; ++first) {
        for (size_t second = first + 1U;
             second < state->anchor_count; ++second) {
            const double residual_m =
                state->ranges[first][second].value_m -
                hypot(x_m[first] - x_m[second],
                      y_m[first] - y_m[second]);
            sse += residual_m * residual_m;
            count++;
        }
    }
    const double rms_m = count > 0U ? sqrt(sse / (double)count) : NAN;
    if (!isfinite(rms_m)) {
        return false;
    }
    for (size_t anchor = 0U; anchor < state->anchor_count; ++anchor) {
        state->anchors[anchor] = (struct passive_ds_position_anchor){
            .id = state->anchor_ids[anchor],
            .x_m = x_m[anchor],
            .y_m = y_m[anchor],
        };
    }
    *fit_rms_m = rms_m;
    return true;
}

static void publish_geometry(struct passive_ds_solver_state *state)
{
    for (size_t anchor = 0U; anchor < state->anchor_count; ++anchor) {
        (void)wireless_telemetry_service_submit_passive_ds_geometry(
            state->anchors[anchor].id, state->anchor_count,
            state->geometry_version,
            (int32_t)lround(state->anchors[anchor].x_m * 1000.0),
            (int32_t)lround(state->anchors[anchor].y_m * 1000.0),
            (int32_t)lround(state->geometry_fit_rms_m * 1000.0));
    }
}

static void try_update_geometry(
    struct passive_ds_solver_state *state, TickType_t now)
{
    const uint32_t expected_mask = expected_pair_mask(state->anchor_count);
    if ((state->range_update_mask & expected_mask) != expected_mask ||
        !ranges_complete_and_fresh(state, now)) {
        return;
    }
    double fit_rms_m = 0.0;
    if (!reconstruct_geometry(state, &fit_rms_m)) {
        state->geometry_rejected++;
        return;
    }
    state->geometry_ready = true;
    state->geometry_fit_rms_m = fit_rms_m;
    state->geometry_version++;
    state->geometry_accepted++;
    state->range_update_mask = 0U;
    publish_geometry(state);
}

static void reset_range_batch(struct passive_ds_solver_state *state)
{
    memset(state->range_batch_sum_m, 0,
           sizeof(state->range_batch_sum_m));
    memset(state->range_batch_count, 0,
           sizeof(state->range_batch_count));
    memset(state->range_batch_slot_id, 0,
           sizeof(state->range_batch_slot_id));
    memset(state->range_batch_updated_tick, 0,
           sizeof(state->range_batch_updated_tick));
    state->range_update_mask = 0U;
}

static void stage_coherent_range(
    struct passive_ds_solver_state *state, size_t first, size_t second,
    double value_m, uint32_t slot_id, TickType_t now)
{
    uint8_t *count = &state->range_batch_count[first][second];
    TickType_t *updated_tick =
        &state->range_batch_updated_tick[first][second];
    const TickType_t max_pair_age =
        pdMS_TO_TICKS(PASSIVE_DS_SOLVER_RANGE_MAX_AGE_MS);
    if (*count > 0U && now - *updated_tick > max_pair_age) {
        state->range_batch_sum_m[first][second] = 0.0;
        state->range_batch_slot_id[first][second] = 0U;
        *count = 0U;
        const size_t index = pair_index(
            first, second, state->anchor_count);
        if (index < 32U) {
            state->range_update_mask &= ~(uint32_t)(1UL << index);
        }
    }

    if (*count < PASSIVE_DS_SOLVER_GEOMETRY_SAMPLES_PER_PAIR) {
        state->range_batch_sum_m[first][second] += value_m;
        state->range_batch_slot_id[first][second] = slot_id;
        *updated_tick = now;
        (*count)++;
    }
    if (*count == PASSIVE_DS_SOLVER_GEOMETRY_SAMPLES_PER_PAIR) {
        const size_t index = pair_index(first, second, state->anchor_count);
        if (index < 32U) {
            state->range_update_mask |= (uint32_t)(1UL << index);
        }
    }

    const uint32_t expected_mask = expected_pair_mask(state->anchor_count);
    if ((state->range_update_mask & expected_mask) != expected_mask) {
        return;
    }
    for (size_t a = 0U; a < state->anchor_count; ++a) {
        for (size_t b = a + 1U; b < state->anchor_count; ++b) {
            const uint8_t pair_count = state->range_batch_count[a][b];
            if (pair_count != PASSIVE_DS_SOLVER_GEOMETRY_SAMPLES_PER_PAIR) {
                reset_range_batch(state);
                return;
            }
            const struct passive_ds_solver_range range = {
                .valid = true,
                .value_m = state->range_batch_sum_m[a][b] /
                           (double)pair_count,
                .slot_id = state->range_batch_slot_id[a][b],
                .updated_tick = state->range_batch_updated_tick[a][b],
            };
            state->ranges[a][b] = range;
            state->ranges[b][a] = range;
        }
    }
    try_update_geometry(state, now);
    reset_range_batch(state);
}

static void reset_frame(struct passive_ds_solver_frame *frame)
{
    memset(frame, 0, sizeof(*frame));
}

static struct passive_ds_solver_frame *acquire_frame(
    struct passive_ds_solver_state *state,
    uint32_t frame_id,
    uint8_t tag_id,
    TickType_t now)
{
    struct passive_ds_solver_frame *available = NULL;
    struct passive_ds_solver_frame *oldest = NULL;
    TickType_t oldest_age = 0U;
    for (size_t index = 0U;
         index < PASSIVE_DS_SOLVER_FRAME_BUCKETS; ++index) {
        struct passive_ds_solver_frame *frame = &state->frames[index];
        if (frame->pending && frame->frame_id == frame_id &&
            frame->tag_id == tag_id) {
            return frame;
        }
        if (!frame->pending) {
            if (available == NULL) {
                available = frame;
            }
            continue;
        }
        const TickType_t age = now - frame->updated_tick;
        if (age > pdMS_TO_TICKS(PASSIVE_DS_SOLVER_FRAME_MAX_AGE_MS)) {
            state->frame_rejected++;
            reset_frame(frame);
            if (available == NULL) {
                available = frame;
            }
            continue;
        }
        if (oldest == NULL || age > oldest_age) {
            oldest = frame;
            oldest_age = age;
        }
    }
    if (available == NULL) {
        available = oldest;
        if (available != NULL) {
            state->frame_rejected++;
            reset_frame(available);
        }
    }
    if (available == NULL) {
        return NULL;
    }
    available->pending = true;
    available->frame_id = frame_id;
    available->tag_id = tag_id;
    for (size_t index = 0U;
         index < PASSIVE_DS_SOLVER_MULTIPOINT_STARS_PER_POSITION; ++index) {
        available->initiator_ids[index] = UINT8_MAX;
    }
    available->updated_tick = now;
    return available;
}

static uint16_t observation_mask_bit(
    const struct passive_ds_solver_state *state,
    uint8_t initiator_id,
    uint8_t responder_id)
{
    const int first = anchor_index(state, initiator_id);
    const int second = anchor_index(state, responder_id);
    if (first < 0 || second < 0) {
        return 0U;
    }
    const size_t index = pair_index(
        (size_t)first, (size_t)second, state->anchor_count);
    return index < 16U ? (uint16_t)(1U << index) : 0U;
}

static void solve_complete_frame(
    struct passive_ds_solver_state *state,
    struct passive_ds_solver_frame *frame,
    TickType_t now)
{
    if (!state->geometry_ready) {
        reset_frame(frame);
        return;
    }
    const app_runtime_config_t *config = app_runtime_config_get();
    const bool multipoint = config->passive_ds_schedule ==
        APP_RUNTIME_PASSIVE_DS_MULTIPOINT_FULL_DS;
    const size_t stars_per_position = multipoint
        ? multipoint_stars_per_position(config)
        : 1U;
    const size_t observations_per_star = state->anchor_count - 1U;
    const size_t expected_count = observations_per_star * stars_per_position;
    struct passive_ds_position_observation
        observations[PASSIVE_DS_SOLVER_MAX_OBSERVATIONS];
    TickType_t oldest_tick = now;
    TickType_t newest_tick = 0U;
    for (size_t index = 0U; index < expected_count; ++index) {
        const struct passive_ds_solver_frame_item *item =
            &frame->items[index];
        const size_t star_index = index / observations_per_star;
        if (!item->valid || star_index >= stars_per_position ||
            item->initiator_id != frame->initiator_ids[star_index]) {
            state->frame_rejected++;
            reset_frame(frame);
            return;
        }
        observations[index] = (struct passive_ds_position_observation){
            .initiator_id = item->initiator_id,
            .responder_id = item->responder_id,
            .difference_m = item->difference_m,
            .delay_ratio = item->delay_ratio,
        };
        if (item->updated_tick < oldest_tick) {
            oldest_tick = item->updated_tick;
        }
        if (item->updated_tick > newest_tick) {
            newest_tick = item->updated_tick;
        }
    }

    struct passive_ds_position_result result = {0};
    const bool solved = multipoint
        ? passive_ds_position_solve_correlated(
              state->anchors, state->anchor_count,
              observations, expected_count,
              PASSIVE_DS_SOLVER_TIMING_COMMON_CORRELATION,
              state->previous_position_valid,
              state->previous_x_m, state->previous_y_m, &result)
        : passive_ds_position_solve(
              state->anchors, state->anchor_count,
              observations, expected_count,
              state->previous_position_valid,
              state->previous_x_m, state->previous_y_m, &result);
    if (!solved) {
        state->position_rejected++;
        reset_frame(frame);
        return;
    }

    state->previous_position_valid = true;
    state->previous_x_m = result.x_m;
    state->previous_y_m = result.y_m;
    state->position_accepted++;
    if (frame->independent_frame) {
        state->position_independent_accepted++;
    }
    const uint32_t measured_span_ms =
        (uint32_t)((newest_tick - oldest_tick) * portTICK_PERIOD_MS);
    const uint32_t measured_age_ms =
        (uint32_t)((now - oldest_tick) * portTICK_PERIOD_MS);
    const uint32_t nominal_span_ms =
        config->passive_ds_schedule ==
                APP_RUNTIME_PASSIVE_DS_MULTIPOINT_FULL_DS
            ? (uint32_t)((stars_per_position *
                              multipoint_period_us(config) +
                          999U) /
                         1000U)
            : ((uint32_t)state->anchor_count - 1U) *
                      config->passive_ds_slot_ms +
                  config->passive_ds_round_gap_ms;
    const uint16_t span_ms = (uint16_t)fmin(
        UINT16_MAX,
        (double)fmax(measured_span_ms, nominal_span_ms));
    const uint16_t age_ms = (uint16_t)fmin(
        UINT16_MAX,
        (double)fmax(measured_age_ms, nominal_span_ms));
    const uint8_t tag_id = frame->tag_id;
    const uint32_t slot_id = frame->last_slot_id;
    const uint16_t mask = frame->observation_mask;
    const bool independent_frame = frame->independent_frame;
    reset_frame(frame);

    (void)wireless_telemetry_service_submit_passive_ds_position(
        tag_id, slot_id,
        (int32_t)lround(result.x_m * 1000.0),
        (int32_t)lround(result.y_m * 1000.0),
        (int32_t)lround(result.x_m * 1000.0),
        (int32_t)lround(result.y_m * 1000.0),
        (int32_t)lround(result.sigma_m * 1000.0),
        (int32_t)lround(result.rms_m * 1000.0),
        result.observation_count, state->anchor_count,
        state->geometry_version, independent_frame, false, false,
        state->position_accepted, state->position_independent_accepted,
        span_ms, age_ms, mask, 0U, 0U,
        state->position_rejected, true);
}

static bool stage_observation(
    struct passive_ds_solver_state *state,
    const struct passive_ds_solver_item *item,
    TickType_t now,
    uint32_t frame_id,
    uint32_t star_index,
    uint32_t radio_slot_index,
    bool independent_frame)
{
    const uint32_t slots_per_star = (uint32_t)state->anchor_count - 1U;
    const app_runtime_config_t *config = app_runtime_config_get();
    const bool multipoint = config->passive_ds_schedule ==
        APP_RUNTIME_PASSIVE_DS_MULTIPOINT_FULL_DS;
    const uint32_t expected_count = slots_per_star *
        (multipoint ? (uint32_t)multipoint_stars_per_position(config) : 1U);
    const uint32_t slot_index = star_index * slots_per_star + radio_slot_index;
    struct passive_ds_solver_frame *frame = acquire_frame(
        state, frame_id, item->tag_id, now);
    if (frame == NULL || slot_index >= expected_count ||
        star_index >= PASSIVE_DS_SOLVER_MULTIPOINT_STARS_PER_POSITION) {
        state->frame_rejected++;
        return false;
    }
    frame->independent_frame = independent_frame;
    if (frame->initiator_ids[star_index] == UINT8_MAX) {
        frame->initiator_ids[star_index] = item->first_id;
    } else if (frame->initiator_ids[star_index] != item->first_id) {
        state->frame_rejected++;
        reset_frame(frame);
        return false;
    }
    const size_t star_begin = (size_t)star_index * slots_per_star;
    const size_t star_end = star_begin + slots_per_star;
    for (size_t index = star_begin; index < star_end; ++index) {
        if (index != slot_index && frame->items[index].valid &&
            frame->items[index].responder_id == item->second_id) {
            state->frame_rejected++;
            reset_frame(frame);
            return false;
        }
    }
    if (!frame->items[slot_index].valid) {
        frame->observation_count++;
    }
    frame->items[slot_index] = (struct passive_ds_solver_frame_item){
        .valid = true,
        .initiator_id = item->first_id,
        .responder_id = item->second_id,
        .difference_m = item->value_mm / 1000.0,
        .delay_ratio = (double)item->delay_ratio_q15 / 32768.0,
        .slot_id = item->slot_id,
        .updated_tick = now,
    };
    frame->observation_mask |= observation_mask_bit(
        state, item->first_id, item->second_id);
    frame->last_slot_id = item->slot_id;
    frame->updated_tick = now;
    if (frame->observation_count == expected_count) {
        solve_complete_frame(state, frame, now);
    }
    return true;
}

static void handle_observation(
    struct passive_ds_solver_state *state,
    const struct passive_ds_solver_item *item,
    TickType_t now)
{
    const int initiator = anchor_index(state, item->first_id);
    const int responder = anchor_index(state, item->second_id);
    if (initiator < 0 || responder < 0 || initiator == responder) {
        state->frame_rejected++;
        return;
    }
    const double difference_m = item->value_mm / 1000.0;
    const struct passive_ds_solver_range *anchor_range =
        &state->ranges[initiator][responder];
    if (!isfinite(difference_m) ||
        (anchor_range->valid &&
         fabs(difference_m) > anchor_range->value_m + 0.15)) {
        state->frame_rejected++;
        return;
    }

    /*
     * Diagnostic only: Welford statistics for each directed radio path.
     * The estimator still consumes the original, unfiltered observation.
     * Pooling the per-path variances later lets us compare the three N+2
     * response positions without mixing their different physical means.
     */
    uint32_t *observation_count =
        &state->observation_count[initiator][responder];
    double *observation_mean_m =
        &state->observation_mean_m[initiator][responder];
    double *observation_m2_m2 =
        &state->observation_m2_m2[initiator][responder];
    (*observation_count)++;
    const double mean_delta = difference_m - *observation_mean_m;
    *observation_mean_m += mean_delta / (double)*observation_count;
    *observation_m2_m2 +=
        mean_delta * (difference_m - *observation_mean_m);

    const app_runtime_config_t *config = app_runtime_config_get();
    const bool multipoint = config->passive_ds_schedule ==
        APP_RUNTIME_PASSIVE_DS_MULTIPOINT_FULL_DS;
    const uint32_t slots_per_star = (uint32_t)state->anchor_count - 1U;
    const uint32_t radio_frame_id = item->slot_id / slots_per_star;
    const uint32_t radio_slot_index = item->slot_id % slots_per_star;
    if (multipoint) {
        if (config->passive_ds_solve_mode ==
            APP_RUNTIME_PASSIVE_DS_SOLVE_SINGLE_STAR) {
            /*
             * Dynamic profile: all three observations originate in one
             * broadcast DS-TWR star.  Solve it once, mark it independent,
             * and never reuse it in an overlapping window.  The previous
             * position is only the nonlinear solver's initial guess; no
             * position averaging or temporal filter is applied.
             */
            (void)stage_observation(
                state, item, now, radio_frame_id, 0U,
                radio_slot_index, true);
            return;
        }

        /*
         * Keep the precision baseline deliberately simple: three consecutive
         * radio stars form one position window and every star is consumed
         * exactly once.  At the validated 6 ms radio cadence this yields a
         * 55.6 Hz independent raw stream.  Nine unfiltered observations
         * reduce measurement noise before the position solve without any
         * temporal position filter or host-side frame synchronization.
         */
        (void)stage_observation(
            state, item, now,
            radio_frame_id /
                PASSIVE_DS_SOLVER_MULTIPOINT_STARS_PER_POSITION,
            radio_frame_id %
                PASSIVE_DS_SOLVER_MULTIPOINT_STARS_PER_POSITION,
            radio_slot_index, true);

        /*
         * Two overlapping three-star windows target Native DS-TWR precision
         * while retaining a theoretical 111.1 raw solves/s.  Window A uses
         * phases 1..3 and window B phases 2..4 of every six-star cycle.  All
         * inputs remain complete native measurements; reuse occurs only
         * between explicitly labelled solver windows.
         */
        const uint32_t supplemental_phase =
            radio_frame_id % PASSIVE_DS_SOLVER_SUPPLEMENTAL_PERIOD_STARS;
        if (supplemental_phase >=
                PASSIVE_DS_SOLVER_SUPPLEMENTAL_A_FIRST_PHASE &&
            supplemental_phase <
                PASSIVE_DS_SOLVER_SUPPLEMENTAL_A_FIRST_PHASE +
                    PASSIVE_DS_SOLVER_MULTIPOINT_STARS_PER_POSITION) {
            const uint32_t supplemental_frame_id =
                PASSIVE_DS_SOLVER_SUPPLEMENTAL_A_FRAME_NAMESPACE |
                (radio_frame_id /
                 PASSIVE_DS_SOLVER_SUPPLEMENTAL_PERIOD_STARS);
            (void)stage_observation(
                state, item, now, supplemental_frame_id,
                supplemental_phase -
                    PASSIVE_DS_SOLVER_SUPPLEMENTAL_A_FIRST_PHASE,
                radio_slot_index, false);
        }
        if (supplemental_phase >=
                PASSIVE_DS_SOLVER_SUPPLEMENTAL_B_FIRST_PHASE &&
            supplemental_phase <
                PASSIVE_DS_SOLVER_SUPPLEMENTAL_B_FIRST_PHASE +
                    PASSIVE_DS_SOLVER_MULTIPOINT_STARS_PER_POSITION) {
            const uint32_t supplemental_frame_id =
                PASSIVE_DS_SOLVER_SUPPLEMENTAL_B_FRAME_NAMESPACE |
                (radio_frame_id /
                 PASSIVE_DS_SOLVER_SUPPLEMENTAL_PERIOD_STARS);
            (void)stage_observation(
                state, item, now, supplemental_frame_id,
                supplemental_phase -
                    PASSIVE_DS_SOLVER_SUPPLEMENTAL_B_FIRST_PHASE,
                radio_slot_index, false);
        }
    } else {
        /* Legacy schedules still form one non-overlapping complete frame. */
        (void)stage_observation(
            state, item, now, radio_frame_id, 0U, radio_slot_index, true);
    }
}

static void apply_runtime_config(struct passive_ds_solver_state *state)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    memset(state, 0, sizeof(*state));
    state->anchor_count = config->anchor_count;
    if (state->anchor_count > APP_RUNTIME_CONFIG_MAX_ANCHORS) {
        state->anchor_count = APP_RUNTIME_CONFIG_MAX_ANCHORS;
    }
    memcpy(state->anchor_ids, config->anchor_ids, sizeof(state->anchor_ids));
}

static void submit_summary(struct passive_ds_solver_state *state)
{
    const TickType_t now = xTaskGetTickCount();
    if (state->summary_tick == 0U) {
        state->summary_tick = now;
        return;
    }
    if (now - state->summary_tick < pdMS_TO_TICKS(1000)) {
        return;
    }
    (void)wireless_log_service_submit(
        'I', TAG,
        "raw pos=%lu/s reject=%lu frame_reject=%lu geom=%lu/%lu "
        "ready=%u version=%lu fit=%ldmm queue_drop=%lu stack_free=%luB",
        (unsigned long)(state->position_accepted -
                        state->previous_position_accepted),
        (unsigned long)(state->position_rejected -
                        state->previous_position_rejected),
        (unsigned long)(state->frame_rejected -
                        state->previous_frame_rejected),
        (unsigned long)(state->geometry_accepted -
                        state->previous_geometry_accepted),
        (unsigned long)(state->geometry_rejected -
                        state->previous_geometry_rejected),
        state->geometry_ready ? 1U : 0U,
        (unsigned long)state->geometry_version,
        (long)lround(state->geometry_fit_rms_m * 1000.0),
        (unsigned long)s_dropped,
        (unsigned long)uxTaskGetStackHighWaterMark(NULL));
    state->previous_position_accepted = state->position_accepted;
    state->previous_position_rejected = state->position_rejected;
    state->previous_frame_rejected = state->frame_rejected;
    state->previous_geometry_accepted = state->geometry_accepted;
    state->previous_geometry_rejected = state->geometry_rejected;
    state->summary_tick = now;

    const app_runtime_config_t *config = app_runtime_config_get();
    if (config->passive_ds_schedule !=
            APP_RUNTIME_PASSIVE_DS_MULTIPOINT_FULL_DS ||
        (state->observation_summary_tick != 0U &&
         now - state->observation_summary_tick < pdMS_TO_TICKS(10000))) {
        return;
    }
    double pooled_m2[APP_RUNTIME_CONFIG_MAX_ANCHORS - 1U] = {0};
    uint32_t pooled_dof[APP_RUNTIME_CONFIG_MAX_ANCHORS - 1U] = {0};
    for (size_t initiator = 0U; initiator < state->anchor_count;
         ++initiator) {
        for (size_t responder = 0U; responder < state->anchor_count;
             ++responder) {
            if (initiator == responder) {
                continue;
            }
            const uint32_t count =
                state->observation_count[initiator][responder];
            const size_t timing_index =
                (responder + state->anchor_count - initiator - 1U) %
                state->anchor_count;
            if (count > 1U && timing_index < state->anchor_count - 1U) {
                pooled_m2[timing_index] +=
                    state->observation_m2_m2[initiator][responder];
                pooled_dof[timing_index] += count - 1U;
            }
        }
    }
    if (state->anchor_count == 4U) {
        const double sigma0_mm = pooled_dof[0] > 0U
            ? 1000.0 * sqrt(pooled_m2[0] / pooled_dof[0])
            : 0.0;
        const double sigma1_mm = pooled_dof[1] > 0U
            ? 1000.0 * sqrt(pooled_m2[1] / pooled_dof[1])
            : 0.0;
        const double sigma2_mm = pooled_dof[2] > 0U
            ? 1000.0 * sqrt(pooled_m2[2] / pooled_dof[2])
            : 0.0;
        (void)wireless_log_service_submit(
            'I', TAG,
            "raw N+2 within-path sigma timing[0/1/2]=%ld/%ld/%ldmm "
            "dof=%lu/%lu/%lu (diagnostic only)",
            (long)lround(sigma0_mm), (long)lround(sigma1_mm),
            (long)lround(sigma2_mm),
            (unsigned long)pooled_dof[0],
            (unsigned long)pooled_dof[1],
            (unsigned long)pooled_dof[2]);
    }
    state->observation_summary_tick = now;
}

static void passive_ds_solver_task(void *arg)
{
    (void)arg;
    static struct passive_ds_solver_state state;
    apply_runtime_config(&state);
    TickType_t last_idle_window_tick = xTaskGetTickCount();
    ESP_LOGI(TAG, "isolated raw tag solver active anchors=%u core=%d",
             (unsigned)state.anchor_count, xPortGetCoreID());
    while (true) {
        struct passive_ds_solver_item item = {0};
        if (xQueueReceive(s_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (item.type == PASSIVE_DS_SOLVER_ITEM_RESET) {
            apply_runtime_config(&state);
            xQueueReset(s_queue);
            continue;
        }
        const int first = anchor_index(&state, item.first_id);
        const int second = anchor_index(&state, item.second_id);
        const TickType_t now = xTaskGetTickCount();
        if (first < 0 || second < 0 || first == second) {
            submit_summary(&state);
            continue;
        }
        if (item.type == PASSIVE_DS_SOLVER_ITEM_RANGE &&
            item.value_mm > 0) {
            const size_t a = (size_t)(first < second ? first : second);
            const size_t b = (size_t)(first < second ? second : first);
            const app_runtime_config_t *config = app_runtime_config_get();
            if (config->passive_ds_schedule ==
                APP_RUNTIME_PASSIVE_DS_MULTIPOINT_FULL_DS) {
                stage_coherent_range(
                    &state, a, b, item.value_mm / 1000.0,
                    item.slot_id, now);
                submit_summary(&state);
                continue;
            }
            const struct passive_ds_solver_range range = {
                .valid = true,
                .value_m = item.value_mm / 1000.0,
                .slot_id = item.slot_id,
                .updated_tick = now,
            };
            state.ranges[a][b] = range;
            state.ranges[b][a] = range;
            const size_t index = pair_index(a, b, state.anchor_count);
            if (index < 32U) {
                state.range_update_mask |= (uint32_t)(1UL << index);
            }
            try_update_geometry(&state, now);
        } else if (item.type == PASSIVE_DS_SOLVER_ITEM_OBSERVATION) {
            handle_observation(&state, &item, now);
        }
        submit_summary(&state);

        /*
         * At high passive-DS rates this queue intentionally remains busy.
         * Give the CPU0 idle task one scheduler tick often enough to feed the
         * task watchdog; queued observations are retained and processed after
         * the window, so this is neither result decimation nor filtering.
         */
        if (now - last_idle_window_tick >=
            pdMS_TO_TICKS(PASSIVE_DS_SOLVER_IDLE_WINDOW_MS)) {
            vTaskDelay(1);
            last_idle_window_tick = xTaskGetTickCount();
        }
    }
}

esp_err_t passive_ds_solver_service_start(void)
{
    if (s_started) {
        return ESP_OK;
    }
    s_queue = xQueueCreate(
        PASSIVE_DS_SOLVER_QUEUE_LEN,
        sizeof(struct passive_ds_solver_item));
    if (s_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    /*
     * The timing-covariance solve intentionally keeps a roughly 25 KiB
     * matrix on this task's stack.  Keep that isolated workspace in PSRAM so
     * Wi-Fi, GPS and the UWB radio retain scarce internal RAM.  This task does
     * not call peripherals or cache-disabled code.
     */
    const BaseType_t created = xTaskCreatePinnedToCoreWithCaps(
        passive_ds_solver_task, "passive_ds_solver",
        PASSIVE_DS_SOLVER_TASK_STACK_BYTES, NULL,
        PASSIVE_DS_SOLVER_TASK_PRIORITY, NULL, 0,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (created != pdPASS) {
        vQueueDelete(s_queue);
        s_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    s_started = true;
    return ESP_OK;
}

static bool submit_item(const struct passive_ds_solver_item *item)
{
    if (!s_started || item == NULL ||
        xQueueSend(s_queue, item, 0) != pdTRUE) {
        s_dropped++;
        return false;
    }
    return true;
}

bool passive_ds_solver_service_reset(void)
{
    const struct passive_ds_solver_item item = {
        .type = PASSIVE_DS_SOLVER_ITEM_RESET,
    };
    return submit_item(&item);
}

bool passive_ds_solver_service_submit_anchor_range(
    uint8_t anchor_a_id, uint8_t anchor_b_id, uint32_t slot_id,
    int32_t distance_mm)
{
    const struct passive_ds_solver_item item = {
        .type = PASSIVE_DS_SOLVER_ITEM_RANGE,
        .first_id = anchor_a_id,
        .second_id = anchor_b_id,
        .slot_id = slot_id,
        .value_mm = distance_mm,
    };
    return submit_item(&item);
}

bool passive_ds_solver_service_submit_observation(
    uint8_t tag_id, uint8_t initiator_id, uint8_t responder_id,
    uint32_t slot_id, int32_t difference_mm, uint16_t delay_ratio_q15)
{
    const struct passive_ds_solver_item item = {
        .type = PASSIVE_DS_SOLVER_ITEM_OBSERVATION,
        .tag_id = tag_id,
        .first_id = initiator_id,
        .second_id = responder_id,
        .slot_id = slot_id,
        .value_mm = difference_mm,
        .delay_ratio_q15 = delay_ratio_q15,
    };
    return submit_item(&item);
}
