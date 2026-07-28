#include "flextdoa_solver_service.h"

#include <math.h>
#include <string.h>

#include "app_config.h"
#include "app_runtime_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "wireless_log_service.h"
#include "wireless_telemetry_service.h"

static const char *TAG = "flextdoa_solver";

enum {
    FLEX_SOLVER_QUEUE_LEN = 256,
    FLEX_SOLVER_TASK_STACK_BYTES = 12288,
    FLEX_SOLVER_TASK_PRIORITY = 3,
    FLEX_SOLVER_POSITION_MAX_AGE_MS = 500,
    FLEX_SOLVER_PASSIVE_POSITION_MAX_AGE_MS = 150,
    FLEX_SOLVER_GEOMETRY_MIN_PERIOD_MS = 100,
    FLEX_SOLVER_MAX_ITEMS_PER_BATCH = 64,
    FLEX_SOLVER_RANGE_HISTORY_LEN = 9,
    FLEX_SOLVER_RANGE_BOOTSTRAP_COUNT = 5,
    FLEX_SOLVER_RANGE_MOVE_CONFIRMATIONS = 3,
    FLEX_SOLVER_MAX_VARIABLES = 2 * APP_RUNTIME_CONFIG_MAX_ANCHORS - 3,
    FLEX_SOLVER_MAX_POSITION_OBSERVATIONS =
        APP_RUNTIME_CONFIG_MAX_ANCHORS *
        (APP_RUNTIME_CONFIG_MAX_ANCHORS - 1),
};

static const double FLEX_SOLVER_RANGE_GATE_MIN_M = 0.08;
static const double FLEX_SOLVER_RANGE_GATE_MAX_M = 0.18;
static const double FLEX_SOLVER_RANGE_CANDIDATE_AGREEMENT_M = 0.08;
static const double FLEX_SOLVER_OBSERVATION_PHYSICAL_MARGIN_M = 0.15;
static const double FLEX_SOLVER_GEOMETRY_MAX_FIT_RMS_M = 0.12;
static const double FLEX_SOLVER_GEOMETRY_MAX_STEP_M = 0.05;

enum flex_solver_item_type {
    FLEX_SOLVER_ITEM_RANGE,
    FLEX_SOLVER_ITEM_OBSERVATION,
    FLEX_SOLVER_ITEM_RELOAD_GEOMETRY,
    FLEX_SOLVER_ITEM_RESET,
};

enum flex_solver_rejection_reason {
    FLEX_SOLVER_REJECT_NONE = 0,
    FLEX_SOLVER_REJECT_FRAME_INCOMPLETE,
    FLEX_SOLVER_REJECT_FRAME_MISMATCH,
    FLEX_SOLVER_REJECT_TOO_FEW,
    FLEX_SOLVER_REJECT_SINGULAR,
    FLEX_SOLVER_REJECT_BOUNDS,
    FLEX_SOLVER_REJECT_RMS,
    FLEX_SOLVER_REJECT_OUT_OF_ORDER,
    FLEX_SOLVER_REJECT_COUNT,
};

struct flex_solver_item {
    enum flex_solver_item_type type;
    uint8_t tag_id;
    uint8_t first_id;
    uint8_t second_id;
    uint32_t slot_id;
    int32_t value_mm;
};

struct flex_solver_measurement {
    bool valid;
    double value_m;
    uint32_t slot_id;
    TickType_t updated_tick;
};

struct flex_solver_frame_observation {
    bool valid;
    uint8_t initiator;
    uint8_t responder;
    struct flex_solver_measurement measurement;
};

struct flex_solver_range_filter {
    float accepted[FLEX_SOLVER_RANGE_HISTORY_LEN];
    uint8_t accepted_count;
    uint8_t accepted_next;
    bool candidate_valid;
    float candidate_m;
    uint8_t candidate_count;
    uint32_t candidate_slot_id;
    TickType_t candidate_tick;
};

struct flex_solver_state {
    uint8_t anchor_count;
    uint8_t anchor_ids[APP_RUNTIME_CONFIG_MAX_ANCHORS];
    struct flex_solver_measurement
        ranges[APP_RUNTIME_CONFIG_MAX_ANCHORS][APP_RUNTIME_CONFIG_MAX_ANCHORS];
    struct flex_solver_measurement
        observations[APP_RUNTIME_CONFIG_MAX_ANCHORS]
                    [APP_RUNTIME_CONFIG_MAX_ANCHORS];
    struct flex_solver_range_filter
        range_filters[APP_RUNTIME_CONFIG_MAX_ANCHORS]
                     [APP_RUNTIME_CONFIG_MAX_ANCHORS];
    double anchor_x[APP_RUNTIME_CONFIG_MAX_ANCHORS];
    double anchor_y[APP_RUNTIME_CONFIG_MAX_ANCHORS];
    bool geometry_ready;
    bool geometry_fixed;
    uint32_t geometry_generation;
    uint32_t geometry_version;
    TickType_t last_geometry_tick;
    bool position_valid;
    double position_x;
    double position_y;
    bool position_pending;
    uint8_t pending_tag_id;
    uint32_t pending_position_slot_id;
    bool coherent_frame_pending;
    uint32_t coherent_frame_id;
    uint8_t coherent_tag_id;
    uint32_t coherent_last_slot_id;
    uint16_t coherent_observation_mask;
    uint8_t coherent_observation_count;
    struct flex_solver_frame_observation
        coherent_observations[APP_RUNTIME_CONFIG_MAX_ANCHORS - 1U];
    uint32_t range_accepted;
    uint32_t range_rejected;
    uint32_t range_relocations;
    uint32_t observation_accepted;
    uint32_t observation_rejected;
    uint32_t position_accepted;
    uint32_t independent_frame_accepted;
    uint32_t complete_superframe_accepted;
    uint32_t filter_correction_accepted;
    uint32_t position_rejected;
    uint32_t rejection_reason_count[FLEX_SOLVER_REJECT_COUNT];
    uint32_t previous_rejection_reason_count[FLEX_SOLVER_REJECT_COUNT];
    uint16_t rejection_reason_mask_since_submit;
    uint16_t rejected_since_submit;
    bool last_raw_position_valid;
    float last_raw_x;
    float last_raw_y;
    float last_sigma;
    float last_rms;
    TickType_t last_raw_tick;
    uint16_t last_observation_count;
    uint16_t last_batch_span_ms;
    uint16_t last_batch_max_age_ms;
    uint16_t last_observation_mask;
    uint32_t previous_range_accepted;
    uint32_t previous_range_rejected;
    uint32_t previous_observation_accepted;
    uint32_t previous_observation_rejected;
    uint32_t previous_position_accepted;
    uint32_t previous_independent_frame_accepted;
    uint32_t previous_complete_superframe_accepted;
    uint32_t previous_filter_correction_accepted;
    uint32_t previous_position_rejected;
    TickType_t last_rolling_attempt_tick;
    TickType_t summary_tick;
    bool position_filter_valid;
    float position_filter_state[4];
    float position_filter_covariance[4][4];
    TickType_t position_filter_tick;
};

struct flex_solver_position_observation {
    float value_m;
    uint32_t slot_id;
    float initiator_x;
    float initiator_y;
    float responder_x;
    float responder_y;
    float position_offset_x;
    float position_offset_y;
};

struct flex_solver_position_batch {
    struct flex_solver_position_observation
        items[FLEX_SOLVER_MAX_POSITION_OBSERVATIONS];
    size_t count;
    uint16_t span_ms;
    uint16_t max_age_ms;
    uint16_t observation_mask;
};

static QueueHandle_t s_queue;
static bool s_started;
static uint32_t s_dropped;

static void flex_solver_sort_float(float *values, size_t count)
{
    for (size_t i = 1U; i < count; ++i) {
        const float value = values[i];
        size_t j = i;
        while (j > 0U && values[j - 1U] > value) {
            values[j] = values[j - 1U];
            --j;
        }
        values[j] = value;
    }
}

static float flex_solver_median(const float *values, size_t count)
{
    float sorted[FLEX_SOLVER_RANGE_HISTORY_LEN];
    if (values == NULL || count == 0U ||
        count > FLEX_SOLVER_RANGE_HISTORY_LEN) {
        return 0.0f;
    }
    memcpy(sorted, values, count * sizeof(sorted[0]));
    flex_solver_sort_float(sorted, count);
    if ((count & 1U) != 0U) {
        return sorted[count / 2U];
    }
    return 0.5f * (sorted[count / 2U - 1U] + sorted[count / 2U]);
}

static float flex_solver_range_filter_sigma(
    const struct flex_solver_range_filter *filter, float center)
{
    float deviations[FLEX_SOLVER_RANGE_HISTORY_LEN];
    for (size_t i = 0; i < filter->accepted_count; ++i) {
        deviations[i] = fabsf(filter->accepted[i] - center);
    }
    return 1.4826f *
           flex_solver_median(deviations, filter->accepted_count);
}

static void flex_solver_range_filter_push(
    struct flex_solver_range_filter *filter, float value_m)
{
    filter->accepted[filter->accepted_next] = value_m;
    filter->accepted_next =
        (uint8_t)((filter->accepted_next + 1U) %
                  FLEX_SOLVER_RANGE_HISTORY_LEN);
    if (filter->accepted_count < FLEX_SOLVER_RANGE_HISTORY_LEN) {
        filter->accepted_count++;
    }
}

static void flex_solver_store_range(
    struct flex_solver_state *state, size_t first, size_t second,
    double value_m, uint32_t slot_id, TickType_t now)
{
    const struct flex_solver_measurement measurement = {
        .valid = true,
        .value_m = value_m,
        .slot_id = slot_id,
        .updated_tick = now,
    };
    state->ranges[first][second] = measurement;
    state->ranges[second][first] = measurement;
}

static void flex_solver_promote_range_candidate(
    struct flex_solver_state *state, size_t first, size_t second)
{
    struct flex_solver_range_filter *filter =
        &state->range_filters[first][second];
    if (!filter->candidate_valid) {
        return;
    }
    const float candidate_m = filter->candidate_m;
    memset(filter->accepted, 0, sizeof(filter->accepted));
    filter->accepted_count = 0;
    filter->accepted_next = 0;
    for (size_t i = 0; i < FLEX_SOLVER_RANGE_BOOTSTRAP_COUNT; ++i) {
        flex_solver_range_filter_push(filter, candidate_m);
    }
    flex_solver_store_range(
        state, first, second, candidate_m, filter->candidate_slot_id,
        filter->candidate_tick);
    filter->candidate_valid = false;
    filter->candidate_count = 0;
}

static bool flex_solver_try_promote_anchor_move(
    struct flex_solver_state *state)
{
    bool promoted = false;
    for (size_t anchor = 0; anchor < state->anchor_count; ++anchor) {
        size_t confirmed_edges = 0;
        for (size_t other = 0; other < state->anchor_count; ++other) {
            if (other == anchor) {
                continue;
            }
            const size_t first = anchor < other ? anchor : other;
            const size_t second = anchor < other ? other : anchor;
            const struct flex_solver_range_filter *filter =
                &state->range_filters[first][second];
            if (filter->candidate_valid &&
                filter->candidate_count >=
                    FLEX_SOLVER_RANGE_MOVE_CONFIRMATIONS) {
                confirmed_edges++;
            }
        }
        // A real move of one anchor changes at least two independent
        // anchor-to-anchor distances. A single bad path is held out.
        if (confirmed_edges < 2U) {
            continue;
        }
        for (size_t other = 0; other < state->anchor_count; ++other) {
            if (other == anchor) {
                continue;
            }
            const size_t first = anchor < other ? anchor : other;
            const size_t second = anchor < other ? other : anchor;
            struct flex_solver_range_filter *filter =
                &state->range_filters[first][second];
            if (filter->candidate_valid &&
                filter->candidate_count >=
                    FLEX_SOLVER_RANGE_MOVE_CONFIRMATIONS) {
                flex_solver_promote_range_candidate(
                    state, first, second);
                state->range_relocations++;
                promoted = true;
            }
        }
    }
    return promoted;
}

static bool flex_solver_accept_range(
    struct flex_solver_state *state, size_t first, size_t second,
    float value_m, uint32_t slot_id, TickType_t now)
{
    if (first > second) {
        const size_t temporary = first;
        first = second;
        second = temporary;
    }
    struct flex_solver_range_filter *filter =
        &state->range_filters[first][second];
    bool accepted =
        filter->accepted_count < FLEX_SOLVER_RANGE_BOOTSTRAP_COUNT;
    if (!accepted) {
        const float center =
            flex_solver_median(filter->accepted, filter->accepted_count);
        const float sigma =
            flex_solver_range_filter_sigma(filter, center);
        const float gate = fminf(
            (float)FLEX_SOLVER_RANGE_GATE_MAX_M,
            fmaxf((float)FLEX_SOLVER_RANGE_GATE_MIN_M, 6.0f * sigma));
        accepted = fabsf(value_m - center) <= gate;
    }

    if (accepted) {
        flex_solver_range_filter_push(filter, value_m);
        filter->candidate_valid = false;
        filter->candidate_count = 0;
        state->range_accepted++;
        if (filter->accepted_count <
            FLEX_SOLVER_RANGE_BOOTSTRAP_COUNT) {
            return false;
        }
        const float stable_m =
            flex_solver_median(filter->accepted,
                               filter->accepted_count);
        flex_solver_store_range(
            state, first, second, stable_m, slot_id, now);
        return true;
    }

    state->range_rejected++;
    if (filter->candidate_valid &&
        fabsf(value_m - filter->candidate_m) <=
            FLEX_SOLVER_RANGE_CANDIDATE_AGREEMENT_M) {
        const float count = (float)filter->candidate_count;
        filter->candidate_m =
            (filter->candidate_m * count + value_m) / (count + 1.0f);
        if (filter->candidate_count < UINT8_MAX) {
            filter->candidate_count++;
        }
    } else {
        filter->candidate_valid = true;
        filter->candidate_m = value_m;
        filter->candidate_count = 1U;
    }
    filter->candidate_slot_id = slot_id;
    filter->candidate_tick = now;
    return flex_solver_try_promote_anchor_move(state);
}

static uint32_t flex_solver_slots_per_position_frame(
    const app_runtime_config_t *config)
{
    if (config->runtime_mode == APP_RUNTIME_MODE_UWB_PASSIVE_DS_TWR &&
        config->anchor_count >= 2U) {
        return (uint32_t)config->anchor_count - 1U;
    }
    return config->flex_tdoa_slot_count > 0U
               ? config->flex_tdoa_slot_count
               : 1U;
}

static void flex_solver_apply_runtime_geometry(struct flex_solver_state *state)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    state->anchor_count = config->anchor_count;
    memcpy(state->anchor_ids, config->anchor_ids, sizeof(state->anchor_ids));
    state->geometry_fixed = config->flex_tdoa_geometry_fixed;
    state->geometry_generation = config->flex_tdoa_geometry_generation;
    state->geometry_ready = state->geometry_fixed;
    state->position_valid = false;
    state->position_pending = false;
    state->coherent_frame_pending = false;
    state->coherent_observation_count = 0U;
    state->coherent_observation_mask = 0U;
    state->position_filter_valid = false;
    state->last_raw_position_valid = false;
    if (!state->geometry_fixed) {
        memset(state->anchor_x, 0, sizeof(state->anchor_x));
        memset(state->anchor_y, 0, sizeof(state->anchor_y));
        return;
    }
    for (size_t i = 0; i < state->anchor_count; ++i) {
        state->anchor_x[i] = config->flex_tdoa_anchor_x_mm[i] / 1000.0;
        state->anchor_y[i] = config->flex_tdoa_anchor_y_mm[i] / 1000.0;
    }
    state->geometry_version = state->geometry_generation;
    ESP_LOGI(TAG,
             "FlexTDOA fixed geometry loaded generation=%lu anchors=%u",
             (unsigned long)state->geometry_generation,
             (unsigned)state->anchor_count);
}

static int flex_solver_anchor_index(const struct flex_solver_state *state,
                                    uint8_t id)
{
    for (size_t i = 0; i < state->anchor_count; ++i) {
        if (state->anchor_ids[i] == id) {
            return (int)i;
        }
    }
    return -1;
}

static int flex_solver_variable_index(size_t anchor, bool y_axis)
{
    // Match the paper/dashboard frame: anchor 0 is the origin and anchor 1
    // lies on +Y. Only anchor 1's Y coordinate remains a variable.
    if (anchor == 0U || (anchor == 1U && !y_axis)) {
        return -1;
    }
    if (anchor == 1U) {
        return 0;
    }
    return 1 + (int)(2U * (anchor - 2U)) + (y_axis ? 1 : 0);
}

static bool flex_solver_linear_solve(double matrix[FLEX_SOLVER_MAX_VARIABLES]
                                                  [FLEX_SOLVER_MAX_VARIABLES + 1],
                                     size_t count, double *solution)
{
    for (size_t column = 0; column < count; ++column) {
        size_t pivot = column;
        double pivot_abs = fabs(matrix[pivot][column]);
        for (size_t row = column + 1U; row < count; ++row) {
            const double candidate = fabs(matrix[row][column]);
            if (candidate > pivot_abs) {
                pivot = row;
                pivot_abs = candidate;
            }
        }
        if (pivot_abs < 1e-10) {
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
        for (size_t row = 0; row < count; ++row) {
            if (row == column) {
                continue;
            }
            const double factor = matrix[row][column];
            for (size_t item = column; item <= count; ++item) {
                matrix[row][item] -= factor * matrix[column][item];
            }
        }
    }
    for (size_t i = 0; i < count; ++i) {
        solution[i] = matrix[i][count];
    }
    return true;
}

static bool flex_solver_initialize_geometry(struct flex_solver_state *state)
{
    if (state->anchor_count < 3U || !state->ranges[0][1].valid) {
        return false;
    }
    const double baseline = state->ranges[0][1].value_m;
    if (baseline <= 0.05) {
        return false;
    }
    state->anchor_x[0] = 0.0;
    state->anchor_y[0] = 0.0;
    state->anchor_x[1] = 0.0;
    state->anchor_y[1] = baseline;

    for (size_t anchor = 2; anchor < state->anchor_count; ++anchor) {
        if (!state->ranges[0][anchor].valid ||
            !state->ranges[1][anchor].valid) {
            return false;
        }
        const double d0 = state->ranges[0][anchor].value_m;
        const double d1 = state->ranges[1][anchor].value_m;
        const double y = (d0 * d0 + baseline * baseline - d1 * d1) /
                         (2.0 * baseline);
        const double x_square = d0 * d0 - y * y;
        if (x_square < -0.02) {
            return false;
        }
        const double x = sqrt(fmax(0.0, x_square));
        state->anchor_x[anchor] = x;
        state->anchor_y[anchor] = y;
        if (anchor > 2U) {
            double positive_error = 0.0;
            double negative_error = 0.0;
            for (size_t known = 2U; known < anchor; ++known) {
                if (!state->ranges[known][anchor].valid) {
                    continue;
                }
                const double dy = y - state->anchor_y[known];
                const double positive = hypot(x - state->anchor_x[known], dy);
                const double negative = hypot(-x - state->anchor_x[known], dy);
                const double measured = state->ranges[known][anchor].value_m;
                positive_error += fabs(positive - measured);
                negative_error += fabs(negative - measured);
            }
            if (negative_error < positive_error) {
                state->anchor_x[anchor] = -x;
            }
        }
    }
    return true;
}

static bool flex_solver_update_geometry(struct flex_solver_state *state)
{
    const bool was_ready = state->geometry_ready;
    if (!was_ready && !flex_solver_initialize_geometry(state)) {
        return false;
    }

    double solved_x[APP_RUNTIME_CONFIG_MAX_ANCHORS];
    double solved_y[APP_RUNTIME_CONFIG_MAX_ANCHORS];
    memcpy(solved_x, state->anchor_x, sizeof(solved_x));
    memcpy(solved_y, state->anchor_y, sizeof(solved_y));

    const size_t variable_count = 2U * state->anchor_count - 3U;
    for (size_t iteration = 0; iteration < 5U; ++iteration) {
        double normal[FLEX_SOLVER_MAX_VARIABLES]
                     [FLEX_SOLVER_MAX_VARIABLES + 1] = {{0}};
        size_t measurement_count = 0;
        for (size_t a = 0; a < state->anchor_count; ++a) {
            for (size_t b = a + 1U; b < state->anchor_count; ++b) {
                if (!state->ranges[a][b].valid) {
                    continue;
                }
                const double dx = solved_x[a] - solved_x[b];
                const double dy = solved_y[a] - solved_y[b];
                const double predicted = hypot(dx, dy);
                if (predicted < 1e-6) {
                    continue;
                }
                const double residual =
                    state->ranges[a][b].value_m - predicted;
                double jacobian[FLEX_SOLVER_MAX_VARIABLES] = {0};
                const int ax = flex_solver_variable_index(a, false);
                const int ay = flex_solver_variable_index(a, true);
                const int bx = flex_solver_variable_index(b, false);
                const int by = flex_solver_variable_index(b, true);
                if (ax >= 0) jacobian[ax] = dx / predicted;
                if (ay >= 0) jacobian[ay] = dy / predicted;
                if (bx >= 0) jacobian[bx] = -dx / predicted;
                if (by >= 0) jacobian[by] = -dy / predicted;
                for (size_t row = 0; row < variable_count; ++row) {
                    normal[row][variable_count] += jacobian[row] * residual;
                    for (size_t column = 0; column < variable_count; ++column) {
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
        for (size_t i = 0; i < variable_count; ++i) {
            normal[i][i] += 1e-8;
        }
        double delta[FLEX_SOLVER_MAX_VARIABLES] = {0};
        if (!flex_solver_linear_solve(normal, variable_count, delta)) {
            return false;
        }
        double max_delta = 0.0;
        for (size_t anchor = 1U; anchor < state->anchor_count; ++anchor) {
            const int x_index = flex_solver_variable_index(anchor, false);
            const int y_index = flex_solver_variable_index(anchor, true);
            if (x_index >= 0) {
                solved_x[anchor] += delta[x_index];
                max_delta = fmax(max_delta, fabs(delta[x_index]));
            }
            if (y_index >= 0) {
                solved_y[anchor] += delta[y_index];
                max_delta = fmax(max_delta, fabs(delta[y_index]));
            }
        }
        if (max_delta < 0.0001) {
            break;
        }
    }

    double fit_sse = 0.0;
    size_t fit_count = 0;
    for (size_t a = 0; a < state->anchor_count; ++a) {
        for (size_t b = a + 1U; b < state->anchor_count; ++b) {
            if (!state->ranges[a][b].valid) {
                continue;
            }
            const double predicted =
                hypot(solved_x[a] - solved_x[b],
                      solved_y[a] - solved_y[b]);
            const double residual =
                state->ranges[a][b].value_m - predicted;
            fit_sse += residual * residual;
            fit_count++;
        }
    }
    if (fit_count < variable_count ||
        sqrt(fit_sse / fit_count) >
            FLEX_SOLVER_GEOMETRY_MAX_FIT_RMS_M) {
        return false;
    }

    double step_scale = 1.0;
    if (was_ready) {
        double max_displacement = 0.0;
        for (size_t anchor = 1U; anchor < state->anchor_count; ++anchor) {
            max_displacement = fmax(
                max_displacement,
                hypot(solved_x[anchor] - state->anchor_x[anchor],
                      solved_y[anchor] - state->anchor_y[anchor]));
        }
        if (max_displacement > FLEX_SOLVER_GEOMETRY_MAX_STEP_M) {
            step_scale =
                FLEX_SOLVER_GEOMETRY_MAX_STEP_M / max_displacement;
        }
    }
    for (size_t anchor = 0; anchor < state->anchor_count; ++anchor) {
        state->anchor_x[anchor] +=
            step_scale * (solved_x[anchor] - state->anchor_x[anchor]);
        state->anchor_y[anchor] +=
            step_scale * (solved_y[anchor] - state->anchor_y[anchor]);
    }
    state->geometry_ready = true;
    state->geometry_version++;
    return true;
}

static uint16_t flex_solver_saturating_u16(uint32_t value)
{
    return value > UINT16_MAX ? UINT16_MAX : (uint16_t)value;
}

static void flex_solver_record_rejection(
    struct flex_solver_state *state,
    enum flex_solver_rejection_reason reason)
{
    if (state == NULL || reason <= FLEX_SOLVER_REJECT_NONE ||
        reason >= FLEX_SOLVER_REJECT_COUNT) {
        return;
    }
    state->position_rejected++;
    state->rejection_reason_count[reason]++;
    state->rejection_reason_mask_since_submit |=
        (uint16_t)(1U << (uint8_t)reason);
    if (state->rejected_since_submit < UINT16_MAX) {
        state->rejected_since_submit++;
    }
}

static uint16_t flex_solver_path_mask(
    const struct flex_solver_state *state, size_t initiator,
    size_t responder)
{
    if (state == NULL || initiator >= state->anchor_count ||
        responder >= state->anchor_count || initiator == responder) {
        return 0U;
    }
    const size_t compressed_responder =
        responder < initiator ? responder : responder - 1U;
    const size_t bit =
        initiator * (state->anchor_count - 1U) + compressed_responder;
    return bit < 16U ? (uint16_t)(1U << bit) : 0U;
}

static uint16_t flex_solver_slot_span_ms(
    const app_runtime_config_t *config, uint32_t first_slot,
    uint32_t last_slot)
{
    if (config == NULL || last_slot <= first_slot) {
        return 0U;
    }
    const uint32_t slots_per_frame =
        flex_solver_slots_per_position_frame(config);
    const uint32_t slot_delta = last_slot - first_slot;
    const uint32_t first_frame = first_slot / slots_per_frame;
    const uint32_t last_frame = last_slot / slots_per_frame;
    const uint32_t boundary_count = last_frame - first_frame;
    const uint64_t span_ms =
        (uint64_t)slot_delta * config->passive_ds_slot_ms +
        (uint64_t)boundary_count * config->passive_ds_round_gap_ms;
    return flex_solver_saturating_u16(
        span_ms > UINT32_MAX ? UINT32_MAX : (uint32_t)span_ms);
}

static void flex_solver_build_rolling_position_batch(
    const struct flex_solver_state *state, TickType_t now,
    bool motion_compensated, struct flex_solver_position_batch *batch)
{
    memset(batch, 0, sizeof(*batch));
    const app_runtime_config_t *config = app_runtime_config_get();
    const uint32_t max_age_ms =
        config->runtime_mode == APP_RUNTIME_MODE_UWB_PASSIVE_DS_TWR
            ? FLEX_SOLVER_PASSIVE_POSITION_MAX_AGE_MS
            : FLEX_SOLVER_POSITION_MAX_AGE_MS;
    const TickType_t max_age_ticks = pdMS_TO_TICKS(max_age_ms);
    TickType_t oldest_tick = now;
    TickType_t newest_tick = 0;
    uint32_t first_slot = UINT32_MAX;
    uint32_t last_slot = 0U;
    for (size_t initiator = 0; initiator < state->anchor_count; ++initiator) {
        for (size_t responder = 0; responder < state->anchor_count;
             ++responder) {
            const struct flex_solver_measurement *measurement =
                &state->observations[initiator][responder];
            if (!measurement->valid || initiator == responder ||
                now - measurement->updated_tick > max_age_ticks ||
                batch->count >= FLEX_SOLVER_MAX_POSITION_OBSERVATIONS) {
                continue;
            }
            batch->items[batch->count++] =
                (struct flex_solver_position_observation){
                    .value_m = (float)measurement->value_m,
                    .slot_id = measurement->slot_id,
                    .initiator_x = (float)state->anchor_x[initiator],
                    .initiator_y = (float)state->anchor_y[initiator],
                    .responder_x = (float)state->anchor_x[responder],
                    .responder_y = (float)state->anchor_y[responder],
                };
            batch->observation_mask |=
                flex_solver_path_mask(state, initiator, responder);
            if (measurement->updated_tick < oldest_tick) {
                oldest_tick = measurement->updated_tick;
            }
            if (measurement->updated_tick > newest_tick) {
                newest_tick = measurement->updated_tick;
            }
            if (measurement->slot_id < first_slot) {
                first_slot = measurement->slot_id;
            }
            if (measurement->slot_id > last_slot) {
                last_slot = measurement->slot_id;
            }
        }
    }
    if (batch->count == 0U) {
        return;
    }
    batch->span_ms =
        flex_solver_slot_span_ms(config, first_slot, last_slot);
    batch->max_age_ms = flex_solver_saturating_u16(
        (uint32_t)((now - oldest_tick) * portTICK_PERIOD_MS));
    if (!motion_compensated || !state->position_filter_valid) {
        return;
    }
    const float velocity_x = state->position_filter_state[2];
    const float velocity_y = state->position_filter_state[3];
    const uint32_t queue_age_ms =
        (uint32_t)((now - newest_tick) * portTICK_PERIOD_MS);
    for (size_t i = 0; i < batch->count; ++i) {
        const uint16_t nominal_age_ms =
            flex_solver_slot_span_ms(
                config, batch->items[i].slot_id, last_slot);
        const float age_s =
            (queue_age_ms + nominal_age_ms) / 1000.0f;
        batch->items[i].position_offset_x = -velocity_x * age_s;
        batch->items[i].position_offset_y = -velocity_y * age_s;
    }
}

static bool flex_solver_build_coherent_position_batch(
    struct flex_solver_state *state, TickType_t now,
    struct flex_solver_position_batch *batch)
{
    memset(batch, 0, sizeof(*batch));
    const app_runtime_config_t *config = app_runtime_config_get();
    const uint32_t slots_per_frame =
        flex_solver_slots_per_position_frame(config);
    const uint16_t expected_mask =
        slots_per_frame >= 16U
            ? UINT16_MAX
            : (uint16_t)((1U << slots_per_frame) - 1U);
    if (!state->coherent_frame_pending ||
        state->coherent_observation_count != slots_per_frame ||
        state->coherent_observation_mask != expected_mask) {
        flex_solver_record_rejection(
            state, FLEX_SOLVER_REJECT_FRAME_INCOMPLETE);
        return false;
    }

    uint8_t initiator = UINT8_MAX;
    uint16_t responder_mask = 0U;
    TickType_t oldest_tick = now;
    uint32_t first_slot = UINT32_MAX;
    uint32_t last_slot = 0U;
    for (size_t index = 0; index < slots_per_frame; ++index) {
        const struct flex_solver_frame_observation *frame_item =
            &state->coherent_observations[index];
        if (!frame_item->valid) {
            flex_solver_record_rejection(
                state, FLEX_SOLVER_REJECT_FRAME_INCOMPLETE);
            return false;
        }
        if (initiator == UINT8_MAX) {
            initiator = frame_item->initiator;
        } else if (initiator != frame_item->initiator) {
            flex_solver_record_rejection(
                state, FLEX_SOLVER_REJECT_FRAME_MISMATCH);
            return false;
        }
        const uint16_t responder_bit =
            frame_item->responder < 16U
                ? (uint16_t)(1U << frame_item->responder)
                : 0U;
        if (responder_bit == 0U ||
            (responder_mask & responder_bit) != 0U) {
            flex_solver_record_rejection(
                state, FLEX_SOLVER_REJECT_FRAME_MISMATCH);
            return false;
        }
        responder_mask |= responder_bit;
        const struct flex_solver_measurement *measurement =
            &frame_item->measurement;
        batch->items[batch->count++] =
            (struct flex_solver_position_observation){
                .value_m = (float)measurement->value_m,
                .slot_id = measurement->slot_id,
                .initiator_x =
                    (float)state->anchor_x[frame_item->initiator],
                .initiator_y =
                    (float)state->anchor_y[frame_item->initiator],
                .responder_x =
                    (float)state->anchor_x[frame_item->responder],
                .responder_y =
                    (float)state->anchor_y[frame_item->responder],
            };
        batch->observation_mask |= flex_solver_path_mask(
            state, frame_item->initiator, frame_item->responder);
        if (measurement->updated_tick < oldest_tick) {
            oldest_tick = measurement->updated_tick;
        }
        if (measurement->slot_id < first_slot) {
            first_slot = measurement->slot_id;
        }
        if (measurement->slot_id > last_slot) {
            last_slot = measurement->slot_id;
        }
    }
    batch->span_ms =
        flex_solver_slot_span_ms(config, first_slot, last_slot);
    const uint32_t measured_max_age_ms =
        (uint32_t)((now - oldest_tick) * portTICK_PERIOD_MS);
    batch->max_age_ms = flex_solver_saturating_u16(
        measured_max_age_ms > batch->span_ms
            ? measured_max_age_ms
            : batch->span_ms);
    return true;
}

static float flex_solver_position_cost(
    const struct flex_solver_position_batch *batch, float x, float y,
    float *h00, float *h01, float *h11, float *g0, float *g1,
    size_t *used_count)
{
    float local_h00 = 0.0f, local_h01 = 0.0f, local_h11 = 0.0f;
    float local_g0 = 0.0f, local_g1 = 0.0f, sse = 0.0f;
    size_t count = 0;
    for (size_t i = 0; i < batch->count; ++i) {
        const struct flex_solver_position_observation *observation =
            &batch->items[i];
        const float observation_x = x + observation->position_offset_x;
        const float observation_y = y + observation->position_offset_y;
        const float initiator_dx =
            observation_x - observation->initiator_x;
        const float initiator_dy =
            observation_y - observation->initiator_y;
        const float responder_dx =
            observation_x - observation->responder_x;
        const float responder_dy =
            observation_y - observation->responder_y;
        const float di = sqrtf(
            initiator_dx * initiator_dx + initiator_dy * initiator_dy);
        const float dr = sqrtf(
            responder_dx * responder_dx + responder_dy * responder_dy);
        if (di < 0.02 || dr < 0.02) {
            continue;
        }
        const float residual = observation->value_m - (dr - di);
        const float jx = responder_dx / dr - initiator_dx / di;
        const float jy = responder_dy / dr - initiator_dy / di;
        const float absolute_residual = fabsf(residual);
        const float huber_delta = 0.10f;
        const float weight =
            absolute_residual <= huber_delta
                ? 1.0f
                : huber_delta / absolute_residual;
        local_h00 += weight * jx * jx;
        local_h01 += weight * jx * jy;
        local_h11 += weight * jy * jy;
        local_g0 += weight * jx * residual;
        local_g1 += weight * jy * residual;
        sse += absolute_residual <= huber_delta
                   ? residual * residual
                   : 2.0f * huber_delta * absolute_residual -
                         huber_delta * huber_delta;
        count++;
    }
    if (h00 != NULL) *h00 = local_h00;
    if (h01 != NULL) *h01 = local_h01;
    if (h11 != NULL) *h11 = local_h11;
    if (g0 != NULL) *g0 = local_g0;
    if (g1 != NULL) *g1 = local_g1;
    if (used_count != NULL) *used_count = count;
    return sse;
}

static float flex_solver_position_raw_sse(
    const struct flex_solver_position_batch *batch, float x, float y,
    size_t *used_count)
{
    float sse = 0.0f;
    size_t count = 0;
    for (size_t i = 0; i < batch->count; ++i) {
        const struct flex_solver_position_observation *observation =
            &batch->items[i];
        const float initiator_distance =
            hypotf(x + observation->position_offset_x -
                       observation->initiator_x,
                   y + observation->position_offset_y -
                       observation->initiator_y);
        const float responder_distance =
            hypotf(x + observation->position_offset_x -
                       observation->responder_x,
                   y + observation->position_offset_y -
                       observation->responder_y);
        if (initiator_distance < 0.02f || responder_distance < 0.02f) {
            continue;
        }
        const float residual =
            observation->value_m -
            (responder_distance - initiator_distance);
        sse += residual * residual;
        count++;
    }
    if (used_count != NULL) {
        *used_count = count;
    }
    return sse;
}

static bool flex_solver_rolling_enabled(uint8_t solve_mode)
{
    return solve_mode != APP_RUNTIME_PASSIVE_DS_SOLVE_FRAME;
}

static bool flex_solver_legacy_rolling_solve(uint8_t solve_mode)
{
    return solve_mode == APP_RUNTIME_PASSIVE_DS_SOLVE_ROLLING_ALL;
}

static bool flex_solver_motion_compensated_rolling_solve(uint8_t solve_mode)
{
    return solve_mode == APP_RUNTIME_PASSIVE_DS_SOLVE_ROLLING_MOTION;
}

static bool flex_solver_filter_correction_requested(
    uint8_t solve_mode, bool independent_frame, bool complete_superframe)
{
    switch (solve_mode) {
    case APP_RUNTIME_PASSIVE_DS_SOLVE_ROLLING_INDEPENDENT:
        return independent_frame;
    case APP_RUNTIME_PASSIVE_DS_SOLVE_ROLLING_SUPERFRAME:
        return complete_superframe;
    case APP_RUNTIME_PASSIVE_DS_SOLVE_FRAME:
    case APP_RUNTIME_PASSIVE_DS_SOLVE_ROLLING_ALL:
    case APP_RUNTIME_PASSIVE_DS_SOLVE_ROLLING_MOTION:
    default:
        return true;
    }
}

static bool flex_solver_complete_superframe(
    const app_runtime_config_t *config, uint32_t completed_frame)
{
    if (config == NULL || config->anchor_count < 2U) {
        return false;
    }
    if (config->passive_ds_schedule !=
        APP_RUNTIME_PASSIVE_DS_ROBUST_ROTATING) {
        // Fast Star does not refresh every directed path in one compact
        // superframe. Keep this experimental policy equivalent to the
        // independent-frame policy outside Robust Rotating.
        return true;
    }
    return completed_frame % (uint32_t)config->anchor_count ==
           (uint32_t)config->anchor_count - 1U;
}

static void flex_solver_filter_position(
    struct flex_solver_state *state, float raw_x, float raw_y,
    float measurement_sigma, TickType_t now, bool measurement_update,
    float *filtered_x, float *filtered_y, bool *correction_applied)
{
    if (correction_applied != NULL) {
        *correction_applied = false;
    }
    const bool filter_state_invalid =
        !state->position_filter_valid ||
        !isfinite(state->position_filter_state[0]) ||
        !isfinite(state->position_filter_state[1]);
    const bool correction_relocation =
        measurement_update && !filter_state_invalid &&
        hypotf(raw_x - state->position_filter_state[0],
               raw_y - state->position_filter_state[1]) > 1.5f;
    if (filter_state_invalid || correction_relocation) {
        // A prediction-only event cannot initialize or relocate the EKF from
        // a measurement. Publish the raw coherent result until the selected
        // correction boundary arrives.
        if (!measurement_update) {
            *filtered_x = raw_x;
            *filtered_y = raw_y;
            return;
        }
        memset(state->position_filter_state, 0,
               sizeof(state->position_filter_state));
        memset(state->position_filter_covariance, 0,
               sizeof(state->position_filter_covariance));
        state->position_filter_state[0] = raw_x;
        state->position_filter_state[1] = raw_y;
        state->position_filter_covariance[0][0] = 0.01f;
        state->position_filter_covariance[1][1] = 0.01f;
        state->position_filter_covariance[2][2] = 1.0f;
        state->position_filter_covariance[3][3] = 1.0f;
        state->position_filter_tick = now;
        state->position_filter_valid = true;
        *filtered_x = raw_x;
        *filtered_y = raw_y;
        if (correction_applied != NULL) {
            *correction_applied = true;
        }
        return;
    }

    float dt =
        (float)((now - state->position_filter_tick) * portTICK_PERIOD_MS) /
        1000.0f;
    dt = fminf(0.2f, fmaxf(0.0f, dt));
    state->position_filter_tick = now;

    float transition[4][4] = {
        {1.0f, 0.0f, dt, 0.0f},
        {0.0f, 1.0f, 0.0f, dt},
        {0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f},
    };
    state->position_filter_state[0] +=
        dt * state->position_filter_state[2];
    state->position_filter_state[1] +=
        dt * state->position_filter_state[3];

    float temporary[4][4] = {{0}};
    float predicted_covariance[4][4] = {{0}};
    for (size_t row = 0; row < 4U; ++row) {
        for (size_t column = 0; column < 4U; ++column) {
            for (size_t inner = 0; inner < 4U; ++inner) {
                temporary[row][column] +=
                    transition[row][inner] *
                    state->position_filter_covariance[inner][column];
            }
        }
    }
    for (size_t row = 0; row < 4U; ++row) {
        for (size_t column = 0; column < 4U; ++column) {
            for (size_t inner = 0; inner < 4U; ++inner) {
                predicted_covariance[row][column] +=
                    temporary[row][inner] *
                    transition[column][inner];
            }
        }
    }

    // Constant-velocity EKF with white acceleration process noise. The
    // relatively generous acceleration budget keeps it responsive to a
    // moving tag while still reducing stationary frame-to-frame jitter.
    const float acceleration_variance = 9.0f;
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;
    const float q_position = acceleration_variance * dt4 * 0.25f;
    const float q_cross = acceleration_variance * dt3 * 0.5f;
    const float q_velocity = acceleration_variance * dt2;
    predicted_covariance[0][0] += q_position;
    predicted_covariance[1][1] += q_position;
    predicted_covariance[0][2] += q_cross;
    predicted_covariance[2][0] += q_cross;
    predicted_covariance[1][3] += q_cross;
    predicted_covariance[3][1] += q_cross;
    predicted_covariance[2][2] += q_velocity;
    predicted_covariance[3][3] += q_velocity;

    if (!measurement_update) {
        memcpy(state->position_filter_covariance, predicted_covariance,
               sizeof(state->position_filter_covariance));
        *filtered_x = state->position_filter_state[0];
        *filtered_y = state->position_filter_state[1];
        return;
    }

    const float measurement_variance = fminf(
        0.01f,
        fmaxf(0.0004f, measurement_sigma * measurement_sigma));
    const float s00 =
        predicted_covariance[0][0] + measurement_variance;
    const float s01 = predicted_covariance[0][1];
    const float s11 =
        predicted_covariance[1][1] + measurement_variance;
    const float determinant = s00 * s11 - s01 * s01;
    if (determinant <= 1e-12f) {
        state->position_filter_valid = false;
        *filtered_x = raw_x;
        *filtered_y = raw_y;
        return;
    }

    float gain[4][2];
    for (size_t row = 0; row < 4U; ++row) {
        gain[row][0] =
            (predicted_covariance[row][0] * s11 -
             predicted_covariance[row][1] * s01) /
            determinant;
        gain[row][1] =
            (-predicted_covariance[row][0] * s01 +
             predicted_covariance[row][1] * s00) /
            determinant;
    }
    const float innovation_x =
        raw_x - state->position_filter_state[0];
    const float innovation_y =
        raw_y - state->position_filter_state[1];
    for (size_t row = 0; row < 4U; ++row) {
        state->position_filter_state[row] +=
            gain[row][0] * innovation_x +
            gain[row][1] * innovation_y;
    }

    float updated_covariance[4][4];
    for (size_t row = 0; row < 4U; ++row) {
        for (size_t column = 0; column < 4U; ++column) {
            updated_covariance[row][column] =
                predicted_covariance[row][column] -
                gain[row][0] * predicted_covariance[0][column] -
                gain[row][1] * predicted_covariance[1][column];
        }
    }
    for (size_t row = 0; row < 4U; ++row) {
        for (size_t column = 0; column < 4U; ++column) {
            state->position_filter_covariance[row][column] =
                0.5f *
                (updated_covariance[row][column] +
                 updated_covariance[column][row]);
        }
        state->position_filter_covariance[row][row] =
            fmaxf(1e-8f,
                  state->position_filter_covariance[row][row]);
    }
    *filtered_x = state->position_filter_state[0];
    *filtered_y = state->position_filter_state[1];
    if (correction_applied != NULL) {
        *correction_applied = true;
    }
}

static bool flex_solver_publish_prediction(
    struct flex_solver_state *state, uint8_t tag_id, uint32_t slot_id)
{
    if (state == NULL || !state->geometry_ready ||
        !state->position_filter_valid || !state->last_raw_position_valid) {
        return false;
    }
    float filtered_x = state->position_filter_state[0];
    float filtered_y = state->position_filter_state[1];
    bool correction_applied = false;
    const TickType_t now = xTaskGetTickCount();
    flex_solver_filter_position(
        state, state->last_raw_x, state->last_raw_y,
        state->last_sigma, now, false,
        &filtered_x, &filtered_y, &correction_applied);
    if (!isfinite(filtered_x) || !isfinite(filtered_y) ||
        correction_applied) {
        return false;
    }
    state->position_x = filtered_x;
    state->position_y = filtered_y;
    state->position_valid = true;
    state->position_accepted++;
    const uint16_t prediction_age_ms = flex_solver_saturating_u16(
        (uint32_t)state->last_batch_max_age_ms +
        (uint32_t)((now - state->last_raw_tick) *
                   portTICK_PERIOD_MS));
    const bool submitted =
        wireless_telemetry_service_submit_passive_ds_position(
            tag_id, slot_id,
            (int32_t)lroundf(filtered_x * 1000.0f),
            (int32_t)lroundf(filtered_y * 1000.0f),
            (int32_t)lroundf(state->last_raw_x * 1000.0f),
            (int32_t)lroundf(state->last_raw_y * 1000.0f),
            (int32_t)lroundf(state->last_sigma * 1000.0f),
            (int32_t)lroundf(state->last_rms * 1000.0f),
            state->last_observation_count, state->anchor_count,
            state->geometry_version, false, false, false,
            state->position_accepted, state->independent_frame_accepted,
            state->last_batch_span_ms, prediction_age_ms,
            state->last_observation_mask,
            state->rejection_reason_mask_since_submit,
            state->rejected_since_submit, state->position_rejected);
    if (submitted) {
        state->rejection_reason_mask_since_submit = 0U;
        state->rejected_since_submit = 0U;
    }
    return true;
}

static bool flex_solver_update_position(
    struct flex_solver_state *state, uint8_t tag_id, uint32_t slot_id,
    bool independent_frame, bool complete_superframe,
    const struct flex_solver_position_batch *batch)
{
    if (!state->geometry_ready || batch == NULL) {
        return false;
    }
    const TickType_t now = xTaskGetTickCount();
    if (batch->count < 3U) {
        flex_solver_record_rejection(
            state, FLEX_SOLVER_REJECT_TOO_FEW);
        return false;
    }
    float min_x = (float)state->anchor_x[0];
    float max_x = (float)state->anchor_x[0];
    float min_y = (float)state->anchor_y[0];
    float max_y = (float)state->anchor_y[0];
    float center_x = 0.0f, center_y = 0.0f;
    for (size_t i = 0; i < state->anchor_count; ++i) {
        const float anchor_x = (float)state->anchor_x[i];
        const float anchor_y = (float)state->anchor_y[i];
        min_x = fminf(min_x, anchor_x);
        max_x = fmaxf(max_x, anchor_x);
        min_y = fminf(min_y, anchor_y);
        max_y = fmaxf(max_y, anchor_y);
        center_x += anchor_x;
        center_y += anchor_y;
    }
    center_x /= state->anchor_count;
    center_y /= state->anchor_count;
    const float span_x = max_x - min_x;
    const float span_y = max_y - min_y;
    const float geometry_span =
        fmaxf(0.5f, sqrtf(span_x * span_x + span_y * span_y));
    const float bound_margin = 2.0f * geometry_span;
    const bool previous_plausible =
        state->position_valid && isfinite(state->position_x) &&
        isfinite(state->position_y) &&
        state->position_x >= min_x - bound_margin &&
        state->position_x <= max_x + bound_margin &&
        state->position_y >= min_y - bound_margin &&
        state->position_y <= max_y + bound_margin;
    float x = previous_plausible ? (float)state->position_x : center_x;
    float y = previous_plausible ? (float)state->position_y : center_y;

    // AlgMin needs a good initial point. Prefer the anchor centroid whenever
    // the previous estimate has a larger raw least-squares cost.
    size_t initial_count = 0, center_count = 0;
    float current_sse = flex_solver_position_cost(
        batch, x, y, NULL, NULL, NULL, NULL, NULL, &initial_count);
    const float center_sse = flex_solver_position_cost(
        batch, center_x, center_y, NULL, NULL, NULL, NULL, NULL,
        &center_count);
    const bool coherent_frame_batch =
        independent_frame &&
        batch->count == state->anchor_count - 1U;
    if (center_count >= 3U &&
        (initial_count < 3U ||
         (!coherent_frame_batch && center_sse < current_sse))) {
        x = center_x;
        y = center_y;
        current_sse = center_sse;
        initial_count = center_count;
    }
    if (initial_count < 3U) {
        flex_solver_record_rejection(
            state, FLEX_SOLVER_REJECT_TOO_FEW);
        return false;
    }

    float damping = 1e-4f;
    for (size_t iteration = 0; iteration < 6U; ++iteration) {
        float h00 = 0.0f, h01 = 0.0f, h11 = 0.0f;
        float g0 = 0.0f, g1 = 0.0f;
        size_t count = 0;
        current_sse = flex_solver_position_cost(
            batch, x, y, &h00, &h01, &h11, &g0, &g1, &count);
        const float damped_h00 = h00 + damping;
        const float damped_h11 = h11 + damping;
        const float determinant = damped_h00 * damped_h11 - h01 * h01;
        if (count < 3U || fabsf(determinant) < 1e-9f) {
            flex_solver_record_rejection(
                state, FLEX_SOLVER_REJECT_SINGULAR);
            return false;
        }
        float dx = (damped_h11 * g0 - h01 * g1) / determinant;
        float dy = (-h01 * g0 + damped_h00 * g1) / determinant;
        const float step_length = sqrtf(dx * dx + dy * dy);
        const float max_step = 0.25f * geometry_span;
        if (step_length > max_step) {
            dx *= max_step / step_length;
            dy *= max_step / step_length;
        }

        bool accepted = false;
        float accepted_scale = 1.0f;
        for (size_t attempt = 0; attempt < 6U; ++attempt) {
            const float candidate_x = x + accepted_scale * dx;
            const float candidate_y = y + accepted_scale * dy;
            size_t candidate_count = 0;
            const float candidate_sse = flex_solver_position_cost(
                batch, candidate_x, candidate_y, NULL, NULL, NULL, NULL,
                NULL, &candidate_count);
            if (candidate_count == count && candidate_sse <= current_sse) {
                x = candidate_x;
                y = candidate_y;
                current_sse = candidate_sse;
                accepted = true;
                break;
            }
            accepted_scale *= 0.5f;
        }
        if (!accepted) {
            damping *= 10.0f;
            continue;
        }
        damping = fmaxf(1e-6f, damping * 0.25f);
        const float accepted_dx = accepted_scale * dx;
        const float accepted_dy = accepted_scale * dy;
        if (sqrtf(accepted_dx * accepted_dx + accepted_dy * accepted_dy) <
            0.0001f) {
            break;
        }
    }

    size_t used_count = 0;
    float final_h00 = 0.0f, final_h01 = 0.0f, final_h11 = 0.0f;
    (void)flex_solver_position_cost(
        batch, x, y, &final_h00, &final_h01, &final_h11, NULL, NULL,
        &used_count);
    const float final_sse =
        flex_solver_position_raw_sse(batch, x, y, &used_count);
    const float determinant =
        final_h00 * final_h11 - final_h01 * final_h01;
    if (!isfinite(x) || !isfinite(y) || used_count < 3U ||
        determinant <= 1e-9 || x < min_x - bound_margin ||
        x > max_x + bound_margin || y < min_y - bound_margin ||
        y > max_y + bound_margin) {
        flex_solver_record_rejection(
            state, FLEX_SOLVER_REJECT_BOUNDS);
        return false;
    }
    const float variance =
        final_sse / fmaxf(1.0f, (float)used_count - 2.0f);
    const float sigma = sqrtf(fmaxf(
        0.0f, variance * (final_h00 + final_h11) / determinant / 2.0f));
    const float rms = sqrtf(final_sse / used_count);
    if (!isfinite(sigma) || !isfinite(rms) || rms > 0.15f) {
        flex_solver_record_rejection(
            state, FLEX_SOLVER_REJECT_RMS);
        return false;
    }
    const app_runtime_config_t *config = app_runtime_config_get();
    float filtered_x = x;
    float filtered_y = y;
    bool filter_correction_applied = false;
    if (config->runtime_mode == APP_RUNTIME_MODE_UWB_PASSIVE_DS_TWR) {
        const bool filter_correction_requested =
            flex_solver_filter_correction_requested(
                config->passive_ds_solve_mode, independent_frame,
                complete_superframe);
        flex_solver_filter_position(
            state, x, y, sigma, now, filter_correction_requested,
            &filtered_x, &filtered_y, &filter_correction_applied);
    }
    state->position_x = filtered_x;
    state->position_y = filtered_y;
    state->position_valid = true;
    state->position_accepted++;
    if (independent_frame) {
        state->independent_frame_accepted++;
    }
    if (complete_superframe) {
        state->complete_superframe_accepted++;
    }
    if (filter_correction_applied) {
        state->filter_correction_accepted++;
    }
    state->last_raw_position_valid = true;
    state->last_raw_x = x;
    state->last_raw_y = y;
    state->last_sigma = sigma;
    state->last_rms = rms;
    state->last_raw_tick = now;
    state->last_observation_count = (uint16_t)used_count;
    state->last_batch_span_ms = batch->span_ms;
    state->last_batch_max_age_ms = batch->max_age_ms;
    state->last_observation_mask = batch->observation_mask;
    if (config->runtime_mode == APP_RUNTIME_MODE_UWB_PASSIVE_DS_TWR) {
        const bool submitted =
            wireless_telemetry_service_submit_passive_ds_position(
            tag_id, slot_id, (int32_t)lroundf(filtered_x * 1000.0f),
            (int32_t)lroundf(filtered_y * 1000.0f),
            (int32_t)lroundf(x * 1000.0f),
            (int32_t)lroundf(y * 1000.0f),
            (int32_t)lroundf(sigma * 1000.0f),
            (int32_t)lroundf(rms * 1000.0f), (uint16_t)used_count,
            state->anchor_count, state->geometry_version,
            independent_frame, complete_superframe,
            filter_correction_applied, state->position_accepted,
            state->independent_frame_accepted, batch->span_ms,
            batch->max_age_ms, batch->observation_mask,
            state->rejection_reason_mask_since_submit,
            state->rejected_since_submit, state->position_rejected);
        if (submitted) {
            state->rejection_reason_mask_since_submit = 0U;
            state->rejected_since_submit = 0U;
        }
    } else {
        (void)wireless_telemetry_service_submit_flex_position(
            tag_id, slot_id, (int32_t)lroundf(x * 1000.0f),
            (int32_t)lroundf(y * 1000.0f),
            (int32_t)lroundf(sigma * 1000.0f),
            (int32_t)lroundf(rms * 1000.0f), (uint16_t)used_count,
            state->anchor_count, state->geometry_version);
    }
    return true;
}

static void flex_solver_reset_coherent_frame(
    struct flex_solver_state *state)
{
    state->coherent_frame_pending = false;
    state->coherent_observation_count = 0U;
    state->coherent_observation_mask = 0U;
    memset(state->coherent_observations, 0,
           sizeof(state->coherent_observations));
}

static void flex_solver_start_coherent_frame(
    struct flex_solver_state *state, uint32_t frame_id, uint8_t tag_id)
{
    flex_solver_reset_coherent_frame(state);
    state->coherent_frame_pending = true;
    state->coherent_frame_id = frame_id;
    state->coherent_tag_id = tag_id;
}

static bool flex_solver_store_coherent_observation(
    struct flex_solver_state *state, size_t initiator, size_t responder,
    uint32_t slot_id, double value_m, TickType_t now)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    const uint32_t slots_per_frame =
        flex_solver_slots_per_position_frame(config);
    const uint32_t slot_index = slot_id % slots_per_frame;
    if (!state->coherent_frame_pending ||
        slot_index >= slots_per_frame ||
        slot_index >= APP_RUNTIME_CONFIG_MAX_ANCHORS - 1U) {
        return false;
    }
    struct flex_solver_frame_observation *frame_item =
        &state->coherent_observations[slot_index];
    if (!frame_item->valid) {
        state->coherent_observation_count++;
    }
    *frame_item = (struct flex_solver_frame_observation){
        .valid = true,
        .initiator = (uint8_t)initiator,
        .responder = (uint8_t)responder,
        .measurement = {
            .valid = true,
            .value_m = value_m,
            .slot_id = slot_id,
            .updated_tick = now,
        },
    };
    state->coherent_observation_mask |=
        (uint16_t)(1U << slot_index);
    state->coherent_last_slot_id = slot_id;
    return true;
}

static void flex_solver_finalize_passive_frame(
    struct flex_solver_state *state, const app_runtime_config_t *config)
{
    if (state == NULL || config == NULL ||
        !state->coherent_frame_pending) {
        return;
    }
    if (!state->geometry_ready) {
        flex_solver_reset_coherent_frame(state);
        return;
    }
    const bool complete_superframe =
        flex_solver_complete_superframe(
            config, state->coherent_frame_id);
    struct flex_solver_position_batch batch;
    if (flex_solver_legacy_rolling_solve(
            config->passive_ds_solve_mode)) {
        flex_solver_build_rolling_position_batch(
            state, xTaskGetTickCount(), false, &batch);
        (void)flex_solver_update_position(
            state, state->coherent_tag_id,
            state->coherent_last_slot_id, true,
            complete_superframe, &batch);
    } else if (flex_solver_build_coherent_position_batch(
                   state, xTaskGetTickCount(), &batch)) {
        (void)flex_solver_update_position(
            state, state->coherent_tag_id,
            state->coherent_last_slot_id, true,
            complete_superframe, &batch);
    }
    flex_solver_reset_coherent_frame(state);
}

static void flex_solver_task(void *arg)
{
    (void)arg;
    const app_runtime_config_t *config = app_runtime_config_get();
    // Keep the robust per-path history out of the task stack. There is one
    // solver task and therefore one persistent state instance.
    static struct flex_solver_state state;
    memset(&state, 0, sizeof(state));
    state.anchor_count = config->anchor_count;
    memcpy(state.anchor_ids, config->anchor_ids, sizeof(state.anchor_ids));
    flex_solver_apply_runtime_geometry(&state);
    ESP_LOGI(TAG, "FlexTDOA local solver active: anchors=%u core=%d",
             (unsigned)state.anchor_count, xPortGetCoreID());

    size_t batch_count = 0;
    while (true) {
        struct flex_solver_item item = {0};
        if (xQueueReceive(s_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (item.type == FLEX_SOLVER_ITEM_RESET) {
            memset(&state, 0, sizeof(state));
            flex_solver_apply_runtime_geometry(&state);
            xQueueReset(s_queue);
            ESP_LOGI(TAG, "FlexTDOA local solver reset for runtime switch");
            continue;
        }
        if (item.type == FLEX_SOLVER_ITEM_RELOAD_GEOMETRY) {
            flex_solver_apply_runtime_geometry(&state);
            continue;
        }
        const int first = flex_solver_anchor_index(&state, item.first_id);
        const int second = flex_solver_anchor_index(&state, item.second_id);
        if (first < 0 || second < 0 || first == second) {
            continue;
        }
        const TickType_t now = xTaskGetTickCount();
        if (item.type == FLEX_SOLVER_ITEM_RANGE && item.value_mm > 0) {
            const bool range_changed = flex_solver_accept_range(
                &state, (size_t)first, (size_t)second,
                item.value_mm / 1000.0f, item.slot_id, now);
            if (range_changed && !state.geometry_fixed &&
                (state.last_geometry_tick == 0 ||
                now - state.last_geometry_tick >=
                    pdMS_TO_TICKS(FLEX_SOLVER_GEOMETRY_MIN_PERIOD_MS))) {
                (void)flex_solver_update_geometry(&state);
                state.last_geometry_tick = now;
            }
        } else if (item.type == FLEX_SOLVER_ITEM_OBSERVATION) {
            const double value_m = item.value_mm / 1000.0;
            const struct flex_solver_measurement *anchor_range =
                &state.ranges[first][second];
            if (!isfinite(value_m) ||
                (anchor_range->valid &&
                 fabs(value_m) >
                     anchor_range->value_m +
                         FLEX_SOLVER_OBSERVATION_PHYSICAL_MARGIN_M)) {
                state.observation_rejected++;
                continue;
            }
            const uint32_t slots_per_frame =
                flex_solver_slots_per_position_frame(config);
            const uint32_t item_frame =
                item.slot_id / slots_per_frame;
            if (config->runtime_mode ==
                    APP_RUNTIME_MODE_UWB_PASSIVE_DS_TWR) {
                if (!state.coherent_frame_pending) {
                    flex_solver_start_coherent_frame(
                        &state, item_frame, item.tag_id);
                } else if (item_frame != state.coherent_frame_id) {
                    const int32_t frame_delta =
                        (int32_t)(item_frame -
                                  state.coherent_frame_id);
                    if (frame_delta <= 0) {
                        flex_solver_record_rejection(
                            &state,
                            FLEX_SOLVER_REJECT_OUT_OF_ORDER);
                        continue;
                    }
                    flex_solver_finalize_passive_frame(
                        &state, config);
                    flex_solver_start_coherent_frame(
                        &state, item_frame, item.tag_id);
                }
            } else {
                const uint32_t pending_frame =
                    state.pending_position_slot_id /
                    slots_per_frame;
                if (state.position_pending &&
                    item_frame != pending_frame) {
                    struct flex_solver_position_batch batch;
                    flex_solver_build_rolling_position_batch(
                        &state, now, false, &batch);
                    (void)flex_solver_update_position(
                        &state, state.pending_tag_id,
                        state.pending_position_slot_id, true,
                        flex_solver_complete_superframe(
                            config, pending_frame),
                        &batch);
                }
            }
            state.observations[first][second] =
                (struct flex_solver_measurement){
                    .valid = true,
                    .value_m = value_m,
                    .slot_id = item.slot_id,
                    .updated_tick = now,
                };
            state.observation_accepted++;
            state.position_pending = true;
            state.pending_tag_id = item.tag_id;
            state.pending_position_slot_id = item.slot_id;
            if (config->runtime_mode ==
                    APP_RUNTIME_MODE_UWB_PASSIVE_DS_TWR &&
                !flex_solver_store_coherent_observation(
                    &state, (size_t)first, (size_t)second,
                    item.slot_id, value_m, now)) {
                flex_solver_record_rejection(
                    &state, FLEX_SOLVER_REJECT_FRAME_MISMATCH);
            }
            if (config->runtime_mode ==
                    APP_RUNTIME_MODE_UWB_PASSIVE_DS_TWR &&
                flex_solver_rolling_enabled(
                    config->passive_ds_solve_mode)) {
                const uint32_t rolling_hz =
                    config->passive_ds_rolling_max_hz;
                const uint32_t interval_ms =
                    rolling_hz >= 1000U
                        ? 1U
                        : (1000U + rolling_hz - 1U) / rolling_hz;
                const TickType_t interval_ticks =
                    pdMS_TO_TICKS(interval_ms) > 0
                        ? pdMS_TO_TICKS(interval_ms)
                        : 1;
                if (state.last_rolling_attempt_tick == 0 ||
                    now - state.last_rolling_attempt_tick >=
                        interval_ticks) {
                    state.last_rolling_attempt_tick = now;
                    if (flex_solver_legacy_rolling_solve(
                            config->passive_ds_solve_mode) ||
                        flex_solver_motion_compensated_rolling_solve(
                            config->passive_ds_solve_mode)) {
                        struct flex_solver_position_batch batch;
                        flex_solver_build_rolling_position_batch(
                            &state, now,
                            flex_solver_motion_compensated_rolling_solve(
                                config->passive_ds_solve_mode),
                            &batch);
                        (void)flex_solver_update_position(
                            &state, item.tag_id, item.slot_id,
                            false, false, &batch);
                    } else {
                        (void)flex_solver_publish_prediction(
                            &state, item.tag_id, item.slot_id);
                    }
                }
            }
        }

        // A continuously fed queue must still let the core-0 idle task run;
        // taskYIELD() alone cannot schedule a lower-priority task.
        if (++batch_count >= FLEX_SOLVER_MAX_ITEMS_PER_BATCH) {
            batch_count = 0;
            vTaskDelay(1);
        }

        if (state.summary_tick == 0) {
            state.summary_tick = now;
        } else if (now - state.summary_tick >= pdMS_TO_TICKS(1000)) {
            const uint32_t elapsed_ms =
                (uint32_t)((now - state.summary_tick) *
                           portTICK_PERIOD_MS);
            const uint32_t position_delta =
                state.position_accepted -
                state.previous_position_accepted;
            const uint32_t independent_frame_delta =
                state.independent_frame_accepted -
                state.previous_independent_frame_accepted;
            const uint32_t complete_superframe_delta =
                state.complete_superframe_accepted -
                state.previous_complete_superframe_accepted;
            const uint32_t filter_correction_delta =
                state.filter_correction_accepted -
                state.previous_filter_correction_accepted;
            const uint32_t position_rate_milli_hz =
                elapsed_ms > 0U
                    ? (uint32_t)(((uint64_t)position_delta * 1000000ULL) /
                                 elapsed_ms)
                    : 0U;
            (void)wireless_log_service_submit(
                'I', TAG,
                "%s solver pos=%lu.%03lu/s accept=%lu frames=%lu "
                "superframes=%lu rolling=%lu corrections=%lu reject=%lu "
                "obs=%lu/%lu range=%lu/%lu reloc=%lu "
                "rej_reason=%lu/%lu/%lu/%lu/%lu/%lu/%lu",
                config->runtime_mode ==
                        APP_RUNTIME_MODE_UWB_PASSIVE_DS_TWR
                    ? "PASSIVE_DS"
                    : "FLEX_TDOA",
                (unsigned long)(position_rate_milli_hz / 1000U),
                (unsigned long)(position_rate_milli_hz % 1000U),
                (unsigned long)(state.position_accepted -
                                state.previous_position_accepted),
                (unsigned long)independent_frame_delta,
                (unsigned long)complete_superframe_delta,
                (unsigned long)(
                    position_delta >= independent_frame_delta
                        ? position_delta - independent_frame_delta
                        : 0U),
                (unsigned long)filter_correction_delta,
                (unsigned long)(state.position_rejected -
                                state.previous_position_rejected),
                (unsigned long)(state.observation_accepted -
                                state.previous_observation_accepted),
                (unsigned long)(state.observation_rejected -
                                state.previous_observation_rejected),
                (unsigned long)(state.range_accepted -
                                state.previous_range_accepted),
                (unsigned long)(state.range_rejected -
                                state.previous_range_rejected),
                (unsigned long)state.range_relocations,
                (unsigned long)(
                    state.rejection_reason_count[
                        FLEX_SOLVER_REJECT_FRAME_INCOMPLETE] -
                    state.previous_rejection_reason_count[
                        FLEX_SOLVER_REJECT_FRAME_INCOMPLETE]),
                (unsigned long)(
                    state.rejection_reason_count[
                        FLEX_SOLVER_REJECT_FRAME_MISMATCH] -
                    state.previous_rejection_reason_count[
                        FLEX_SOLVER_REJECT_FRAME_MISMATCH]),
                (unsigned long)(
                    state.rejection_reason_count[
                        FLEX_SOLVER_REJECT_TOO_FEW] -
                    state.previous_rejection_reason_count[
                        FLEX_SOLVER_REJECT_TOO_FEW]),
                (unsigned long)(
                    state.rejection_reason_count[
                        FLEX_SOLVER_REJECT_SINGULAR] -
                    state.previous_rejection_reason_count[
                        FLEX_SOLVER_REJECT_SINGULAR]),
                (unsigned long)(
                    state.rejection_reason_count[
                        FLEX_SOLVER_REJECT_BOUNDS] -
                    state.previous_rejection_reason_count[
                        FLEX_SOLVER_REJECT_BOUNDS]),
                (unsigned long)(
                    state.rejection_reason_count[
                        FLEX_SOLVER_REJECT_RMS] -
                    state.previous_rejection_reason_count[
                        FLEX_SOLVER_REJECT_RMS]),
                (unsigned long)(
                    state.rejection_reason_count[
                        FLEX_SOLVER_REJECT_OUT_OF_ORDER] -
                    state.previous_rejection_reason_count[
                        FLEX_SOLVER_REJECT_OUT_OF_ORDER]));
            state.previous_range_accepted = state.range_accepted;
            state.previous_range_rejected = state.range_rejected;
            state.previous_observation_accepted =
                state.observation_accepted;
            state.previous_observation_rejected =
                state.observation_rejected;
            state.previous_position_accepted =
                state.position_accepted;
            state.previous_independent_frame_accepted =
                state.independent_frame_accepted;
            state.previous_complete_superframe_accepted =
                state.complete_superframe_accepted;
            state.previous_filter_correction_accepted =
                state.filter_correction_accepted;
            state.previous_position_rejected =
                state.position_rejected;
            memcpy(
                state.previous_rejection_reason_count,
                state.rejection_reason_count,
                sizeof(state.previous_rejection_reason_count));
            state.summary_tick = now;
        }
    }
}

esp_err_t flextdoa_solver_service_start(void)
{
    if (s_started) {
        return ESP_OK;
    }
    s_queue = xQueueCreate(FLEX_SOLVER_QUEUE_LEN, sizeof(struct flex_solver_item));
    if (s_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    const BaseType_t created = xTaskCreatePinnedToCore(
        flex_solver_task, "flex_solver", FLEX_SOLVER_TASK_STACK_BYTES, NULL,
        FLEX_SOLVER_TASK_PRIORITY, NULL, 0);
    if (created != pdPASS) {
        vQueueDelete(s_queue);
        s_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    s_started = true;
    return ESP_OK;
}

static bool flex_solver_submit(const struct flex_solver_item *item)
{
    if (!s_started || item == NULL ||
        xQueueSend(s_queue, item, 0) != pdTRUE) {
        s_dropped++;
        return false;
    }
    return true;
}

bool flextdoa_solver_service_submit_anchor_range(
    uint8_t anchor_a_id, uint8_t anchor_b_id, uint32_t slot_id,
    int32_t distance_mm)
{
    const struct flex_solver_item item = {
        .type = FLEX_SOLVER_ITEM_RANGE,
        .first_id = anchor_a_id,
        .second_id = anchor_b_id,
        .slot_id = slot_id,
        .value_mm = distance_mm,
    };
    return flex_solver_submit(&item);
}

bool flextdoa_solver_service_reload_geometry(void)
{
    const struct flex_solver_item item = {
        .type = FLEX_SOLVER_ITEM_RELOAD_GEOMETRY,
    };
    return flex_solver_submit(&item);
}

bool flextdoa_solver_service_reset(void)
{
    const struct flex_solver_item item = {
        .type = FLEX_SOLVER_ITEM_RESET,
    };
    return flex_solver_submit(&item);
}

bool flextdoa_solver_service_submit_observation(
    uint8_t tag_id, uint8_t initiator_id, uint8_t responder_id,
    uint32_t slot_id, int32_t difference_mm)
{
    const struct flex_solver_item item = {
        .type = FLEX_SOLVER_ITEM_OBSERVATION,
        .tag_id = tag_id,
        .first_id = initiator_id,
        .second_id = responder_id,
        .slot_id = slot_id,
        .value_mm = difference_mm,
    };
    return flex_solver_submit(&item);
}
