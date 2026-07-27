#include "flextdoa_solver_service.h"

#include <math.h>
#include <string.h>

#include "app_config.h"
#include "app_runtime_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "wireless_telemetry_service.h"

static const char *TAG = "flextdoa_solver";

enum {
    FLEX_SOLVER_QUEUE_LEN = 256,
    FLEX_SOLVER_TASK_STACK_BYTES = 12288,
    FLEX_SOLVER_TASK_PRIORITY = 3,
    FLEX_SOLVER_POSITION_MAX_AGE_MS = 500,
    FLEX_SOLVER_GEOMETRY_MIN_PERIOD_MS = 100,
    FLEX_SOLVER_MAX_ITEMS_PER_BATCH = 64,
    FLEX_SOLVER_MAX_VARIABLES = 2 * APP_RUNTIME_CONFIG_MAX_ANCHORS - 3,
    FLEX_SOLVER_MAX_POSITION_OBSERVATIONS =
        APP_RUNTIME_CONFIG_MAX_ANCHORS *
        (APP_RUNTIME_CONFIG_MAX_ANCHORS - 1),
};

enum flex_solver_item_type {
    FLEX_SOLVER_ITEM_RANGE,
    FLEX_SOLVER_ITEM_OBSERVATION,
    FLEX_SOLVER_ITEM_RELOAD_GEOMETRY,
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

struct flex_solver_state {
    uint8_t anchor_count;
    uint8_t anchor_ids[APP_RUNTIME_CONFIG_MAX_ANCHORS];
    struct flex_solver_measurement
        ranges[APP_RUNTIME_CONFIG_MAX_ANCHORS][APP_RUNTIME_CONFIG_MAX_ANCHORS];
    struct flex_solver_measurement
        observations[APP_RUNTIME_CONFIG_MAX_ANCHORS]
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
};

struct flex_solver_position_observation {
    float value_m;
    float initiator_x;
    float initiator_y;
    float responder_x;
    float responder_y;
};

struct flex_solver_position_batch {
    struct flex_solver_position_observation
        items[FLEX_SOLVER_MAX_POSITION_OBSERVATIONS];
    size_t count;
};

static QueueHandle_t s_queue;
static bool s_started;
static uint32_t s_dropped;

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
    if (!state->geometry_ready && !flex_solver_initialize_geometry(state)) {
        return false;
    }

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
                const double dx = state->anchor_x[a] - state->anchor_x[b];
                const double dy = state->anchor_y[a] - state->anchor_y[b];
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
                state->anchor_x[anchor] += delta[x_index];
                max_delta = fmax(max_delta, fabs(delta[x_index]));
            }
            if (y_index >= 0) {
                state->anchor_y[anchor] += delta[y_index];
                max_delta = fmax(max_delta, fabs(delta[y_index]));
            }
        }
        if (max_delta < 0.0001) {
            break;
        }
    }
    state->geometry_ready = true;
    state->geometry_version++;
    return true;
}

static void flex_solver_build_position_batch(
    const struct flex_solver_state *state, TickType_t now,
    struct flex_solver_position_batch *batch)
{
    batch->count = 0;
    const TickType_t max_age_ticks =
        pdMS_TO_TICKS(FLEX_SOLVER_POSITION_MAX_AGE_MS);
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
                    .initiator_x = (float)state->anchor_x[initiator],
                    .initiator_y = (float)state->anchor_y[initiator],
                    .responder_x = (float)state->anchor_x[responder],
                    .responder_y = (float)state->anchor_y[responder],
                };
        }
    }
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
        const float initiator_dx = x - observation->initiator_x;
        const float initiator_dy = y - observation->initiator_y;
        const float responder_dx = x - observation->responder_x;
        const float responder_dy = y - observation->responder_y;
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
        local_h00 += jx * jx;
        local_h01 += jx * jy;
        local_h11 += jy * jy;
        local_g0 += jx * residual;
        local_g1 += jy * residual;
        sse += residual * residual;
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

static void flex_solver_update_position(struct flex_solver_state *state,
                                        uint8_t tag_id, uint32_t slot_id)
{
    if (!state->geometry_ready) {
        return;
    }
    const TickType_t now = xTaskGetTickCount();
    struct flex_solver_position_batch batch;
    flex_solver_build_position_batch(state, now, &batch);
    if (batch.count < 3U) {
        return;
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
        &batch, x, y, NULL, NULL, NULL, NULL, NULL, &initial_count);
    const float center_sse = flex_solver_position_cost(
        &batch, center_x, center_y, NULL, NULL, NULL, NULL, NULL,
        &center_count);
    if (center_count >= 3U &&
        (initial_count < 3U || center_sse < current_sse)) {
        x = center_x;
        y = center_y;
        current_sse = center_sse;
        initial_count = center_count;
    }
    if (initial_count < 3U) {
        return;
    }

    float damping = 1e-4f;
    for (size_t iteration = 0; iteration < 6U; ++iteration) {
        float h00 = 0.0f, h01 = 0.0f, h11 = 0.0f;
        float g0 = 0.0f, g1 = 0.0f;
        size_t count = 0;
        current_sse = flex_solver_position_cost(
            &batch, x, y, &h00, &h01, &h11, &g0, &g1, &count);
        const float damped_h00 = h00 + damping;
        const float damped_h11 = h11 + damping;
        const float determinant = damped_h00 * damped_h11 - h01 * h01;
        if (count < 3U || fabsf(determinant) < 1e-9f) {
            return;
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
                &batch, candidate_x, candidate_y, NULL, NULL, NULL, NULL,
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
    const float final_sse = flex_solver_position_cost(
        &batch, x, y, &final_h00, &final_h01, &final_h11, NULL, NULL,
        &used_count);
    const float determinant =
        final_h00 * final_h11 - final_h01 * final_h01;
    if (!isfinite(x) || !isfinite(y) || used_count < 3U ||
        determinant <= 1e-9 || x < min_x - bound_margin ||
        x > max_x + bound_margin || y < min_y - bound_margin ||
        y > max_y + bound_margin) {
        state->position_valid = false;
        return;
    }
    const float variance =
        final_sse / fmaxf(1.0f, (float)used_count - 2.0f);
    const float sigma = sqrtf(fmaxf(
        0.0f, variance * (final_h00 + final_h11) / determinant / 2.0f));
    const float rms = sqrtf(final_sse / used_count);
    state->position_x = x;
    state->position_y = y;
    state->position_valid = true;
    const app_runtime_config_t *config = app_runtime_config_get();
    if (config->runtime_mode == APP_RUNTIME_MODE_UWB_PASSIVE_DS_TWR) {
        (void)wireless_telemetry_service_submit_passive_ds_position(
            tag_id, slot_id, (int32_t)lroundf(x * 1000.0f),
            (int32_t)lroundf(y * 1000.0f),
            (int32_t)lroundf(sigma * 1000.0f),
            (int32_t)lroundf(rms * 1000.0f), (uint16_t)used_count,
            state->anchor_count, state->geometry_version);
    } else {
        (void)wireless_telemetry_service_submit_flex_position(
            tag_id, slot_id, (int32_t)lroundf(x * 1000.0f),
            (int32_t)lroundf(y * 1000.0f),
            (int32_t)lroundf(sigma * 1000.0f),
            (int32_t)lroundf(rms * 1000.0f), (uint16_t)used_count,
            state->anchor_count, state->geometry_version);
    }
}

static void flex_solver_task(void *arg)
{
    (void)arg;
    const app_runtime_config_t *config = app_runtime_config_get();
    struct flex_solver_state state = {
        .anchor_count = config->anchor_count,
    };
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
            struct flex_solver_measurement measurement = {
                .valid = true,
                .value_m = item.value_mm / 1000.0,
                .slot_id = item.slot_id,
                .updated_tick = now,
            };
            state.ranges[first][second] = measurement;
            state.ranges[second][first] = measurement;
            if (!state.geometry_fixed &&
                (state.last_geometry_tick == 0 ||
                now - state.last_geometry_tick >=
                    pdMS_TO_TICKS(FLEX_SOLVER_GEOMETRY_MIN_PERIOD_MS))) {
                (void)flex_solver_update_geometry(&state);
                state.last_geometry_tick = now;
            }
        } else if (item.type == FLEX_SOLVER_ITEM_OBSERVATION) {
            const uint32_t slots_per_frame =
                flex_solver_slots_per_position_frame(config);
            const uint32_t pending_frame =
                state.pending_position_slot_id / slots_per_frame;
            const uint32_t item_frame =
                item.slot_id / slots_per_frame;
            if (state.position_pending && item_frame != pending_frame) {
                flex_solver_update_position(
                    &state, state.pending_tag_id,
                    state.pending_position_slot_id);
            }
            state.observations[first][second] =
                (struct flex_solver_measurement){
                    .valid = true,
                    .value_m = item.value_mm / 1000.0,
                    .slot_id = item.slot_id,
                    .updated_tick = now,
                };
            state.position_pending = true;
            state.pending_tag_id = item.tag_id;
            state.pending_position_slot_id = item.slot_id;
        }

        // A continuously fed queue must still let the core-0 idle task run;
        // taskYIELD() alone cannot schedule a lower-priority task.
        if (++batch_count >= FLEX_SOLVER_MAX_ITEMS_PER_BATCH) {
            batch_count = 0;
            vTaskDelay(1);
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
