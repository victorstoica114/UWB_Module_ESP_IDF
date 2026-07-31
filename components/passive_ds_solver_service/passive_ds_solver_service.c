#include "passive_ds_solver_service.h"

#include <math.h>
#include <string.h>

#include "app_runtime_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "passive_ds_position_solver.h"
#include "wireless_log_service.h"
#include "wireless_telemetry_service.h"

static const char *TAG = "passive_ds_solver";

enum {
    PASSIVE_DS_SOLVER_QUEUE_LEN = 256,
    PASSIVE_DS_SOLVER_TASK_STACK_BYTES = 10240,
    PASSIVE_DS_SOLVER_TASK_PRIORITY = 3,
    PASSIVE_DS_SOLVER_FRAME_BUCKETS = 8,
    PASSIVE_DS_SOLVER_RANGE_MAX_AGE_MS = 2000,
    PASSIVE_DS_SOLVER_FRAME_MAX_AGE_MS = 250,
    PASSIVE_DS_SOLVER_MAX_VARIABLES =
        2 * APP_RUNTIME_CONFIG_MAX_ANCHORS - 3,
};

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
    uint32_t slot_id;
    TickType_t updated_tick;
};

struct passive_ds_solver_frame {
    bool pending;
    uint8_t tag_id;
    uint8_t observation_count;
    uint8_t initiator_id;
    uint16_t observation_mask;
    uint32_t frame_id;
    uint32_t last_slot_id;
    TickType_t updated_tick;
    struct passive_ds_solver_frame_item
        items[APP_RUNTIME_CONFIG_MAX_ANCHORS - 1U];
};

struct passive_ds_solver_state {
    uint8_t anchor_count;
    uint8_t anchor_ids[APP_RUNTIME_CONFIG_MAX_ANCHORS];
    struct passive_ds_solver_range
        ranges[APP_RUNTIME_CONFIG_MAX_ANCHORS]
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
    uint32_t position_rejected;
    uint32_t frame_rejected;
    uint32_t geometry_accepted;
    uint32_t geometry_rejected;
    uint32_t previous_position_accepted;
    uint32_t previous_position_rejected;
    uint32_t previous_frame_rejected;
    uint32_t previous_geometry_accepted;
    uint32_t previous_geometry_rejected;
    TickType_t summary_tick;
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
    available->initiator_id = UINT8_MAX;
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
    const size_t expected_count = state->anchor_count - 1U;
    struct passive_ds_position_observation
        observations[APP_RUNTIME_CONFIG_MAX_ANCHORS - 1U];
    TickType_t oldest_tick = now;
    TickType_t newest_tick = 0U;
    for (size_t index = 0U; index < expected_count; ++index) {
        const struct passive_ds_solver_frame_item *item =
            &frame->items[index];
        if (!item->valid || item->initiator_id != frame->initiator_id) {
            state->frame_rejected++;
            reset_frame(frame);
            return;
        }
        observations[index] = (struct passive_ds_position_observation){
            .initiator_id = item->initiator_id,
            .responder_id = item->responder_id,
            .difference_m = item->difference_m,
        };
        if (item->updated_tick < oldest_tick) {
            oldest_tick = item->updated_tick;
        }
        if (item->updated_tick > newest_tick) {
            newest_tick = item->updated_tick;
        }
    }

    struct passive_ds_position_result result = {0};
    const bool solved = passive_ds_position_solve(
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
    const uint32_t measured_span_ms =
        (uint32_t)((newest_tick - oldest_tick) * portTICK_PERIOD_MS);
    const uint32_t measured_age_ms =
        (uint32_t)((now - oldest_tick) * portTICK_PERIOD_MS);
    const app_runtime_config_t *config = app_runtime_config_get();
    const uint32_t nominal_span_ms =
        ((uint32_t)state->anchor_count - 1U) *
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
        state->geometry_version, true, false, false,
        state->position_accepted, state->position_accepted,
        span_ms, age_ms, mask, 0U, 0U,
        state->position_rejected, true);
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

    const uint32_t slots_per_frame = (uint32_t)state->anchor_count - 1U;
    const uint32_t frame_id = item->slot_id / slots_per_frame;
    const uint32_t slot_index = item->slot_id % slots_per_frame;
    struct passive_ds_solver_frame *frame = acquire_frame(
        state, frame_id, item->tag_id, now);
    if (frame == NULL || slot_index >= slots_per_frame) {
        state->frame_rejected++;
        return;
    }
    if (frame->initiator_id == UINT8_MAX) {
        frame->initiator_id = item->first_id;
    } else if (frame->initiator_id != item->first_id) {
        state->frame_rejected++;
        reset_frame(frame);
        return;
    }
    for (size_t index = 0U; index < slots_per_frame; ++index) {
        if (index != slot_index && frame->items[index].valid &&
            frame->items[index].responder_id == item->second_id) {
            state->frame_rejected++;
            reset_frame(frame);
            return;
        }
    }
    if (!frame->items[slot_index].valid) {
        frame->observation_count++;
    }
    frame->items[slot_index] = (struct passive_ds_solver_frame_item){
        .valid = true,
        .initiator_id = item->first_id,
        .responder_id = item->second_id,
        .difference_m = difference_m,
        .slot_id = item->slot_id,
        .updated_tick = now,
    };
    frame->observation_mask |= observation_mask_bit(
        state, item->first_id, item->second_id);
    frame->last_slot_id = item->slot_id;
    frame->updated_tick = now;
    if (frame->observation_count == slots_per_frame) {
        solve_complete_frame(state, frame, now);
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
        "ready=%u version=%lu fit=%ldmm queue_drop=%lu",
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
        (unsigned long)s_dropped);
    state->previous_position_accepted = state->position_accepted;
    state->previous_position_rejected = state->position_rejected;
    state->previous_frame_rejected = state->frame_rejected;
    state->previous_geometry_accepted = state->geometry_accepted;
    state->previous_geometry_rejected = state->geometry_rejected;
    state->summary_tick = now;
}

static void passive_ds_solver_task(void *arg)
{
    (void)arg;
    static struct passive_ds_solver_state state;
    apply_runtime_config(&state);
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
    const BaseType_t created = xTaskCreatePinnedToCore(
        passive_ds_solver_task, "passive_ds_solver",
        PASSIVE_DS_SOLVER_TASK_STACK_BYTES, NULL,
        PASSIVE_DS_SOLVER_TASK_PRIORITY, NULL, 0);
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
    uint32_t slot_id, int32_t difference_mm)
{
    const struct passive_ds_solver_item item = {
        .type = PASSIVE_DS_SOLVER_ITEM_OBSERVATION,
        .tag_id = tag_id,
        .first_id = initiator_id,
        .second_id = responder_id,
        .slot_id = slot_id,
        .value_mm = difference_mm,
    };
    return submit_item(&item);
}
