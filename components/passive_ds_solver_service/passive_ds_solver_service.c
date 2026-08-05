#include "passive_ds_solver_service.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "app_runtime_config.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "passive_ds_batch_policy.h"
#include "passive_ds_position_solver.h"
#include "uwb_config.h"
#include "wireless_log_service.h"
#include "wireless_telemetry_service.h"

static const char *TAG = "passive_ds_solver";

enum {
    /* Preserve the validated burst margin while three-star GLS is active. */
    PASSIVE_DS_SOLVER_QUEUE_LEN = 512,
    /*
     * The position solver has a worst-case 56 x 56 double precision matrix
     * in its timing-covariance path.  Keep the established 40 KiB stack so
     * the raw and covariance solvers can share this task safely.  A 12 KiB
     * stack overflows as soon as the first complete Passive DS star is
     * solved, which makes the receive-only tag reboot intermittently.
     */
    PASSIVE_DS_SOLVER_TASK_STACK_BYTES = 40960,
    PASSIVE_DS_SOLVER_TASK_PRIORITY = 5,
    PASSIVE_DS_SOLVER_FRAME_BUCKETS = 32,
    PASSIVE_DS_SOLVER_FRAME_MAX_AGE_MS = 250,
    PASSIVE_DS_SOLVER_INDEPENDENT_MAX_FRAME_SPAN = 3,
    PASSIVE_DS_SOLVER_OVERLAP_MAX_FRAME_SPAN = 6,
    PASSIVE_DS_SOLVER_MAX_OBSERVATIONS =
        PASSIVE_DS_BATCH_STARS_PER_POSITION *
        (APP_RUNTIME_CONFIG_MAX_ANCHORS - 1U),
};

/* Same-star spatial covariance used by the field-validated raw solver. */
static const double PASSIVE_DS_SOLVER_COMMON_CORRELATION = 0.25;

enum passive_ds_solver_item_type {
    PASSIVE_DS_SOLVER_ITEM_OBSERVATION,
    PASSIVE_DS_SOLVER_ITEM_RESET,
};

struct passive_ds_solver_item {
    enum passive_ds_solver_item_type type;
    uint8_t tag_id;
    uint8_t initiator_id;
    uint8_t responder_id;
    uint32_t session_id;
    uint32_t frame_id;
    int32_t difference_mm;
    uint16_t delay_ratio_q15;
};

struct passive_ds_solver_frame_observation {
    bool valid;
    uint8_t initiator_id;
    uint8_t responder_id;
    int32_t difference_mm;
    uint16_t delay_ratio_q15;
};

struct passive_ds_solver_frame {
    bool active;
    uint8_t tag_id;
    uint8_t initiator_id;
    uint8_t observation_count;
    uint16_t observation_mask;
    uint32_t session_id;
    uint32_t frame_id;
    TickType_t updated_tick;
    struct passive_ds_solver_frame_observation
        observations[APP_RUNTIME_CONFIG_MAX_ANCHORS - 1U];
};

struct passive_ds_solver_complete_star {
    bool valid;
    uint8_t tag_id;
    uint8_t initiator_id;
    uint8_t observation_count;
    uint16_t observation_mask;
    uint32_t session_id;
    uint32_t frame_id;
    TickType_t completed_tick;
    struct passive_ds_solver_frame_observation
        observations[APP_RUNTIME_CONFIG_MAX_ANCHORS - 1U];
};

struct passive_ds_solver_state {
    bool geometry_ready;
    bool geometry_published;
    uint8_t anchor_count;
    uint8_t anchor_ids[APP_RUNTIME_CONFIG_MAX_ANCHORS];
    uint32_t geometry_version;
    struct passive_ds_position_anchor
        anchors[APP_RUNTIME_CONFIG_MAX_ANCHORS];
    struct passive_ds_solver_frame frames[PASSIVE_DS_SOLVER_FRAME_BUCKETS];
    struct passive_ds_solver_complete_star
        recent_stars[APP_RUNTIME_CONFIG_MAX_ANCHORS];
    struct passive_ds_solver_complete_star
        independent_stars[APP_RUNTIME_CONFIG_MAX_ANCHORS];
    bool session_valid;
    uint32_t session_id;
    bool last_published_frame_valid;
    uint32_t last_published_frame_id;
    bool previous_position_valid;
    double previous_x_m;
    double previous_y_m;
    uint32_t solved_count;
    uint32_t independent_solved_count;
    uint32_t rejected_count;
    uint32_t incomplete_count;
    uint32_t previous_solved_count;
    uint32_t previous_independent_solved_count;
    uint32_t previous_rejected_count;
    uint32_t previous_incomplete_count;
    TickType_t summary_tick;
};

static QueueHandle_t s_queue;
static bool s_started;
static uint32_t s_dropped;

static int anchor_index(const struct passive_ds_solver_state *state,
                        uint8_t anchor_id)
{
    if (state == NULL || anchor_id == 0U) {
        return -1;
    }
    for (uint8_t index = 0U; index < state->anchor_count; ++index) {
        if (state->anchor_ids[index] == anchor_id) {
            return (int)index;
        }
    }
    return -1;
}

static bool publish_geometry(struct passive_ds_solver_state *state)
{
    if (state == NULL || !state->geometry_ready) {
        return false;
    }
    bool published = true;
    for (uint8_t index = 0U; index < state->anchor_count; ++index) {
        published &=
            wireless_telemetry_service_submit_passive_ds_geometry(
                state->anchors[index].id, state->anchor_count,
                state->geometry_version,
                (int32_t)lround(state->anchors[index].x_m * 1000.0),
                (int32_t)lround(state->anchors[index].y_m * 1000.0),
                0);
    }
    state->geometry_published = published;
    return published;
}

static void apply_runtime_config(struct passive_ds_solver_state *state)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    memset(state, 0, sizeof(*state));
    if (config == NULL || !config->flex_tdoa_geometry_fixed ||
        config->anchor_count < 3U ||
        config->anchor_count > PASSIVE_DS_POSITION_MAX_ANCHORS) {
        ESP_LOGE(TAG,
                 "fixed GPS RTK anchor geometry is required");
        return;
    }
    state->anchor_count = config->anchor_count;
    state->geometry_version = config->flex_tdoa_geometry_generation;
    for (uint8_t index = 0U; index < state->anchor_count; ++index) {
        state->anchor_ids[index] = config->anchor_ids[index];
        state->anchors[index] = (struct passive_ds_position_anchor){
            .id = config->anchor_ids[index],
            .x_m = config->flex_tdoa_anchor_x_mm[index] / 1000.0,
            .y_m = config->flex_tdoa_anchor_y_mm[index] / 1000.0,
        };
    }
    state->geometry_ready = true;
    (void)publish_geometry(state);
    ESP_LOGI(TAG,
             "raw solver geometry=fixed_rtk anchors=%u generation=%lu",
             (unsigned)state->anchor_count,
             (unsigned long)state->geometry_version);
}

static void reset_frame(struct passive_ds_solver_frame *frame)
{
    if (frame != NULL) {
        memset(frame, 0, sizeof(*frame));
    }
}

static void expire_frames(struct passive_ds_solver_state *state,
                          TickType_t now)
{
    const TickType_t maximum_age =
        pdMS_TO_TICKS(PASSIVE_DS_SOLVER_FRAME_MAX_AGE_MS);
    for (size_t index = 0U; index < PASSIVE_DS_SOLVER_FRAME_BUCKETS;
         ++index) {
        struct passive_ds_solver_frame *frame = &state->frames[index];
        if (frame->active && now - frame->updated_tick > maximum_age) {
            state->incomplete_count++;
            reset_frame(frame);
        }
    }
}

static struct passive_ds_solver_frame *acquire_frame(
    struct passive_ds_solver_state *state,
    const struct passive_ds_solver_item *item,
    TickType_t now)
{
    if (state == NULL || item == NULL) {
        return NULL;
    }
    struct passive_ds_solver_frame *available = NULL;
    struct passive_ds_solver_frame *oldest = NULL;
    for (size_t index = 0U; index < PASSIVE_DS_SOLVER_FRAME_BUCKETS;
         ++index) {
        struct passive_ds_solver_frame *frame = &state->frames[index];
        if (frame->active && frame->tag_id == item->tag_id &&
            frame->session_id == item->session_id &&
            frame->frame_id == item->frame_id) {
            if (frame->initiator_id != item->initiator_id) {
                return NULL;
            }
            frame->updated_tick = now;
            return frame;
        }
        if (!frame->active && available == NULL) {
            available = frame;
        }
        if (frame->active &&
            (oldest == NULL ||
             frame->updated_tick < oldest->updated_tick)) {
            oldest = frame;
        }
    }
    struct passive_ds_solver_frame *frame =
        available != NULL ? available : oldest;
    if (frame == NULL) {
        return NULL;
    }
    if (frame->active) {
        state->incomplete_count++;
    }
    reset_frame(frame);
    frame->active = true;
    frame->tag_id = item->tag_id;
    frame->initiator_id = item->initiator_id;
    frame->session_id = item->session_id;
    frame->frame_id = item->frame_id;
    frame->updated_tick = now;
    return frame;
}

static void reset_session_state(
    struct passive_ds_solver_state *state, uint32_t session_id)
{
    memset(state->frames, 0, sizeof(state->frames));
    memset(state->recent_stars, 0, sizeof(state->recent_stars));
    memset(state->independent_stars, 0,
           sizeof(state->independent_stars));
    state->previous_position_valid = false;
    state->last_published_frame_valid = false;
    state->session_valid = true;
    state->session_id = session_id;
}

static bool residual_metrics(
    const struct passive_ds_solver_state *state,
    const struct passive_ds_position_observation *observations,
    size_t observation_count,
    const struct passive_ds_position_result *result,
    double *equation_rms_m,
    double *max_abs_residual_m)
{
    if (state == NULL || observations == NULL || result == NULL ||
        equation_rms_m == NULL || max_abs_residual_m == NULL ||
        observation_count == 0U) {
        return false;
    }
    double sse = 0.0;
    double maximum = 0.0;
    for (size_t index = 0U; index < observation_count; ++index) {
        const int initiator = anchor_index(
            state, observations[index].initiator_id);
        const int responder = anchor_index(
            state, observations[index].responder_id);
        if (initiator < 0 || responder < 0 || initiator == responder) {
            return false;
        }
        const double predicted =
            hypot(result->x_m - state->anchors[responder].x_m,
                  result->y_m - state->anchors[responder].y_m) -
            hypot(result->x_m - state->anchors[initiator].x_m,
                  result->y_m - state->anchors[initiator].y_m);
        const double residual = observations[index].difference_m - predicted;
        if (!isfinite(residual)) {
            return false;
        }
        sse += residual * residual;
        maximum = fmax(maximum, fabs(residual));
    }
    *equation_rms_m = sqrt(sse / (double)observation_count);
    *max_abs_residual_m = maximum;
    return isfinite(*equation_rms_m) && isfinite(*max_abs_residual_m);
}

static bool solve_observations(
    struct passive_ds_solver_state *state,
    const struct passive_ds_position_observation *observations,
    size_t observation_count,
    bool correlated,
    struct passive_ds_position_result *result,
    double *equation_rms_m,
    double *max_abs_residual_m)
{
    bool use_previous = state->previous_position_valid;
    for (size_t attempt = 0U; attempt < (use_previous ? 2U : 1U);
         ++attempt) {
        const bool solved = correlated
            ? passive_ds_position_solve_correlated(
                  state->anchors, state->anchor_count,
                  observations, observation_count,
                  PASSIVE_DS_SOLVER_COMMON_CORRELATION,
                  use_previous, state->previous_x_m,
                  state->previous_y_m, result)
            : passive_ds_position_solve(
                  state->anchors, state->anchor_count,
                  observations, observation_count,
                  use_previous, state->previous_x_m,
                  state->previous_y_m, result);
        if (solved && residual_metrics(
                state, observations, observation_count, result,
                equation_rms_m, max_abs_residual_m) &&
            passive_ds_batch_residuals_valid(
                *equation_rms_m, *max_abs_residual_m)) {
            return true;
        }
        use_previous = false;
    }
    return false;
}

static bool solve_stars(
    struct passive_ds_solver_state *state,
    const struct passive_ds_solver_complete_star *stars,
    size_t star_count,
    bool independent,
    uint32_t frame_span)
{
    if (state == NULL || stars == NULL ||
        (star_count != 1U &&
         star_count != PASSIVE_DS_BATCH_STARS_PER_POSITION)) {
        return false;
    }
    const size_t observations_per_star = state->anchor_count - 1U;
    const size_t expected_count = star_count * observations_per_star;
    if (expected_count > PASSIVE_DS_SOLVER_MAX_OBSERVATIONS) {
        return false;
    }
    struct passive_ds_position_observation
        observations[PASSIVE_DS_SOLVER_MAX_OBSERVATIONS] = {0};
    size_t output = 0U;
    uint16_t observation_mask = 0U;
    for (size_t star_index = 0U; star_index < star_count; ++star_index) {
        if (!stars[star_index].valid ||
            stars[star_index].tag_id != stars[0].tag_id ||
            stars[star_index].session_id != stars[0].session_id ||
            stars[star_index].observation_count != observations_per_star) {
            return false;
        }
        observation_mask |= stars[star_index].observation_mask;
        for (size_t item = 0U; item < observations_per_star; ++item) {
            const struct passive_ds_solver_frame_observation *source =
                &stars[star_index].observations[item];
            if (!source->valid ||
                source->initiator_id != stars[star_index].initiator_id ||
                source->delay_ratio_q15 == 0U ||
                source->delay_ratio_q15 >= 32768U) {
                return false;
            }
            observations[output++] =
                (struct passive_ds_position_observation){
                    .initiator_id = source->initiator_id,
                    .responder_id = source->responder_id,
                    .difference_m = source->difference_mm / 1000.0,
                    .delay_ratio =
                        (double)source->delay_ratio_q15 / 32768.0,
                };
        }
    }

    struct passive_ds_position_result result = {0};
    double equation_rms_m = NAN;
    double max_abs_residual_m = NAN;
    if (!solve_observations(
            state, observations, output, star_count > 1U, &result,
            &equation_rms_m, &max_abs_residual_m)) {
        state->rejected_count++;
        return false;
    }
    state->previous_position_valid = true;
    state->previous_x_m = result.x_m;
    state->previous_y_m = result.y_m;
    state->solved_count++;
    if (independent) {
        state->independent_solved_count++;
    }

    const app_runtime_config_t *config = app_runtime_config_get();
    const uint32_t response_train_us =
        config->passive_ds_resp_delay_us +
        APP_UWB_PASSIVE_DS_MULTI_RESPONSE_SPACING_US *
            (state->anchor_count - 2U);
    const uint64_t batch_span_us = star_count > 1U
        ? (uint64_t)frame_span *
                  (config->passive_ds_slot_ms * 1000ULL +
                   config->passive_ds_round_gap_ms * 1000ULL) +
              response_train_us + config->passive_ds_final_delay_us
        : response_train_us;
    const uint16_t batch_span_ms = (uint16_t)fmin(
        UINT16_MAX, (double)((batch_span_us + 999ULL) / 1000ULL));
    const uint8_t tag_id = stars[0].tag_id;
    const uint32_t frame_id = stars[0].frame_id;
    const bool complete_superframe =
        star_count == 1U &&
        frame_id % state->anchor_count == state->anchor_count - 1U;

    (void)wireless_telemetry_service_submit_passive_ds_position(
        tag_id, frame_id,
        (int32_t)lround(result.x_m * 1000.0),
        (int32_t)lround(result.y_m * 1000.0),
        (int32_t)lround(result.x_m * 1000.0),
        (int32_t)lround(result.y_m * 1000.0),
        (int32_t)lround(result.sigma_m * 1000.0),
        (int32_t)lround(equation_rms_m * 1000.0),
        result.observation_count, state->anchor_count,
        state->geometry_version, independent, complete_superframe,
        false, state->solved_count, state->independent_solved_count,
        batch_span_ms, batch_span_ms, observation_mask, 0U, 0U,
        state->rejected_count, true);
    return true;
}

static int responder_slot_index(
    const struct passive_ds_solver_state *state,
    int initiator_index,
    int responder_index)
{
    if (state == NULL || initiator_index < 0 || responder_index < 0 ||
        initiator_index == responder_index || state->anchor_count < 2U) {
        return -1;
    }
    const int offset =
        (responder_index - initiator_index + state->anchor_count) %
        state->anchor_count;
    return offset > 0 ? offset - 1 : -1;
}

static size_t select_stars(
    struct passive_ds_solver_state *state,
    const struct passive_ds_solver_complete_star *cache,
    uint32_t max_frame_span,
    struct passive_ds_solver_complete_star
        selected[PASSIVE_DS_BATCH_STARS_PER_POSITION],
    size_t selected_cache_indices[PASSIVE_DS_BATCH_STARS_PER_POSITION],
    uint32_t *newest_frame_id,
    uint32_t *frame_span)
{
    struct passive_ds_batch_star_ref
        references[APP_RUNTIME_CONFIG_MAX_ANCHORS] = {0};
    for (size_t index = 0U; index < state->anchor_count; ++index) {
        references[index] = (struct passive_ds_batch_star_ref){
            .valid = cache[index].valid,
            .initiator_id = cache[index].initiator_id,
            .frame_id = cache[index].frame_id,
        };
    }
    const size_t count = passive_ds_batch_select_recent_unique(
        references, state->anchor_count, max_frame_span,
        selected_cache_indices, newest_frame_id, frame_span);
    for (size_t index = 0U; index < count; ++index) {
        selected[index] = cache[selected_cache_indices[index]];
    }
    return count;
}

static void expire_independent_stars(
    struct passive_ds_solver_state *state, uint32_t newest_frame_id)
{
    for (size_t index = 0U; index < state->anchor_count; ++index) {
        struct passive_ds_solver_complete_star *star =
            &state->independent_stars[index];
        if (star->valid &&
            passive_ds_batch_frame_after(
                newest_frame_id, star->frame_id) &&
            newest_frame_id - star->frame_id >
                PASSIVE_DS_SOLVER_INDEPENDENT_MAX_FRAME_SPAN) {
            memset(star, 0, sizeof(*star));
        }
    }
}

static void process_complete_star(
    struct passive_ds_solver_state *state,
    const struct passive_ds_solver_complete_star *star)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    if (config == NULL || star == NULL || !star->valid) {
        return;
    }
    if (config->passive_ds_solve_mode ==
        APP_RUNTIME_PASSIVE_DS_SOLVE_SINGLE_STAR) {
        (void)solve_stars(state, star, 1U, true, 0U);
        return;
    }
    if (config->passive_ds_solve_mode !=
        APP_RUNTIME_PASSIVE_DS_SOLVE_PRECISION_THREE_STAR) {
        state->rejected_count++;
        return;
    }

    const int initiator = anchor_index(state, star->initiator_id);
    if (initiator < 0) {
        state->rejected_count++;
        return;
    }
    struct passive_ds_solver_complete_star *recent =
        &state->recent_stars[initiator];
    if (!recent->valid || passive_ds_batch_frame_after(
            star->frame_id, recent->frame_id)) {
        *recent = *star;
    }
    struct passive_ds_solver_complete_star *pending =
        &state->independent_stars[initiator];
    if (!pending->valid || passive_ds_batch_frame_after(
            star->frame_id, pending->frame_id)) {
        *pending = *star;
    }
    expire_independent_stars(state, star->frame_id);

    struct passive_ds_solver_complete_star
        selected[PASSIVE_DS_BATCH_STARS_PER_POSITION] = {0};
    size_t selected_indices[PASSIVE_DS_BATCH_STARS_PER_POSITION] = {0};
    uint32_t newest_frame_id = 0U;
    uint32_t frame_span = 0U;
    size_t count = select_stars(
        state, state->independent_stars,
        PASSIVE_DS_SOLVER_INDEPENDENT_MAX_FRAME_SPAN,
        selected, selected_indices, &newest_frame_id, &frame_span);
    if (count == PASSIVE_DS_BATCH_STARS_PER_POSITION) {
        for (size_t index = 0U; index < count; ++index) {
            memset(&state->independent_stars[selected_indices[index]],
                   0, sizeof(state->independent_stars[0]));
        }
        if (!state->last_published_frame_valid ||
            passive_ds_batch_frame_after(
                newest_frame_id, state->last_published_frame_id)) {
            if (solve_stars(
                    state, selected, count, true, frame_span)) {
                state->last_published_frame_valid = true;
                state->last_published_frame_id = newest_frame_id;
            }
        }
        return;
    }

    count = select_stars(
        state, state->recent_stars,
        PASSIVE_DS_SOLVER_OVERLAP_MAX_FRAME_SPAN,
        selected, selected_indices, &newest_frame_id, &frame_span);
    if (count == PASSIVE_DS_BATCH_STARS_PER_POSITION &&
        (!state->last_published_frame_valid ||
         passive_ds_batch_frame_after(
             newest_frame_id, state->last_published_frame_id)) &&
        solve_stars(state, selected, count, false, frame_span)) {
        state->last_published_frame_valid = true;
        state->last_published_frame_id = newest_frame_id;
    }
}

static void handle_observation(
    struct passive_ds_solver_state *state,
    const struct passive_ds_solver_item *item, TickType_t now)
{
    if (!state->geometry_ready || item->tag_id == 0U ||
        item->session_id == 0U ||
        item->initiator_id == item->responder_id ||
        item->delay_ratio_q15 == 0U ||
        item->delay_ratio_q15 >= 32768U) {
        state->rejected_count++;
        return;
    }
    if (!state->session_valid || state->session_id != item->session_id) {
        reset_session_state(state, item->session_id);
    }
    const int initiator_index = anchor_index(
        state, item->initiator_id);
    const int responder_index = anchor_index(
        state, item->responder_id);
    const uint8_t expected_initiator =
        state->anchor_ids[item->frame_id % state->anchor_count];
    if (initiator_index < 0 || responder_index < 0 ||
        expected_initiator != item->initiator_id) {
        state->rejected_count++;
        return;
    }
    const int responder_slot = responder_slot_index(
        state, initiator_index, responder_index);
    if (responder_slot < 0) {
        state->rejected_count++;
        return;
    }

    struct passive_ds_solver_frame *frame = acquire_frame(
        state, item, now);
    if (frame == NULL) {
        state->rejected_count++;
        return;
    }
    struct passive_ds_solver_frame_observation *observation =
        &frame->observations[responder_slot];
    if (observation->valid) {
        if (observation->initiator_id != item->initiator_id ||
            observation->responder_id != item->responder_id ||
            observation->difference_mm != item->difference_mm ||
            observation->delay_ratio_q15 != item->delay_ratio_q15) {
            state->rejected_count++;
            reset_frame(frame);
        }
        return;
    }
    *observation = (struct passive_ds_solver_frame_observation){
        .valid = true,
        .initiator_id = item->initiator_id,
        .responder_id = item->responder_id,
        .difference_mm = item->difference_mm,
        .delay_ratio_q15 = item->delay_ratio_q15,
    };
    frame->observation_mask |=
        (uint16_t)(1U << (uint8_t)responder_index);
    frame->observation_count++;
    frame->updated_tick = now;
    if (frame->observation_count == state->anchor_count - 1U) {
        struct passive_ds_solver_complete_star star = {
            .valid = true,
            .tag_id = frame->tag_id,
            .initiator_id = frame->initiator_id,
            .observation_count = frame->observation_count,
            .observation_mask = frame->observation_mask,
            .session_id = frame->session_id,
            .frame_id = frame->frame_id,
            .completed_tick = now,
        };
        memcpy(star.observations, frame->observations,
               sizeof(star.observations));
        reset_frame(frame);
        process_complete_star(state, &star);
    }
}

static void submit_summary(struct passive_ds_solver_state *state,
                           TickType_t now)
{
    if (state->summary_tick == 0U) {
        state->summary_tick = now;
        return;
    }
    if (now - state->summary_tick < pdMS_TO_TICKS(1000)) {
        return;
    }
    (void)wireless_log_service_submit(
        'I', TAG,
        "raw fixed-RTK pos=%lu/s independent=%lu/s reject=%lu "
        "batch_incomplete=%lu "
        "geometry=%u generation=%lu queue_drop=%lu stack_free=%luB",
        (unsigned long)(state->solved_count -
                        state->previous_solved_count),
        (unsigned long)(state->independent_solved_count -
                        state->previous_independent_solved_count),
        (unsigned long)(state->rejected_count -
                        state->previous_rejected_count),
        (unsigned long)(state->incomplete_count -
                        state->previous_incomplete_count),
        state->geometry_ready ? 1U : 0U,
        (unsigned long)state->geometry_version,
        (unsigned long)s_dropped,
        (unsigned long)uxTaskGetStackHighWaterMark(NULL));
    state->previous_solved_count = state->solved_count;
    state->previous_independent_solved_count =
        state->independent_solved_count;
    state->previous_rejected_count = state->rejected_count;
    state->previous_incomplete_count = state->incomplete_count;
    state->summary_tick = now;
    if (!state->geometry_published) {
        (void)publish_geometry(state);
    }
}

static void passive_ds_solver_task(void *arg)
{
    (void)arg;
    static struct passive_ds_solver_state state;
    apply_runtime_config(&state);
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
        const TickType_t now = xTaskGetTickCount();
        expire_frames(&state, now);
        handle_observation(&state, &item, now);
        submit_summary(&state, now);
    }
}

esp_err_t passive_ds_solver_service_start(void)
{
    if (s_started) {
        return ESP_OK;
    }
    s_queue = xQueueCreate(PASSIVE_DS_SOLVER_QUEUE_LEN,
                           sizeof(struct passive_ds_solver_item));
    if (s_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
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
    if (!s_started) {
        return true;
    }
    const struct passive_ds_solver_item item = {
        .type = PASSIVE_DS_SOLVER_ITEM_RESET,
    };
    return submit_item(&item);
}

bool passive_ds_solver_service_submit_anchor_range(
    uint8_t anchor_a_id, uint8_t anchor_b_id, uint32_t slot_id,
    int32_t distance_mm)
{
    (void)slot_id;
    /* Anchor ranges are diagnostics only; fixed RTK coordinates drive the
     * raw solver and no range-derived geometry state is retained here. */
    return anchor_a_id != 0U && anchor_b_id != 0U &&
           anchor_a_id != anchor_b_id && distance_mm > 0;
}

bool passive_ds_solver_service_submit_observation(
    uint8_t tag_id, uint8_t initiator_id, uint8_t responder_id,
    uint32_t session_id, uint32_t frame_id, int32_t difference_mm,
    uint16_t delay_ratio_q15)
{
    const struct passive_ds_solver_item item = {
        .type = PASSIVE_DS_SOLVER_ITEM_OBSERVATION,
        .tag_id = tag_id,
        .initiator_id = initiator_id,
        .responder_id = responder_id,
        .session_id = session_id,
        .frame_id = frame_id,
        .difference_mm = difference_mm,
        .delay_ratio_q15 = delay_ratio_q15,
    };
    return submit_item(&item);
}
