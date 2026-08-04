#include "flextdoa_solver_service.h"

#include <math.h>
#include <string.h>

#include "app_runtime_config.h"
#include "esp_log.h"
#include "flextdoa_algmin.h"
#include "flextdoa_frame_aggregator.h"
#include "flextdoa_protocol.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "wireless_log_service.h"
#include "wireless_telemetry_service.h"

static const char *TAG = "flextdoa_solver";

enum {
    FLEX_SOLVER_QUEUE_LEN = 128,
    FLEX_SOLVER_TASK_STACK_BYTES = 6144,
    FLEX_SOLVER_TASK_PRIORITY = 3,
    FLEX_SOLVER_FRAME_CAPACITY =
        APP_RUNTIME_CONFIG_FLEX_MAX_SLOTS *
        (APP_RUNTIME_CONFIG_MAX_ANCHORS - 1U),
};

enum flex_solver_item_type {
    FLEX_SOLVER_ITEM_OBSERVATION,
    FLEX_SOLVER_ITEM_RELOAD_GEOMETRY,
    FLEX_SOLVER_ITEM_RESET,
};

struct flex_solver_item {
    enum flex_solver_item_type type;
    uint8_t tag_id;
    uint8_t initiator_id;
    uint8_t responder_id;
    uint32_t slot_id;
    int32_t difference_mm;
};

struct flex_solver_state {
    bool geometry_ready;
    uint8_t anchor_count;
    uint32_t geometry_generation;
    struct flextdoa_anchor_position anchors[FLEXTDOA_MAX_ANCHORS];
    struct flextdoa_algmin_seed previous_position;
    uint32_t frame_slot_ids[FLEX_SOLVER_FRAME_CAPACITY];
    struct flextdoa_range_difference
        frame_observations[FLEX_SOLVER_FRAME_CAPACITY];
    struct flextdoa_range_difference
        frame_solver_observations[FLEX_SOLVER_FRAME_CAPACITY];
    struct flextdoa_frame_aggregator frame;
    uint8_t frame_tag_id;
    bool last_closed_frame_valid;
    uint32_t last_closed_frame;
    uint32_t last_published_geometry_generation;
    TickType_t last_geometry_publish_tick;
    TickType_t summary_tick;
    uint32_t observations_accepted;
    uint32_t observations_rejected;
    uint32_t frames_complete;
    uint32_t frames_incomplete;
    uint32_t positions_published;
    uint32_t solver_rejected;
    uint32_t previous_observations_accepted;
    uint32_t previous_observations_rejected;
    uint32_t previous_frames_complete;
    uint32_t previous_frames_incomplete;
    uint32_t previous_positions_published;
    uint32_t previous_solver_rejected;
};

static QueueHandle_t s_queue;
static bool s_started;
static volatile uint32_t s_dropped;

static void flex_solver_load_geometry(struct flex_solver_state *state)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    state->anchor_count = config->anchor_count;
    state->geometry_generation = config->flex_tdoa_geometry_generation;
    state->geometry_ready =
        config->flex_tdoa_geometry_fixed && config->anchor_count >= 3U &&
        config->anchor_count <= FLEXTDOA_MAX_ANCHORS;
    memset(state->anchors, 0, sizeof(state->anchors));
    for (size_t index = 0U;
         state->geometry_ready && index < config->anchor_count; ++index) {
        state->anchors[index].anchor_id = config->anchor_ids[index];
        state->anchors[index].x_m =
            config->flex_tdoa_anchor_x_mm[index] / 1000.0;
        state->anchors[index].y_m =
            config->flex_tdoa_anchor_y_mm[index] / 1000.0;
    }
    state->previous_position.valid = false;
    state->frame_tag_id = 0U;
    state->last_closed_frame_valid = false;
    state->last_published_geometry_generation = 0U;
    state->last_geometry_publish_tick = 0U;
    flextdoa_frame_aggregator_init(
        &state->frame, state->frame_slot_ids,
        state->frame_observations, FLEX_SOLVER_FRAME_CAPACITY);
    if (state->geometry_ready) {
        ESP_LOGI(TAG,
                 "paper AlgMin geometry loaded generation=%lu anchors=%u",
                 (unsigned long)state->geometry_generation,
                 (unsigned)state->anchor_count);
    } else {
        ESP_LOGW(TAG,
                 "paper AlgMin waiting for fixed anchor geometry; dynamic ranging geometry is disabled");
    }
}

static bool flex_solver_build_plan(
    uint32_t slot_id, struct flextdoa_slot_plan *plan)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    if (config->anchor_count > FLEXTDOA_MAX_ANCHORS ||
        config->flex_tdoa_slot_count == 0U) {
        return false;
    }
    uint16_t anchor_ids[FLEXTDOA_MAX_ANCHORS] = {0};
    uint16_t slot_initiators[APP_RUNTIME_CONFIG_FLEX_MAX_SLOTS] = {0};
    for (size_t index = 0U; index < config->anchor_count; ++index) {
        anchor_ids[index] = config->anchor_ids[index];
    }
    for (size_t index = 0U; index < config->flex_tdoa_slot_count; ++index) {
        slot_initiators[index] =
            config->flex_tdoa_slot_initiator_ids[index];
    }
    return flextdoa_build_ci_cr_slot(
        anchor_ids, config->anchor_count, slot_initiators,
        config->flex_tdoa_slot_count,
        config->flex_tdoa_slot_responder_masks,
        config->flex_tdoa_responder_count, slot_id, plan);
}

static const struct flextdoa_anchor_position *flex_solver_find_anchor(
    const struct flex_solver_state *state, uint16_t anchor_id)
{
    for (size_t index = 0U; index < state->anchor_count; ++index) {
        if (state->anchors[index].anchor_id == anchor_id) {
            return &state->anchors[index];
        }
    }
    return NULL;
}

static bool flex_solver_observation_is_physical(
    const struct flex_solver_state *state, uint16_t initiator_id,
    uint16_t responder_id, double difference_m)
{
    const struct flextdoa_anchor_position *initiator =
        flex_solver_find_anchor(state, initiator_id);
    const struct flextdoa_anchor_position *responder =
        flex_solver_find_anchor(state, responder_id);
    if (initiator == NULL || responder == NULL || !isfinite(difference_m)) {
        return false;
    }
    const double baseline_m =
        hypot(responder->x_m - initiator->x_m,
              responder->y_m - initiator->y_m);
    return fabs(difference_m) <= baseline_m + 0.15;
}

static bool flex_solver_publish_geometry(
    struct flex_solver_state *state, uint8_t tag_id)
{
    bool submitted = true;
    for (size_t index = 0U; index < state->anchor_count; ++index) {
        submitted = wireless_telemetry_service_submit_flex_geometry(
                        tag_id, (uint8_t)state->anchors[index].anchor_id,
                        state->anchor_count, state->geometry_generation,
                        (int32_t)lround(state->anchors[index].x_m * 1000.0),
                        (int32_t)lround(state->anchors[index].y_m * 1000.0),
                        0) &&
                    submitted;
    }
    return submitted;
}

static void flex_solver_close_frame(struct flex_solver_state *state)
{
    struct flextdoa_frame_aggregator *frame = &state->frame;
    if (!frame->active) {
        return;
    }
    const uint32_t closed_frame = frame->frame_id;
    const uint32_t closing_slot = frame->last_slot_id;
    const uint8_t tag_id = state->frame_tag_id;
    const uint16_t observation_count =
        (uint16_t)frame->observation_count;
    const bool complete = flextdoa_frame_aggregator_complete(frame);
    if (complete) {
        state->frames_complete++;
    } else {
        state->frames_incomplete++;
    }
    state->last_closed_frame = closed_frame;
    state->last_closed_frame_valid = true;

    if (!flextdoa_frame_aggregator_solvable(frame)) {
        state->solver_rejected++;
        state->frame_tag_id = 0U;
        flextdoa_frame_aggregator_reset(frame);
        return;
    }

    const size_t solver_observation_count =
        flextdoa_frame_aggregator_copy_corrected(
            frame, NULL, 0U, state->frame_solver_observations,
            FLEX_SOLVER_FRAME_CAPACITY);
    if (solver_observation_count != frame->observation_count) {
        state->solver_rejected++;
        state->frame_tag_id = 0U;
        flextdoa_frame_aggregator_reset(frame);
        return;
    }

    struct flextdoa_algmin_result result = {0};
    if (!flextdoa_algmin_solve_2d(
            state->anchors, state->anchor_count,
            state->frame_solver_observations,
            solver_observation_count, &state->previous_position, &result) ||
        !result.valid || fabs(result.x_m) > 100000.0 ||
        fabs(result.y_m) > 100000.0) {
        state->solver_rejected++;
        state->frame_tag_id = 0U;
        flextdoa_frame_aggregator_reset(frame);
        return;
    }

    state->previous_position.valid = true;
    state->previous_position.x_m = result.x_m;
    state->previous_position.y_m = result.y_m;
    const TickType_t now = xTaskGetTickCount();
    if (state->last_published_geometry_generation !=
            state->geometry_generation ||
        now - state->last_geometry_publish_tick >= pdMS_TO_TICKS(1000U)) {
        if (flex_solver_publish_geometry(state, tag_id)) {
            state->last_published_geometry_generation =
                state->geometry_generation;
            state->last_geometry_publish_tick = now;
        }
    }
    const int32_t rms_mm =
        (int32_t)lround(result.residual_rms_m * 1000.0);
    if (wireless_telemetry_service_submit_flex_position(
            tag_id, closing_slot,
            (int32_t)lround(result.x_m * 1000.0),
            (int32_t)lround(result.y_m * 1000.0), rms_mm, rms_mm,
            observation_count, state->anchor_count,
            state->geometry_generation)) {
        state->positions_published++;
    }
    state->frame_tag_id = 0U;
    flextdoa_frame_aggregator_reset(frame);
}

static void flex_solver_accept_observation(
    struct flex_solver_state *state, const struct flex_solver_item *item)
{
    if (!state->geometry_ready || item->tag_id == 0U ||
        item->initiator_id == item->responder_id) {
        state->observations_rejected++;
        return;
    }

    struct flextdoa_slot_plan plan = {0};
    if (!flex_solver_build_plan(item->slot_id, &plan) ||
        plan.initiator_id != item->initiator_id) {
        state->observations_rejected++;
        return;
    }
    const int responder_index =
        flextdoa_responder_index(&plan, item->responder_id);
    if (responder_index < 0) {
        state->observations_rejected++;
        return;
    }

    const double difference_m = item->difference_mm / 1000.0;
    if (!flex_solver_observation_is_physical(
            state, item->initiator_id, item->responder_id,
            difference_m)) {
        state->observations_rejected++;
        return;
    }

    const app_runtime_config_t *config = app_runtime_config_get();
    const uint32_t frame_id =
        item->slot_id / config->flex_tdoa_slot_count;
    if (state->last_closed_frame_valid &&
        (int32_t)(frame_id - state->last_closed_frame) <= 0) {
        state->observations_rejected++;
        return;
    }
    if (state->frame.active && state->frame_tag_id != item->tag_id) {
        state->observations_rejected++;
        return;
    }

    const struct flextdoa_range_difference observation = {
        .initiator_id = item->initiator_id,
        .responder_id = item->responder_id,
        .range_difference_m = difference_m,
    };
    const uint16_t complete_frame_observations =
        (uint16_t)((uint16_t)config->flex_tdoa_slot_count *
                   config->flex_tdoa_responder_count);
    for (uint8_t attempt = 0U; attempt < 2U; ++attempt) {
        const enum flextdoa_frame_ingest_result ingest =
            flextdoa_frame_aggregator_ingest(
                &state->frame, item->slot_id,
                config->flex_tdoa_slot_count,
                config->flex_tdoa_responder_count,
                complete_frame_observations, &observation);
        if (ingest == FLEXTDOA_FRAME_BOUNDARY) {
            flex_solver_close_frame(state);
            continue;
        }
        if (ingest == FLEXTDOA_FRAME_ACCEPTED ||
            ingest == FLEXTDOA_FRAME_COMPLETE) {
            if (state->frame.observation_count == 1U) {
                state->frame_tag_id = item->tag_id;
            }
            state->observations_accepted++;
            if (ingest == FLEXTDOA_FRAME_COMPLETE) {
                flex_solver_close_frame(state);
            }
            return;
        }
        state->observations_rejected++;
        return;
    }
    state->observations_rejected++;
}

static void flex_solver_log_summary(struct flex_solver_state *state,
                                    TickType_t now)
{
    if (state->summary_tick == 0U) {
        state->summary_tick = now;
        return;
    }
    if (now - state->summary_tick < pdMS_TO_TICKS(1000U)) {
        return;
    }
    (void)wireless_log_service_submit(
        'I', TAG,
        "FLEX_TDOA raw frame AlgMin geometry=%u obs=%lu/%lu frames=%lu/%lu pos=%lu reject=%lu queue_drop=%lu",
        state->geometry_ready ? 1U : 0U,
        (unsigned long)(state->observations_accepted -
                        state->previous_observations_accepted),
        (unsigned long)(state->observations_rejected -
                        state->previous_observations_rejected),
        (unsigned long)(state->frames_complete -
                        state->previous_frames_complete),
        (unsigned long)(state->frames_incomplete -
                        state->previous_frames_incomplete),
        (unsigned long)(state->positions_published -
                        state->previous_positions_published),
        (unsigned long)(state->solver_rejected -
                        state->previous_solver_rejected),
        (unsigned long)s_dropped);
    state->previous_observations_accepted = state->observations_accepted;
    state->previous_observations_rejected = state->observations_rejected;
    state->previous_frames_complete = state->frames_complete;
    state->previous_frames_incomplete = state->frames_incomplete;
    state->previous_positions_published = state->positions_published;
    state->previous_solver_rejected = state->solver_rejected;
    state->summary_tick = now;
}

static void flex_solver_task(void *arg)
{
    (void)arg;
    static struct flex_solver_state state;
    memset(&state, 0, sizeof(state));
    flex_solver_load_geometry(&state);
    ESP_LOGI(TAG, "raw frame-local AlgMin active on core=%d",
             xPortGetCoreID());

    while (true) {
        struct flex_solver_item item = {0};
        if (xQueueReceive(s_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (item.type == FLEX_SOLVER_ITEM_RESET) {
            memset(&state, 0, sizeof(state));
            flex_solver_load_geometry(&state);
            xQueueReset(s_queue);
            continue;
        }
        if (item.type == FLEX_SOLVER_ITEM_RELOAD_GEOMETRY) {
            flex_solver_load_geometry(&state);
            continue;
        }
        flex_solver_accept_observation(&state, &item);
        flex_solver_log_summary(&state, xTaskGetTickCount());
    }
}

esp_err_t flextdoa_solver_service_start(void)
{
    if (s_started) {
        return ESP_OK;
    }
    s_queue = xQueueCreate(FLEX_SOLVER_QUEUE_LEN,
                           sizeof(struct flex_solver_item));
    if (s_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    const BaseType_t created = xTaskCreatePinnedToCore(
        flex_solver_task, "flex_algmin", FLEX_SOLVER_TASK_STACK_BYTES, NULL,
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
    (void)anchor_a_id;
    (void)anchor_b_id;
    (void)slot_id;
    /* Anchor-to-anchor ranging remains available for diagnostics and packet
     * piggybacking, but paper AlgMin uses only fixed configured geometry. */
    return distance_mm > 0;
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
    if (!s_started) {
        return true;
    }
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
        .initiator_id = initiator_id,
        .responder_id = responder_id,
        .slot_id = slot_id,
        .difference_mm = difference_mm,
    };
    return flex_solver_submit(&item);
}
