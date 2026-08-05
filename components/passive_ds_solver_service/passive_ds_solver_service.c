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
#include "passive_ds_position_solver.h"
#include "uwb_config.h"
#include "wireless_log_service.h"
#include "wireless_telemetry_service.h"

static const char *TAG = "passive_ds_solver";

enum {
    PASSIVE_DS_SOLVER_QUEUE_LEN = 128,
    /*
     * The position solver has a worst-case 56 x 56 double precision matrix
     * in its timing-covariance path.  Keep the established 40 KiB stack so
     * the raw and covariance solvers can share this task safely.  A 12 KiB
     * stack overflows as soon as the first complete Passive DS star is
     * solved, which makes the receive-only tag reboot intermittently.
     */
    PASSIVE_DS_SOLVER_TASK_STACK_BYTES = 40960,
    PASSIVE_DS_SOLVER_TASK_PRIORITY = 5,
    PASSIVE_DS_SOLVER_FRAME_BUCKETS = 16,
    PASSIVE_DS_SOLVER_FRAME_MAX_AGE_MS = 250,
};

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
};

struct passive_ds_solver_frame_observation {
    bool valid;
    uint8_t responder_id;
    double difference_m;
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
        observations[APP_RUNTIME_CONFIG_MAX_ANCHORS];
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
    bool previous_position_valid;
    double previous_x_m;
    double previous_y_m;
    uint32_t solved_count;
    uint32_t rejected_count;
    uint32_t incomplete_count;
    uint32_t previous_solved_count;
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
        config->anchor_count > APP_RUNTIME_CONFIG_MAX_ANCHORS) {
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
    const struct passive_ds_solver_item *item, TickType_t now)
{
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

static void solve_frame(struct passive_ds_solver_state *state,
                        struct passive_ds_solver_frame *frame)
{
    struct passive_ds_position_observation
        observations[APP_RUNTIME_CONFIG_MAX_ANCHORS - 1U] = {0};
    size_t output = 0U;
    for (uint8_t index = 0U; index < state->anchor_count; ++index) {
        if (state->anchor_ids[index] == frame->initiator_id) {
            continue;
        }
        const struct passive_ds_solver_frame_observation *source =
            &frame->observations[index];
        if (!source->valid ||
            source->responder_id != state->anchor_ids[index]) {
            state->incomplete_count++;
            reset_frame(frame);
            return;
        }
        observations[output++] =
            (struct passive_ds_position_observation){
                .initiator_id = frame->initiator_id,
                .responder_id = source->responder_id,
                .difference_m = source->difference_m,
                .delay_ratio = 0.0,
            };
    }
    if (output != state->anchor_count - 1U) {
        state->incomplete_count++;
        reset_frame(frame);
        return;
    }

    struct passive_ds_position_result result = {0};
    if (!passive_ds_position_solve(
            state->anchors, state->anchor_count, observations, output,
            state->previous_position_valid, state->previous_x_m,
            state->previous_y_m, &result)) {
        state->rejected_count++;
        reset_frame(frame);
        return;
    }
    state->previous_position_valid = true;
    state->previous_x_m = result.x_m;
    state->previous_y_m = result.y_m;
    state->solved_count++;

    const app_runtime_config_t *config = app_runtime_config_get();
    const uint32_t response_train_us =
        config->passive_ds_resp_delay_us +
        APP_UWB_PASSIVE_DS_MULTI_RESPONSE_SPACING_US *
            (state->anchor_count - 2U);
    const uint16_t batch_span_ms =
        (uint16_t)((response_train_us + 999U) / 1000U);
    const uint8_t tag_id = frame->tag_id;
    const uint32_t frame_id = frame->frame_id;
    const uint16_t observation_mask = frame->observation_mask;
    reset_frame(frame);

    (void)wireless_telemetry_service_submit_passive_ds_position(
        tag_id, frame_id,
        (int32_t)lround(result.x_m * 1000.0),
        (int32_t)lround(result.y_m * 1000.0),
        (int32_t)lround(result.x_m * 1000.0),
        (int32_t)lround(result.y_m * 1000.0),
        (int32_t)lround(result.sigma_m * 1000.0),
        (int32_t)lround(result.rms_m * 1000.0),
        result.observation_count, state->anchor_count,
        state->geometry_version, true,
        frame_id % state->anchor_count == state->anchor_count - 1U,
        false, state->solved_count, state->solved_count,
        batch_span_ms, batch_span_ms, observation_mask, 0U, 0U,
        state->rejected_count, true);
}

static void handle_observation(
    struct passive_ds_solver_state *state,
    const struct passive_ds_solver_item *item, TickType_t now)
{
    if (!state->geometry_ready || item->tag_id == 0U ||
        item->session_id == 0U ||
        item->initiator_id == item->responder_id ||
        anchor_index(state, item->initiator_id) < 0) {
        state->rejected_count++;
        return;
    }
    const int responder_index = anchor_index(
        state, item->responder_id);
    const uint8_t expected_initiator =
        state->anchor_ids[item->frame_id % state->anchor_count];
    if (responder_index < 0 ||
        expected_initiator != item->initiator_id) {
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
        &frame->observations[responder_index];
    if (observation->valid) {
        return;
    }
    *observation = (struct passive_ds_solver_frame_observation){
        .valid = true,
        .responder_id = item->responder_id,
        .difference_m = item->difference_mm / 1000.0,
    };
    frame->observation_mask |=
        (uint16_t)(1U << (uint8_t)responder_index);
    frame->observation_count++;
    frame->updated_tick = now;
    if (frame->observation_count == state->anchor_count - 1U) {
        solve_frame(state, frame);
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
        "raw fixed-RTK pos=%lu/s reject=%lu incomplete=%lu "
        "geometry=%u generation=%lu queue_drop=%lu stack_free=%luB",
        (unsigned long)(state->solved_count -
                        state->previous_solved_count),
        (unsigned long)(state->rejected_count -
                        state->previous_rejected_count),
        (unsigned long)(state->incomplete_count -
                        state->previous_incomplete_count),
        state->geometry_ready ? 1U : 0U,
        (unsigned long)state->geometry_version,
        (unsigned long)s_dropped,
        (unsigned long)uxTaskGetStackHighWaterMark(NULL));
    state->previous_solved_count = state->solved_count;
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
    (void)delay_ratio_q15;
    const struct passive_ds_solver_item item = {
        .type = PASSIVE_DS_SOLVER_ITEM_OBSERVATION,
        .tag_id = tag_id,
        .initiator_id = initiator_id,
        .responder_id = responder_id,
        .session_id = session_id,
        .frame_id = frame_id,
        .difference_mm = difference_mm,
    };
    return submit_item(&item);
}
