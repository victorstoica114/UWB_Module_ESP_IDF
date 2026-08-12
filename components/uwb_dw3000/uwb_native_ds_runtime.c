#include "uwb_native_ds_runtime.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "uwb_mobile_geometry.h"
#include "uwb_native_ds_position_solver.h"
#include "wireless_telemetry_service.h"

static const char *TAG = "native_ds_runtime";

struct uwb_native_ds_runtime_context {
    struct uwb_native_ds_radio_ops backend;
    struct uwb_native_ds_position_solver solver;
    bool solver_ready;
    uint32_t position_publish_count;
    struct uwb_mobile_geometry mobile_geometry;
    bool mobile_geometry_active;
    bool mobile_geometry_all_rtk_fixed;
    int32_t mobile_geometry_fit_rms_mm;
};

static struct uwb_native_ds_runtime_context *runtime_context(void *context)
{
    return (struct uwb_native_ds_runtime_context *)context;
}

static esp_err_t runtime_send_immediate_expect_rx(
    void *context, const uint8_t *payload, size_t payload_len,
    uint32_t rx_after_tx_delay_uus, uint32_t rx_timeout_ms,
    uint64_t *tx_timestamp)
{
    struct uwb_native_ds_runtime_context *runtime = runtime_context(context);
    return runtime->backend.send_immediate_expect_rx(
        runtime->backend.context, payload, payload_len,
        rx_after_tx_delay_uus, rx_timeout_ms, tx_timestamp);
}

static esp_err_t runtime_send_delayed(
    void *context, const uint8_t *payload, size_t payload_len,
    uint64_t due_timestamp, uint64_t *programmed_tx_timestamp,
    uint64_t *actual_tx_timestamp)
{
    struct uwb_native_ds_runtime_context *runtime = runtime_context(context);
    return runtime->backend.send_delayed(
        runtime->backend.context, payload, payload_len, due_timestamp,
        programmed_tx_timestamp, actual_tx_timestamp);
}

static esp_err_t runtime_send_delayed_expect_rx(
    void *context, const uint8_t *payload, size_t payload_len,
    uint64_t due_timestamp, uint32_t rx_after_tx_delay_uus,
    uint32_t rx_timeout_ms, uint64_t *programmed_tx_timestamp,
    uint64_t *actual_tx_timestamp)
{
    struct uwb_native_ds_runtime_context *runtime = runtime_context(context);
    return runtime->backend.send_delayed_expect_rx(
        runtime->backend.context, payload, payload_len, due_timestamp,
        rx_after_tx_delay_uus, rx_timeout_ms, programmed_tx_timestamp,
        actual_tx_timestamp);
}

static esp_err_t runtime_receive(
    void *context, struct uwb_native_ds_rx_frame *frame,
    uint32_t timeout_ms)
{
    struct uwb_native_ds_runtime_context *runtime = runtime_context(context);
    return runtime->backend.receive(
        runtime->backend.context, frame, timeout_ms);
}

static uint64_t runtime_add_delay_ms(
    void *context, uint64_t timestamp, uint32_t delay_ms)
{
    struct uwb_native_ds_runtime_context *runtime = runtime_context(context);
    return runtime->backend.add_delay_ms(
        runtime->backend.context, timestamp, delay_ms);
}

static uint64_t runtime_programmed_tx_timestamp(
    void *context, uint64_t due_timestamp)
{
    struct uwb_native_ds_runtime_context *runtime = runtime_context(context);
    return runtime->backend.programmed_tx_timestamp(
        runtime->backend.context, due_timestamp);
}

static int64_t runtime_now_us(void *context)
{
    struct uwb_native_ds_runtime_context *runtime = runtime_context(context);
    return runtime->backend.now_us(runtime->backend.context);
}

static void runtime_wait_until_us(void *context, int64_t deadline_us)
{
    struct uwb_native_ds_runtime_context *runtime = runtime_context(context);
    runtime->backend.wait_until_us(runtime->backend.context, deadline_us);
}

static bool runtime_stop_requested(void *context)
{
    struct uwb_native_ds_runtime_context *runtime = runtime_context(context);
    return runtime->backend.stop_requested(runtime->backend.context);
}

static void runtime_set_ready(void *context)
{
    struct uwb_native_ds_runtime_context *runtime = runtime_context(context);
    runtime->backend.set_ready(runtime->backend.context);
}

static void runtime_capture_anchor_position(
    void *context, struct uwb_mobile_position *position)
{
    struct uwb_native_ds_runtime_context *runtime = runtime_context(context);
    runtime->backend.capture_anchor_position(
        runtime->backend.context, position);
}

static int32_t meters_to_mm(float value_m)
{
    const float value_mm = value_m * 1000.0f;
    return (int32_t)(value_mm >= 0.0f ? value_mm + 0.5f
                                     : value_mm - 0.5f);
}

static void runtime_publish_geometry(
    const struct uwb_native_ds_runtime_context *runtime)
{
    struct uwb_native_ds_position_output geometry = {0};
    if (runtime == NULL ||
        !uwb_native_ds_position_solver_geometry(
            &runtime->solver, &geometry)) {
        return;
    }
    for (size_t index = 0U; index < geometry.anchor_count; ++index) {
        (void)wireless_telemetry_service_submit_native_ds_geometry(
            runtime->solver.anchor_ids[index], geometry.anchor_count,
            geometry.geometry_version,
            meters_to_mm(geometry.anchor_x_m[index]),
            meters_to_mm(geometry.anchor_y_m[index]),
            runtime->mobile_geometry_fit_rms_mm,
            runtime->mobile_geometry_active,
            runtime->mobile_geometry_all_rtk_fixed);
    }
}

static void runtime_update_mobile_geometry(
    struct uwb_native_ds_runtime_context *runtime, uint8_t anchor_id,
    uint32_t frame_id,
    const struct uwb_mobile_position *anchor_position)
{
    if (runtime == NULL || anchor_position == NULL) {
        return;
    }
    const bool activated = uwb_mobile_geometry_ingest(
        &runtime->mobile_geometry, anchor_id, anchor_position);
    if (!runtime->mobile_geometry.active) {
        return;
    }
    float x_m[UWB_NATIVE_DS_POSITION_MAX_ANCHORS] = {0};
    float y_m[UWB_NATIVE_DS_POSITION_MAX_ANCHORS] = {0};
    for (size_t index = 0U; index < runtime->solver.anchor_count; ++index) {
        double x = 0.0;
        double y = 0.0;
        if (!uwb_mobile_geometry_latest(
                &runtime->mobile_geometry,
                runtime->solver.anchor_ids[index], &x, &y)) {
            return;
        }
        x_m[index] = (float)x;
        y_m[index] = (float)y;
    }
    const uint32_t version = uwb_mobile_geometry_version(frame_id);
    if (!uwb_native_ds_position_solver_update_geometry(
            &runtime->solver, x_m, y_m, version)) {
        return;
    }
    runtime->mobile_geometry_active = true;
    runtime->mobile_geometry_all_rtk_fixed =
        uwb_mobile_geometry_all_rtk_fixed(&runtime->mobile_geometry);
    runtime->mobile_geometry_fit_rms_mm =
        meters_to_mm((float)runtime->mobile_geometry.fit_rms_m);
    if (activated) {
        ESP_LOGI(TAG,
                 "mobile RTK geometry active fit_rms=%.3fm generation=%lu",
                 runtime->mobile_geometry.fit_rms_m,
                 (unsigned long)version);
    }
}

static void runtime_consume_report(
    void *context, bool tag_range, uint8_t initiator_id,
    uint8_t responder_id, uint32_t frame_id, double distance_m,
    const struct uwb_mobile_position *anchor_position)
{
    struct uwb_native_ds_runtime_context *runtime = runtime_context(context);
    if (runtime == NULL || !runtime->solver_ready) {
        return;
    }

    const double raw_distance_m = distance_m;

    if (tag_range) {
        runtime_update_mobile_geometry(
            runtime, responder_id, frame_id, anchor_position);
    }

    struct uwb_native_ds_position_output output = {0};
    const bool accepted = tag_range
        ? uwb_native_ds_position_solver_submit_tag_range(
              &runtime->solver, initiator_id, responder_id, frame_id,
              (float)distance_m, &output)
        : uwb_native_ds_position_solver_submit_anchor_range(
              &runtime->solver, initiator_id, responder_id, frame_id,
              (float)distance_m, &output);
    if (!accepted) {
        return;
    }

    const int32_t distance_mm = meters_to_mm((float)distance_m);
    const int32_t raw_distance_mm = meters_to_mm((float)raw_distance_m);
    if (tag_range) {
        (void)wireless_telemetry_service_submit_native_ds_tag_range(
            initiator_id, responder_id, (uint16_t)frame_id, frame_id,
            distance_mm, raw_distance_mm);
    } else {
        (void)wireless_telemetry_service_submit_native_ds_anchor_range(
            initiator_id, responder_id, (uint16_t)frame_id, frame_id,
            distance_mm, distance_mm);
    }

    if (output.geometry_updated) {
        for (size_t index = 0U; index < output.anchor_count; ++index) {
            (void)wireless_telemetry_service_submit_native_ds_geometry(
                runtime->solver.anchor_ids[index], output.anchor_count,
                output.geometry_version,
                meters_to_mm(output.anchor_x_m[index]),
                meters_to_mm(output.anchor_y_m[index]),
                meters_to_mm(output.geometry_fit_rms_m),
                runtime->mobile_geometry_active,
                runtime->mobile_geometry_all_rtk_fixed);
        }
    }
    if (output.position_valid) {
        (void)wireless_telemetry_service_submit_native_ds_position(
            output.tag_id, output.frame_id, meters_to_mm(output.x_m),
            meters_to_mm(output.y_m), meters_to_mm(output.sigma_m),
            meters_to_mm(output.rms_m), output.observation_count,
            output.anchor_count, output.geometry_version);
        runtime->position_publish_count++;
        if (runtime->mobile_geometry_active) {
            runtime_publish_geometry(runtime);
        }
        if ((runtime->position_publish_count % 32U) == 0U) {
            runtime_publish_geometry(runtime);
        }
    }
}

static bool backend_valid(const struct uwb_native_ds_radio_ops *backend)
{
    return backend != NULL &&
           backend->send_immediate_expect_rx != NULL &&
           backend->send_delayed != NULL &&
           backend->send_delayed_expect_rx != NULL &&
           backend->receive != NULL && backend->add_delay_ms != NULL &&
           backend->programmed_tx_timestamp != NULL &&
           backend->now_us != NULL && backend->wait_until_us != NULL &&
           backend->stop_requested != NULL && backend->set_ready != NULL &&
           backend->capture_anchor_position != NULL;
}

esp_err_t uwb_native_ds_runtime_run(
    const struct uwb_native_ds_config *config,
    const struct uwb_native_ds_radio_ops *radio_backend)
{
    if (config == NULL || !backend_valid(radio_backend)) {
        return ESP_ERR_INVALID_ARG;
    }

    struct uwb_native_ds_runtime_context runtime = {
        .backend = *radio_backend,
    };
    if (config->source_id == config->tag_id) {
        if (!config->fixed_geometry) {
            ESP_LOGE(TAG, "Native DS-TWR requires fixed RTK geometry");
            return ESP_ERR_INVALID_STATE;
        }
        float anchor_x_m[UWB_NATIVE_DS_POSITION_MAX_ANCHORS] = {0};
        float anchor_y_m[UWB_NATIVE_DS_POSITION_MAX_ANCHORS] = {0};
        for (size_t index = 0U; index < config->anchor_count; ++index) {
            anchor_x_m[index] = (float)config->anchor_x_mm[index] / 1000.0f;
            anchor_y_m[index] = (float)config->anchor_y_mm[index] / 1000.0f;
        }
        struct uwb_mobile_anchor fixed[UWB_NATIVE_DS_MAX_ANCHORS] = {0};
        for (size_t index = 0U; index < config->anchor_count; ++index) {
            fixed[index] = (struct uwb_mobile_anchor){
                .id = config->anchor_ids[index],
                .x_m = anchor_x_m[index],
                .y_m = anchor_y_m[index],
            };
        }
        uwb_mobile_geometry_init(
            &runtime.mobile_geometry, fixed, config->anchor_count);
        runtime.solver_ready = uwb_native_ds_position_solver_init(
            &runtime.solver, config->tag_id, config->anchor_ids,
            anchor_x_m, anchor_y_m, config->anchor_count,
            config->geometry_version);
        if (!runtime.solver_ready) {
            ESP_LOGE(TAG, "Native DS-TWR position solver init failed");
            return ESP_ERR_INVALID_STATE;
        }
        runtime_publish_geometry(&runtime);
    }

    const struct uwb_native_ds_radio_ops radio = {
        .context = &runtime,
        .send_immediate_expect_rx = runtime_send_immediate_expect_rx,
        .send_delayed = runtime_send_delayed,
        .send_delayed_expect_rx = runtime_send_delayed_expect_rx,
        .receive = runtime_receive,
        .add_delay_ms = runtime_add_delay_ms,
        .programmed_tx_timestamp = runtime_programmed_tx_timestamp,
        .now_us = runtime_now_us,
        .wait_until_us = runtime_wait_until_us,
        .stop_requested = runtime_stop_requested,
        .set_ready = runtime_set_ready,
        .capture_anchor_position = runtime_capture_anchor_position,
        .consume_report = runtime_consume_report,
    };
    return uwb_native_ds_twr_run(config, &radio);
}
