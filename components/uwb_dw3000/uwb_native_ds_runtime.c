#include "uwb_native_ds_runtime.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "uwb_native_ds_position_solver.h"
#include "wireless_telemetry_service.h"

static const char *TAG = "native_ds_runtime";

struct uwb_native_ds_runtime_context {
    struct uwb_native_ds_radio_ops backend;
    struct uwb_native_ds_position_solver solver;
    bool solver_ready;
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

static void runtime_delay_ms(void *context, uint32_t delay_ms)
{
    struct uwb_native_ds_runtime_context *runtime = runtime_context(context);
    runtime->backend.delay_ms(runtime->backend.context, delay_ms);
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

static void runtime_publish_range(
    void *context, uint8_t initiator_id, uint8_t responder_id,
    uint16_t frame_id, double distance_m)
{
    struct uwb_native_ds_runtime_context *runtime = runtime_context(context);
    runtime->backend.publish_range(
        runtime->backend.context, initiator_id, responder_id, frame_id,
        distance_m);
}

static int32_t meters_to_mm(float value_m)
{
    const float value_mm = value_m * 1000.0f;
    return (int32_t)(value_mm >= 0.0f ? value_mm + 0.5f
                                     : value_mm - 0.5f);
}

static void runtime_consume_report(
    void *context, bool tag_range, uint8_t initiator_id,
    uint8_t responder_id, uint16_t frame_id, double distance_m)
{
    struct uwb_native_ds_runtime_context *runtime = runtime_context(context);
    if (runtime == NULL || !runtime->solver_ready) {
        return;
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
    if (tag_range) {
        (void)wireless_telemetry_service_submit_native_ds_tag_range(
            initiator_id, responder_id, frame_id, (uint32_t)frame_id,
            distance_mm, distance_mm);
    } else {
        (void)wireless_telemetry_service_submit_native_ds_anchor_range(
            initiator_id, responder_id, frame_id, (uint32_t)frame_id,
            distance_mm, distance_mm);
    }

    if (output.geometry_updated) {
        for (size_t index = 0U; index < output.anchor_count; ++index) {
            (void)wireless_telemetry_service_submit_native_ds_geometry(
                runtime->solver.anchor_ids[index], output.anchor_count,
                output.geometry_version,
                meters_to_mm(output.anchor_x_m[index]),
                meters_to_mm(output.anchor_y_m[index]),
                meters_to_mm(output.geometry_fit_rms_m));
        }
    }
    if (output.position_valid) {
        (void)wireless_telemetry_service_submit_native_ds_position(
            output.tag_id, output.frame_id, meters_to_mm(output.x_m),
            meters_to_mm(output.y_m), meters_to_mm(output.sigma_m),
            meters_to_mm(output.rms_m), output.observation_count,
            output.anchor_count, output.geometry_version);
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
           backend->now_us != NULL && backend->delay_ms != NULL &&
           backend->stop_requested != NULL && backend->set_ready != NULL &&
           backend->publish_range != NULL;
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
        runtime.solver_ready = uwb_native_ds_position_solver_init(
            &runtime.solver, config->tag_id, config->anchor_ids,
            config->anchor_count);
        if (!runtime.solver_ready) {
            ESP_LOGE(TAG, "Native DS-TWR position solver init failed");
            return ESP_ERR_INVALID_STATE;
        }
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
        .delay_ms = runtime_delay_ms,
        .stop_requested = runtime_stop_requested,
        .set_ready = runtime_set_ready,
        .publish_range = runtime_publish_range,
        .consume_report = runtime_consume_report,
    };
    return uwb_native_ds_twr_run(config, &radio);
}
