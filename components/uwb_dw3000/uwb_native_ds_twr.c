#include "uwb_native_ds_twr.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "uwb_native_ds_protocol.h"

static const char *TAG = "native_ds_twr";

#define NATIVE_DS_SLOT_GUARD_MS 1U
#define NATIVE_DS_DIAGNOSTIC_INTERVAL_FRAMES 16U

struct native_ds_received_packet {
    struct uwb_native_ds_packet packet;
    uint64_t rx_timestamp;
};

static struct uwb_native_ds_pipeline_stats s_stats;

static uint32_t remaining_ms(const struct uwb_native_ds_radio_ops *radio,
                             int64_t started_us, uint32_t timeout_ms)
{
    const int64_t elapsed_us = radio->now_us(radio->context) - started_us;
    const int64_t timeout_us = (int64_t)timeout_ms * 1000LL;
    if (elapsed_us >= timeout_us) {
        return 0U;
    }
    return (uint32_t)((timeout_us - elapsed_us + 999LL) / 1000LL);
}

static esp_err_t receive_matching(
    const struct uwb_native_ds_config *config,
    const struct uwb_native_ds_radio_ops *radio,
    enum uwb_native_ds_message_type expected_type, uint8_t expected_source,
    uint32_t expected_session_id, uint32_t expected_frame_id,
    uint32_t timeout_ms, struct native_ds_received_packet *received_packet)
{
    const int64_t started_us = radio->now_us(radio->context);
    while (!radio->stop_requested(radio->context)) {
        const uint32_t wait_ms = remaining_ms(radio, started_us, timeout_ms);
        if (wait_ms == 0U) {
            s_stats.rx_timeout_count++;
            return ESP_ERR_TIMEOUT;
        }
        struct uwb_native_ds_rx_frame received = {0};
        const esp_err_t err =
            radio->receive(radio->context, &received, wait_ms);
        if (err != ESP_OK) {
            if (err == ESP_ERR_TIMEOUT) {
                s_stats.rx_timeout_count++;
            } else if (err == ESP_ERR_INVALID_RESPONSE) {
                /*
                 * A DW3000 PHY error can precede the expected frame in the
                 * same receive window.  The driver has already stopped RX,
                 * cleared the error status and marked the receiver for
                 * re-arm.  Keep the original absolute timeout and let the
                 * next receive call re-arm instead of discarding the whole
                 * exchange immediately.
                 */
                s_stats.recovered_rx_error_count++;
                continue;
            }
            return err;
        }
        struct uwb_native_ds_packet packet = {0};
        const enum uwb_native_ds_decode_result decode_result =
            uwb_native_ds_protocol_decode_ex(
                received.payload, received.payload_len, &packet);
        if (decode_result != UWB_NATIVE_DS_DECODE_OK) {
            if (decode_result == UWB_NATIVE_DS_DECODE_CRC_ERROR) {
                s_stats.crc_error_count++;
            }
            continue;
        }
        if (packet.type == expected_type &&
            packet.source_id == expected_source &&
            packet.destination_id == config->source_id &&
            packet.session_id == expected_session_id &&
            packet.frame_id == expected_frame_id) {
            received_packet->packet = packet;
            received_packet->rx_timestamp =
                received.rx_timestamp & UWB_NATIVE_DS_TIMESTAMP_MASK;
            return ESP_OK;
        }
        if (packet.destination_id == config->source_id) {
            s_stats.invalid_frame_count++;
        }
    }
    return ESP_ERR_INVALID_STATE;
}

static bool encode_packet(const struct uwb_native_ds_packet *packet,
                          uint8_t payload[UWB_NATIVE_DS_MAX_FRAME_LEN],
                          size_t *payload_len)
{
    return uwb_native_ds_protocol_encode(
        packet, payload, UWB_NATIVE_DS_MAX_FRAME_LEN, payload_len);
}

static bool delayed_timestamp_valid(uint64_t expected, uint64_t programmed,
                                    uint64_t actual)
{
    return (expected & UWB_NATIVE_DS_TIMESTAMP_MASK) ==
               (programmed & UWB_NATIVE_DS_TIMESTAMP_MASK) &&
           (programmed & UWB_NATIVE_DS_TIMESTAMP_MASK) ==
               (actual & UWB_NATIVE_DS_TIMESTAMP_MASK);
}

static esp_err_t tag_exchange(
    const struct uwb_native_ds_config *config,
    const struct uwb_native_ds_radio_ops *radio, size_t anchor_index,
    uint32_t session_id, uint32_t frame_id)
{
    const uint8_t anchor_id = config->anchor_ids[anchor_index];
    struct uwb_native_ds_tag_anchor_stats *anchor_stats =
        &s_stats.tag_anchors[anchor_index];
    anchor_stats->attempt_count++;

    uint8_t payload[UWB_NATIVE_DS_MAX_FRAME_LEN] = {0};
    size_t payload_len = 0U;
    struct uwb_native_ds_packet packet = {
        .type = UWB_NATIVE_DS_MESSAGE_POLL,
        .exchange_kind = UWB_NATIVE_DS_EXCHANGE_TAG_RANGE,
        .source_id = config->tag_id,
        .destination_id = anchor_id,
        .session_id = session_id,
        .frame_id = frame_id,
    };
    if (!encode_packet(&packet, payload, &payload_len)) {
        return ESP_ERR_INVALID_STATE;
    }

    uint64_t poll_tx = 0U;
    esp_err_t err = radio->send_immediate_expect_rx(
        radio->context, payload, payload_len, config->auto_rx_delay_uus,
        config->rx_timeout_ms, &poll_tx);
    if (err != ESP_OK) {
        s_stats.poll_tx_error_count++;
        anchor_stats->poll_tx_error_count++;
        return err;
    }
    s_stats.poll_tx_count++;

    struct native_ds_received_packet response = {0};
    err = receive_matching(config, radio, UWB_NATIVE_DS_MESSAGE_RESPONSE,
                           anchor_id, session_id, frame_id,
                           config->rx_timeout_ms, &response);
    if (err != ESP_OK) {
        if (err == ESP_ERR_TIMEOUT) {
            s_stats.response_timeout_count++;
            anchor_stats->response_timeout_count++;
        } else if (!radio->stop_requested(radio->context)) {
            s_stats.response_rx_error_count++;
            anchor_stats->response_rx_error_count++;
        }
        return err;
    }
    s_stats.response_rx_count++;

    const uint64_t final_due = radio->add_delay_ms(
        radio->context, response.rx_timestamp, config->final_delay_ms);
    const uint64_t expected_final_tx = radio->programmed_tx_timestamp(
        radio->context, final_due);
    packet.type = UWB_NATIVE_DS_MESSAGE_FINAL;
    packet.poll_tx_timestamp = poll_tx;
    packet.response_rx_timestamp = response.rx_timestamp;
    packet.final_tx_timestamp = expected_final_tx;
    if (!encode_packet(&packet, payload, &payload_len)) {
        return ESP_ERR_INVALID_STATE;
    }

    uint64_t programmed_final_tx = 0U;
    uint64_t actual_final_tx = 0U;
    err = radio->send_delayed_expect_rx(
        radio->context, payload, payload_len, final_due,
        config->auto_rx_delay_uus, config->rx_timeout_ms,
        &programmed_final_tx, &actual_final_tx);
    if (err != ESP_OK ||
        !delayed_timestamp_valid(expected_final_tx, programmed_final_tx,
                                 actual_final_tx)) {
        s_stats.delayed_tx_error_count++;
        s_stats.final_tx_error_count++;
        anchor_stats->final_tx_error_count++;
        return err == ESP_OK ? ESP_ERR_INVALID_STATE : err;
    }
    s_stats.final_tx_count++;

    struct native_ds_received_packet result = {0};
    err = receive_matching(config, radio, UWB_NATIVE_DS_MESSAGE_RESULT,
                           anchor_id, session_id, frame_id,
                           config->rx_timeout_ms, &result);
    if (err != ESP_OK) {
        if (err == ESP_ERR_TIMEOUT) {
            s_stats.result_timeout_count++;
            anchor_stats->result_timeout_count++;
        } else if (!radio->stop_requested(radio->context)) {
            s_stats.result_rx_error_count++;
            anchor_stats->result_rx_error_count++;
        }
        return err;
    }
    s_stats.result_rx_count++;
    const double distance_m = (double)result.packet.distance_mm / 1000.0;
    if (!isfinite(distance_m) || distance_m <= 0.0 ||
        distance_m > config->maximum_distance_m) {
        s_stats.rejected_range_count++;
        return ESP_ERR_INVALID_RESPONSE;
    }
    s_stats.completed_range_count++;
    anchor_stats->completed_range_count++;
    s_stats.last_distance_mm = (int32_t)result.packet.distance_mm;
    radio->consume_report(radio->context, true, config->tag_id, anchor_id,
                          frame_id, distance_m);
    return ESP_OK;
}

static void run_tag(const struct uwb_native_ds_config *config,
                    const struct uwb_native_ds_radio_ops *radio)
{
    uint32_t session_id = (uint32_t)radio->now_us(radio->context);
    if (session_id == 0U) {
        session_id = 1U;
    }
    uint32_t frame_id = 1U;
    radio->set_ready(radio->context);
    ESP_LOGI(TAG,
             "tag active id=%u session=%lu anchors=%u slot=%lu ms "
             "resp/final/result=%lu/%lu/%lu ms",
             (unsigned)config->tag_id, (unsigned long)session_id,
             (unsigned)config->anchor_count, (unsigned long)config->slot_ms,
             (unsigned long)config->response_delay_ms,
             (unsigned long)config->final_delay_ms,
             (unsigned long)config->response_delay_ms);

    while (!radio->stop_requested(radio->context)) {
        const uint32_t current_frame_id = frame_id++;
        uint32_t completed_anchor_mask = 0U;
        if (frame_id == 0U) {
            frame_id = 1U;
        }
        for (size_t index = 0U;
             index < config->anchor_count &&
             !radio->stop_requested(radio->context);
             ++index) {
            const int64_t slot_started_us = radio->now_us(radio->context);
            const esp_err_t err = tag_exchange(
                config, radio, index, session_id, current_frame_id);
            if (err == ESP_OK) {
                completed_anchor_mask |= 1UL << index;
            }
            if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "tag exchange anchor=%u frame=%lu failed: %s",
                         (unsigned)config->anchor_ids[index],
                         (unsigned long)current_frame_id,
                         esp_err_to_name(err));
            }
            const int64_t elapsed_us =
                radio->now_us(radio->context) - slot_started_us;
            const int64_t slot_us = (int64_t)config->slot_ms * 1000LL;
            if (elapsed_us < slot_us) {
                radio->wait_until_us(radio->context,
                                     slot_started_us + slot_us);
            } else {
                s_stats.slot_overrun_count++;
            }
        }
        const uint32_t expected_anchor_mask =
            (1UL << config->anchor_count) - 1UL;
        s_stats.last_frame_missing_anchor_mask =
            expected_anchor_mask & ~completed_anchor_mask;
        if (completed_anchor_mask == expected_anchor_mask) {
            s_stats.complete_frame_count++;
        } else {
            s_stats.incomplete_frame_count++;
        }
        if (config->round_gap_ms > 0U &&
            !radio->stop_requested(radio->context)) {
            const int64_t gap_started_us =
                radio->now_us(radio->context);
            radio->wait_until_us(
                radio->context,
                gap_started_us + (int64_t)config->round_gap_ms * 1000LL);
        }
    }
}

static esp_err_t anchor_exchange(
    const struct uwb_native_ds_config *config,
    const struct uwb_native_ds_radio_ops *radio,
    const struct native_ds_received_packet *poll)
{
    uint8_t payload[UWB_NATIVE_DS_MAX_FRAME_LEN] = {0};
    size_t payload_len = 0U;
    struct uwb_native_ds_packet packet = {
        .type = UWB_NATIVE_DS_MESSAGE_RESPONSE,
        .exchange_kind = UWB_NATIVE_DS_EXCHANGE_TAG_RANGE,
        .source_id = config->source_id,
        .destination_id = config->tag_id,
        .session_id = poll->packet.session_id,
        .frame_id = poll->packet.frame_id,
    };
    if (!encode_packet(&packet, payload, &payload_len)) {
        return ESP_ERR_INVALID_STATE;
    }

    const uint64_t response_due = radio->add_delay_ms(
        radio->context, poll->rx_timestamp, config->response_delay_ms);
    const uint64_t expected_response_tx = radio->programmed_tx_timestamp(
        radio->context, response_due);
    uint64_t programmed_response_tx = 0U;
    uint64_t actual_response_tx = 0U;
    esp_err_t err = radio->send_delayed_expect_rx(
        radio->context, payload, payload_len, response_due,
        config->auto_rx_delay_uus, config->rx_timeout_ms,
        &programmed_response_tx, &actual_response_tx);
    if (err != ESP_OK ||
        !delayed_timestamp_valid(expected_response_tx,
                                 programmed_response_tx,
                                 actual_response_tx)) {
        s_stats.delayed_tx_error_count++;
        s_stats.response_tx_error_count++;
        return err == ESP_OK ? ESP_ERR_INVALID_STATE : err;
    }
    s_stats.response_tx_count++;

    struct native_ds_received_packet final = {0};
    err = receive_matching(config, radio, UWB_NATIVE_DS_MESSAGE_FINAL,
                           config->tag_id, poll->packet.session_id,
                           poll->packet.frame_id, config->rx_timeout_ms,
                           &final);
    if (err != ESP_OK) {
        if (err == ESP_ERR_TIMEOUT) {
            s_stats.final_timeout_count++;
        } else if (!radio->stop_requested(radio->context)) {
            s_stats.final_rx_error_count++;
        }
        return err;
    }
    s_stats.final_rx_count++;

    double distance_m = 0.0;
    if (!uwb_native_ds_protocol_calculate_distance(
            final.packet.poll_tx_timestamp, poll->rx_timestamp,
            programmed_response_tx, final.packet.response_rx_timestamp,
            final.packet.final_tx_timestamp, final.rx_timestamp,
            &distance_m) ||
        distance_m > config->maximum_distance_m) {
        s_stats.rejected_range_count++;
        return ESP_ERR_INVALID_RESPONSE;
    }
    const uint32_t distance_mm = (uint32_t)(distance_m * 1000.0 + 0.5);

    packet.type = UWB_NATIVE_DS_MESSAGE_RESULT;
    packet.distance_mm = distance_mm;
    if (!encode_packet(&packet, payload, &payload_len)) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint64_t result_due = radio->add_delay_ms(
        radio->context, final.rx_timestamp, config->response_delay_ms);
    const uint64_t expected_result_tx = radio->programmed_tx_timestamp(
        radio->context, result_due);
    uint64_t programmed_result_tx = 0U;
    uint64_t actual_result_tx = 0U;
    err = radio->send_delayed(
        radio->context, payload, payload_len, result_due,
        &programmed_result_tx, &actual_result_tx);
    if (err != ESP_OK) {
        s_stats.delayed_tx_error_count++;
        s_stats.result_tx_error_count++;
        return err;
    }
    if (!delayed_timestamp_valid(expected_result_tx, programmed_result_tx,
                                 actual_result_tx)) {
        /*
         * RESULT carries the already computed distance and its TX timestamp is
         * not used by the ADS-TWR equation.  send_delayed() returning ESP_OK
         * proves that TX completed, so keep the valid range even if the DW3000
         * reports a quantized timestamp that differs from the programmed one.
         * RESPONSE and FINAL retain the strict check above because their
         * timestamps are inputs to the distance calculation.
         */
        ESP_LOGW(TAG,
                 "RESULT TX timestamp mismatch accepted expected=0x%010llx "
                 "programmed=0x%010llx actual=0x%010llx",
                 (unsigned long long)expected_result_tx,
                 (unsigned long long)programmed_result_tx,
                 (unsigned long long)actual_result_tx);
    }
    s_stats.result_tx_count++;
    s_stats.completed_range_count++;
    s_stats.last_distance_mm = (int32_t)distance_mm;

    if ((poll->packet.frame_id %
         NATIVE_DS_DIAGNOSTIC_INTERVAL_FRAMES) == 0U) {
        ESP_LOGI(TAG,
                 "NATIVE_DS_RANGE_DIAG local=%u session=%lu frame=%lu "
                 "poll_tx=0x%010llx poll_rx=0x%010llx "
                 "response_tx=0x%010llx response_rx=0x%010llx "
                 "final_tx=0x%010llx final_rx=0x%010llx distance=%.4f",
                 (unsigned)config->source_id,
                 (unsigned long)poll->packet.session_id,
                 (unsigned long)poll->packet.frame_id,
                 (unsigned long long)final.packet.poll_tx_timestamp,
                 (unsigned long long)poll->rx_timestamp,
                 (unsigned long long)programmed_response_tx,
                 (unsigned long long)final.packet.response_rx_timestamp,
                 (unsigned long long)final.packet.final_tx_timestamp,
                 (unsigned long long)final.rx_timestamp, distance_m);
    }
    return ESP_OK;
}

static void run_anchor(const struct uwb_native_ds_config *config,
                       const struct uwb_native_ds_radio_ops *radio)
{
    radio->set_ready(radio->context);
    ESP_LOGI(TAG, "anchor active id=%u tag=%u", (unsigned)config->source_id,
             (unsigned)config->tag_id);
    while (!radio->stop_requested(radio->context)) {
        struct uwb_native_ds_rx_frame received = {0};
        const esp_err_t rx_err = radio->receive(
            radio->context, &received, config->rx_slice_ms);
        if (rx_err == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (rx_err == ESP_ERR_INVALID_RESPONSE) {
            continue;
        }
        if (rx_err != ESP_OK) {
            if (!radio->stop_requested(radio->context)) {
                ESP_LOGW(TAG, "anchor receive failed: %s",
                         esp_err_to_name(rx_err));
            }
            continue;
        }
        struct native_ds_received_packet poll = {
            .rx_timestamp =
                received.rx_timestamp & UWB_NATIVE_DS_TIMESTAMP_MASK,
        };
        const enum uwb_native_ds_decode_result decode_result =
            uwb_native_ds_protocol_decode_ex(
                received.payload, received.payload_len, &poll.packet);
        if (decode_result != UWB_NATIVE_DS_DECODE_OK) {
            if (decode_result == UWB_NATIVE_DS_DECODE_CRC_ERROR) {
                s_stats.crc_error_count++;
            }
            continue;
        }
        if (poll.packet.type != UWB_NATIVE_DS_MESSAGE_POLL ||
            poll.packet.exchange_kind != UWB_NATIVE_DS_EXCHANGE_TAG_RANGE ||
            poll.packet.source_id != config->tag_id ||
            poll.packet.destination_id != config->source_id) {
            if (poll.packet.destination_id == config->source_id) {
                s_stats.invalid_frame_count++;
            }
            continue;
        }
        s_stats.poll_rx_count++;
        const esp_err_t err = anchor_exchange(config, radio, &poll);
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "anchor exchange frame=%lu failed: %s",
                     (unsigned long)poll.packet.frame_id,
                     esp_err_to_name(err));
        }
    }
}

static bool config_valid(const struct uwb_native_ds_config *config,
                         const struct uwb_native_ds_radio_ops *radio)
{
    if (config == NULL || radio == NULL || config->source_id == 0U ||
        config->tag_id == 0U || config->anchor_count < 3U ||
        config->anchor_count > UWB_NATIVE_DS_MAX_ANCHORS ||
        config->slot_ms == 0U || config->rx_timeout_ms == 0U ||
        config->rx_slice_ms == 0U || config->response_delay_ms == 0U ||
        config->final_delay_ms == 0U || config->maximum_distance_m <= 0.0 ||
        radio->send_immediate_expect_rx == NULL ||
        radio->send_delayed == NULL ||
        radio->send_delayed_expect_rx == NULL || radio->receive == NULL ||
        radio->add_delay_ms == NULL ||
        radio->programmed_tx_timestamp == NULL || radio->now_us == NULL ||
        radio->wait_until_us == NULL || radio->stop_requested == NULL ||
        radio->set_ready == NULL || radio->consume_report == NULL) {
        return false;
    }
    const uint64_t minimum_slot_ms =
        2ULL * config->response_delay_ms + config->final_delay_ms +
        NATIVE_DS_SLOT_GUARD_MS;
    if ((uint64_t)config->slot_ms < minimum_slot_ms) {
        return false;
    }
    for (size_t first = 0U; first < config->anchor_count; ++first) {
        if (config->anchor_ids[first] == 0U ||
            config->anchor_ids[first] == config->tag_id) {
            return false;
        }
        for (size_t second = first + 1U; second < config->anchor_count;
             ++second) {
            if (config->anchor_ids[first] == config->anchor_ids[second]) {
                return false;
            }
        }
    }
    return true;
}

esp_err_t uwb_native_ds_twr_run(const struct uwb_native_ds_config *config,
                                const struct uwb_native_ds_radio_ops *radio)
{
    if (!config_valid(config, radio)) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(&s_stats, 0, sizeof(s_stats));
    if (config->source_id == config->tag_id) {
        s_stats.tag_anchor_count = config->anchor_count;
        for (size_t index = 0U; index < config->anchor_count; ++index) {
            s_stats.tag_anchors[index].anchor_id =
                config->anchor_ids[index];
        }
        run_tag(config, radio);
        return ESP_OK;
    }
    for (size_t index = 0U; index < config->anchor_count; ++index) {
        if (config->anchor_ids[index] == config->source_id) {
            run_anchor(config, radio);
            return ESP_OK;
        }
    }
    radio->set_ready(radio->context);
    ESP_LOGW(TAG, "idle source id=%u is not the tag or a configured anchor",
             (unsigned)config->source_id);
    while (!radio->stop_requested(radio->context)) {
        const int64_t idle_started_us = radio->now_us(radio->context);
        radio->wait_until_us(radio->context,
                             idle_started_us + 1000000LL);
    }
    return ESP_OK;
}

void uwb_native_ds_twr_get_stats(
    struct uwb_native_ds_pipeline_stats *stats)
{
    if (stats != NULL) {
        memcpy(stats, &s_stats, sizeof(*stats));
    }
}
