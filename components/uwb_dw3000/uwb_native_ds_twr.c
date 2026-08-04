#include "uwb_native_ds_twr.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"

static const char *TAG = "native_ds_twr";

#define NATIVE_DS_MAGIC_0 'N'
#define NATIVE_DS_MAGIC_1 'D'
#define NATIVE_DS_MAGIC_2 'S'
#define NATIVE_DS_MAGIC_3 '2'
#define NATIVE_DS_VERSION 3U
#define NATIVE_DS_HEADER_LEN 10U
#define NATIVE_DS_POLL_LEN NATIVE_DS_HEADER_LEN
#define NATIVE_DS_RESPONSE_FLAGS_OFFSET NATIVE_DS_HEADER_LEN
#define NATIVE_DS_RESPONSE_TAG_FRAME_OFFSET 11U
#define NATIVE_DS_RESPONSE_TAG_DISTANCE_OFFSET 13U
#define NATIVE_DS_RESPONSE_SURVEY_INITIATOR_OFFSET 17U
#define NATIVE_DS_RESPONSE_SURVEY_RESPONDER_OFFSET 18U
#define NATIVE_DS_RESPONSE_SURVEY_FRAME_OFFSET 19U
#define NATIVE_DS_RESPONSE_SURVEY_DISTANCE_OFFSET 21U
#define NATIVE_DS_RESPONSE_LEN 25U
#define NATIVE_DS_RESPONSE_HAS_TAG_RANGE (1U << 0U)
#define NATIVE_DS_RESPONSE_HAS_SURVEY_RANGE (1U << 1U)
#define NATIVE_DS_FINAL_POLL_TX_OFFSET NATIVE_DS_HEADER_LEN
#define NATIVE_DS_FINAL_RESPONSE_RX_OFFSET 15U
#define NATIVE_DS_FINAL_FINAL_TX_OFFSET 20U
#define NATIVE_DS_FINAL_SURVEY_PEER_OFFSET 25U
#define NATIVE_DS_FINAL_LEN 26U
#define NATIVE_DS_SURVEY_GUARD_MS 1U
#define NATIVE_DS_SURVEY_INTERVAL_FRAMES 4U
#define NATIVE_DS_DIAGNOSTIC_INTERVAL_FRAMES 5U
#define NATIVE_DS_TIMESTAMP_MASK ((1ULL << 40U) - 1ULL)
#define NATIVE_DS_TIME_UNIT_SECONDS 15.650040064102564e-12
#define NATIVE_DS_SPEED_OF_LIGHT_MPS 299702547.0

enum native_ds_frame_type {
    NATIVE_DS_POLL = 1,
    NATIVE_DS_RESPONSE = 2,
    NATIVE_DS_FINAL = 3,
};

struct native_ds_frame {
    enum native_ds_frame_type type;
    uint8_t source_id;
    uint8_t destination_id;
    uint16_t frame_id;
    uint64_t rx_timestamp;
    uint8_t payload[UWB_NATIVE_DS_MAX_FRAME_LEN];
    uint16_t payload_len;
};

struct native_ds_completed_report {
    bool valid;
    uint8_t initiator_id;
    uint8_t responder_id;
    uint16_t frame_id;
    uint32_t distance_mm;
};

struct native_ds_anchor_reports {
    struct native_ds_completed_report tag;
    struct native_ds_completed_report survey;
};

static struct uwb_native_ds_pipeline_stats s_stats;

static void put_u16(uint8_t *payload, size_t offset, uint16_t value)
{
    payload[offset] = (uint8_t)(value & 0xffU);
    payload[offset + 1U] = (uint8_t)(value >> 8U);
}

static uint16_t get_u16(const uint8_t *payload, size_t offset)
{
    return (uint16_t)((uint16_t)payload[offset] |
                      ((uint16_t)payload[offset + 1U] << 8U));
}

static void put_u32(uint8_t *payload, size_t offset, uint32_t value)
{
    payload[offset] = (uint8_t)(value & 0xffU);
    payload[offset + 1U] = (uint8_t)((value >> 8U) & 0xffU);
    payload[offset + 2U] = (uint8_t)((value >> 16U) & 0xffU);
    payload[offset + 3U] = (uint8_t)((value >> 24U) & 0xffU);
}

static uint32_t get_u32(const uint8_t *payload, size_t offset)
{
    return (uint32_t)payload[offset] |
           ((uint32_t)payload[offset + 1U] << 8U) |
           ((uint32_t)payload[offset + 2U] << 16U) |
           ((uint32_t)payload[offset + 3U] << 24U);
}

static void put_ts40(uint8_t *payload, size_t offset, uint64_t timestamp)
{
    timestamp &= NATIVE_DS_TIMESTAMP_MASK;
    for (size_t index = 0; index < 5U; ++index) {
        payload[offset + index] =
            (uint8_t)((timestamp >> (index * 8U)) & 0xffU);
    }
}

static uint64_t get_ts40(const uint8_t *payload, size_t offset)
{
    uint64_t timestamp = 0;
    for (size_t index = 0; index < 5U; ++index) {
        timestamp |= (uint64_t)payload[offset + index] << (index * 8U);
    }
    return timestamp & NATIVE_DS_TIMESTAMP_MASK;
}

static void build_frame(const struct uwb_native_ds_config *config,
                        enum native_ds_frame_type type,
                        uint8_t destination_id, uint16_t frame_id,
                        uint8_t payload[UWB_NATIVE_DS_MAX_FRAME_LEN])
{
    memset(payload, 0, UWB_NATIVE_DS_MAX_FRAME_LEN);
    payload[0] = NATIVE_DS_MAGIC_0;
    payload[1] = NATIVE_DS_MAGIC_1;
    payload[2] = NATIVE_DS_MAGIC_2;
    payload[3] = NATIVE_DS_MAGIC_3;
    payload[4] = NATIVE_DS_VERSION;
    payload[5] = (uint8_t)type;
    payload[6] = config->source_id;
    payload[7] = destination_id;
    put_u16(payload, 8U, frame_id);
}

static bool parse_frame(const struct uwb_native_ds_rx_frame *input,
                        struct native_ds_frame *output)
{
    if (input == NULL || output == NULL ||
        input->payload_len < NATIVE_DS_HEADER_LEN ||
        input->payload_len > UWB_NATIVE_DS_MAX_FRAME_LEN) {
        return false;
    }
    const uint8_t *payload = input->payload;
    if (payload[0] != NATIVE_DS_MAGIC_0 ||
        payload[1] != NATIVE_DS_MAGIC_1 ||
        payload[2] != NATIVE_DS_MAGIC_2 ||
        payload[3] != NATIVE_DS_MAGIC_3 ||
        payload[4] != NATIVE_DS_VERSION ||
        payload[5] < NATIVE_DS_POLL ||
        payload[5] > NATIVE_DS_FINAL) {
        return false;
    }
    memset(output, 0, sizeof(*output));
    output->type = (enum native_ds_frame_type)payload[5];
    output->source_id = payload[6];
    output->destination_id = payload[7];
    output->frame_id = get_u16(payload, 8U);
    output->rx_timestamp = input->rx_timestamp & NATIVE_DS_TIMESTAMP_MASK;
    output->payload_len = input->payload_len;
    memcpy(output->payload, input->payload, input->payload_len);
    return true;
}

static uint32_t remaining_ms(const struct uwb_native_ds_radio_ops *radio,
                             int64_t started_us, uint32_t timeout_ms)
{
    const int64_t elapsed_us = radio->now_us(radio->context) - started_us;
    const int64_t timeout_us = (int64_t)timeout_ms * 1000LL;
    if (elapsed_us >= timeout_us) {
        return 0;
    }
    return (uint32_t)((timeout_us - elapsed_us + 999LL) / 1000LL);
}

static esp_err_t receive_matching(
    const struct uwb_native_ds_config *config,
    const struct uwb_native_ds_radio_ops *radio,
    enum native_ds_frame_type expected_type, uint8_t expected_source,
    uint16_t expected_frame_id, uint32_t timeout_ms,
    struct native_ds_frame *frame)
{
    const int64_t started_us = radio->now_us(radio->context);
    while (!radio->stop_requested(radio->context)) {
        const uint32_t wait_ms = remaining_ms(
            radio, started_us, timeout_ms);
        if (wait_ms == 0U) {
            s_stats.rx_timeout_count++;
            return ESP_ERR_TIMEOUT;
        }
        struct uwb_native_ds_rx_frame received = {0};
        const esp_err_t err = radio->receive(
            radio->context, &received, wait_ms);
        if (err != ESP_OK) {
            if (err == ESP_ERR_TIMEOUT) {
                s_stats.rx_timeout_count++;
            }
            return err;
        }
        struct native_ds_frame parsed = {0};
        if (!parse_frame(&received, &parsed)) {
            continue;
        }
        if (parsed.type == expected_type &&
            parsed.source_id == expected_source &&
            parsed.destination_id == config->source_id &&
            parsed.frame_id == expected_frame_id) {
            *frame = parsed;
            return ESP_OK;
        }
        if (parsed.destination_id == config->source_id) {
            s_stats.invalid_frame_count++;
        }
    }
    return ESP_ERR_INVALID_STATE;
}

static uint64_t timestamp_delta(uint64_t later, uint64_t earlier)
{
    return (later - earlier) & NATIVE_DS_TIMESTAMP_MASK;
}

static bool calculate_distance(
    const struct uwb_native_ds_config *config, uint64_t poll_tx,
    uint64_t poll_rx, uint64_t response_tx, uint64_t response_rx,
    uint64_t final_tx, uint64_t final_rx, double *distance_m)
{
    const double round_a = (double)timestamp_delta(response_rx, poll_tx);
    const double round_b = (double)timestamp_delta(final_rx, response_tx);
    const double reply_a = (double)timestamp_delta(final_tx, response_rx);
    const double reply_b = (double)timestamp_delta(response_tx, poll_rx);
    const double denominator = round_a + round_b + reply_a + reply_b;
    if (denominator <= 0.0) {
        return false;
    }
    const double tof_dtu =
        ((round_a * round_b) - (reply_a * reply_b)) / denominator;
    const double result = tof_dtu * NATIVE_DS_TIME_UNIT_SECONDS *
                          NATIVE_DS_SPEED_OF_LIGHT_MPS;
    if (!isfinite(result) || result <= 0.0 ||
        result > config->maximum_distance_m) {
        return false;
    }
    *distance_m = result;
    return true;
}

static void encode_reports(uint8_t *payload,
                           const struct native_ds_anchor_reports *reports)
{
    uint8_t flags = 0U;
    if (reports != NULL && reports->tag.valid) {
        flags |= NATIVE_DS_RESPONSE_HAS_TAG_RANGE;
        put_u16(payload, NATIVE_DS_RESPONSE_TAG_FRAME_OFFSET,
                reports->tag.frame_id);
        put_u32(payload, NATIVE_DS_RESPONSE_TAG_DISTANCE_OFFSET,
                reports->tag.distance_mm);
    }
    if (reports != NULL && reports->survey.valid) {
        flags |= NATIVE_DS_RESPONSE_HAS_SURVEY_RANGE;
        payload[NATIVE_DS_RESPONSE_SURVEY_INITIATOR_OFFSET] =
            reports->survey.initiator_id;
        payload[NATIVE_DS_RESPONSE_SURVEY_RESPONDER_OFFSET] =
            reports->survey.responder_id;
        put_u16(payload, NATIVE_DS_RESPONSE_SURVEY_FRAME_OFFSET,
                reports->survey.frame_id);
        put_u32(payload, NATIVE_DS_RESPONSE_SURVEY_DISTANCE_OFFSET,
                reports->survey.distance_mm);
    }
    payload[NATIVE_DS_RESPONSE_FLAGS_OFFSET] = flags;
}

static void consume_reports(const struct uwb_native_ds_config *config,
                            const struct uwb_native_ds_radio_ops *radio,
                            const struct native_ds_frame *response,
                            uint8_t anchor_id)
{
    if (config->source_id != config->tag_id ||
        response->payload_len < NATIVE_DS_RESPONSE_LEN) {
        return;
    }
    const uint8_t flags =
        response->payload[NATIVE_DS_RESPONSE_FLAGS_OFFSET];
    if ((flags & NATIVE_DS_RESPONSE_HAS_TAG_RANGE) != 0U) {
        radio->consume_report(
            radio->context, true, config->tag_id, anchor_id,
            get_u16(response->payload,
                    NATIVE_DS_RESPONSE_TAG_FRAME_OFFSET),
            (double)get_u32(response->payload,
                            NATIVE_DS_RESPONSE_TAG_DISTANCE_OFFSET) /
                1000.0);
    }
    if ((flags & NATIVE_DS_RESPONSE_HAS_SURVEY_RANGE) != 0U) {
        radio->consume_report(
            radio->context, false,
            response->payload[NATIVE_DS_RESPONSE_SURVEY_INITIATOR_OFFSET],
            response->payload[NATIVE_DS_RESPONSE_SURVEY_RESPONDER_OFFSET],
            get_u16(response->payload,
                    NATIVE_DS_RESPONSE_SURVEY_FRAME_OFFSET),
            (double)get_u32(response->payload,
                            NATIVE_DS_RESPONSE_SURVEY_DISTANCE_OFFSET) /
                1000.0);
    }
}

static esp_err_t initiator_exchange(
    const struct uwb_native_ds_config *config,
    const struct uwb_native_ds_radio_ops *radio, uint8_t anchor_id,
    uint16_t frame_id, uint8_t survey_peer_id, uint32_t rx_timeout_ms)
{
    uint8_t payload[UWB_NATIVE_DS_MAX_FRAME_LEN] = {0};
    uint64_t poll_tx = 0;
    build_frame(config, NATIVE_DS_POLL, anchor_id, frame_id, payload);
    esp_err_t err = radio->send_immediate_expect_rx(
        radio->context, payload, NATIVE_DS_POLL_LEN,
        config->auto_rx_delay_uus, rx_timeout_ms, &poll_tx);
    if (err != ESP_OK) {
        return err;
    }
    s_stats.poll_tx_count++;

    struct native_ds_frame response = {0};
    err = receive_matching(config, radio, NATIVE_DS_RESPONSE, anchor_id,
                           frame_id, rx_timeout_ms, &response);
    if (err != ESP_OK) {
        return err;
    }
    s_stats.response_rx_count++;
    consume_reports(config, radio, &response, anchor_id);

    const uint64_t final_due = radio->add_delay_ms(
        radio->context, response.rx_timestamp, config->final_delay_ms);
    const uint64_t expected_final_tx = radio->programmed_tx_timestamp(
        radio->context, final_due);
    build_frame(config, NATIVE_DS_FINAL, anchor_id, frame_id, payload);
    put_ts40(payload, NATIVE_DS_FINAL_POLL_TX_OFFSET, poll_tx);
    put_ts40(payload, NATIVE_DS_FINAL_RESPONSE_RX_OFFSET,
             response.rx_timestamp);
    put_ts40(payload, NATIVE_DS_FINAL_FINAL_TX_OFFSET, expected_final_tx);
    payload[NATIVE_DS_FINAL_SURVEY_PEER_OFFSET] = survey_peer_id;

    uint64_t programmed_final_tx = 0;
    uint64_t actual_final_tx = 0;
    err = radio->send_delayed(
        radio->context, payload, NATIVE_DS_FINAL_LEN, final_due,
        &programmed_final_tx, &actual_final_tx);
    if (err != ESP_OK || programmed_final_tx != expected_final_tx) {
        s_stats.delayed_tx_error_count++;
        return err == ESP_OK ? ESP_ERR_INVALID_STATE : err;
    }
    if (actual_final_tx != programmed_final_tx) {
        ESP_LOGW(TAG,
                 "final TX timestamp mismatch frame=%u dst=%u "
                 "programmed=0x%010llx actual=0x%010llx delta=%llu dtu",
                 (unsigned)frame_id, (unsigned)anchor_id,
                 (unsigned long long)programmed_final_tx,
                 (unsigned long long)actual_final_tx,
                 (unsigned long long)timestamp_delta(actual_final_tx,
                                                     programmed_final_tx));
    }
    s_stats.final_tx_count++;
    return ESP_OK;
}

static bool survey_pair_for_frame(
    const struct uwb_native_ds_config *config, uint16_t frame_id,
    uint8_t *initiator_id, uint8_t *responder_id)
{
    if (config->anchor_count < 2U || initiator_id == NULL ||
        responder_id == NULL) {
        return false;
    }
    if ((frame_id % NATIVE_DS_SURVEY_INTERVAL_FRAMES) != 0U) {
        return false;
    }
    const size_t pair_count =
        ((size_t)config->anchor_count * (config->anchor_count - 1U)) / 2U;
    size_t selected =
        ((size_t)frame_id / NATIVE_DS_SURVEY_INTERVAL_FRAMES) % pair_count;
    for (size_t first = 0; first < config->anchor_count; ++first) {
        for (size_t second = first + 1U;
             second < config->anchor_count; ++second) {
            if (selected-- == 0U) {
                *initiator_id = config->anchor_ids[first];
                *responder_id = config->anchor_ids[second];
                return true;
            }
        }
    }
    return false;
}

static void run_tag(const struct uwb_native_ds_config *config,
                    const struct uwb_native_ds_radio_ops *radio)
{
    uint16_t frame_id = 0;
    radio->set_ready(radio->context);
    ESP_LOGI(TAG,
             "tag active id=%u anchors=%u slot=%lu ms gap=%lu ms "
             "resp/final=%lu/%lu ms",
             (unsigned)config->tag_id, (unsigned)config->anchor_count,
             (unsigned long)config->slot_ms,
             (unsigned long)config->round_gap_ms,
             (unsigned long)config->response_delay_ms,
             (unsigned long)config->final_delay_ms);

    while (!radio->stop_requested(radio->context)) {
        const uint16_t current_frame_id = frame_id++;
        uint8_t survey_initiator_id = 0;
        uint8_t survey_responder_id = 0;
        (void)survey_pair_for_frame(
            config, current_frame_id, &survey_initiator_id,
            &survey_responder_id);
        for (size_t index = 0;
             index < config->anchor_count &&
             !radio->stop_requested(radio->context);
             ++index) {
            const int64_t slot_started_us = radio->now_us(radio->context);
            const uint8_t anchor_id = config->anchor_ids[index];
            const uint8_t survey_peer_id =
                anchor_id == survey_initiator_id
                    ? survey_responder_id
                    : 0U;
            const esp_err_t err = initiator_exchange(
                config, radio, anchor_id, current_frame_id,
                survey_peer_id, config->rx_timeout_ms);
            if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "tag exchange anchor=%u failed: %s",
                         (unsigned)config->anchor_ids[index],
                         esp_err_to_name(err));
            }
            const int64_t elapsed_us =
                radio->now_us(radio->context) - slot_started_us;
            const int64_t slot_us = (int64_t)config->slot_ms * 1000LL;
            if (elapsed_us < slot_us) {
                radio->delay_ms(
                    radio->context,
                    (uint32_t)((slot_us - elapsed_us + 999LL) / 1000LL));
            } else {
                s_stats.slot_overrun_count++;
            }
        }
        if (config->round_gap_ms > 0U &&
            !radio->stop_requested(radio->context)) {
            radio->delay_ms(radio->context, config->round_gap_ms);
        }
    }
}

static esp_err_t anchor_exchange(
    const struct uwb_native_ds_config *config,
    const struct uwb_native_ds_radio_ops *radio,
    const struct native_ds_frame *poll,
    struct native_ds_anchor_reports *reports, uint8_t *survey_peer_id)
{
    uint8_t payload[UWB_NATIVE_DS_MAX_FRAME_LEN] = {0};
    const uint64_t response_due = radio->add_delay_ms(
        radio->context, poll->rx_timestamp, config->response_delay_ms);
    build_frame(config, NATIVE_DS_RESPONSE, poll->source_id,
                poll->frame_id, payload);
    encode_reports(payload, reports);
    uint64_t response_tx = 0;
    uint64_t response_actual = 0;
    esp_err_t err = radio->send_delayed_expect_rx(
        radio->context, payload, NATIVE_DS_RESPONSE_LEN, response_due,
        config->auto_rx_delay_uus, config->rx_timeout_ms,
        &response_tx, &response_actual);
    if (err != ESP_OK) {
        s_stats.delayed_tx_error_count++;
        return err;
    }
    if (response_actual != response_tx) {
        ESP_LOGW(TAG,
                 "response TX timestamp mismatch frame=%u dst=%u "
                 "programmed=0x%010llx actual=0x%010llx delta=%llu dtu",
                 (unsigned)poll->frame_id, (unsigned)poll->source_id,
                 (unsigned long long)response_tx,
                 (unsigned long long)response_actual,
                 (unsigned long long)timestamp_delta(response_actual,
                                                     response_tx));
    }
    s_stats.response_tx_count++;

    struct native_ds_frame final = {0};
    err = receive_matching(config, radio, NATIVE_DS_FINAL,
                           poll->source_id, poll->frame_id,
                           config->rx_timeout_ms, &final);
    if (err != ESP_OK) {
        return err;
    }
    if (final.payload_len < NATIVE_DS_FINAL_LEN) {
        s_stats.invalid_frame_count++;
        return ESP_ERR_INVALID_SIZE;
    }
    s_stats.final_rx_count++;

    double distance_m = 0.0;
    if (!calculate_distance(
            config,
            get_ts40(final.payload, NATIVE_DS_FINAL_POLL_TX_OFFSET),
            poll->rx_timestamp, response_tx,
            get_ts40(final.payload, NATIVE_DS_FINAL_RESPONSE_RX_OFFSET),
            get_ts40(final.payload, NATIVE_DS_FINAL_FINAL_TX_OFFSET),
            final.rx_timestamp, &distance_m)) {
        s_stats.rejected_range_count++;
        return ESP_ERR_INVALID_RESPONSE;
    }
    if ((poll->frame_id % NATIVE_DS_DIAGNOSTIC_INTERVAL_FRAMES) == 0U) {
        ESP_LOGI(
            TAG,
            "NATIVE_DS_RANGE_DIAG local=%u frame=%u exchange=%s src=%u "
            "poll_tx=0x%010llx poll_rx=0x%010llx "
            "response_tx=0x%010llx response_rx=0x%010llx "
            "final_tx=0x%010llx final_rx=0x%010llx distance=%.4f",
            (unsigned)config->source_id, (unsigned)poll->frame_id,
            poll->source_id == config->tag_id ? "tag" : "survey",
            (unsigned)poll->source_id,
            (unsigned long long)get_ts40(
                final.payload, NATIVE_DS_FINAL_POLL_TX_OFFSET),
            (unsigned long long)poll->rx_timestamp,
            (unsigned long long)response_tx,
            (unsigned long long)get_ts40(
                final.payload, NATIVE_DS_FINAL_RESPONSE_RX_OFFSET),
            (unsigned long long)get_ts40(
                final.payload, NATIVE_DS_FINAL_FINAL_TX_OFFSET),
            (unsigned long long)final.rx_timestamp, distance_m);
    }
    s_stats.completed_range_count++;
    s_stats.last_distance_mm = (int32_t)(distance_m * 1000.0 + 0.5);
    radio->publish_range(radio->context, poll->source_id, config->source_id,
                         poll->frame_id, distance_m);
    struct native_ds_completed_report *completed =
        poll->source_id == config->tag_id ? &reports->tag : &reports->survey;
    completed->valid = true;
    completed->initiator_id = poll->source_id;
    completed->responder_id = config->source_id;
    completed->frame_id = poll->frame_id;
    completed->distance_mm = (uint32_t)(distance_m * 1000.0 + 0.5);
    if (survey_peer_id != NULL) {
        *survey_peer_id = poll->source_id == config->tag_id
            ? final.payload[NATIVE_DS_FINAL_SURVEY_PEER_OFFSET]
            : 0U;
    }
    return ESP_OK;
}

static bool configured_anchor(
    const struct uwb_native_ds_config *config, uint8_t source_id)
{
    for (size_t index = 0; index < config->anchor_count; ++index) {
        if (config->anchor_ids[index] == source_id) {
            return true;
        }
    }
    return false;
}

static void run_anchor(const struct uwb_native_ds_config *config,
                       const struct uwb_native_ds_radio_ops *radio)
{
    struct native_ds_anchor_reports reports = {0};
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
        if (rx_err != ESP_OK) {
            if (!radio->stop_requested(radio->context)) {
                ESP_LOGW(TAG, "anchor receive failed: %s",
                         esp_err_to_name(rx_err));
            }
            continue;
        }
        struct native_ds_frame poll = {0};
        if (!parse_frame(&received, &poll)) {
            continue;
        }
        if (poll.destination_id != config->source_id) {
            continue;
        }
        if (poll.type != NATIVE_DS_POLL ||
            (poll.source_id != config->tag_id &&
             !configured_anchor(config, poll.source_id))) {
            s_stats.invalid_frame_count++;
            continue;
        }
        s_stats.poll_rx_count++;
        uint8_t survey_peer_id = 0;
        const esp_err_t err = anchor_exchange(
            config, radio, &poll, &reports, &survey_peer_id);
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "anchor exchange frame=%u failed: %s",
                     (unsigned)poll.frame_id, esp_err_to_name(err));
        }
        if (err == ESP_OK && survey_peer_id != 0U &&
            survey_peer_id != config->source_id &&
            configured_anchor(config, survey_peer_id) &&
            !radio->stop_requested(radio->context)) {
            radio->delay_ms(radio->context, NATIVE_DS_SURVEY_GUARD_MS);
            const esp_err_t survey_err = initiator_exchange(
                config, radio, survey_peer_id, poll.frame_id, 0U,
                config->rx_timeout_ms);
            if (survey_err != ESP_OK && survey_err != ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG,
                         "geometry exchange peer=%u frame=%u failed: %s",
                         (unsigned)survey_peer_id,
                         (unsigned)poll.frame_id,
                         esp_err_to_name(survey_err));
            }
        }
    }
}

static bool config_valid(const struct uwb_native_ds_config *config,
                         const struct uwb_native_ds_radio_ops *radio)
{
    if (config == NULL || radio == NULL || config->source_id == 0U ||
        config->tag_id == 0U || config->anchor_count == 0U ||
        config->anchor_count > UWB_NATIVE_DS_MAX_ANCHORS ||
        config->slot_ms == 0U ||
        config->rx_timeout_ms == 0U || config->rx_slice_ms == 0U ||
        config->response_delay_ms == 0U || config->final_delay_ms == 0U ||
        config->maximum_distance_m <= 0.0 ||
        radio->send_immediate_expect_rx == NULL ||
        radio->send_delayed == NULL ||
        radio->send_delayed_expect_rx == NULL || radio->receive == NULL ||
        radio->add_delay_ms == NULL ||
        radio->programmed_tx_timestamp == NULL || radio->now_us == NULL ||
        radio->delay_ms == NULL || radio->stop_requested == NULL ||
        radio->set_ready == NULL || radio->publish_range == NULL) {
        return false;
    }
    if (radio->consume_report == NULL) {
        return false;
    }
    for (size_t index = 0; index < config->anchor_count; ++index) {
        if (config->anchor_ids[index] == 0U ||
            config->anchor_ids[index] == config->tag_id) {
            return false;
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
        run_tag(config, radio);
        return ESP_OK;
    }
    for (size_t index = 0; index < config->anchor_count; ++index) {
        if (config->anchor_ids[index] == config->source_id) {
            run_anchor(config, radio);
            return ESP_OK;
        }
    }
    radio->set_ready(radio->context);
    ESP_LOGW(TAG, "idle source id=%u is not the tag or a configured anchor",
             (unsigned)config->source_id);
    while (!radio->stop_requested(radio->context)) {
        radio->delay_ms(radio->context, 1000U);
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
