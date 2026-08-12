#include "uwb_native_ds_twr.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "uwb_native_ds_protocol.h"

struct mock_radio {
    int64_t now_us;
    uint8_t scenario;
    uint8_t stop_after_slots;
    uint8_t attempt_index;
    uint8_t receive_index;
    uint8_t slot_delay_count;
    bool injected_phy_error;
    uint32_t report_count;
    uint8_t attempted_anchor_ids[16];
    uint8_t reported_anchor_ids[16];
    uint32_t reported_frame_ids[16];
    struct uwb_native_ds_packet poll;
};

enum {
    MOCK_BASELINE = 0,
    MOCK_SINGLE_MISS_RECOVERY,
    MOCK_ROTATING_ORDER,
};

static esp_err_t mock_send_poll(void *context, const uint8_t *payload,
                                size_t payload_len,
                                uint32_t rx_after_tx_delay_uus,
                                uint32_t rx_timeout_ms,
                                uint64_t *tx_timestamp)
{
    struct mock_radio *mock = context;
    (void)rx_after_tx_delay_uus;
    (void)rx_timeout_ms;
    assert(uwb_native_ds_protocol_decode(payload, payload_len,
                                         &mock->poll));
    mock->attempt_index = mock->slot_delay_count;
    assert(mock->attempt_index < sizeof(mock->attempted_anchor_ids));
    mock->attempted_anchor_ids[mock->attempt_index] =
        mock->poll.destination_id;
    mock->receive_index = 0U;
    *tx_timestamp = 1000U + mock->attempt_index * 100U;
    return ESP_OK;
}

static esp_err_t mock_send_delayed(void *context, const uint8_t *payload,
                                   size_t payload_len, uint64_t due_timestamp,
                                   uint64_t *programmed_tx_timestamp,
                                   uint64_t *actual_tx_timestamp)
{
    (void)context;
    (void)payload;
    (void)payload_len;
    *programmed_tx_timestamp = due_timestamp;
    *actual_tx_timestamp = due_timestamp;
    return ESP_OK;
}

static esp_err_t mock_send_delayed_expect_rx(
    void *context, const uint8_t *payload, size_t payload_len,
    uint64_t due_timestamp, uint32_t rx_after_tx_delay_uus,
    uint32_t rx_timeout_ms, uint64_t *programmed_tx_timestamp,
    uint64_t *actual_tx_timestamp)
{
    (void)context;
    (void)payload;
    (void)payload_len;
    (void)rx_after_tx_delay_uus;
    (void)rx_timeout_ms;
    *programmed_tx_timestamp = due_timestamp;
    *actual_tx_timestamp = due_timestamp;
    return ESP_OK;
}

static esp_err_t mock_receive(void *context,
                              struct uwb_native_ds_rx_frame *frame,
                              uint32_t timeout_ms)
{
    struct mock_radio *mock = context;
    (void)timeout_ms;
    if (mock->scenario == MOCK_BASELINE &&
        mock->attempt_index == 2U && mock->receive_index == 0U &&
        !mock->injected_phy_error) {
        mock->injected_phy_error = true;
        return ESP_ERR_INVALID_RESPONSE;
    }
    if ((mock->scenario == MOCK_BASELINE &&
         (mock->attempt_index == 0U ||
          (mock->attempt_index == 1U && mock->receive_index == 1U))) ||
        (mock->scenario == MOCK_SINGLE_MISS_RECOVERY &&
         mock->attempt_index == 1U && mock->receive_index == 0U)) {
        mock->receive_index++;
        return ESP_ERR_TIMEOUT;
    }

    struct uwb_native_ds_packet packet = {
        .type = mock->receive_index == 0U
                    ? UWB_NATIVE_DS_MESSAGE_RESPONSE
                    : UWB_NATIVE_DS_MESSAGE_RESULT,
        .exchange_kind = UWB_NATIVE_DS_EXCHANGE_TAG_RANGE,
        .source_id = mock->poll.destination_id,
        .destination_id = mock->poll.source_id,
        .session_id = mock->poll.session_id,
        .frame_id = mock->poll.frame_id,
        .distance_mm = 1234U,
    };
    size_t encoded_len = 0U;
    assert(uwb_native_ds_protocol_encode(
        &packet, frame->payload, sizeof(frame->payload), &encoded_len));
    frame->payload_len = (uint16_t)encoded_len;
    frame->rx_timestamp = 2000U + mock->receive_index * 100U;
    mock->receive_index++;
    return ESP_OK;
}

static uint64_t mock_add_delay_ms(void *context, uint64_t timestamp,
                                  uint32_t delay_ms)
{
    (void)context;
    return (timestamp + (uint64_t)delay_ms * 1000U) &
           UWB_NATIVE_DS_TIMESTAMP_MASK;
}

static uint64_t mock_programmed_tx(void *context, uint64_t due_timestamp)
{
    (void)context;
    return due_timestamp & UWB_NATIVE_DS_TIMESTAMP_MASK;
}

static int64_t mock_now_us(void *context)
{
    return ((struct mock_radio *)context)->now_us;
}

static void mock_wait_until_us(void *context, int64_t deadline_us)
{
    struct mock_radio *mock = context;
    assert(deadline_us >= mock->now_us);
    mock->now_us = deadline_us;
    mock->slot_delay_count++;
}

static bool mock_stop_requested(void *context)
{
    const struct mock_radio *mock = context;
    return mock->slot_delay_count >= mock->stop_after_slots;
}

static void mock_set_ready(void *context)
{
    (void)context;
}

static void mock_capture_anchor_position(
    void *context, struct uwb_mobile_position *position)
{
    (void)context;
    memset(position, 0, sizeof(*position));
}

static void mock_consume_report(void *context, bool tag_range,
                                uint8_t initiator_id, uint8_t responder_id,
                                uint32_t frame_id, double distance_m,
                                const struct uwb_mobile_position *position)
{
    struct mock_radio *mock = context;
    assert(tag_range);
    assert(initiator_id == 1U);
    if (mock->scenario == MOCK_BASELINE) {
        assert(responder_id == 4U);
        assert(frame_id == 1U);
    }
    assert(distance_m == 1.234);
    assert(position != NULL);
    assert(mock->report_count < sizeof(mock->reported_anchor_ids));
    mock->reported_anchor_ids[mock->report_count] = responder_id;
    mock->reported_frame_ids[mock->report_count] = frame_id;
    mock->report_count++;
}

static struct uwb_native_ds_radio_ops mock_ops(struct mock_radio *mock)
{
    const struct uwb_native_ds_radio_ops radio = {
        .context = mock,
        .send_immediate_expect_rx = mock_send_poll,
        .send_delayed = mock_send_delayed,
        .send_delayed_expect_rx = mock_send_delayed_expect_rx,
        .receive = mock_receive,
        .add_delay_ms = mock_add_delay_ms,
        .programmed_tx_timestamp = mock_programmed_tx,
        .now_us = mock_now_us,
        .wait_until_us = mock_wait_until_us,
        .stop_requested = mock_stop_requested,
        .set_ready = mock_set_ready,
        .capture_anchor_position = mock_capture_anchor_position,
        .consume_report = mock_consume_report,
    };
    return radio;
}

int main(void)
{
    struct mock_radio mock = {
        .now_us = 1000,
        .scenario = MOCK_BASELINE,
        .stop_after_slots = 3U,
    };
    const struct uwb_native_ds_config config = {
        .source_id = 1U,
        .tag_id = 1U,
        .anchor_count = 3U,
        .anchor_ids = {2U, 3U, 4U},
        .slot_ms = 5U,
        .rx_slice_ms = 10U,
        .rx_timeout_ms = 5U,
        .response_delay_ms = 1U,
        .final_delay_ms = 1U,
        .maximum_distance_m = 100.0,
    };
    const struct uwb_native_ds_radio_ops radio = mock_ops(&mock);
    assert(uwb_native_ds_twr_run(&config, &radio) == ESP_OK);

    struct uwb_native_ds_pipeline_stats stats = {0};
    uwb_native_ds_twr_get_stats(&stats);
    assert(stats.poll_tx_count == 3U);
    assert(stats.response_rx_count == 2U);
    assert(stats.final_tx_count == 2U);
    assert(stats.result_rx_count == 1U);
    assert(stats.completed_range_count == 1U);
    assert(stats.rx_timeout_count == 2U);
    assert(stats.response_timeout_count == 1U);
    assert(stats.result_timeout_count == 1U);
    assert(stats.recovered_rx_error_count == 1U);
    assert(stats.complete_frame_count == 0U);
    assert(stats.incomplete_frame_count == 1U);
    assert(stats.recovery_attempt_count == 0U);
    assert(stats.recovery_success_count == 0U);
    assert(stats.last_frame_missing_anchor_mask == 0x03U);
    assert(stats.tag_anchor_count == 3U);
    assert(stats.tag_anchors[0].anchor_id == 2U);
    assert(stats.tag_anchors[0].attempt_count == 1U);
    assert(stats.tag_anchors[0].response_timeout_count == 1U);
    assert(stats.tag_anchors[1].anchor_id == 3U);
    assert(stats.tag_anchors[1].result_timeout_count == 1U);
    assert(stats.tag_anchors[2].anchor_id == 4U);
    assert(stats.tag_anchors[2].completed_range_count == 1U);
    assert(mock.report_count == 1U);

    struct mock_radio recovery = {
        .now_us = 1000,
        .scenario = MOCK_SINGLE_MISS_RECOVERY,
        .stop_after_slots = 5U,
    };
    const struct uwb_native_ds_config recovery_config = {
        .source_id = 1U,
        .tag_id = 1U,
        .anchor_count = 4U,
        .anchor_ids = {2U, 3U, 4U, 5U},
        .slot_ms = 5U,
        .rx_slice_ms = 10U,
        .rx_timeout_ms = 5U,
        .response_delay_ms = 1U,
        .final_delay_ms = 1U,
        .maximum_distance_m = 100.0,
    };
    const struct uwb_native_ds_radio_ops recovery_radio =
        mock_ops(&recovery);
    assert(uwb_native_ds_twr_run(&recovery_config, &recovery_radio) ==
           ESP_OK);
    uwb_native_ds_twr_get_stats(&stats);
    assert(stats.complete_frame_count == 1U);
    assert(stats.incomplete_frame_count == 0U);
    assert(stats.recovery_attempt_count == 1U);
    assert(stats.recovery_success_count == 1U);
    assert(stats.last_recovery_anchor_id == 3U);
    assert(stats.tag_anchors[1].attempt_count == 2U);
    assert(stats.tag_anchors[1].completed_range_count == 1U);
    const uint8_t expected_recovery_order[] = {2U, 3U, 4U, 5U, 3U};
    assert(memcmp(recovery.attempted_anchor_ids,
                  expected_recovery_order,
                  sizeof(expected_recovery_order)) == 0);
    assert(recovery.report_count == 4U);

    struct mock_radio rotation = {
        .now_us = 1000,
        .scenario = MOCK_ROTATING_ORDER,
        .stop_after_slots = 8U,
    };
    const struct uwb_native_ds_radio_ops rotation_radio =
        mock_ops(&rotation);
    assert(uwb_native_ds_twr_run(&recovery_config, &rotation_radio) ==
           ESP_OK);
    uwb_native_ds_twr_get_stats(&stats);
    assert(stats.complete_frame_count == 2U);
    assert(stats.incomplete_frame_count == 0U);
    assert(stats.recovery_attempt_count == 0U);
    const uint8_t expected_rotating_order[] = {
        2U, 3U, 4U, 5U, 3U, 4U, 5U, 2U,
    };
    assert(memcmp(rotation.attempted_anchor_ids,
                  expected_rotating_order,
                  sizeof(expected_rotating_order)) == 0);
    assert(rotation.report_count == 8U);
    assert(rotation.reported_frame_ids[0] == 1U);
    assert(rotation.reported_frame_ids[3] == 1U);
    assert(rotation.reported_frame_ids[4] == 2U);
    assert(rotation.reported_frame_ids[7] == 2U);

    puts("uwb_native_ds_twr_stats_test: PASS");
    return 0;
}
