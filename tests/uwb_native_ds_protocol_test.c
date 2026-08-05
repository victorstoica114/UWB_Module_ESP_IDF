#include "uwb_native_ds_protocol.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void codec_round_trip(enum uwb_native_ds_message_type type)
{
    const struct uwb_native_ds_packet source = {
        .type = type,
        .exchange_kind = UWB_NATIVE_DS_EXCHANGE_TAG_RANGE,
        .source_id = 1U,
        .destination_id = 5U,
        .session_id = 0x89abcdefU,
        .frame_id = 0xf1234567U,
        .poll_tx_timestamp = UWB_NATIVE_DS_TIMESTAMP_MASK - 7U,
        .response_rx_timestamp = 123456789U,
        .final_tx_timestamp = 987654321U,
        .distance_mm = 3472U,
    };
    uint8_t payload[UWB_NATIVE_DS_PROTOCOL_MAX_PACKET_SIZE] = {0};
    size_t payload_len = 0U;
    assert(uwb_native_ds_protocol_encode(
        &source, payload, sizeof(payload), &payload_len));
    assert(payload_len == uwb_native_ds_protocol_packet_size(type));

    struct uwb_native_ds_packet decoded = {0};
    assert(uwb_native_ds_protocol_decode(payload, payload_len, &decoded));
    assert(decoded.type == source.type);
    assert(decoded.exchange_kind == source.exchange_kind);
    assert(decoded.source_id == source.source_id);
    assert(decoded.destination_id == source.destination_id);
    assert(decoded.session_id == source.session_id);
    assert(decoded.frame_id == source.frame_id);
    if (type == UWB_NATIVE_DS_MESSAGE_FINAL) {
        assert(decoded.poll_tx_timestamp == source.poll_tx_timestamp);
        assert(decoded.response_rx_timestamp == source.response_rx_timestamp);
        assert(decoded.final_tx_timestamp == source.final_tx_timestamp);
    }
    if (type == UWB_NATIVE_DS_MESSAGE_RESULT) {
        assert(decoded.distance_mm == source.distance_mm);
    }

    assert(!uwb_native_ds_protocol_decode(payload, payload_len - 1U,
                                          &decoded));
    payload[4]++;
    assert(!uwb_native_ds_protocol_decode(payload, payload_len, &decoded));
}

static uint64_t wrap_add(uint64_t value, uint64_t delta)
{
    return (value + delta) & UWB_NATIVE_DS_TIMESTAMP_MASK;
}

int main(void)
{
    codec_round_trip(UWB_NATIVE_DS_MESSAGE_POLL);
    codec_round_trip(UWB_NATIVE_DS_MESSAGE_RESPONSE);
    codec_round_trip(UWB_NATIVE_DS_MESSAGE_FINAL);
    codec_round_trip(UWB_NATIVE_DS_MESSAGE_RESULT);

    assert(uwb_native_ds_protocol_timestamp_delta(
               5U, UWB_NATIVE_DS_TIMESTAMP_MASK - 4U) == 10U);

    /* Synthetic asymmetric DS-TWR exchange crossing the 40-bit wrap. */
    const uint64_t tof = 640U;
    const uint64_t responder_reply = 190000U;
    const uint64_t initiator_reply = 310000U;
    const uint64_t poll_tx = UWB_NATIVE_DS_TIMESTAMP_MASK - 100000U;
    const uint64_t poll_rx = wrap_add(poll_tx, tof);
    const uint64_t response_tx = wrap_add(poll_rx, responder_reply);
    const uint64_t response_rx = wrap_add(response_tx, tof);
    const uint64_t final_tx = wrap_add(response_rx, initiator_reply);
    const uint64_t final_rx = wrap_add(final_tx, tof);
    double distance_m = 0.0;
    assert(uwb_native_ds_protocol_calculate_distance(
        poll_tx, poll_rx, response_tx, response_rx, final_tx, final_rx,
        &distance_m));
    const double expected_m =
        (double)tof * 15.650040064102564e-12 * 299702547.0;
    assert(fabs(distance_m - expected_m) < 1.0e-6);

    puts("uwb_native_ds_protocol_test: PASS");
    return 0;
}
