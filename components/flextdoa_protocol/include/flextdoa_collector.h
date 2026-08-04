#ifndef FLEXTDOA_COLLECTOR_H
#define FLEXTDOA_COLLECTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "flextdoa_protocol.h"

enum flextdoa_collect_result {
    FLEXTDOA_COLLECT_REJECTED = 0,
    FLEXTDOA_COLLECT_REQUEST,
    FLEXTDOA_COLLECT_RESPONSE,
    FLEXTDOA_COLLECT_COMPLETE,
};

struct flextdoa_collected_response {
    bool present;
    uint16_t responder_id;
    uint64_t rx_timestamp_dtu;
    uint32_t processing_time_dtu;
    bool cfo_valid;
    double cfo_fraction;
};

struct flextdoa_slot_collection {
    bool active;
    uint32_t slot_id;
    uint16_t initiator_id;
    uint64_t request_rx_timestamp_dtu;
    uint8_t responder_count;
    uint16_t received_mask;
    struct flextdoa_collected_response
        responses[FLEXTDOA_MAX_RESPONDERS];
};

void flextdoa_collector_reset(struct flextdoa_slot_collection *collection);
enum flextdoa_collect_result flextdoa_collector_ingest(
    struct flextdoa_slot_collection *collection,
    const struct flextdoa_packet *packet, uint64_t rx_timestamp_dtu,
    bool cfo_valid, double cfo_fraction);
bool flextdoa_collector_complete(
    const struct flextdoa_slot_collection *collection);
uint16_t flextdoa_collector_missing_mask(
    const struct flextdoa_slot_collection *collection);
uint16_t flextdoa_missing_mask_from_presence(
    const bool *present, uint8_t responder_count);

#endif
