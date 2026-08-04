#include "flextdoa_collector.h"

#include <math.h>
#include <string.h>

void flextdoa_collector_reset(struct flextdoa_slot_collection *collection)
{
    if (collection != NULL) {
        memset(collection, 0, sizeof(*collection));
    }
}

bool flextdoa_collector_complete(
    const struct flextdoa_slot_collection *collection)
{
    if (collection == NULL || !collection->active ||
        collection->responder_count == 0U) {
        return false;
    }
    const uint16_t expected_mask =
        (uint16_t)((1U << collection->responder_count) - 1U);
    return collection->received_mask == expected_mask;
}

uint16_t flextdoa_collector_missing_mask(
    const struct flextdoa_slot_collection *collection)
{
    if (collection == NULL || !collection->active ||
        collection->responder_count == 0U ||
        collection->responder_count > FLEXTDOA_MAX_RESPONDERS) {
        return 0U;
    }
    const uint16_t expected_mask =
        (uint16_t)((1U << collection->responder_count) - 1U);
    return (uint16_t)(expected_mask & ~collection->received_mask);
}

uint16_t flextdoa_missing_mask_from_presence(
    const bool *present, uint8_t responder_count)
{
    if (present == NULL || responder_count == 0U ||
        responder_count > FLEXTDOA_MAX_RESPONDERS) {
        return 0U;
    }

    uint16_t missing_mask = 0U;
    for (uint8_t index = 0U; index < responder_count; ++index) {
        if (!present[index]) {
            missing_mask |= (uint16_t)(1U << index);
        }
    }
    return missing_mask;
}

enum flextdoa_collect_result flextdoa_collector_ingest(
    struct flextdoa_slot_collection *collection,
    const struct flextdoa_packet *packet, uint64_t rx_timestamp_dtu,
    bool cfo_valid, double cfo_fraction)
{
    if (collection == NULL || packet == NULL ||
        (cfo_valid && !isfinite(cfo_fraction))) {
        return FLEXTDOA_COLLECT_REJECTED;
    }

    if (packet->type == FLEXTDOA_MESSAGE_REQUEST) {
        if (packet->destination_count == 0U ||
            packet->destination_count > FLEXTDOA_MAX_RESPONDERS) {
            return FLEXTDOA_COLLECT_REJECTED;
        }
        flextdoa_collector_reset(collection);
        collection->active = true;
        collection->slot_id = packet->slot_id;
        collection->initiator_id = packet->source_id;
        collection->request_rx_timestamp_dtu = rx_timestamp_dtu;
        collection->responder_count = packet->destination_count;
        for (uint8_t index = 0U; index < packet->destination_count;
             ++index) {
            const uint16_t responder_id = packet->destination_ids[index];
            if (responder_id == packet->source_id) {
                flextdoa_collector_reset(collection);
                return FLEXTDOA_COLLECT_REJECTED;
            }
            for (uint8_t previous = 0U; previous < index; ++previous) {
                if (collection->responses[previous].responder_id ==
                    responder_id) {
                    flextdoa_collector_reset(collection);
                    return FLEXTDOA_COLLECT_REJECTED;
                }
            }
            collection->responses[index].responder_id = responder_id;
        }
        return FLEXTDOA_COLLECT_REQUEST;
    }

    if (packet->type != FLEXTDOA_MESSAGE_RESPONSE ||
        !collection->active || packet->slot_id != collection->slot_id ||
        packet->processing_time_dtu == 0U) {
        return FLEXTDOA_COLLECT_REJECTED;
    }

    uint8_t response_index = collection->responder_count;
    for (uint8_t index = 0U; index < collection->responder_count; ++index) {
        if (collection->responses[index].responder_id == packet->source_id) {
            response_index = index;
            break;
        }
    }
    if (response_index == collection->responder_count ||
        collection->responses[response_index].present) {
        return FLEXTDOA_COLLECT_REJECTED;
    }

    struct flextdoa_collected_response *response =
        &collection->responses[response_index];
    response->present = true;
    response->rx_timestamp_dtu = rx_timestamp_dtu;
    response->processing_time_dtu = packet->processing_time_dtu;
    response->cfo_valid = cfo_valid;
    response->cfo_fraction = cfo_valid ? cfo_fraction : 0.0;
    collection->received_mask |= (uint16_t)(1U << response_index);
    return flextdoa_collector_complete(collection)
               ? FLEXTDOA_COLLECT_COMPLETE
               : FLEXTDOA_COLLECT_RESPONSE;
}
