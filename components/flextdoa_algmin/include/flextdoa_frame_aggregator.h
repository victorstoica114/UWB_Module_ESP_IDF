#ifndef FLEXTDOA_FRAME_AGGREGATOR_H
#define FLEXTDOA_FRAME_AGGREGATOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "flextdoa_algmin.h"

enum flextdoa_frame_ingest_result {
    FLEXTDOA_FRAME_REJECTED = 0,
    FLEXTDOA_FRAME_ACCEPTED,
    FLEXTDOA_FRAME_DUPLICATE,
    FLEXTDOA_FRAME_BOUNDARY,
    FLEXTDOA_FRAME_COMPLETE,
};

struct flextdoa_anchor_bias {
    uint16_t anchor_id;
    /*
     * Signed additive correction, not the measured hardware offset itself.
     * If calibration reports raw_offset_m for an anchor, configure
     * bias_m = -raw_offset_m so raw + bias_responder - bias_initiator
     * cancels the directed observation offset.
     */
    double bias_m;
};

/*
 * Collects raw directed range differences from all slots in one TDMA frame.
 * The caller owns the storage, which keeps the collector usable in both the
 * ESP32 runtime and host-side unit tests without dynamic allocation.
 */
struct flextdoa_frame_aggregator {
    bool active;
    uint32_t frame_id;
    uint32_t first_slot_id;
    uint32_t last_slot_id;
    uint8_t slot_count;
    uint8_t responses_per_slot;
    uint16_t expected_observation_count;
    uint16_t minimum_observation_count;
    size_t observation_count;
    size_t capacity;
    uint32_t *slot_ids;
    struct flextdoa_range_difference *observations;
};

void flextdoa_frame_aggregator_init(
    struct flextdoa_frame_aggregator *aggregator,
    uint32_t *slot_ids,
    struct flextdoa_range_difference *observations,
    size_t capacity);
void flextdoa_frame_aggregator_reset(
    struct flextdoa_frame_aggregator *aggregator);
enum flextdoa_frame_ingest_result flextdoa_frame_aggregator_ingest(
    struct flextdoa_frame_aggregator *aggregator,
    uint32_t slot_id, uint8_t slot_count, uint8_t responses_per_slot,
    uint16_t minimum_observation_count,
    const struct flextdoa_range_difference *observation);
bool flextdoa_frame_aggregator_solvable(
    const struct flextdoa_frame_aggregator *aggregator);
bool flextdoa_frame_aggregator_complete(
    const struct flextdoa_frame_aggregator *aggregator);
/*
 * Materialize the observations used by AlgMin. The accumulator always keeps
 * the immutable raw values. A future calibration layer can supply static
 * per-anchor correction terms; each directed i->j observation receives
 * (bias_j - bias_i). Here bias is explicitly the negative of a measured
 * per-anchor raw offset. NULL/zero biases are the exact raw, zero-default
 * path.
 * Returns the copied observation count, or zero on invalid input.
 */
size_t flextdoa_frame_aggregator_copy_corrected(
    const struct flextdoa_frame_aggregator *aggregator,
    const struct flextdoa_anchor_bias *biases, size_t bias_count,
    struct flextdoa_range_difference *output, size_t output_capacity);

#endif /* FLEXTDOA_FRAME_AGGREGATOR_H */
