#include "flextdoa_frame_aggregator.h"

#include <math.h>
#include <string.h>

static bool parameters_valid(
    const struct flextdoa_frame_aggregator *aggregator,
    uint8_t slot_count, uint8_t responses_per_slot,
    uint16_t minimum_observation_count)
{
    if (aggregator == NULL || aggregator->slot_ids == NULL ||
        aggregator->observations == NULL || aggregator->capacity == 0U ||
        slot_count == 0U || responses_per_slot == 0U) {
        return false;
    }
    const uint32_t expected =
        (uint32_t)slot_count * (uint32_t)responses_per_slot;
    return expected <= UINT16_MAX && expected <= aggregator->capacity &&
           minimum_observation_count > 0U &&
           minimum_observation_count <= expected;
}

void flextdoa_frame_aggregator_init(
    struct flextdoa_frame_aggregator *aggregator,
    uint32_t *slot_ids,
    struct flextdoa_range_difference *observations,
    size_t capacity)
{
    if (aggregator == NULL) {
        return;
    }
    memset(aggregator, 0, sizeof(*aggregator));
    aggregator->slot_ids = slot_ids;
    aggregator->observations = observations;
    aggregator->capacity = capacity;
}

void flextdoa_frame_aggregator_reset(
    struct flextdoa_frame_aggregator *aggregator)
{
    if (aggregator == NULL) {
        return;
    }
    uint32_t *slot_ids = aggregator->slot_ids;
    struct flextdoa_range_difference *observations =
        aggregator->observations;
    const size_t capacity = aggregator->capacity;
    memset(aggregator, 0, sizeof(*aggregator));
    aggregator->slot_ids = slot_ids;
    aggregator->observations = observations;
    aggregator->capacity = capacity;
}

bool flextdoa_frame_aggregator_solvable(
    const struct flextdoa_frame_aggregator *aggregator)
{
    return aggregator != NULL && aggregator->active &&
           aggregator->minimum_observation_count > 0U &&
           aggregator->observation_count >=
               aggregator->minimum_observation_count;
}

bool flextdoa_frame_aggregator_complete(
    const struct flextdoa_frame_aggregator *aggregator)
{
    return aggregator != NULL && aggregator->active &&
           aggregator->expected_observation_count > 0U &&
           aggregator->observation_count ==
               aggregator->expected_observation_count;
}

static bool find_bias(
    const struct flextdoa_anchor_bias *biases, size_t bias_count,
    uint16_t anchor_id, double *bias_m)
{
    *bias_m = 0.0;
    for (size_t index = 0U; index < bias_count; ++index) {
        if (!isfinite(biases[index].bias_m)) {
            return false;
        }
        if (biases[index].anchor_id == anchor_id) {
            *bias_m = biases[index].bias_m;
            return true;
        }
    }
    return true;
}

size_t flextdoa_frame_aggregator_copy_corrected(
    const struct flextdoa_frame_aggregator *aggregator,
    const struct flextdoa_anchor_bias *biases, size_t bias_count,
    struct flextdoa_range_difference *output, size_t output_capacity)
{
    if (aggregator == NULL || !aggregator->active || output == NULL ||
        output_capacity < aggregator->observation_count ||
        (bias_count > 0U && biases == NULL)) {
        return 0U;
    }
    for (size_t index = 0U;
         index < aggregator->observation_count; ++index) {
        output[index] = aggregator->observations[index];
        double initiator_bias_m = 0.0;
        double responder_bias_m = 0.0;
        if (!find_bias(biases, bias_count, output[index].initiator_id,
                       &initiator_bias_m) ||
            !find_bias(biases, bias_count, output[index].responder_id,
                       &responder_bias_m)) {
            return 0U;
        }
        output[index].range_difference_m +=
            responder_bias_m - initiator_bias_m;
    }
    return aggregator->observation_count;
}

enum flextdoa_frame_ingest_result flextdoa_frame_aggregator_ingest(
    struct flextdoa_frame_aggregator *aggregator,
    uint32_t slot_id, uint8_t slot_count, uint8_t responses_per_slot,
    uint16_t minimum_observation_count,
    const struct flextdoa_range_difference *observation)
{
    if (!parameters_valid(aggregator, slot_count, responses_per_slot,
                          minimum_observation_count) ||
        observation == NULL ||
        observation->initiator_id == observation->responder_id ||
        !isfinite(observation->range_difference_m) ||
        (observation->dynamic_geometry &&
         (!isfinite(observation->initiator_x_m) ||
          !isfinite(observation->initiator_y_m) ||
          !isfinite(observation->responder_x_m) ||
          !isfinite(observation->responder_y_m)))) {
        return FLEXTDOA_FRAME_REJECTED;
    }

    const uint32_t frame_id = slot_id / slot_count;
    if (!aggregator->active) {
        aggregator->active = true;
        aggregator->frame_id = frame_id;
        aggregator->first_slot_id = slot_id;
        aggregator->last_slot_id = slot_id;
        aggregator->slot_count = slot_count;
        aggregator->responses_per_slot = responses_per_slot;
        aggregator->expected_observation_count =
            (uint16_t)((uint16_t)slot_count * responses_per_slot);
        aggregator->minimum_observation_count =
            minimum_observation_count;
    } else {
        const int32_t frame_delta =
            (int32_t)(frame_id - aggregator->frame_id);
        if (frame_delta < 0) {
            return FLEXTDOA_FRAME_REJECTED;
        }
        if (frame_delta > 0) {
            return FLEXTDOA_FRAME_BOUNDARY;
        }
        if (aggregator->slot_count != slot_count ||
            aggregator->responses_per_slot != responses_per_slot ||
            aggregator->minimum_observation_count !=
                minimum_observation_count) {
            return FLEXTDOA_FRAME_REJECTED;
        }
    }

    for (size_t index = 0U;
         index < aggregator->observation_count; ++index) {
        const struct flextdoa_range_difference *stored =
            &aggregator->observations[index];
        if (aggregator->slot_ids[index] == slot_id &&
            stored->initiator_id == observation->initiator_id &&
            stored->responder_id == observation->responder_id) {
            /* First value wins, including for conflicting retransmissions. */
            return FLEXTDOA_FRAME_DUPLICATE;
        }
    }

    if (aggregator->observation_count >= aggregator->capacity ||
        aggregator->observation_count >=
            aggregator->expected_observation_count) {
        return FLEXTDOA_FRAME_REJECTED;
    }
    const size_t output = aggregator->observation_count++;
    aggregator->slot_ids[output] = slot_id;
    aggregator->observations[output] = *observation;
    aggregator->last_slot_id = slot_id;
    return flextdoa_frame_aggregator_complete(aggregator)
               ? FLEXTDOA_FRAME_COMPLETE
               : FLEXTDOA_FRAME_ACCEPTED;
}
