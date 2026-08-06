#include "passive_ds_batch_policy.h"

#include <math.h>

bool passive_ds_batch_frame_after(uint32_t candidate, uint32_t reference)
{
    return (int32_t)(candidate - reference) > 0;
}

bool passive_ds_batch_residuals_valid(double equation_rms_m,
                                      double max_abs_residual_m)
{
    return isfinite(equation_rms_m) && isfinite(max_abs_residual_m) &&
           equation_rms_m >= 0.0 && max_abs_residual_m >= 0.0 &&
           equation_rms_m <= PASSIVE_DS_BATCH_MAX_EQUATION_RMS_M &&
           max_abs_residual_m <= PASSIVE_DS_BATCH_MAX_ABS_RESIDUAL_M;
}

bool passive_ds_batch_star_observation_count_valid(
    size_t anchor_count, size_t observation_count)
{
    if (anchor_count < 3U || observation_count < 2U) {
        return false;
    }
    const size_t complete_count = anchor_count - 1U;
    const size_t minimum_count = complete_count > 2U
                                     ? complete_count - 1U
                                     : complete_count;
    return observation_count >= minimum_count &&
           observation_count <= complete_count;
}

size_t passive_ds_batch_select_recent_unique(
    const struct passive_ds_batch_star_ref *stars,
    size_t star_count,
    uint32_t max_frame_span,
    size_t selected_indices[PASSIVE_DS_BATCH_STARS_PER_POSITION],
    uint32_t *newest_frame_id,
    uint32_t *frame_span)
{
    if (stars == NULL || selected_indices == NULL ||
        newest_frame_id == NULL || frame_span == NULL) {
        return 0U;
    }
    bool have_previous = false;
    uint32_t previous_frame = 0U;
    uint8_t used_initiators[PASSIVE_DS_BATCH_STARS_PER_POSITION] = {0};
    size_t selected_count = 0U;
    while (selected_count < PASSIVE_DS_BATCH_STARS_PER_POSITION) {
        size_t best = SIZE_MAX;
        for (size_t index = 0U; index < star_count; ++index) {
            if (!stars[index].valid || stars[index].initiator_id == 0U) {
                continue;
            }
            bool initiator_used = false;
            for (size_t used = 0U; used < selected_count; ++used) {
                initiator_used |=
                    used_initiators[used] == stars[index].initiator_id;
            }
            if (initiator_used ||
                (have_previous &&
                 !passive_ds_batch_frame_after(
                     previous_frame, stars[index].frame_id))) {
                continue;
            }
            if (best == SIZE_MAX ||
                passive_ds_batch_frame_after(
                    stars[index].frame_id, stars[best].frame_id)) {
                best = index;
            }
        }
        if (best == SIZE_MAX) {
            return 0U;
        }
        selected_indices[selected_count] = best;
        used_initiators[selected_count] = stars[best].initiator_id;
        previous_frame = stars[best].frame_id;
        have_previous = true;
        selected_count++;
    }
    const uint32_t newest = stars[selected_indices[0]].frame_id;
    const uint32_t oldest =
        stars[selected_indices[PASSIVE_DS_BATCH_STARS_PER_POSITION - 1U]]
            .frame_id;
    const uint32_t span = newest - oldest;
    if (span > max_frame_span) {
        return 0U;
    }
    *newest_frame_id = newest;
    *frame_span = span;
    return PASSIVE_DS_BATCH_STARS_PER_POSITION;
}
