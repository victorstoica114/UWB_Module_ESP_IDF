#ifndef PASSIVE_DS_BATCH_POLICY_H
#define PASSIVE_DS_BATCH_POLICY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PASSIVE_DS_BATCH_STARS_PER_POSITION 3U
#define PASSIVE_DS_BATCH_MAX_EQUATION_RMS_M 0.25
#define PASSIVE_DS_BATCH_MAX_ABS_RESIDUAL_M 0.50

struct passive_ds_batch_star_ref {
    bool valid;
    uint8_t initiator_id;
    uint32_t frame_id;
};

/*
 * Frame ordering is valid for spans below INT32_MAX frames and remains
 * correct across uint32_t wrap.
 */
bool passive_ds_batch_frame_after(uint32_t candidate, uint32_t reference);

/* Apply the raw, unweighted equation-residual gate. */
bool passive_ds_batch_residuals_valid(double equation_rms_m,
                                      double max_abs_residual_m);

/*
 * Select the newest three complete stars with distinct initiators.  Returns
 * zero unless their newest-to-oldest frame span is within max_frame_span.
 * selected_indices are ordered newest first.
 */
size_t passive_ds_batch_select_recent_unique(
    const struct passive_ds_batch_star_ref *stars,
    size_t star_count,
    uint32_t max_frame_span,
    size_t selected_indices[PASSIVE_DS_BATCH_STARS_PER_POSITION],
    uint32_t *newest_frame_id,
    uint32_t *frame_span);

#ifdef __cplusplus
}
#endif

#endif /* PASSIVE_DS_BATCH_POLICY_H */
