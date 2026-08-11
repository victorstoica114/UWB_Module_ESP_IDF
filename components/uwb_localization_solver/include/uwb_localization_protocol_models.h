#ifndef UWB_LOCALIZATION_PROTOCOL_MODELS_H
#define UWB_LOCALIZATION_PROTOCOL_MODELS_H

#include "uwb_localization_solver.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UWB_LOCALIZATION_MODEL_MAX_ANCHORS 10U

struct uwb_localization_model_anchor {
    uint16_t id;
    double x_m;
    double y_m;
};

struct uwb_localization_native_ds_range {
    uint16_t anchor_id;
    double range_m;
    double base_weight;
};

struct uwb_localization_tdoa_difference {
    uint16_t initiator_id;
    uint16_t responder_id;
    double difference_m;
    double base_weight;
    bool dynamic_geometry;
    double initiator_x_m;
    double initiator_y_m;
    double responder_x_m;
    double responder_y_m;
};

/* Native DS-TWR: one direct tag-to-anchor range per observation. */
bool uwb_localization_solve_native_ds_2d(
    const struct uwb_localization_model_anchor *anchors,
    size_t anchor_count,
    const struct uwb_localization_native_ds_range *ranges,
    size_t range_count,
    const struct uwb_localization_seed *seed,
    const struct uwb_localization_solver_config *config,
    struct uwb_localization_result *result);

/* FlexTDOA: CFO correction is upstream; this consumes corrected Eq. (14)
 * range differences for one coherent Flex frame. */
bool uwb_localization_solve_flextdoa_2d(
    const struct uwb_localization_model_anchor *anchors,
    size_t anchor_count,
    const struct uwb_localization_tdoa_difference *differences,
    size_t difference_count,
    const struct uwb_localization_seed *seed,
    const struct uwb_localization_solver_config *config,
    struct uwb_localization_result *result);

/* Passive DS-TWR: the coherent 3/4 selection remains upstream in the batch
 * policy.  The optional precision matrix preserves the correlated timing
 * model of observations sharing the same initiator. */
bool uwb_localization_solve_passive_ds_2d(
    const struct uwb_localization_model_anchor *anchors,
    size_t anchor_count,
    const struct uwb_localization_tdoa_difference *differences,
    size_t difference_count,
    const double *timing_precision,
    size_t precision_row_stride,
    const struct uwb_localization_seed *seed,
    const struct uwb_localization_solver_config *config,
    struct uwb_localization_result *result);

#ifdef __cplusplus
}
#endif

#endif /* UWB_LOCALIZATION_PROTOCOL_MODELS_H */
