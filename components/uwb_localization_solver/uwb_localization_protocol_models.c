#include "uwb_localization_protocol_models.h"

#include <math.h>

static const struct uwb_localization_model_anchor *find_anchor(
    const struct uwb_localization_model_anchor *anchors,
    size_t anchor_count,
    uint16_t id)
{
    for (size_t index = 0U; index < anchor_count; ++index) {
        if (anchors[index].id == id) {
            return &anchors[index];
        }
    }
    return NULL;
}

static bool geometry_valid(
    const struct uwb_localization_model_anchor *anchors,
    size_t anchor_count)
{
    if (anchors == NULL || anchor_count < 3U ||
        anchor_count > UWB_LOCALIZATION_MODEL_MAX_ANCHORS) {
        return false;
    }
    for (size_t index = 0U; index < anchor_count; ++index) {
        if (anchors[index].id == 0U || !isfinite(anchors[index].x_m) ||
            !isfinite(anchors[index].y_m)) {
            return false;
        }
        for (size_t previous = 0U; previous < index; ++previous) {
            if (anchors[previous].id == anchors[index].id) {
                return false;
            }
        }
    }
    return true;
}

bool uwb_localization_solve_native_ds_2d(
    const struct uwb_localization_model_anchor *anchors,
    size_t anchor_count,
    const struct uwb_localization_native_ds_range *ranges,
    size_t range_count,
    const struct uwb_localization_seed *seed,
    const struct uwb_localization_solver_config *config,
    struct uwb_localization_result *result)
{
    if (!geometry_valid(anchors, anchor_count) || ranges == NULL ||
        range_count < 3U ||
        range_count > UWB_LOCALIZATION_MAX_OBSERVATIONS) {
        return false;
    }
    struct uwb_localization_observation observations[
        UWB_LOCALIZATION_MAX_OBSERVATIONS] = {0};
    for (size_t index = 0U; index < range_count; ++index) {
        const struct uwb_localization_model_anchor *anchor = find_anchor(
            anchors, anchor_count, ranges[index].anchor_id);
        if (anchor == NULL || !isfinite(ranges[index].range_m) ||
            ranges[index].range_m <= 0.0 ||
            !isfinite(ranges[index].base_weight) ||
            ranges[index].base_weight <= 0.0) {
            return false;
        }
        for (size_t previous = 0U; previous < index; ++previous) {
            if (ranges[previous].anchor_id == ranges[index].anchor_id) {
                return false;
            }
        }
        observations[index] = (struct uwb_localization_observation){
            .type = UWB_LOCALIZATION_MEASUREMENT_RANGE,
            .first_x_m = anchor->x_m,
            .first_y_m = anchor->y_m,
            .measured_m = ranges[index].range_m,
            .base_weight = ranges[index].base_weight,
        };
    }
    return uwb_localization_solve_2d(
        observations, range_count, seed, config, result);
}

static bool solve_tdoa(
    const struct uwb_localization_model_anchor *anchors,
    size_t anchor_count,
    const struct uwb_localization_tdoa_difference *differences,
    size_t difference_count,
    size_t minimum_difference_count,
    const double *precision_matrix,
    size_t precision_row_stride,
    const struct uwb_localization_seed *seed,
    const struct uwb_localization_solver_config *config,
    struct uwb_localization_result *result)
{
    if (!geometry_valid(anchors, anchor_count) || differences == NULL ||
        difference_count < minimum_difference_count ||
        difference_count > UWB_LOCALIZATION_MAX_OBSERVATIONS) {
        return false;
    }
    struct uwb_localization_observation observations[
        UWB_LOCALIZATION_MAX_OBSERVATIONS] = {0};
    for (size_t index = 0U; index < difference_count; ++index) {
        const struct uwb_localization_model_anchor *initiator = find_anchor(
            anchors, anchor_count, differences[index].initiator_id);
        const struct uwb_localization_model_anchor *responder = find_anchor(
            anchors, anchor_count, differences[index].responder_id);
        if (initiator == NULL || responder == NULL ||
            initiator == responder ||
            !isfinite(differences[index].difference_m) ||
            !isfinite(differences[index].base_weight) ||
            differences[index].base_weight <= 0.0) {
            return false;
        }
        const double initiator_x_m = differences[index].dynamic_geometry
            ? differences[index].initiator_x_m : initiator->x_m;
        const double initiator_y_m = differences[index].dynamic_geometry
            ? differences[index].initiator_y_m : initiator->y_m;
        const double responder_x_m = differences[index].dynamic_geometry
            ? differences[index].responder_x_m : responder->x_m;
        const double responder_y_m = differences[index].dynamic_geometry
            ? differences[index].responder_y_m : responder->y_m;
        if (!isfinite(initiator_x_m) || !isfinite(initiator_y_m) ||
            !isfinite(responder_x_m) || !isfinite(responder_y_m)) {
            return false;
        }
        observations[index] = (struct uwb_localization_observation){
            .type = UWB_LOCALIZATION_MEASUREMENT_RANGE_DIFFERENCE,
            .first_x_m = initiator_x_m,
            .first_y_m = initiator_y_m,
            .second_x_m = responder_x_m,
            .second_y_m = responder_y_m,
            .measured_m = differences[index].difference_m,
            .base_weight = differences[index].base_weight,
        };
    }
    return uwb_localization_solve_2d_correlated(
        observations, difference_count, precision_matrix,
        precision_row_stride, seed, config, result);
}

bool uwb_localization_solve_flextdoa_2d(
    const struct uwb_localization_model_anchor *anchors,
    size_t anchor_count,
    const struct uwb_localization_tdoa_difference *differences,
    size_t difference_count,
    const struct uwb_localization_seed *seed,
    const struct uwb_localization_solver_config *config,
    struct uwb_localization_result *result)
{
    return solve_tdoa(
        anchors, anchor_count, differences, difference_count, 3U,
        NULL, 0U, seed, config, result);
}

bool uwb_localization_solve_passive_ds_2d(
    const struct uwb_localization_model_anchor *anchors,
    size_t anchor_count,
    const struct uwb_localization_tdoa_difference *differences,
    size_t difference_count,
    const double *timing_precision,
    size_t precision_row_stride,
    const struct uwb_localization_seed *seed,
    const struct uwb_localization_solver_config *config,
    struct uwb_localization_result *result)
{
    return solve_tdoa(
        anchors, anchor_count, differences, difference_count, 2U,
        timing_precision, precision_row_stride, seed, config, result);
}
