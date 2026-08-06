#include "passive_ds_position_solver.h"

#include <float.h>
#include <math.h>
#include <string.h>

struct passive_ds_cost_result {
    double sse;
    double h00;
    double h01;
    double h11;
    double g0;
    double g1;
    size_t count;
};

struct passive_ds_timing_precision {
    double weight[PASSIVE_DS_POSITION_MAX_OBSERVATIONS]
                 [PASSIVE_DS_POSITION_MAX_OBSERVATIONS];
};

static int anchor_index(
    const struct passive_ds_position_anchor *anchors,
    size_t anchor_count,
    uint8_t id)
{
    for (size_t index = 0U; index < anchor_count; ++index) {
        if (anchors[index].id == id) {
            return (int)index;
        }
    }
    return -1;
}

static bool solve_covariance_system(
    const double matrix[PASSIVE_DS_POSITION_MAX_ANCHORS]
                       [PASSIVE_DS_POSITION_MAX_ANCHORS],
    const double *rhs,
    size_t count,
    double *solution)
{
    double augmented[PASSIVE_DS_POSITION_MAX_ANCHORS]
                    [PASSIVE_DS_POSITION_MAX_ANCHORS + 1U] = {{0}};
    for (size_t row = 0U; row < count; ++row) {
        for (size_t column = 0U; column < count; ++column) {
            augmented[row][column] = matrix[row][column];
        }
        augmented[row][count] = rhs[row];
    }
    for (size_t pivot = 0U; pivot < count; ++pivot) {
        size_t best = pivot;
        for (size_t row = pivot + 1U; row < count; ++row) {
            if (fabs(augmented[row][pivot]) >
                fabs(augmented[best][pivot])) {
                best = row;
            }
        }
        if (fabs(augmented[best][pivot]) < 1e-12) {
            return false;
        }
        if (best != pivot) {
            for (size_t column = pivot; column <= count; ++column) {
                const double temporary = augmented[pivot][column];
                augmented[pivot][column] = augmented[best][column];
                augmented[best][column] = temporary;
            }
        }
        const double divisor = augmented[pivot][pivot];
        for (size_t column = pivot; column <= count; ++column) {
            augmented[pivot][column] /= divisor;
        }
        for (size_t row = 0U; row < count; ++row) {
            if (row == pivot) {
                continue;
            }
            const double factor = augmented[row][pivot];
            for (size_t column = pivot; column <= count; ++column) {
                augmented[row][column] -=
                    factor * augmented[pivot][column];
            }
        }
    }
    for (size_t row = 0U; row < count; ++row) {
        solution[row] = augmented[row][count];
    }
    return true;
}

static bool build_timing_precision(
    const struct passive_ds_position_anchor *anchors,
    size_t anchor_count,
    const struct passive_ds_position_observation *observations,
    size_t observation_count,
    struct passive_ds_timing_precision *precision)
{
    if (observation_count > PASSIVE_DS_POSITION_MAX_OBSERVATIONS ||
        precision == NULL) {
        return false;
    }
    memset(precision, 0, sizeof(*precision));
    for (size_t group = 0U; group < anchor_count; ++group) {
        size_t source_index[PASSIVE_DS_POSITION_MAX_ANCHORS] = {0};
        double ratio[PASSIVE_DS_POSITION_MAX_ANCHORS] = {0};
        size_t count = 0U;
        for (size_t index = 0U; index < observation_count; ++index) {
            const struct passive_ds_position_observation *observation =
                &observations[index];
            const int initiator = anchor_index(
                anchors, anchor_count, observation->initiator_id);
            const int responder = anchor_index(
                anchors, anchor_count, observation->responder_id);
            if (initiator != (int)group) {
                continue;
            }
            if (responder < 0 || responder == initiator ||
                !isfinite(observation->difference_m) ||
                count >= PASSIVE_DS_POSITION_MAX_ANCHORS) {
                return false;
            }
            source_index[count] = index;
            ratio[count] =
                isfinite(observation->delay_ratio) &&
                        observation->delay_ratio > 0.0 &&
                        observation->delay_ratio < 1.0
                    ? observation->delay_ratio
                    : 0.5;
            count++;
        }
        if (count == 0U) {
            continue;
        }

        double covariance[PASSIVE_DS_POSITION_MAX_ANCHORS]
                         [PASSIVE_DS_POSITION_MAX_ANCHORS] = {{0}};
        for (size_t row = 0U; row < count; ++row) {
            const double one_minus = 1.0 - ratio[row];
            covariance[row][row] = 1.25 *
                (1.0 + one_minus * one_minus + ratio[row] * ratio[row]);
            for (size_t column = 0U; column < row; ++column) {
                const double shared =
                    (ratio[row] - 1.0) * (ratio[column] - 1.0) +
                    ratio[row] * ratio[column];
                covariance[row][column] = shared;
                covariance[column][row] = shared;
            }
        }
        for (size_t column = 0U; column < count; ++column) {
            double unit[PASSIVE_DS_POSITION_MAX_ANCHORS] = {0};
            double inverse_column[PASSIVE_DS_POSITION_MAX_ANCHORS] = {0};
            unit[column] = 1.0;
            if (!solve_covariance_system(
                    covariance, unit, count, inverse_column)) {
                return false;
            }
            for (size_t row = 0U; row < count; ++row) {
                precision->weight[source_index[row]][source_index[column]] =
                    inverse_column[row];
            }
        }
    }
    return true;
}

static struct passive_ds_cost_result position_cost(
    const struct passive_ds_position_anchor *anchors,
    size_t anchor_count,
    const struct passive_ds_position_observation *observations,
    size_t observation_count,
    double x_m,
    double y_m,
    double common_correlation,
    const struct passive_ds_timing_precision *timing_precision)
{
    struct passive_ds_cost_result cost = {0};
    double residual_sum[PASSIVE_DS_POSITION_MAX_ANCHORS] = {0};
    double jx_sum[PASSIVE_DS_POSITION_MAX_ANCHORS] = {0};
    double jy_sum[PASSIVE_DS_POSITION_MAX_ANCHORS] = {0};
    size_t group_count[PASSIVE_DS_POSITION_MAX_ANCHORS] = {0};
    double sample_residual[PASSIVE_DS_POSITION_MAX_OBSERVATIONS] = {0};
    double sample_jx[PASSIVE_DS_POSITION_MAX_OBSERVATIONS] = {0};
    double sample_jy[PASSIVE_DS_POSITION_MAX_OBSERVATIONS] = {0};
    size_t sample_source_index[PASSIVE_DS_POSITION_MAX_OBSERVATIONS] = {0};
    size_t sample_count = 0U;
    for (size_t index = 0U; index < observation_count; ++index) {
        const struct passive_ds_position_observation *observation =
            &observations[index];
        const int initiator = anchor_index(
            anchors, anchor_count, observation->initiator_id);
        const int responder = anchor_index(
            anchors, anchor_count, observation->responder_id);
        if (initiator < 0 || responder < 0 || initiator == responder ||
            !isfinite(observation->difference_m)) {
            continue;
        }

        const double initiator_x = observation->dynamic_geometry
            ? observation->initiator_x_m
            : anchors[initiator].x_m;
        const double initiator_y = observation->dynamic_geometry
            ? observation->initiator_y_m
            : anchors[initiator].y_m;
        const double responder_x = observation->dynamic_geometry
            ? observation->responder_x_m
            : anchors[responder].x_m;
        const double responder_y = observation->dynamic_geometry
            ? observation->responder_y_m
            : anchors[responder].y_m;
        if (!isfinite(initiator_x) || !isfinite(initiator_y) ||
            !isfinite(responder_x) || !isfinite(responder_y)) {
            continue;
        }
        const double initiator_dx = x_m - initiator_x;
        const double initiator_dy = y_m - initiator_y;
        const double responder_dx = x_m - responder_x;
        const double responder_dy = y_m - responder_y;
        const double initiator_distance = hypot(initiator_dx, initiator_dy);
        const double responder_distance = hypot(responder_dx, responder_dy);
        if (initiator_distance < 0.02 || responder_distance < 0.02) {
            continue;
        }

        const double predicted = responder_distance - initiator_distance;
        const double residual = observation->difference_m - predicted;
        const double jx =
            responder_dx / responder_distance -
            initiator_dx / initiator_distance;
        const double jy =
            responder_dy / responder_distance -
            initiator_dy / initiator_distance;

        if (sample_count >= PASSIVE_DS_POSITION_MAX_OBSERVATIONS) {
            return (struct passive_ds_cost_result){.sse = NAN};
        }
        sample_residual[sample_count] = residual;
        sample_jx[sample_count] = jx;
        sample_jy[sample_count] = jy;
        sample_source_index[sample_count] = index;
        sample_count++;

        /* Raw quadratic terms; optional same-frame covariance is below. */
        cost.sse += residual * residual;
        cost.h00 += jx * jx;
        cost.h01 += jx * jy;
        cost.h11 += jy * jy;
        cost.g0 += jx * residual;
        cost.g1 += jy * residual;
        residual_sum[initiator] += residual;
        jx_sum[initiator] += jx;
        jy_sum[initiator] += jy;
        group_count[initiator]++;
        cost.count++;
    }
    if (timing_precision != NULL) {
        memset(&cost, 0, sizeof(cost));
        for (size_t row = 0U; row < sample_count; ++row) {
            const size_t source_row = sample_source_index[row];
            for (size_t column = 0U; column < sample_count; ++column) {
                const size_t source_column = sample_source_index[column];
                const double weight =
                    timing_precision->weight[source_row][source_column];
                cost.sse += sample_residual[row] * weight *
                            sample_residual[column];
                cost.h00 += sample_jx[row] * weight * sample_jx[column];
                cost.h01 += sample_jx[row] * weight * sample_jy[column];
                cost.h11 += sample_jy[row] * weight * sample_jy[column];
                cost.g0 += sample_jx[row] * weight *
                           sample_residual[column];
                cost.g1 += sample_jy[row] * weight *
                           sample_residual[column];
            }
        }
        cost.count = sample_count;
        return cost;
    }
    if (common_correlation > 0.0) {
        const double bounded_correlation = fmin(0.95,
            fmax(0.0, common_correlation));
        for (size_t group = 0U; group < anchor_count; ++group) {
            if (group_count[group] < 2U) {
                continue;
            }
            const double alpha = bounded_correlation /
                (1.0 + ((double)group_count[group] - 1.0) *
                           bounded_correlation);
            cost.sse -= alpha * residual_sum[group] * residual_sum[group];
            cost.h00 -= alpha * jx_sum[group] * jx_sum[group];
            cost.h01 -= alpha * jx_sum[group] * jy_sum[group];
            cost.h11 -= alpha * jy_sum[group] * jy_sum[group];
            cost.g0 -= alpha * jx_sum[group] * residual_sum[group];
            cost.g1 -= alpha * jy_sum[group] * residual_sum[group];
        }
    }
    return cost;
}

static bool solve_from_seed(
    const struct passive_ds_position_anchor *anchors,
    size_t anchor_count,
    const struct passive_ds_position_observation *observations,
    size_t observation_count,
    double seed_x_m,
    double seed_y_m,
    double geometry_span_m,
    double min_x_m,
    double max_x_m,
    double min_y_m,
    double max_y_m,
    double common_correlation,
    const struct passive_ds_timing_precision *timing_precision,
    struct passive_ds_position_result *result)
{
    double x_m = seed_x_m;
    double y_m = seed_y_m;
    double damping = 1e-6;
    uint8_t completed_iterations = 0U;

    for (uint8_t iteration = 0U; iteration < 8U; ++iteration) {
        const struct passive_ds_cost_result cost = position_cost(
            anchors, anchor_count, observations, observation_count,
            x_m, y_m, common_correlation, timing_precision);
        const double h00 = cost.h00 + damping;
        const double h11 = cost.h11 + damping;
        const double determinant = h00 * h11 - cost.h01 * cost.h01;
        if (cost.count < 2U || !isfinite(cost.sse) ||
            fabs(determinant) < 1e-12) {
            return false;
        }

        double dx =
            (h11 * cost.g0 - cost.h01 * cost.g1) / determinant;
        double dy =
            (-cost.h01 * cost.g0 + h00 * cost.g1) / determinant;
        const double step_m = hypot(dx, dy);
        const double max_step_m = fmax(0.05, geometry_span_m * 0.25);
        if (step_m > max_step_m) {
            dx *= max_step_m / step_m;
            dy *= max_step_m / step_m;
        }

        bool accepted = false;
        double scale = 1.0;
        for (uint8_t attempt = 0U; attempt < 8U; ++attempt) {
            const double candidate_x_m = x_m + scale * dx;
            const double candidate_y_m = y_m + scale * dy;
            const struct passive_ds_cost_result candidate = position_cost(
                anchors, anchor_count, observations, observation_count,
                candidate_x_m, candidate_y_m, common_correlation,
                timing_precision);
            if (candidate.count == cost.count &&
                isfinite(candidate.sse) && candidate.sse <= cost.sse) {
                x_m = candidate_x_m;
                y_m = candidate_y_m;
                accepted = true;
                completed_iterations = (uint8_t)(iteration + 1U);
                break;
            }
            scale *= 0.5;
        }
        if (!accepted) {
            damping *= 10.0;
            continue;
        }
        damping = fmax(1e-9, damping * 0.25);
        if (hypot(scale * dx, scale * dy) < 0.00001) {
            break;
        }
    }

    const struct passive_ds_cost_result final_cost = position_cost(
        anchors, anchor_count, observations, observation_count, x_m, y_m,
        common_correlation, timing_precision);
    const double determinant =
        final_cost.h00 * final_cost.h11 - final_cost.h01 * final_cost.h01;
    const double margin_m = 2.0 * geometry_span_m;
    if (final_cost.count < 2U || !isfinite(x_m) || !isfinite(y_m) ||
        !isfinite(final_cost.sse) || determinant <= 1e-12 ||
        x_m < min_x_m - margin_m || x_m > max_x_m + margin_m ||
        y_m < min_y_m - margin_m || y_m > max_y_m + margin_m) {
        return false;
    }

    const double variance = final_cost.sse /
        fmax(1.0, (double)final_cost.count - 2.0);
    const double sigma_m = sqrt(fmax(
        0.0,
        variance * (final_cost.h00 + final_cost.h11) /
            determinant / 2.0));
    const double rms_m = sqrt(final_cost.sse / (double)final_cost.count);
    if (!isfinite(sigma_m) || !isfinite(rms_m)) {
        return false;
    }

    *result = (struct passive_ds_position_result){
        .x_m = x_m,
        .y_m = y_m,
        .sigma_m = sigma_m,
        .rms_m = rms_m,
        .observation_count = (uint8_t)final_cost.count,
        .iteration_count = completed_iterations,
    };
    return true;
}

static bool passive_ds_position_solve_internal(
    const struct passive_ds_position_anchor *anchors,
    size_t anchor_count,
    const struct passive_ds_position_observation *observations,
    size_t observation_count,
    double common_correlation,
    bool timing_covariance,
    bool previous_valid,
    double previous_x_m,
    double previous_y_m,
    struct passive_ds_position_result *result)
{
    if (anchors == NULL || observations == NULL || result == NULL ||
        anchor_count < 3U ||
        anchor_count > PASSIVE_DS_POSITION_MAX_ANCHORS ||
        observation_count < 2U) {
        return false;
    }

    double min_x_m = anchors[0].x_m;
    double max_x_m = anchors[0].x_m;
    double min_y_m = anchors[0].y_m;
    double max_y_m = anchors[0].y_m;
    double center_x_m = 0.0;
    double center_y_m = 0.0;
    for (size_t index = 0U; index < anchor_count; ++index) {
        if (!isfinite(anchors[index].x_m) ||
            !isfinite(anchors[index].y_m) || anchors[index].id == 0U) {
            return false;
        }
        min_x_m = fmin(min_x_m, anchors[index].x_m);
        max_x_m = fmax(max_x_m, anchors[index].x_m);
        min_y_m = fmin(min_y_m, anchors[index].y_m);
        max_y_m = fmax(max_y_m, anchors[index].y_m);
        center_x_m += anchors[index].x_m;
        center_y_m += anchors[index].y_m;
    }
    center_x_m /= (double)anchor_count;
    center_y_m /= (double)anchor_count;
    const double geometry_span_m = fmax(
        0.5, hypot(max_x_m - min_x_m, max_y_m - min_y_m));

    struct passive_ds_timing_precision precision = {0};
    const struct passive_ds_timing_precision *timing_precision = NULL;
    if (timing_covariance) {
        if (!build_timing_precision(
                anchors, anchor_count, observations, observation_count,
                &precision)) {
            return false;
        }
        timing_precision = &precision;
    }

    struct passive_ds_position_result best = {0};
    bool best_valid = false;
    if (previous_valid && isfinite(previous_x_m) && isfinite(previous_y_m)) {
        best_valid = solve_from_seed(
            anchors, anchor_count, observations, observation_count,
            previous_x_m, previous_y_m, geometry_span_m,
            min_x_m, max_x_m, min_y_m, max_y_m,
            common_correlation, timing_precision, &best);
    }
    if (!best_valid) {
        best_valid = solve_from_seed(
            anchors, anchor_count, observations, observation_count,
            center_x_m, center_y_m, geometry_span_m,
            min_x_m, max_x_m, min_y_m, max_y_m, common_correlation,
            timing_precision, &best);
    }
    if (!best_valid) {
        return false;
    }
    *result = best;
    return true;
}

bool passive_ds_position_solve(
    const struct passive_ds_position_anchor *anchors,
    size_t anchor_count,
    const struct passive_ds_position_observation *observations,
    size_t observation_count,
    bool previous_valid,
    double previous_x_m,
    double previous_y_m,
    struct passive_ds_position_result *result)
{
    return passive_ds_position_solve_internal(
        anchors, anchor_count, observations, observation_count, 0.0,
        false, previous_valid, previous_x_m, previous_y_m, result);
}

bool passive_ds_position_solve_correlated(
    const struct passive_ds_position_anchor *anchors,
    size_t anchor_count,
    const struct passive_ds_position_observation *observations,
    size_t observation_count,
    double common_correlation,
    bool previous_valid,
    double previous_x_m,
    double previous_y_m,
    struct passive_ds_position_result *result)
{
    if (!isfinite(common_correlation) || common_correlation < 0.0 ||
        common_correlation >= 1.0) {
        return false;
    }
    return passive_ds_position_solve_internal(
        anchors, anchor_count, observations, observation_count,
        common_correlation, false, previous_valid, previous_x_m,
        previous_y_m, result);
}

bool passive_ds_position_solve_timing_covariance(
    const struct passive_ds_position_anchor *anchors,
    size_t anchor_count,
    const struct passive_ds_position_observation *observations,
    size_t observation_count,
    bool previous_valid,
    double previous_x_m,
    double previous_y_m,
    struct passive_ds_position_result *result)
{
    return passive_ds_position_solve_internal(
        anchors, anchor_count, observations, observation_count, 0.0,
        true, previous_valid, previous_x_m, previous_y_m, result);
}
