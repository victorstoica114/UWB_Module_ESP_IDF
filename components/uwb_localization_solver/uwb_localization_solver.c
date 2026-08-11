#include "uwb_localization_solver.h"

#include <float.h>
#include <math.h>
#include <string.h>

struct evaluation {
    double cost;
    double raw_sse;
    double weighted_sse;
    double h00;
    double h01;
    double h11;
    double g0;
    double g1;
    double maximum_abs_residual;
    uint8_t downweighted_count;
};

static bool config_valid(const struct uwb_localization_solver_config *config)
{
    return config != NULL &&
           config->loss >= UWB_LOCALIZATION_LOSS_NONE &&
           config->loss <= UWB_LOCALIZATION_LOSS_CAUCHY &&
           isfinite(config->loss_scale_m) && config->loss_scale_m > 0.0 &&
           isfinite(config->minimum_distance_m) &&
           config->minimum_distance_m > 0.0 &&
           isfinite(config->step_tolerance_m) &&
           config->step_tolerance_m > 0.0 &&
           isfinite(config->maximum_step_m) &&
           config->maximum_step_m > 0.0 &&
           isfinite(config->initial_damping) &&
           config->initial_damping > 0.0 &&
           config->maximum_iterations > 0U;
}

static bool observation_valid(
    const struct uwb_localization_observation *observation)
{
    if (observation == NULL ||
        observation->type < UWB_LOCALIZATION_MEASUREMENT_RANGE ||
        observation->type >
            UWB_LOCALIZATION_MEASUREMENT_RANGE_DIFFERENCE ||
        !isfinite(observation->first_x_m) ||
        !isfinite(observation->first_y_m) ||
        !isfinite(observation->measured_m) ||
        !isfinite(observation->base_weight) ||
        observation->base_weight <= 0.0) {
        return false;
    }
    return observation->type == UWB_LOCALIZATION_MEASUREMENT_RANGE ||
           (isfinite(observation->second_x_m) &&
            isfinite(observation->second_y_m));
}

static double robust_weight(enum uwb_localization_robust_loss loss,
                            double residual, double scale)
{
    const double absolute = fabs(residual);
    switch (loss) {
    case UWB_LOCALIZATION_LOSS_NONE:
        return 1.0;
    case UWB_LOCALIZATION_LOSS_HUBER:
        return absolute <= scale ? 1.0 : scale / absolute;
    case UWB_LOCALIZATION_LOSS_CAUCHY: {
        const double normalized = residual / scale;
        return 1.0 / (1.0 + normalized * normalized);
    }
    default:
        return NAN;
    }
}

static double robust_cost(enum uwb_localization_robust_loss loss,
                          double residual, double scale)
{
    const double absolute = fabs(residual);
    switch (loss) {
    case UWB_LOCALIZATION_LOSS_NONE:
        return residual * residual;
    case UWB_LOCALIZATION_LOSS_HUBER:
        return absolute <= scale
                   ? residual * residual
                   : 2.0 * scale * absolute - scale * scale;
    case UWB_LOCALIZATION_LOSS_CAUCHY: {
        const double normalized = residual / scale;
        return scale * scale * log1p(normalized * normalized);
    }
    default:
        return NAN;
    }
}

static bool evaluate(
    const struct uwb_localization_observation *observations,
    size_t observation_count, double x_m, double y_m,
    const struct uwb_localization_solver_config *config,
    const double *precision_matrix, size_t precision_row_stride,
    bool build_normal, struct evaluation *evaluation)
{
    memset(evaluation, 0, sizeof(*evaluation));
    double residuals[UWB_LOCALIZATION_MAX_OBSERVATIONS] = {0};
    double sample_jx[UWB_LOCALIZATION_MAX_OBSERVATIONS] = {0};
    double sample_jy[UWB_LOCALIZATION_MAX_OBSERVATIONS] = {0};
    double row_scale[UWB_LOCALIZATION_MAX_OBSERVATIONS] = {0};
    for (size_t index = 0U; index < observation_count; ++index) {
        const struct uwb_localization_observation *observation =
            &observations[index];
        if (!observation_valid(observation)) {
            return false;
        }
        const double first_dx = x_m - observation->first_x_m;
        const double first_dy = y_m - observation->first_y_m;
        const double first_distance = hypot(first_dx, first_dy);
        if (first_distance < config->minimum_distance_m) {
            return false;
        }
        double predicted = first_distance;
        double jacobian_x = first_dx / first_distance;
        double jacobian_y = first_dy / first_distance;
        if (observation->type ==
            UWB_LOCALIZATION_MEASUREMENT_RANGE_DIFFERENCE) {
            const double second_dx = x_m - observation->second_x_m;
            const double second_dy = y_m - observation->second_y_m;
            const double second_distance = hypot(second_dx, second_dy);
            if (second_distance < config->minimum_distance_m) {
                return false;
            }
            predicted = second_distance - first_distance;
            jacobian_x = second_dx / second_distance - jacobian_x;
            jacobian_y = second_dy / second_distance - jacobian_y;
        }
        const double residual = observation->measured_m - predicted;
        const double robust = robust_weight(
            config->loss, residual, config->loss_scale_m);
        const double weight = observation->base_weight * robust;
        const double absolute = fabs(residual);
        if (!isfinite(residual) || !isfinite(weight) || weight <= 0.0) {
            return false;
        }
        residuals[index] = residual;
        sample_jx[index] = jacobian_x;
        sample_jy[index] = jacobian_y;
        row_scale[index] = sqrt(weight);
        evaluation->raw_sse += residual * residual;
        evaluation->maximum_abs_residual =
            fmax(evaluation->maximum_abs_residual, absolute);
        if (robust < 0.999) {
            evaluation->downweighted_count++;
        }
        if (precision_matrix == NULL) {
            evaluation->cost += observation->base_weight * robust_cost(
                config->loss, residual, config->loss_scale_m);
            evaluation->weighted_sse += weight * residual * residual;
            if (build_normal) {
                evaluation->h00 += weight * jacobian_x * jacobian_x;
                evaluation->h01 += weight * jacobian_x * jacobian_y;
                evaluation->h11 += weight * jacobian_y * jacobian_y;
                evaluation->g0 += weight * jacobian_x * residual;
                evaluation->g1 += weight * jacobian_y * residual;
            }
        }
    }
    if (precision_matrix != NULL) {
        for (size_t row = 0U; row < observation_count; ++row) {
            const double scaled_residual_row =
                row_scale[row] * residuals[row];
            const double scaled_jx_row =
                row_scale[row] * sample_jx[row];
            const double scaled_jy_row =
                row_scale[row] * sample_jy[row];
            for (size_t column = 0U; column < observation_count;
                 ++column) {
                const double precision = precision_matrix[
                    row * precision_row_stride + column];
                const double weighted_residual_column =
                    precision * row_scale[column] * residuals[column];
                evaluation->weighted_sse +=
                    scaled_residual_row * weighted_residual_column;
                if (build_normal) {
                    evaluation->h00 += scaled_jx_row * precision *
                        row_scale[column] * sample_jx[column];
                    evaluation->h01 += scaled_jx_row * precision *
                        row_scale[column] * sample_jy[column];
                    evaluation->h11 += scaled_jy_row * precision *
                        row_scale[column] * sample_jy[column];
                    evaluation->g0 += scaled_jx_row *
                        weighted_residual_column;
                    evaluation->g1 += scaled_jy_row *
                        weighted_residual_column;
                }
            }
        }
        if (evaluation->weighted_sse < -1.0e-9) {
            return false;
        }
        evaluation->weighted_sse = fmax(0.0, evaluation->weighted_sse);
        evaluation->cost = evaluation->weighted_sse;
    }
    return isfinite(evaluation->cost) &&
           isfinite(evaluation->raw_sse) &&
           isfinite(evaluation->weighted_sse);
}

struct uwb_localization_solver_config
uwb_localization_solver_default_config(void)
{
    return (struct uwb_localization_solver_config){
        .loss = UWB_LOCALIZATION_LOSS_HUBER,
        .loss_scale_m = 0.10,
        .minimum_distance_m = 1.0e-5,
        .step_tolerance_m = 1.0e-5,
        .maximum_step_m = 2.0,
        .initial_damping = 1.0e-3,
        .maximum_iterations = 30U,
    };
}

bool uwb_localization_solve_2d(
    const struct uwb_localization_observation *observations,
    size_t observation_count,
    const struct uwb_localization_seed *seed,
    const struct uwb_localization_solver_config *config,
    struct uwb_localization_result *result)
{
    return uwb_localization_solve_2d_correlated(
        observations, observation_count, NULL, 0U, seed, config,
        result);
}

bool uwb_localization_solve_2d_correlated(
    const struct uwb_localization_observation *observations,
    size_t observation_count,
    const double *precision_matrix,
    size_t precision_row_stride,
    const struct uwb_localization_seed *seed,
    const struct uwb_localization_solver_config *config,
    struct uwb_localization_result *result)
{
    if (result == NULL) {
        return false;
    }
    memset(result, 0, sizeof(*result));
    if (observations == NULL || observation_count < 2U ||
        observation_count > UWB_LOCALIZATION_MAX_OBSERVATIONS ||
        !config_valid(config) ||
        (precision_matrix != NULL &&
         precision_row_stride < observation_count)) {
        return false;
    }
    if (precision_matrix != NULL) {
        for (size_t row = 0U; row < observation_count; ++row) {
            if (!isfinite(precision_matrix[
                    row * precision_row_stride + row]) ||
                precision_matrix[row * precision_row_stride + row] <= 0.0) {
                return false;
            }
            for (size_t column = 0U; column < observation_count;
                 ++column) {
                const double value = precision_matrix[
                    row * precision_row_stride + column];
                const double transpose = precision_matrix[
                    column * precision_row_stride + row];
                if (!isfinite(value) ||
                    fabs(value - transpose) >
                        1.0e-9 * fmax(1.0, fmax(fabs(value),
                                                fabs(transpose)))) {
                    return false;
                }
            }
        }
    }
    double x_m = 0.0;
    double y_m = 0.0;
    if (seed != NULL && seed->valid && isfinite(seed->x_m) &&
        isfinite(seed->y_m)) {
        x_m = seed->x_m;
        y_m = seed->y_m;
    } else {
        size_t point_count = 0U;
        for (size_t index = 0U; index < observation_count; ++index) {
            if (!observation_valid(&observations[index])) {
                return false;
            }
            x_m += observations[index].first_x_m;
            y_m += observations[index].first_y_m;
            point_count++;
            if (observations[index].type ==
                UWB_LOCALIZATION_MEASUREMENT_RANGE_DIFFERENCE) {
                x_m += observations[index].second_x_m;
                y_m += observations[index].second_y_m;
                point_count++;
            }
        }
        x_m /= (double)point_count;
        y_m /= (double)point_count;
    }

    double damping = config->initial_damping;
    uint8_t accepted_iterations = 0U;
    for (uint8_t attempt = 0U; attempt < config->maximum_iterations;
         ++attempt) {
        struct evaluation current = {0};
        if (!evaluate(observations, observation_count, x_m, y_m,
                      config, precision_matrix, precision_row_stride,
                      true, &current)) {
            return false;
        }
        const double h00 = current.h00 + damping;
        const double h11 = current.h11 + damping;
        const double determinant = h00 * h11 - current.h01 * current.h01;
        if (!isfinite(determinant) || fabs(determinant) < DBL_EPSILON) {
            damping *= 10.0;
            continue;
        }
        double step_x =
            (h11 * current.g0 - current.h01 * current.g1) /
            determinant;
        double step_y =
            (h00 * current.g1 - current.h01 * current.g0) /
            determinant;
        double step_m = hypot(step_x, step_y);
        if (!isfinite(step_m)) {
            return false;
        }
        if (step_m > config->maximum_step_m) {
            step_x *= config->maximum_step_m / step_m;
            step_y *= config->maximum_step_m / step_m;
            step_m = config->maximum_step_m;
        }
        struct evaluation candidate = {0};
        if (!evaluate(observations, observation_count,
                      x_m + step_x, y_m + step_y, config,
                      precision_matrix, precision_row_stride, false,
                      &candidate)) {
            damping *= 10.0;
            continue;
        }
        if (candidate.cost < current.cost) {
            x_m += step_x;
            y_m += step_y;
            accepted_iterations++;
            damping = fmax(1.0e-12, damping * 0.3);
            if (step_m < config->step_tolerance_m) {
                break;
            }
        } else {
            damping *= 10.0;
        }
    }

    struct evaluation final = {0};
    if (!evaluate(observations, observation_count, x_m, y_m, config,
                  precision_matrix, precision_row_stride, true,
                  &final)) {
        return false;
    }
    const double determinant =
        final.h00 * final.h11 - final.h01 * final.h01;
    const double trace = final.h00 + final.h11;
    const double discriminant = hypot(final.h00 - final.h11,
                                      2.0 * final.h01);
    const double lambda_max = 0.5 * (trace + discriminant);
    const double lambda_min = 0.5 * (trace - discriminant);
    if (!isfinite(x_m) || !isfinite(y_m) || determinant <= 1.0e-12 ||
        lambda_min <= DBL_EPSILON * lambda_max) {
        return false;
    }
    const double degrees_of_freedom =
        (double)(observation_count > 2U ? observation_count - 2U : 1U);
    const double variance = final.weighted_sse / degrees_of_freedom;
    const double covariance_trace =
        variance * (final.h00 + final.h11) / determinant;
    *result = (struct uwb_localization_result){
        .valid = true,
        .x_m = x_m,
        .y_m = y_m,
        .residual_rms_m =
            sqrt(final.raw_sse / (double)observation_count),
        .weighted_residual_rms_m =
            sqrt(final.weighted_sse / (double)observation_count),
        .maximum_abs_residual_m = final.maximum_abs_residual,
        .sigma_m = sqrt(fmax(0.0, covariance_trace / 2.0)),
        .geometry_hdop =
            sqrt(fmax(0.0, (final.h00 + final.h11) / determinant)),
        .normal_condition_number = lambda_max / lambda_min,
        .observation_count = (uint8_t)observation_count,
        .downweighted_count = final.downweighted_count,
        .iterations = accepted_iterations,
    };
    return true;
}
