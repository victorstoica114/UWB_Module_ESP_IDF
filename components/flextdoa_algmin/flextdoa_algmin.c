#include "flextdoa_algmin.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define FLEXTDOA_ALGMIN_MAX_ITERATIONS 40U
#define FLEXTDOA_ALGMIN_MIN_DISTANCE_M 1.0e-6
#define FLEXTDOA_ALGMIN_STEP_TOLERANCE_M 1.0e-6
#define FLEXTDOA_ALGMIN_GATE_MAX_ANCHORS 64U

static const struct flextdoa_anchor_position *find_anchor(
    const struct flextdoa_anchor_position *anchors, size_t anchor_count,
    uint16_t anchor_id)
{
    for (size_t index = 0U; index < anchor_count; ++index) {
        if (anchors[index].anchor_id == anchor_id) {
            return &anchors[index];
        }
    }
    return NULL;
}

static size_t find_anchor_index(
    const struct flextdoa_anchor_position *anchors, size_t anchor_count,
    uint16_t anchor_id)
{
    for (size_t index = 0U; index < anchor_count; ++index) {
        if (anchors[index].anchor_id == anchor_id) {
            return index;
        }
    }
    return SIZE_MAX;
}

static bool geometry_valid(
    const struct flextdoa_anchor_position *anchors, size_t anchor_count)
{
    if (anchors == NULL || anchor_count < 3U) {
        return false;
    }
    for (size_t index = 0U; index < anchor_count; ++index) {
        if (anchors[index].anchor_id == 0U ||
            !isfinite(anchors[index].x_m) ||
            !isfinite(anchors[index].y_m)) {
            return false;
        }
        for (size_t previous = 0U; previous < index; ++previous) {
            if (anchors[previous].anchor_id == anchors[index].anchor_id) {
                return false;
            }
        }
    }
    return true;
}

static bool evaluate(
    const struct flextdoa_anchor_position *anchors, size_t anchor_count,
    const struct flextdoa_range_difference *observations,
    size_t observation_count, double x_m, double y_m,
    bool build_normal_equations, double *cost, double normal[3],
    double gradient[2])
{
    double sum_squared = 0.0;
    if (build_normal_equations) {
        memset(normal, 0, 3U * sizeof(normal[0]));
        memset(gradient, 0, 2U * sizeof(gradient[0]));
    }

    for (size_t index = 0U; index < observation_count; ++index) {
        const struct flextdoa_range_difference *observation =
            &observations[index];
        const struct flextdoa_anchor_position *initiator = find_anchor(
            anchors, anchor_count, observation->initiator_id);
        const struct flextdoa_anchor_position *responder = find_anchor(
            anchors, anchor_count, observation->responder_id);
        if (initiator == NULL || responder == NULL ||
            initiator == responder ||
            !isfinite(observation->range_difference_m)) {
            return false;
        }

        const double initiator_x_m = observation->dynamic_geometry
            ? observation->initiator_x_m : initiator->x_m;
        const double initiator_y_m = observation->dynamic_geometry
            ? observation->initiator_y_m : initiator->y_m;
        const double responder_x_m = observation->dynamic_geometry
            ? observation->responder_x_m : responder->x_m;
        const double responder_y_m = observation->dynamic_geometry
            ? observation->responder_y_m : responder->y_m;
        if (!isfinite(initiator_x_m) || !isfinite(initiator_y_m) ||
            !isfinite(responder_x_m) || !isfinite(responder_y_m)) {
            return false;
        }

        const double initiator_dx = x_m - initiator_x_m;
        const double initiator_dy = y_m - initiator_y_m;
        const double responder_dx = x_m - responder_x_m;
        const double responder_dy = y_m - responder_y_m;
        const double initiator_distance =
            hypot(initiator_dx, initiator_dy);
        const double responder_distance =
            hypot(responder_dx, responder_dy);
        if (initiator_distance < FLEXTDOA_ALGMIN_MIN_DISTANCE_M ||
            responder_distance < FLEXTDOA_ALGMIN_MIN_DISTANCE_M) {
            return false;
        }

        const double predicted = responder_distance - initiator_distance;
        const double residual =
            predicted - observation->range_difference_m;
        sum_squared += residual * residual;

        if (build_normal_equations) {
            const double jacobian_x =
                responder_dx / responder_distance -
                initiator_dx / initiator_distance;
            const double jacobian_y =
                responder_dy / responder_distance -
                initiator_dy / initiator_distance;
            normal[0] += jacobian_x * jacobian_x;
            normal[1] += jacobian_x * jacobian_y;
            normal[2] += jacobian_y * jacobian_y;
            gradient[0] += jacobian_x * residual;
            gradient[1] += jacobian_y * residual;
        }
    }

    if (!isfinite(sum_squared)) {
        return false;
    }
    *cost = sum_squared;
    return true;
}

bool flextdoa_algmin_solve_2d(
    const struct flextdoa_anchor_position *anchors, size_t anchor_count,
    const struct flextdoa_range_difference *observations,
    size_t observation_count, const struct flextdoa_algmin_seed *seed,
    struct flextdoa_algmin_result *result)
{
    if (result == NULL) {
        return false;
    }
    memset(result, 0, sizeof(*result));
    if (!geometry_valid(anchors, anchor_count) || observations == NULL ||
        observation_count < 3U) {
        return false;
    }

    double x_m = 0.0;
    double y_m = 0.0;
    if (seed != NULL && seed->valid && isfinite(seed->x_m) &&
        isfinite(seed->y_m)) {
        x_m = seed->x_m;
        y_m = seed->y_m;
    } else {
        for (size_t index = 0U; index < anchor_count; ++index) {
            x_m += anchors[index].x_m;
            y_m += anchors[index].y_m;
        }
        x_m /= (double)anchor_count;
        y_m /= (double)anchor_count;
    }

    double cost = 0.0;
    double normal[3] = {0.0};
    double gradient[2] = {0.0};
    if (!evaluate(anchors, anchor_count, observations, observation_count,
                  x_m, y_m, true, &cost, normal, gradient)) {
        return false;
    }

    double damping = 1.0e-3;
    uint8_t accepted_iterations = 0U;
    for (uint8_t attempt = 0U;
         attempt < FLEXTDOA_ALGMIN_MAX_ITERATIONS; ++attempt) {
        const double a = normal[0] + damping;
        const double b = normal[1];
        const double c = normal[2] + damping;
        const double determinant = a * c - b * b;
        if (!isfinite(determinant) || fabs(determinant) < DBL_EPSILON) {
            damping *= 10.0;
            continue;
        }

        const double step_x =
            (-c * gradient[0] + b * gradient[1]) / determinant;
        const double step_y =
            (b * gradient[0] - a * gradient[1]) / determinant;
        if (!isfinite(step_x) || !isfinite(step_y)) {
            return false;
        }

        const double candidate_x = x_m + step_x;
        const double candidate_y = y_m + step_y;
        double candidate_cost = 0.0;
        double unused_normal[3] = {0.0};
        double unused_gradient[2] = {0.0};
        if (!evaluate(anchors, anchor_count, observations,
                      observation_count, candidate_x, candidate_y, false,
                      &candidate_cost, unused_normal, unused_gradient)) {
            damping *= 10.0;
            continue;
        }

        if (candidate_cost < cost) {
            x_m = candidate_x;
            y_m = candidate_y;
            cost = candidate_cost;
            accepted_iterations++;
            damping = fmax(damping * 0.3, 1.0e-12);
            if (hypot(step_x, step_y) <
                FLEXTDOA_ALGMIN_STEP_TOLERANCE_M) {
                break;
            }
            if (!evaluate(anchors, anchor_count, observations,
                          observation_count, x_m, y_m, true, &cost,
                          normal, gradient)) {
                return false;
            }
        } else {
            damping *= 10.0;
        }
    }

    const double rms = sqrt(cost / (double)observation_count);
    if (!isfinite(x_m) || !isfinite(y_m) || !isfinite(rms)) {
        return false;
    }
    result->valid = true;
    result->x_m = x_m;
    result->y_m = y_m;
    result->residual_rms_m = rms;
    result->iterations = accepted_iterations;
    return true;
}

enum flextdoa_solution_gate_result flextdoa_algmin_gate_solution_2d(
    const struct flextdoa_anchor_position *anchors, size_t anchor_count,
    const struct flextdoa_range_difference *observations,
    size_t observation_count, double x_m, double y_m,
    size_t minimum_initiator_count, double maximum_condition_number,
    struct flextdoa_solution_gate_metrics *metrics)
{
    struct flextdoa_solution_gate_metrics local_metrics = {0};
    if (metrics == NULL) {
        metrics = &local_metrics;
    }
    memset(metrics, 0, sizeof(*metrics));
    if (!geometry_valid(anchors, anchor_count) || observations == NULL ||
        observation_count < 3U || anchor_count >
            FLEXTDOA_ALGMIN_GATE_MAX_ANCHORS ||
        minimum_initiator_count == 0U ||
        minimum_initiator_count > anchor_count || !isfinite(x_m) ||
        !isfinite(y_m) || !isfinite(maximum_condition_number) ||
        maximum_condition_number < 1.0) {
        return FLEXTDOA_SOLUTION_GATE_INVALID_ARGUMENT;
    }

    uint64_t adjacency[FLEXTDOA_ALGMIN_GATE_MAX_ANCHORS] = {0};
    uint64_t initiator_mask = 0U;
    for (size_t index = 0U; index < observation_count; ++index) {
        const size_t initiator_index = find_anchor_index(
            anchors, anchor_count, observations[index].initiator_id);
        const size_t responder_index = find_anchor_index(
            anchors, anchor_count, observations[index].responder_id);
        if (initiator_index == SIZE_MAX || responder_index == SIZE_MAX ||
            initiator_index == responder_index ||
            !isfinite(observations[index].range_difference_m)) {
            return FLEXTDOA_SOLUTION_GATE_INVALID_ARGUMENT;
        }
        initiator_mask |= UINT64_C(1) << initiator_index;
        adjacency[initiator_index] |= UINT64_C(1) << responder_index;
        adjacency[responder_index] |= UINT64_C(1) << initiator_index;
    }

    uint64_t initiators = initiator_mask;
    while (initiators != 0U) {
        metrics->initiator_count += initiators & UINT64_C(1);
        initiators >>= 1U;
    }
    if (metrics->initiator_count < minimum_initiator_count) {
        return FLEXTDOA_SOLUTION_GATE_TOO_FEW_INITIATORS;
    }

    uint64_t visited = UINT64_C(1);
    uint64_t frontier = visited;
    while (frontier != 0U) {
        uint64_t next = 0U;
        for (size_t index = 0U; index < anchor_count; ++index) {
            if ((frontier & (UINT64_C(1) << index)) != 0U) {
                next |= adjacency[index];
            }
        }
        next &= ~visited;
        visited |= next;
        frontier = next;
    }
    uint64_t connected = visited;
    while (connected != 0U) {
        metrics->connected_anchor_count += connected & UINT64_C(1);
        connected >>= 1U;
    }
    if (metrics->connected_anchor_count != anchor_count) {
        return FLEXTDOA_SOLUTION_GATE_DISCONNECTED;
    }

    double cost = 0.0;
    double normal[3] = {0.0};
    double gradient[2] = {0.0};
    if (!evaluate(anchors, anchor_count, observations, observation_count,
                  x_m, y_m, true, &cost, normal, gradient)) {
        return FLEXTDOA_SOLUTION_GATE_RANK_DEFICIENT;
    }
    const double trace = normal[0] + normal[2];
    const double discriminant =
        hypot(normal[0] - normal[2], 2.0 * normal[1]);
    const double lambda_max = 0.5 * (trace + discriminant);
    const double determinant =
        fma(normal[0], normal[2], -normal[1] * normal[1]);
    const double lambda_min = lambda_max > 0.0
                                  ? determinant / lambda_max
                                  : 0.0;
    metrics->normal_lambda_min = lambda_min;
    metrics->normal_lambda_max = lambda_max;
    metrics->normal_condition_number =
        lambda_min > 0.0 ? lambda_max / lambda_min : INFINITY;
    if (!isfinite(lambda_max) || !isfinite(lambda_min) ||
        lambda_max <= DBL_EPSILON ||
        lambda_min <= DBL_EPSILON * lambda_max) {
        return FLEXTDOA_SOLUTION_GATE_RANK_DEFICIENT;
    }
    if (!isfinite(metrics->normal_condition_number) ||
        metrics->normal_condition_number > maximum_condition_number) {
        return FLEXTDOA_SOLUTION_GATE_ILL_CONDITIONED;
    }
    return FLEXTDOA_SOLUTION_GATE_OK;
}
