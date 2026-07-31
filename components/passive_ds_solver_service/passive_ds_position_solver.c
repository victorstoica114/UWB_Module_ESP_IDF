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

static struct passive_ds_cost_result position_cost(
    const struct passive_ds_position_anchor *anchors,
    size_t anchor_count,
    const struct passive_ds_position_observation *observations,
    size_t observation_count,
    double x_m,
    double y_m)
{
    struct passive_ds_cost_result cost = {0};
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

        const double initiator_dx = x_m - anchors[initiator].x_m;
        const double initiator_dy = y_m - anchors[initiator].y_m;
        const double responder_dx = x_m - anchors[responder].x_m;
        const double responder_dy = y_m - anchors[responder].y_m;
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

        /* Deliberately unweighted least squares: no Huber or range gate. */
        cost.sse += residual * residual;
        cost.h00 += jx * jx;
        cost.h01 += jx * jy;
        cost.h11 += jy * jy;
        cost.g0 += jx * residual;
        cost.g1 += jy * residual;
        cost.count++;
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
    struct passive_ds_position_result *result)
{
    double x_m = seed_x_m;
    double y_m = seed_y_m;
    double damping = 1e-6;
    uint8_t completed_iterations = 0U;

    for (uint8_t iteration = 0U; iteration < 8U; ++iteration) {
        const struct passive_ds_cost_result cost = position_cost(
            anchors, anchor_count, observations, observation_count,
            x_m, y_m);
        const double h00 = cost.h00 + damping;
        const double h11 = cost.h11 + damping;
        const double determinant = h00 * h11 - cost.h01 * cost.h01;
        if (cost.count < 3U || !isfinite(cost.sse) ||
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
                candidate_x_m, candidate_y_m);
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
        anchors, anchor_count, observations, observation_count, x_m, y_m);
    const double determinant =
        final_cost.h00 * final_cost.h11 - final_cost.h01 * final_cost.h01;
    const double margin_m = 2.0 * geometry_span_m;
    if (final_cost.count < 3U || !isfinite(x_m) || !isfinite(y_m) ||
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
    if (anchors == NULL || observations == NULL || result == NULL ||
        anchor_count < 4U ||
        anchor_count > PASSIVE_DS_POSITION_MAX_ANCHORS ||
        observation_count < 3U) {
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

    struct passive_ds_position_result best = {0};
    bool best_valid = solve_from_seed(
        anchors, anchor_count, observations, observation_count,
        center_x_m, center_y_m, geometry_span_m,
        min_x_m, max_x_m, min_y_m, max_y_m, &best);
    if (previous_valid && isfinite(previous_x_m) && isfinite(previous_y_m)) {
        struct passive_ds_position_result previous_result = {0};
        if (solve_from_seed(
                anchors, anchor_count, observations, observation_count,
                previous_x_m, previous_y_m, geometry_span_m,
                min_x_m, max_x_m, min_y_m, max_y_m,
                &previous_result) &&
            (!best_valid || previous_result.rms_m < best.rms_m)) {
            best = previous_result;
            best_valid = true;
        }
    }
    if (!best_valid) {
        return false;
    }
    *result = best;
    return true;
}
