#include "uwb_native_ds_position_solver.h"

#include <math.h>
#include <string.h>

#define NATIVE_DS_POSITION_ITERATIONS 8U
#define NATIVE_DS_MIN_RANGE_M 0.02f
#define NATIVE_DS_MAX_RANGE_M 100.0f
#define NATIVE_DS_MIN_GEOMETRY_AREA_M2 1.0e-4f
#define NATIVE_DS_MAX_EQUATION_RMS_M 0.40f
#define NATIVE_DS_MAX_ABS_RESIDUAL_M 0.50f
#define NATIVE_DS_RANGE_COHERENCE_MARGIN_M 1.00f

static int anchor_index(const struct uwb_native_ds_position_solver *solver,
                        uint8_t anchor_id)
{
    for (size_t index = 0U; index < solver->anchor_count; ++index) {
        if (solver->anchor_ids[index] == anchor_id) {
            return (int)index;
        }
    }
    return -1;
}

static size_t pair_index(size_t anchor_count, size_t first, size_t second)
{
    size_t index = 0U;
    for (size_t left = 0U; left < anchor_count; ++left) {
        for (size_t right = left + 1U; right < anchor_count; ++right) {
            if (left == first && right == second) {
                return index;
            }
            ++index;
        }
    }
    return index;
}

static uint64_t complete_pair_mask(size_t anchor_count)
{
    const size_t count = anchor_count * (anchor_count - 1U) / 2U;
    return count >= 64U ? UINT64_MAX : ((1ULL << count) - 1ULL);
}

static uint16_t complete_anchor_mask(size_t anchor_count)
{
    return (uint16_t)((1UL << anchor_count) - 1UL);
}

static size_t range_count(uint16_t range_mask)
{
    size_t count = 0U;
    while (range_mask != 0U) {
        count += range_mask & 1U;
        range_mask >>= 1U;
    }
    return count;
}

static bool frame_is_newer(uint32_t candidate, uint32_t current)
{
    return (int32_t)(candidate - current) > 0;
}

static void initialize_output(
    const struct uwb_native_ds_position_solver *solver,
    struct uwb_native_ds_position_output *output)
{
    memset(output, 0, sizeof(*output));
    output->tag_id = solver->tag_id;
    output->anchor_count = solver->anchor_count;
    output->geometry_version = solver->geometry_version;
}

static void copy_geometry(
    const struct uwb_native_ds_position_solver *solver,
    struct uwb_native_ds_position_output *output)
{
    memcpy(output->anchor_x_m, solver->anchor_x_m,
           sizeof(output->anchor_x_m));
    memcpy(output->anchor_y_m, solver->anchor_y_m,
           sizeof(output->anchor_y_m));
}

static bool geometry_is_observable(const float *x_m, const float *y_m,
                                   size_t anchor_count)
{
    float maximum_twice_area = 0.0f;
    for (size_t second = 1U; second + 1U < anchor_count; ++second) {
        for (size_t third = second + 1U; third < anchor_count; ++third) {
            const float twice_area = fabsf(
                (x_m[second] - x_m[0]) * (y_m[third] - y_m[0]) -
                (y_m[second] - y_m[0]) * (x_m[third] - x_m[0]));
            maximum_twice_area = fmaxf(maximum_twice_area, twice_area);
        }
    }
    return maximum_twice_area >= 2.0f * NATIVE_DS_MIN_GEOMETRY_AREA_M2;
}

bool uwb_native_ds_position_solver_init(
    struct uwb_native_ds_position_solver *solver, uint8_t tag_id,
    const uint8_t *anchor_ids, const float *anchor_x_m,
    const float *anchor_y_m, size_t anchor_count,
    uint32_t geometry_version)
{
    if (solver == NULL || anchor_ids == NULL || anchor_x_m == NULL ||
        anchor_y_m == NULL || tag_id == 0U || anchor_count < 3U ||
        anchor_count > UWB_NATIVE_DS_POSITION_MAX_ANCHORS) {
        return false;
    }
    for (size_t first = 0U; first < anchor_count; ++first) {
        if (anchor_ids[first] == 0U || anchor_ids[first] == tag_id ||
            !isfinite(anchor_x_m[first]) || !isfinite(anchor_y_m[first])) {
            return false;
        }
        for (size_t second = first + 1U; second < anchor_count; ++second) {
            if (anchor_ids[first] == anchor_ids[second]) {
                return false;
            }
        }
    }
    if (!geometry_is_observable(anchor_x_m, anchor_y_m, anchor_count)) {
        return false;
    }

    memset(solver, 0, sizeof(*solver));
    solver->tag_id = tag_id;
    solver->anchor_count = (uint8_t)anchor_count;
    solver->geometry_version = geometry_version == 0U ? 1U : geometry_version;
    memcpy(solver->anchor_ids, anchor_ids, anchor_count);
    memcpy(solver->anchor_x_m, anchor_x_m, anchor_count * sizeof(float));
    memcpy(solver->anchor_y_m, anchor_y_m, anchor_count * sizeof(float));
    return true;
}

bool uwb_native_ds_position_solver_geometry(
    const struct uwb_native_ds_position_solver *solver,
    struct uwb_native_ds_position_output *output)
{
    if (solver == NULL || output == NULL || solver->anchor_count < 3U) {
        return false;
    }
    initialize_output(solver, output);
    output->geometry_updated = true;
    copy_geometry(solver, output);
    return true;
}

bool uwb_native_ds_position_solver_update_geometry(
    struct uwb_native_ds_position_solver *solver,
    const float *anchor_x_m, const float *anchor_y_m,
    uint32_t geometry_version)
{
    if (solver == NULL || anchor_x_m == NULL || anchor_y_m == NULL ||
        solver->anchor_count < 3U || geometry_version == 0U) {
        return false;
    }
    for (size_t index = 0U; index < solver->anchor_count; ++index) {
        if (!isfinite(anchor_x_m[index]) ||
            !isfinite(anchor_y_m[index])) {
            return false;
        }
    }
    if (!geometry_is_observable(
            anchor_x_m, anchor_y_m, solver->anchor_count)) {
        return false;
    }
    memcpy(solver->anchor_x_m, anchor_x_m,
           solver->anchor_count * sizeof(solver->anchor_x_m[0]));
    memcpy(solver->anchor_y_m, anchor_y_m,
           solver->anchor_count * sizeof(solver->anchor_y_m[0]));
    solver->geometry_version = geometry_version;
    return true;
}

static bool selected_geometry_is_observable(
    const struct uwb_native_ds_position_solver *solver, uint16_t range_mask)
{
    float maximum_twice_area = 0.0f;
    for (size_t first = 0U; first < solver->anchor_count; ++first) {
        if ((range_mask & (uint16_t)(1U << first)) == 0U) {
            continue;
        }
        for (size_t second = first + 1U; second < solver->anchor_count;
             ++second) {
            if ((range_mask & (uint16_t)(1U << second)) == 0U) {
                continue;
            }
            for (size_t third = second + 1U; third < solver->anchor_count;
                 ++third) {
                if ((range_mask & (uint16_t)(1U << third)) == 0U) {
                    continue;
                }
                const float twice_area = fabsf(
                    (solver->anchor_x_m[second] -
                     solver->anchor_x_m[first]) *
                        (solver->anchor_y_m[third] -
                         solver->anchor_y_m[first]) -
                    (solver->anchor_y_m[second] -
                     solver->anchor_y_m[first]) *
                        (solver->anchor_x_m[third] -
                         solver->anchor_x_m[first]));
                maximum_twice_area =
                    fmaxf(maximum_twice_area, twice_area);
            }
        }
    }
    return maximum_twice_area >= 2.0f * NATIVE_DS_MIN_GEOMETRY_AREA_M2;
}

static bool selected_ranges_are_coherent(
    const struct uwb_native_ds_position_solver *solver, uint16_t range_mask)
{
    for (size_t first = 0U; first < solver->anchor_count; ++first) {
        if ((range_mask & (uint16_t)(1U << first)) == 0U) {
            continue;
        }
        for (size_t second = first + 1U; second < solver->anchor_count;
             ++second) {
            if ((range_mask & (uint16_t)(1U << second)) == 0U) {
                continue;
            }
            const float anchor_distance = hypotf(
                solver->anchor_x_m[first] - solver->anchor_x_m[second],
                solver->anchor_y_m[first] - solver->anchor_y_m[second]);
            const float first_range = solver->tag_range_m[first];
            const float second_range = solver->tag_range_m[second];
            if (fabsf(first_range - second_range) >
                    anchor_distance + NATIVE_DS_RANGE_COHERENCE_MARGIN_M ||
                first_range + second_range +
                        NATIVE_DS_RANGE_COHERENCE_MARGIN_M <
                    anchor_distance) {
                return false;
            }
        }
    }
    return true;
}

static bool initial_position(const struct uwb_native_ds_position_solver *solver,
                             uint16_t range_mask, float *x_m, float *y_m)
{
    size_t reference = solver->anchor_count;
    for (size_t index = 0U; index < solver->anchor_count; ++index) {
        if ((range_mask & (uint16_t)(1U << index)) != 0U) {
            reference = index;
            break;
        }
    }
    if (reference == solver->anchor_count) {
        return false;
    }
    const float x0 = solver->anchor_x_m[reference];
    const float y0 = solver->anchor_y_m[reference];
    const float r0 = solver->tag_range_m[reference];
    float h00 = 0.0f;
    float h01 = 0.0f;
    float h11 = 0.0f;
    float g0 = 0.0f;
    float g1 = 0.0f;
    for (size_t index = 0U; index < solver->anchor_count; ++index) {
        if (index == reference ||
            (range_mask & (uint16_t)(1U << index)) == 0U) {
            continue;
        }
        const float xi = solver->anchor_x_m[index];
        const float yi = solver->anchor_y_m[index];
        const float ri = solver->tag_range_m[index];
        const float ax = 2.0f * (xi - x0);
        const float ay = 2.0f * (yi - y0);
        const float b = xi * xi + yi * yi - x0 * x0 - y0 * y0 -
                        ri * ri + r0 * r0;
        h00 += ax * ax;
        h01 += ax * ay;
        h11 += ay * ay;
        g0 += ax * b;
        g1 += ay * b;
    }
    const float determinant = h00 * h11 - h01 * h01;
    if (fabsf(determinant) < 1.0e-8f) {
        return false;
    }
    *x_m = (h11 * g0 - h01 * g1) / determinant;
    *y_m = (h00 * g1 - h01 * g0) / determinant;
    return isfinite(*x_m) && isfinite(*y_m);
}

static bool solve_position(struct uwb_native_ds_position_solver *solver,
                           uint16_t range_mask,
                           bool apply_quality_gate,
                           struct uwb_native_ds_position_output *output)
{
    const size_t observation_count = range_count(range_mask);
    if (observation_count < 3U ||
        !selected_geometry_is_observable(solver, range_mask) ||
        (apply_quality_gate &&
         !selected_ranges_are_coherent(solver, range_mask))) {
        return false;
    }
    float x_m = 0.0f;
    float y_m = 0.0f;
    if (!initial_position(solver, range_mask, &x_m, &y_m)) {
        return false;
    }

    uint8_t iterations = 0U;
    for (; iterations < NATIVE_DS_POSITION_ITERATIONS; ++iterations) {
        float h00 = 1.0e-6f;
        float h01 = 0.0f;
        float h11 = 1.0e-6f;
        float g0 = 0.0f;
        float g1 = 0.0f;
        for (size_t index = 0U; index < solver->anchor_count; ++index) {
            if ((range_mask & (uint16_t)(1U << index)) == 0U) {
                continue;
            }
            const float dx = x_m - solver->anchor_x_m[index];
            const float dy = y_m - solver->anchor_y_m[index];
            const float predicted = hypotf(dx, dy);
            if (predicted < 1.0e-5f) {
                return false;
            }
            const float residual = solver->tag_range_m[index] - predicted;
            const float jx = dx / predicted;
            const float jy = dy / predicted;
            h00 += jx * jx;
            h01 += jx * jy;
            h11 += jy * jy;
            g0 += jx * residual;
            g1 += jy * residual;
        }
        const float determinant = h00 * h11 - h01 * h01;
        if (fabsf(determinant) < 1.0e-8f) {
            return false;
        }
        const float dx = (h11 * g0 - h01 * g1) / determinant;
        const float dy = (h00 * g1 - h01 * g0) / determinant;
        x_m += dx;
        y_m += dy;
        if (!isfinite(x_m) || !isfinite(y_m)) {
            return false;
        }
        if (hypotf(dx, dy) < 0.0001f) {
            ++iterations;
            break;
        }
    }

    float sse = 0.0f;
    float h00 = 0.0f;
    float h01 = 0.0f;
    float h11 = 0.0f;
    float maximum_abs_residual = 0.0f;
    for (size_t index = 0U; index < solver->anchor_count; ++index) {
        if ((range_mask & (uint16_t)(1U << index)) == 0U) {
            continue;
        }
        const float dx = x_m - solver->anchor_x_m[index];
        const float dy = y_m - solver->anchor_y_m[index];
        const float predicted = hypotf(dx, dy);
        const float residual = solver->tag_range_m[index] - predicted;
        const float safe_predicted = fmaxf(predicted, 1.0e-5f);
        const float jx = dx / safe_predicted;
        const float jy = dy / safe_predicted;
        sse += residual * residual;
        maximum_abs_residual =
            fmaxf(maximum_abs_residual, fabsf(residual));
        h00 += jx * jx;
        h01 += jx * jy;
        h11 += jy * jy;
    }

    float sigma = 0.0f;
    const float determinant = h00 * h11 - h01 * h01;
    if (determinant > 1.0e-8f) {
        const float degrees_of_freedom =
            (float)(observation_count > 2U ? observation_count - 2U : 1U);
        const float variance = sse / degrees_of_freedom;
        const float cov00 = variance * h11 / determinant;
        const float cov01 = -variance * h01 / determinant;
        const float cov11 = variance * h00 / determinant;
        const float eigen_term = sqrtf(fmaxf(
            0.0f, (cov00 - cov11) * (cov00 - cov11) +
                      4.0f * cov01 * cov01));
        sigma = sqrtf(fmaxf(0.0f,
                            0.5f * (cov00 + cov11 + eigen_term)));
    }

    const float rms_m = sqrtf(sse / (float)observation_count);
    if (apply_quality_gate &&
        (rms_m > NATIVE_DS_MAX_EQUATION_RMS_M ||
         maximum_abs_residual > NATIVE_DS_MAX_ABS_RESIDUAL_M)) {
        return false;
    }

    output->position_valid = true;
    output->frame_id = solver->tag_frame_id;
    output->x_m = x_m;
    output->y_m = y_m;
    output->sigma_m = sigma;
    output->rms_m = rms_m;
    output->observation_count = (uint8_t)observation_count;
    output->iteration_count = iterations;
    return true;
}

bool uwb_native_ds_position_solver_submit_anchor_range(
    struct uwb_native_ds_position_solver *solver, uint8_t initiator_id,
    uint8_t responder_id, uint32_t frame_id, float distance_m,
    struct uwb_native_ds_position_output *output)
{
    (void)frame_id;
    if (solver == NULL || output == NULL || !isfinite(distance_m) ||
        distance_m < NATIVE_DS_MIN_RANGE_M ||
        distance_m > NATIVE_DS_MAX_RANGE_M) {
        return false;
    }
    initialize_output(solver, output);
    int first = anchor_index(solver, initiator_id);
    int second = anchor_index(solver, responder_id);
    if (first < 0 || second < 0 || first == second) {
        return false;
    }
    if (first > second) {
        const int temporary = first;
        first = second;
        second = temporary;
    }
    solver->pair_valid[first][second] = true;
    solver->pair_valid[second][first] = true;
    solver->pair_range_m[first][second] = distance_m;
    solver->pair_range_m[second][first] = distance_m;
    solver->pair_cycle_mask |=
        1ULL << pair_index(solver->anchor_count, (size_t)first,
                           (size_t)second);

    const uint64_t complete = complete_pair_mask(solver->anchor_count);
    if ((solver->pair_cycle_mask & complete) == complete) {
        float sse = 0.0f;
        size_t count = 0U;
        for (size_t left = 0U; left < solver->anchor_count; ++left) {
            for (size_t right = left + 1U; right < solver->anchor_count;
                 ++right) {
                const float expected = hypotf(
                    solver->anchor_x_m[left] - solver->anchor_x_m[right],
                    solver->anchor_y_m[left] - solver->anchor_y_m[right]);
                const float residual =
                    solver->pair_range_m[left][right] - expected;
                sse += residual * residual;
                ++count;
            }
        }
        solver->pair_cycle_mask = 0U;
        output->geometry_updated = true;
        output->geometry_fit_rms_m =
            count == 0U ? 0.0f : sqrtf(sse / (float)count);
        copy_geometry(solver, output);
    }
    return true;
}

bool uwb_native_ds_position_solver_submit_tag_range(
    struct uwb_native_ds_position_solver *solver, uint8_t tag_id,
    uint8_t anchor_id, uint32_t frame_id, float distance_m,
    struct uwb_native_ds_position_output *output)
{
    if (solver == NULL || output == NULL || tag_id != solver->tag_id ||
        !isfinite(distance_m) || distance_m < NATIVE_DS_MIN_RANGE_M ||
        distance_m > NATIVE_DS_MAX_RANGE_M) {
        return false;
    }
    initialize_output(solver, output);
    const int index = anchor_index(solver, anchor_id);
    if (index < 0) {
        return false;
    }

    if (!solver->tag_frame_active) {
        solver->tag_frame_active = true;
        solver->tag_frame_id = frame_id;
        solver->tag_frame_finalized = false;
        solver->tag_range_mask = 0U;
    } else if (frame_id != solver->tag_frame_id) {
        if (!frame_is_newer(frame_id, solver->tag_frame_id)) {
            return true;
        }
        if (!solver->tag_frame_finalized &&
            range_count(solver->tag_range_mask) >= 3U) {
            solver->tag_frame_finalized = true;
            (void)solve_position(solver, solver->tag_range_mask, true,
                                 output);
        }
        solver->tag_frame_id = frame_id;
        solver->tag_frame_finalized = false;
        solver->tag_range_mask = 0U;
    }

    solver->tag_range_m[index] = distance_m;
    solver->tag_range_mask |= (uint16_t)(1U << index);
    if (solver->tag_frame_finalized ||
        solver->tag_range_mask != complete_anchor_mask(solver->anchor_count)) {
        return true;
    }
    solver->tag_frame_finalized = true;
    /*
     * A complete frame is not automatically a coherent frame. A damaged but
     * syntactically valid range can still make all four bits arrive and used
     * to bypass both the triangle-inequality check and the residual gate.
     * In field captures that produced isolated 1.8--2.9 m position jumps
     * whose own equation RMS was 1.3--2.2 m. Apply the same physical and
     * residual validation used by the 3/4 boundary fallback.
     */
    (void)solve_position(solver, solver->tag_range_mask, true, output);
    return true;
}
