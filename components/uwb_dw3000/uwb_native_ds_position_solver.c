#include "uwb_native_ds_position_solver.h"

#include <math.h>
#include <string.h>

#define NATIVE_DS_GEOMETRY_MAX_VARIABLES \
    (2U * UWB_NATIVE_DS_POSITION_MAX_ANCHORS - 3U)
#define NATIVE_DS_GEOMETRY_ITERATIONS 5U
#define NATIVE_DS_POSITION_ITERATIONS 5U
#define NATIVE_DS_MIN_RANGE_M 0.02f
#define NATIVE_DS_MAX_RANGE_M 100.0f

static int anchor_index(const struct uwb_native_ds_position_solver *solver,
                        uint8_t anchor_id)
{
    for (size_t index = 0; index < solver->anchor_count; ++index) {
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

static int geometry_variable_index(size_t anchor, bool y_axis)
{
    if (anchor == 0U || (anchor == 1U && !y_axis)) {
        return -1;
    }
    if (anchor == 1U) {
        return 0;
    }
    return 1 + (int)(2U * (anchor - 2U)) + (y_axis ? 1 : 0);
}

static bool solve_linear(
    float matrix[NATIVE_DS_GEOMETRY_MAX_VARIABLES]
                [NATIVE_DS_GEOMETRY_MAX_VARIABLES + 1U],
    size_t count, float *solution)
{
    for (size_t column = 0U; column < count; ++column) {
        size_t pivot = column;
        float pivot_abs = fabsf(matrix[pivot][column]);
        for (size_t row = column + 1U; row < count; ++row) {
            const float candidate = fabsf(matrix[row][column]);
            if (candidate > pivot_abs) {
                pivot = row;
                pivot_abs = candidate;
            }
        }
        if (pivot_abs < 1.0e-8f) {
            return false;
        }
        if (pivot != column) {
            for (size_t item = column; item <= count; ++item) {
                const float temporary = matrix[column][item];
                matrix[column][item] = matrix[pivot][item];
                matrix[pivot][item] = temporary;
            }
        }
        const float divisor = matrix[column][column];
        for (size_t item = column; item <= count; ++item) {
            matrix[column][item] /= divisor;
        }
        for (size_t row = 0U; row < count; ++row) {
            if (row == column) {
                continue;
            }
            const float factor = matrix[row][column];
            for (size_t item = column; item <= count; ++item) {
                matrix[row][item] -= factor * matrix[column][item];
            }
        }
    }
    for (size_t index = 0U; index < count; ++index) {
        solution[index] = matrix[index][count];
    }
    return true;
}

static bool initialize_geometry(struct uwb_native_ds_position_solver *solver,
                                float *x_m, float *y_m)
{
    const float baseline = solver->pair_range_m[0][1];
    if (baseline <= NATIVE_DS_MIN_RANGE_M) {
        return false;
    }
    x_m[0] = 0.0f;
    y_m[0] = 0.0f;
    x_m[1] = 0.0f;
    y_m[1] = baseline;
    for (size_t anchor = 2U; anchor < solver->anchor_count; ++anchor) {
        const float d0 = solver->pair_range_m[0][anchor];
        const float d1 = solver->pair_range_m[1][anchor];
        const float y = (d0 * d0 + baseline * baseline - d1 * d1) /
                        (2.0f * baseline);
        const float x_square = d0 * d0 - y * y;
        if (x_square < -0.05f) {
            return false;
        }
        float x = sqrtf(fmaxf(0.0f, x_square));
        if (anchor > 2U) {
            float positive_error = 0.0f;
            float negative_error = 0.0f;
            for (size_t known = 2U; known < anchor; ++known) {
                const float measured = solver->pair_range_m[known][anchor];
                const float dy = y - y_m[known];
                positive_error +=
                    fabsf(hypotf(x - x_m[known], dy) - measured);
                negative_error +=
                    fabsf(hypotf(-x - x_m[known], dy) - measured);
            }
            if (negative_error < positive_error) {
                x = -x;
            }
        }
        x_m[anchor] = x;
        y_m[anchor] = y;
    }
    return true;
}

static bool update_geometry(struct uwb_native_ds_position_solver *solver,
                            struct uwb_native_ds_position_output *output)
{
    float x_m[UWB_NATIVE_DS_POSITION_MAX_ANCHORS] = {0};
    float y_m[UWB_NATIVE_DS_POSITION_MAX_ANCHORS] = {0};
    if (solver->geometry_ready) {
        memcpy(x_m, solver->anchor_x_m, sizeof(x_m));
        memcpy(y_m, solver->anchor_y_m, sizeof(y_m));
    } else if (!initialize_geometry(solver, x_m, y_m)) {
        return false;
    }

    const size_t variable_count = 2U * solver->anchor_count - 3U;
    for (size_t iteration = 0U;
         iteration < NATIVE_DS_GEOMETRY_ITERATIONS; ++iteration) {
        float normal[NATIVE_DS_GEOMETRY_MAX_VARIABLES]
                    [NATIVE_DS_GEOMETRY_MAX_VARIABLES + 1U] = {{0}};
        for (size_t first = 0U; first < solver->anchor_count; ++first) {
            for (size_t second = first + 1U;
                 second < solver->anchor_count; ++second) {
                const float dx = x_m[first] - x_m[second];
                const float dy = y_m[first] - y_m[second];
                const float predicted = hypotf(dx, dy);
                if (predicted < 1.0e-5f) {
                    return false;
                }
                const float residual =
                    solver->pair_range_m[first][second] - predicted;
                float jacobian[NATIVE_DS_GEOMETRY_MAX_VARIABLES] = {0};
                const int first_x = geometry_variable_index(first, false);
                const int first_y = geometry_variable_index(first, true);
                const int second_x = geometry_variable_index(second, false);
                const int second_y = geometry_variable_index(second, true);
                if (first_x >= 0) jacobian[first_x] = dx / predicted;
                if (first_y >= 0) jacobian[first_y] = dy / predicted;
                if (second_x >= 0) jacobian[second_x] = -dx / predicted;
                if (second_y >= 0) jacobian[second_y] = -dy / predicted;
                for (size_t row = 0U; row < variable_count; ++row) {
                    normal[row][variable_count] += jacobian[row] * residual;
                    for (size_t column = 0U; column < variable_count;
                         ++column) {
                        normal[row][column] +=
                            jacobian[row] * jacobian[column];
                    }
                }
            }
        }
        for (size_t index = 0U; index < variable_count; ++index) {
            normal[index][index] += 1.0e-6f;
        }
        float delta[NATIVE_DS_GEOMETRY_MAX_VARIABLES] = {0};
        if (!solve_linear(normal, variable_count, delta)) {
            return false;
        }
        float max_delta = 0.0f;
        for (size_t anchor = 1U; anchor < solver->anchor_count; ++anchor) {
            const int x_index = geometry_variable_index(anchor, false);
            const int y_index = geometry_variable_index(anchor, true);
            if (x_index >= 0) {
                x_m[anchor] += delta[x_index];
                max_delta = fmaxf(max_delta, fabsf(delta[x_index]));
            }
            if (y_index >= 0) {
                y_m[anchor] += delta[y_index];
                max_delta = fmaxf(max_delta, fabsf(delta[y_index]));
            }
        }
        if (max_delta < 0.0001f) {
            break;
        }
    }

    float fit_sse = 0.0f;
    size_t fit_count = 0U;
    for (size_t first = 0U; first < solver->anchor_count; ++first) {
        for (size_t second = first + 1U;
             second < solver->anchor_count; ++second) {
            const float predicted = hypotf(
                x_m[first] - x_m[second], y_m[first] - y_m[second]);
            const float residual =
                solver->pair_range_m[first][second] - predicted;
            fit_sse += residual * residual;
            ++fit_count;
        }
    }
    if (fit_count == 0U) {
        return false;
    }
    memcpy(solver->anchor_x_m, x_m, sizeof(solver->anchor_x_m));
    memcpy(solver->anchor_y_m, y_m, sizeof(solver->anchor_y_m));
    solver->geometry_fit_rms_m = sqrtf(fit_sse / (float)fit_count);
    solver->geometry_ready = true;
    ++solver->geometry_version;
    output->geometry_updated = true;
    output->geometry_version = solver->geometry_version;
    output->geometry_fit_rms_m = solver->geometry_fit_rms_m;
    memcpy(output->anchor_x_m, solver->anchor_x_m,
           sizeof(output->anchor_x_m));
    memcpy(output->anchor_y_m, solver->anchor_y_m,
           sizeof(output->anchor_y_m));
    return true;
}

static bool solve_position(struct uwb_native_ds_position_solver *solver,
                           uint16_t frame_id,
                           struct uwb_native_ds_position_output *output)
{
    float x_m = solver->position_valid ? solver->position_x_m : 0.0f;
    float y_m = solver->position_valid ? solver->position_y_m : 0.0f;
    if (!solver->position_valid) {
        for (size_t index = 0U; index < solver->anchor_count; ++index) {
            x_m += solver->anchor_x_m[index];
            y_m += solver->anchor_y_m[index];
        }
        x_m /= (float)solver->anchor_count;
        y_m /= (float)solver->anchor_count;
    }

    uint8_t iterations = 0U;
    for (; iterations < NATIVE_DS_POSITION_ITERATIONS; ++iterations) {
        float h00 = 1.0e-6f;
        float h01 = 0.0f;
        float h11 = 1.0e-6f;
        float g0 = 0.0f;
        float g1 = 0.0f;
        for (size_t index = 0U; index < solver->anchor_count; ++index) {
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
        if (hypotf(dx, dy) < 0.0001f) {
            ++iterations;
            break;
        }
    }

    float sse = 0.0f;
    float h00 = 0.0f;
    float h01 = 0.0f;
    float h11 = 0.0f;
    for (size_t index = 0U; index < solver->anchor_count; ++index) {
        const float dx = x_m - solver->anchor_x_m[index];
        const float dy = y_m - solver->anchor_y_m[index];
        const float predicted = hypotf(dx, dy);
        const float residual = solver->tag_range_m[index] - predicted;
        const float jx = dx / fmaxf(predicted, 1.0e-5f);
        const float jy = dy / fmaxf(predicted, 1.0e-5f);
        sse += residual * residual;
        h00 += jx * jx;
        h01 += jx * jy;
        h11 += jy * jy;
    }
    const float determinant = h00 * h11 - h01 * h01;
    float sigma = 0.0f;
    if (determinant > 1.0e-8f) {
        const float variance =
            sse / (float)(solver->anchor_count > 2U
                              ? solver->anchor_count - 2U
                              : 1U);
        const float cov00 = variance * h11 / determinant;
        const float cov01 = -variance * h01 / determinant;
        const float cov11 = variance * h00 / determinant;
        const float eigen_term = sqrtf(fmaxf(
            0.0f, (cov00 - cov11) * (cov00 - cov11) +
                      4.0f * cov01 * cov01));
        sigma = sqrtf(fmaxf(0.0f,
                            0.5f * (cov00 + cov11 + eigen_term)));
    }
    solver->position_valid = true;
    solver->last_position_frame_id = frame_id;
    solver->position_x_m = x_m;
    solver->position_y_m = y_m;
    output->position_valid = true;
    output->frame_id = frame_id;
    output->geometry_version = solver->geometry_version;
    output->x_m = x_m;
    output->y_m = y_m;
    output->sigma_m = sigma;
    output->rms_m = sqrtf(sse / (float)solver->anchor_count);
    output->observation_count = solver->anchor_count;
    output->iteration_count = iterations;
    return true;
}

bool uwb_native_ds_position_solver_init(
    struct uwb_native_ds_position_solver *solver, uint8_t tag_id,
    const uint8_t *anchor_ids, size_t anchor_count)
{
    if (solver == NULL || anchor_ids == NULL || tag_id == 0U ||
        anchor_count < 3U ||
        anchor_count > UWB_NATIVE_DS_POSITION_MAX_ANCHORS) {
        return false;
    }
    memset(solver, 0, sizeof(*solver));
    solver->tag_id = tag_id;
    solver->anchor_count = (uint8_t)anchor_count;
    memcpy(solver->anchor_ids, anchor_ids, anchor_count);
    for (size_t first = 0U; first < anchor_count; ++first) {
        if (anchor_ids[first] == 0U || anchor_ids[first] == tag_id) {
            return false;
        }
        for (size_t second = first + 1U; second < anchor_count; ++second) {
            if (anchor_ids[first] == anchor_ids[second]) {
                return false;
            }
        }
    }
    return true;
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

bool uwb_native_ds_position_solver_submit_anchor_range(
    struct uwb_native_ds_position_solver *solver, uint8_t initiator_id,
    uint8_t responder_id, uint16_t frame_id, float distance_m,
    struct uwb_native_ds_position_output *output)
{
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
    if (solver->pair_valid[first][second] &&
        solver->pair_frame_id[first][second] == frame_id) {
        return true;
    }
    solver->pair_valid[first][second] = true;
    solver->pair_valid[second][first] = true;
    solver->pair_range_m[first][second] = distance_m;
    solver->pair_range_m[second][first] = distance_m;
    solver->pair_frame_id[first][second] = frame_id;
    solver->pair_frame_id[second][first] = frame_id;
    solver->pair_cycle_mask |=
        1ULL << pair_index(solver->anchor_count, (size_t)first,
                           (size_t)second);
    const uint64_t complete = complete_pair_mask(solver->anchor_count);
    if ((solver->pair_cycle_mask & complete) == complete) {
        solver->pair_cycle_mask = 0U;
        (void)update_geometry(solver, output);
    }
    return true;
}

bool uwb_native_ds_position_solver_submit_tag_range(
    struct uwb_native_ds_position_solver *solver, uint8_t tag_id,
    uint8_t anchor_id, uint16_t frame_id, float distance_m,
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
    solver->tag_range_valid[index] = true;
    solver->tag_range_m[index] = distance_m;
    solver->tag_frame_id[index] = frame_id;
    if (!solver->geometry_ready ||
        (solver->position_valid && solver->last_position_frame_id == frame_id)) {
        return true;
    }
    for (size_t anchor = 0U; anchor < solver->anchor_count; ++anchor) {
        if (!solver->tag_range_valid[anchor] ||
            solver->tag_frame_id[anchor] != frame_id) {
            return true;
        }
    }
    (void)solve_position(solver, frame_id, output);
    return true;
}
