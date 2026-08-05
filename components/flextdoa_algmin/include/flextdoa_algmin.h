#ifndef FLEXTDOA_ALGMIN_H
#define FLEXTDOA_ALGMIN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct flextdoa_anchor_position {
    uint16_t anchor_id;
    double x_m;
    double y_m;
};

struct flextdoa_range_difference {
    uint16_t initiator_id;
    uint16_t responder_id;
    /* Paper Eq. (14): distance(tag, responder) - distance(tag, initiator). */
    double range_difference_m;
};

struct flextdoa_algmin_seed {
    bool valid;
    double x_m;
    double y_m;
};

struct flextdoa_algmin_result {
    bool valid;
    double x_m;
    double y_m;
    double residual_rms_m;
    uint8_t iterations;
};

enum flextdoa_solution_gate_result {
    FLEXTDOA_SOLUTION_GATE_OK = 0,
    FLEXTDOA_SOLUTION_GATE_INVALID_ARGUMENT,
    FLEXTDOA_SOLUTION_GATE_TOO_FEW_INITIATORS,
    FLEXTDOA_SOLUTION_GATE_DISCONNECTED,
    FLEXTDOA_SOLUTION_GATE_RANK_DEFICIENT,
    FLEXTDOA_SOLUTION_GATE_ILL_CONDITIONED,
};

struct flextdoa_solution_gate_metrics {
    size_t initiator_count;
    size_t connected_anchor_count;
    double normal_lambda_min;
    double normal_lambda_max;
    double normal_condition_number;
};

bool flextdoa_algmin_solve_2d(
    const struct flextdoa_anchor_position *anchors, size_t anchor_count,
    const struct flextdoa_range_difference *observations,
    size_t observation_count, const struct flextdoa_algmin_seed *seed,
    struct flextdoa_algmin_result *result);

/* Rejects partial frames whose observation graph or local 2-D Jacobian cannot
 * constrain a stable solution. This is a geometry gate, not a time filter. */
enum flextdoa_solution_gate_result flextdoa_algmin_gate_solution_2d(
    const struct flextdoa_anchor_position *anchors, size_t anchor_count,
    const struct flextdoa_range_difference *observations,
    size_t observation_count, double x_m, double y_m,
    size_t minimum_initiator_count, double maximum_condition_number,
    struct flextdoa_solution_gate_metrics *metrics);

#endif
