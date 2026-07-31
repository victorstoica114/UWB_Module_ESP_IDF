#ifndef PASSIVE_DS_POSITION_SOLVER_H
#define PASSIVE_DS_POSITION_SOLVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PASSIVE_DS_POSITION_MAX_ANCHORS 8U

struct passive_ds_position_anchor {
    uint8_t id;
    double x_m;
    double y_m;
};

struct passive_ds_position_observation {
    uint8_t initiator_id;
    uint8_t responder_id;
    double difference_m;
};

struct passive_ds_position_result {
    double x_m;
    double y_m;
    double sigma_m;
    double rms_m;
    uint8_t observation_count;
    uint8_t iteration_count;
};

/*
 * Unweighted AlgMin solve for
 *
 *   measured = distance(tag, responder) - distance(tag, initiator)
 *
 * The previous position is only an optimization seed. It is never averaged
 * with, or otherwise blended into, the returned raw solution.
 */
bool passive_ds_position_solve(
    const struct passive_ds_position_anchor *anchors,
    size_t anchor_count,
    const struct passive_ds_position_observation *observations,
    size_t observation_count,
    bool previous_valid,
    double previous_x_m,
    double previous_y_m,
    struct passive_ds_position_result *result);

#ifdef __cplusplus
}
#endif

#endif /* PASSIVE_DS_POSITION_SOLVER_H */
