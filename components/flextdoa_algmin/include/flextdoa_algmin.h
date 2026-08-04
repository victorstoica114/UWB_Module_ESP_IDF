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

bool flextdoa_algmin_solve_2d(
    const struct flextdoa_anchor_position *anchors, size_t anchor_count,
    const struct flextdoa_range_difference *observations,
    size_t observation_count, const struct flextdoa_algmin_seed *seed,
    struct flextdoa_algmin_result *result);

#endif
