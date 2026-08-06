#ifndef PASSIVE_DS_DYNAMIC_GEOMETRY_H
#define PASSIVE_DS_DYNAMIC_GEOMETRY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "passive_ds_position_solver.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PASSIVE_DS_DYNAMIC_POSITION_VALID (1U << 0U)
#define PASSIVE_DS_DYNAMIC_POSITION_RTK_FIXED (1U << 1U)
#define PASSIVE_DS_DYNAMIC_POSITION_VELOCITY_VALID (1U << 2U)

struct passive_ds_dynamic_position_sample {
    uint8_t flags;
    uint16_t age_ms;
    int32_t latitude_e7;
    int32_t longitude_e7;
    int16_t velocity_east_mmps;
    int16_t velocity_north_mmps;
};

struct passive_ds_dynamic_anchor_state {
    bool valid;
    bool rtk_fixed;
    uint8_t id;
    double east_m;
    double north_m;
    double velocity_east_mps;
    double velocity_north_mps;
};

struct passive_ds_dynamic_geometry {
    bool origin_valid;
    bool active;
    uint8_t anchor_count;
    double origin_latitude_rad;
    double origin_longitude_rad;
    double rotation_cos;
    double rotation_sin;
    double translation_x_m;
    double translation_y_m;
    double fit_rms_m;
    struct passive_ds_position_anchor fixed[
        PASSIVE_DS_POSITION_MAX_ANCHORS];
    struct passive_ds_dynamic_anchor_state latest[
        PASSIVE_DS_POSITION_MAX_ANCHORS];
};

void passive_ds_dynamic_geometry_init(
    struct passive_ds_dynamic_geometry *geometry,
    const struct passive_ds_position_anchor *fixed,
    size_t anchor_count);

/* Returns true only on the transition from surveyed fallback to mobile mode. */
bool passive_ds_dynamic_geometry_ingest(
    struct passive_ds_dynamic_geometry *geometry, uint8_t anchor_id,
    const struct passive_ds_dynamic_position_sample *sample);

bool passive_ds_dynamic_geometry_project(
    const struct passive_ds_dynamic_geometry *geometry,
    const struct passive_ds_dynamic_position_sample *sample,
    double *x_m, double *y_m);

bool passive_ds_dynamic_geometry_latest(
    const struct passive_ds_dynamic_geometry *geometry, uint8_t anchor_id,
    double *x_m, double *y_m);

bool passive_ds_dynamic_geometry_all_rtk_fixed(
    const struct passive_ds_dynamic_geometry *geometry);

#ifdef __cplusplus
}
#endif

#endif /* PASSIVE_DS_DYNAMIC_GEOMETRY_H */
