#include "passive_ds_dynamic_geometry.h"

#include <math.h>
#include <string.h>

#define PASSIVE_DS_EARTH_RADIUS_M 6378137.0
#define PASSIVE_DS_DEG_TO_RAD 0.01745329251994329577
#define PASSIVE_DS_DYNAMIC_MAX_INITIAL_FIT_RMS_M 0.15
#define PASSIVE_DS_DYNAMIC_BASE_JUMP_M 0.25
#define PASSIVE_DS_DYNAMIC_MOTION_HORIZON_S 0.50

static int anchor_index(const struct passive_ds_dynamic_geometry *geometry,
                        uint8_t anchor_id)
{
    if (geometry == NULL || anchor_id == 0U) {
        return -1;
    }
    for (uint8_t index = 0U; index < geometry->anchor_count; ++index) {
        if (geometry->fixed[index].id == anchor_id) {
            return (int)index;
        }
    }
    return -1;
}

static bool sample_coordinate_valid(
    const struct passive_ds_dynamic_position_sample *sample)
{
    return sample != NULL &&
           (sample->flags & PASSIVE_DS_DYNAMIC_POSITION_VALID) != 0U &&
           sample->age_ms <= 1000U &&
           sample->latitude_e7 >= -900000000 &&
           sample->latitude_e7 <= 900000000 &&
           sample->longitude_e7 >= -1800000000 &&
           sample->longitude_e7 <= 1800000000;
}

static bool sample_rtk_fixed(
    const struct passive_ds_dynamic_position_sample *sample)
{
    return sample_coordinate_valid(sample) &&
           (sample->flags &
            PASSIVE_DS_DYNAMIC_POSITION_RTK_FIXED) != 0U;
}

static void raw_position(
    const struct passive_ds_dynamic_geometry *geometry,
    const struct passive_ds_dynamic_position_sample *sample,
    double *east_m, double *north_m,
    double *velocity_east_mps, double *velocity_north_mps)
{
    const double latitude_rad =
        sample->latitude_e7 * 1.0e-7 * PASSIVE_DS_DEG_TO_RAD;
    const double longitude_rad =
        sample->longitude_e7 * 1.0e-7 * PASSIVE_DS_DEG_TO_RAD;
    double east = (longitude_rad - geometry->origin_longitude_rad) *
                  PASSIVE_DS_EARTH_RADIUS_M *
                  cos(0.5 * (latitude_rad +
                             geometry->origin_latitude_rad));
    double north = (latitude_rad - geometry->origin_latitude_rad) *
                   PASSIVE_DS_EARTH_RADIUS_M;
    double velocity_east = 0.0;
    double velocity_north = 0.0;
    if ((sample->flags &
         PASSIVE_DS_DYNAMIC_POSITION_VELOCITY_VALID) != 0U) {
        velocity_east = sample->velocity_east_mmps / 1000.0;
        velocity_north = sample->velocity_north_mmps / 1000.0;
        const double age_s = sample->age_ms / 1000.0;
        east += velocity_east * age_s;
        north += velocity_north * age_s;
    }
    *east_m = east;
    *north_m = north;
    *velocity_east_mps = velocity_east;
    *velocity_north_mps = velocity_north;
}

static bool try_activate(struct passive_ds_dynamic_geometry *geometry)
{
    double raw_center_east = 0.0;
    double raw_center_north = 0.0;
    double fixed_center_x = 0.0;
    double fixed_center_y = 0.0;
    for (uint8_t index = 0U; index < geometry->anchor_count; ++index) {
        if (!geometry->latest[index].valid) {
            return false;
        }
        raw_center_east += geometry->latest[index].east_m;
        raw_center_north += geometry->latest[index].north_m;
        fixed_center_x += geometry->fixed[index].x_m;
        fixed_center_y += geometry->fixed[index].y_m;
    }
    const double divisor = geometry->anchor_count;
    raw_center_east /= divisor;
    raw_center_north /= divisor;
    fixed_center_x /= divisor;
    fixed_center_y /= divisor;

    double dot = 0.0;
    double cross = 0.0;
    double raw_energy = 0.0;
    double fixed_energy = 0.0;
    for (uint8_t index = 0U; index < geometry->anchor_count; ++index) {
        const double east =
            geometry->latest[index].east_m - raw_center_east;
        const double north =
            geometry->latest[index].north_m - raw_center_north;
        const double x = geometry->fixed[index].x_m - fixed_center_x;
        const double y = geometry->fixed[index].y_m - fixed_center_y;
        dot += east * x + north * y;
        cross += east * y - north * x;
        raw_energy += east * east + north * north;
        fixed_energy += x * x + y * y;
    }
    if (raw_energy < 0.25 || fixed_energy < 0.25 ||
        hypot(dot, cross) < 1.0e-9) {
        return false;
    }
    const double angle = atan2(cross, dot);
    const double rotation_cos = cos(angle);
    const double rotation_sin = sin(angle);
    const double translation_x = fixed_center_x -
        (rotation_cos * raw_center_east -
         rotation_sin * raw_center_north);
    const double translation_y = fixed_center_y -
        (rotation_sin * raw_center_east +
         rotation_cos * raw_center_north);
    double squared_error = 0.0;
    for (uint8_t index = 0U; index < geometry->anchor_count; ++index) {
        const double x = rotation_cos * geometry->latest[index].east_m -
                         rotation_sin * geometry->latest[index].north_m +
                         translation_x;
        const double y = rotation_sin * geometry->latest[index].east_m +
                         rotation_cos * geometry->latest[index].north_m +
                         translation_y;
        const double dx = x - geometry->fixed[index].x_m;
        const double dy = y - geometry->fixed[index].y_m;
        squared_error += dx * dx + dy * dy;
    }
    const double fit_rms = sqrt(squared_error / divisor);
    if (!isfinite(fit_rms) ||
        fit_rms > PASSIVE_DS_DYNAMIC_MAX_INITIAL_FIT_RMS_M) {
        return false;
    }
    geometry->rotation_cos = rotation_cos;
    geometry->rotation_sin = rotation_sin;
    geometry->translation_x_m = translation_x;
    geometry->translation_y_m = translation_y;
    geometry->fit_rms_m = fit_rms;
    geometry->active = true;
    return true;
}

void passive_ds_dynamic_geometry_init(
    struct passive_ds_dynamic_geometry *geometry,
    const struct passive_ds_position_anchor *fixed,
    size_t anchor_count)
{
    if (geometry == NULL) {
        return;
    }
    memset(geometry, 0, sizeof(*geometry));
    if (fixed == NULL || anchor_count < 3U ||
        anchor_count > PASSIVE_DS_POSITION_MAX_ANCHORS) {
        return;
    }
    geometry->anchor_count = (uint8_t)anchor_count;
    memcpy(geometry->fixed, fixed,
           anchor_count * sizeof(geometry->fixed[0]));
    for (uint8_t index = 0U; index < geometry->anchor_count; ++index) {
        geometry->latest[index].id = fixed[index].id;
    }
}

bool passive_ds_dynamic_geometry_ingest(
    struct passive_ds_dynamic_geometry *geometry, uint8_t anchor_id,
    const struct passive_ds_dynamic_position_sample *sample)
{
    const int index = anchor_index(geometry, anchor_id);
    if (index < 0 || !sample_coordinate_valid(sample) ||
        (!geometry->active && !sample_rtk_fixed(sample))) {
        return false;
    }
    if (!geometry->origin_valid) {
        geometry->origin_latitude_rad =
            sample->latitude_e7 * 1.0e-7 * PASSIVE_DS_DEG_TO_RAD;
        geometry->origin_longitude_rad =
            sample->longitude_e7 * 1.0e-7 * PASSIVE_DS_DEG_TO_RAD;
        geometry->origin_valid = true;
    }
    struct passive_ds_dynamic_anchor_state *latest =
        &geometry->latest[index];
    if (geometry->active && !sample_rtk_fixed(sample)) {
        /*
         * RTK Float/SPS coordinates can jump by enough to make an otherwise
         * coherent Passive DS star physically inconsistent.  Keep the last
         * RTK Fixed coordinate as the degraded-continuity hold point while
         * still exposing the current quality through all_rtk_fixed().
         */
        latest->rtk_fixed = false;
        return false;
    }
    double east_m = 0.0;
    double north_m = 0.0;
    double velocity_east_mps = 0.0;
    double velocity_north_mps = 0.0;
    raw_position(geometry, sample, &east_m, &north_m,
                 &velocity_east_mps, &velocity_north_mps);
    if (geometry->active && latest->valid) {
        const double displacement_m = hypot(
            east_m - latest->east_m, north_m - latest->north_m);
        const double previous_speed_mps = hypot(
            latest->velocity_east_mps,
            latest->velocity_north_mps);
        const double candidate_speed_mps = hypot(
            velocity_east_mps, velocity_north_mps);
        const double allowed_displacement_m =
            PASSIVE_DS_DYNAMIC_BASE_JUMP_M +
            fmax(previous_speed_mps, candidate_speed_mps) *
                PASSIVE_DS_DYNAMIC_MOTION_HORIZON_S;
        if (!isfinite(displacement_m) ||
            displacement_m > allowed_displacement_m) {
            /*
             * An RTK receiver can briefly label an ambiguity jump as Fixed.
             * Holding the last plausible Fixed coordinate is safer than
             * poisoning every TDOA equation with a metre-scale zero-speed
             * anchor jump.  Normal continuous motion remains below the base
             * step, while faster motion is admitted by the RMC velocity.
             */
            latest->rtk_fixed = false;
            return false;
        }
    }
    latest->east_m = east_m;
    latest->north_m = north_m;
    latest->velocity_east_mps = velocity_east_mps;
    latest->velocity_north_mps = velocity_north_mps;
    latest->valid = true;
    latest->rtk_fixed = sample_rtk_fixed(sample);
    const bool was_active = geometry->active;
    if (!geometry->active) {
        (void)try_activate(geometry);
    }
    return !was_active && geometry->active;
}

bool passive_ds_dynamic_geometry_project(
    const struct passive_ds_dynamic_geometry *geometry,
    const struct passive_ds_dynamic_position_sample *sample,
    double *x_m, double *y_m)
{
    if (geometry == NULL || !geometry->active ||
        !sample_coordinate_valid(sample) || x_m == NULL || y_m == NULL) {
        return false;
    }
    double east = 0.0;
    double north = 0.0;
    double velocity_east = 0.0;
    double velocity_north = 0.0;
    raw_position(geometry, sample, &east, &north,
                 &velocity_east, &velocity_north);
    *x_m = geometry->rotation_cos * east -
           geometry->rotation_sin * north +
           geometry->translation_x_m;
    *y_m = geometry->rotation_sin * east +
           geometry->rotation_cos * north +
           geometry->translation_y_m;
    return isfinite(*x_m) && isfinite(*y_m);
}

bool passive_ds_dynamic_geometry_latest(
    const struct passive_ds_dynamic_geometry *geometry, uint8_t anchor_id,
    double *x_m, double *y_m)
{
    const int index = anchor_index(geometry, anchor_id);
    if (index < 0 || geometry == NULL || !geometry->active ||
        !geometry->latest[index].valid || x_m == NULL || y_m == NULL) {
        return false;
    }
    const struct passive_ds_dynamic_anchor_state *latest =
        &geometry->latest[index];
    *x_m = geometry->rotation_cos * latest->east_m -
           geometry->rotation_sin * latest->north_m +
           geometry->translation_x_m;
    *y_m = geometry->rotation_sin * latest->east_m +
           geometry->rotation_cos * latest->north_m +
           geometry->translation_y_m;
    return isfinite(*x_m) && isfinite(*y_m);
}

bool passive_ds_dynamic_geometry_all_rtk_fixed(
    const struct passive_ds_dynamic_geometry *geometry)
{
    if (geometry == NULL || !geometry->active) {
        return false;
    }
    for (uint8_t index = 0U; index < geometry->anchor_count; ++index) {
        if (!geometry->latest[index].valid ||
            !geometry->latest[index].rtk_fixed) {
            return false;
        }
    }
    return true;
}
