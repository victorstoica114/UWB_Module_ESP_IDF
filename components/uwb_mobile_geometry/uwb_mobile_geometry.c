#include "uwb_mobile_geometry.h"

#include <math.h>
#include <string.h>

#define UWB_MOBILE_EARTH_RADIUS_M 6378137.0
#define UWB_MOBILE_DEG_TO_RAD 0.01745329251994329577
#define UWB_MOBILE_MAX_INITIAL_FIT_RMS_M 0.15
#define UWB_MOBILE_MAX_POSITION_AGE_MS 1000U
#define UWB_MOBILE_BASE_JUMP_M 0.25
#define UWB_MOBILE_MOTION_HORIZON_S 0.50

static void put_u16(uint8_t *payload, uint16_t value)
{
    payload[0] = (uint8_t)value;
    payload[1] = (uint8_t)(value >> 8U);
}

static void put_u32(uint8_t *payload, uint32_t value)
{
    payload[0] = (uint8_t)value;
    payload[1] = (uint8_t)(value >> 8U);
    payload[2] = (uint8_t)(value >> 16U);
    payload[3] = (uint8_t)(value >> 24U);
}

static uint16_t get_u16(const uint8_t *payload)
{
    return (uint16_t)payload[0] | (uint16_t)((uint16_t)payload[1] << 8U);
}

static uint32_t get_u32(const uint8_t *payload)
{
    return (uint32_t)payload[0] | ((uint32_t)payload[1] << 8U) |
           ((uint32_t)payload[2] << 16U) |
           ((uint32_t)payload[3] << 24U);
}

bool uwb_mobile_position_valid(const struct uwb_mobile_position *position)
{
    if (position == NULL ||
        (position->flags & ~UWB_MOBILE_POSITION_KNOWN_FLAGS) != 0U) {
        return false;
    }
    if ((position->flags & UWB_MOBILE_POSITION_VALID) == 0U) {
        return position->flags == 0U && position->age_ms == 0U &&
               position->latitude_e7 == 0 && position->longitude_e7 == 0 &&
               position->velocity_east_mmps == 0 &&
               position->velocity_north_mmps == 0;
    }
    return position->latitude_e7 >= -900000000 &&
           position->latitude_e7 <= 900000000 &&
           position->longitude_e7 >= -1800000000 &&
           position->longitude_e7 <= 1800000000;
}

bool uwb_mobile_position_encode(const struct uwb_mobile_position *position,
                                uint8_t *payload, size_t capacity)
{
    if (!uwb_mobile_position_valid(position) || payload == NULL ||
        capacity < UWB_MOBILE_POSITION_WIRE_SIZE) {
        return false;
    }
    payload[0] = position->flags;
    put_u16(&payload[1], position->age_ms);
    put_u32(&payload[3], (uint32_t)position->latitude_e7);
    put_u32(&payload[7], (uint32_t)position->longitude_e7);
    put_u16(&payload[11], (uint16_t)position->velocity_east_mmps);
    put_u16(&payload[13], (uint16_t)position->velocity_north_mmps);
    return true;
}

bool uwb_mobile_position_decode(const uint8_t *payload, size_t payload_len,
                                struct uwb_mobile_position *position)
{
    if (payload == NULL || position == NULL ||
        payload_len < UWB_MOBILE_POSITION_WIRE_SIZE) {
        return false;
    }
    *position = (struct uwb_mobile_position){
        .flags = payload[0],
        .age_ms = get_u16(&payload[1]),
        .latitude_e7 = (int32_t)get_u32(&payload[3]),
        .longitude_e7 = (int32_t)get_u32(&payload[7]),
        .velocity_east_mmps = (int16_t)get_u16(&payload[11]),
        .velocity_north_mmps = (int16_t)get_u16(&payload[13]),
    };
    return uwb_mobile_position_valid(position);
}

static int anchor_index(const struct uwb_mobile_geometry *geometry,
                        uint16_t anchor_id)
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

static bool coordinate_valid(const struct uwb_mobile_position *position)
{
    return uwb_mobile_position_valid(position) &&
           (position->flags & UWB_MOBILE_POSITION_VALID) != 0U &&
           position->age_ms <= UWB_MOBILE_MAX_POSITION_AGE_MS;
}

static bool rtk_fixed(const struct uwb_mobile_position *position)
{
    return coordinate_valid(position) &&
           (position->flags & UWB_MOBILE_POSITION_RTK_FIXED) != 0U;
}

static void raw_position(const struct uwb_mobile_geometry *geometry,
                         const struct uwb_mobile_position *position,
                         double *east_m, double *north_m)
{
    const double latitude_rad =
        position->latitude_e7 * 1.0e-7 * UWB_MOBILE_DEG_TO_RAD;
    const double longitude_rad =
        position->longitude_e7 * 1.0e-7 * UWB_MOBILE_DEG_TO_RAD;
    double east = (longitude_rad - geometry->origin_longitude_rad) *
                  UWB_MOBILE_EARTH_RADIUS_M *
                  cos(0.5 * (latitude_rad + geometry->origin_latitude_rad));
    double north = (latitude_rad - geometry->origin_latitude_rad) *
                   UWB_MOBILE_EARTH_RADIUS_M;
    if ((position->flags & UWB_MOBILE_POSITION_VELOCITY_VALID) != 0U) {
        const double age_s = position->age_ms / 1000.0;
        east += position->velocity_east_mmps / 1000.0 * age_s;
        north += position->velocity_north_mmps / 1000.0 * age_s;
    }
    *east_m = east;
    *north_m = north;
}

static bool try_activate(struct uwb_mobile_geometry *geometry)
{
    double raw_east = 0.0;
    double raw_north = 0.0;
    double fixed_x = 0.0;
    double fixed_y = 0.0;
    for (uint8_t index = 0U; index < geometry->anchor_count; ++index) {
        if (!geometry->latest[index].valid ||
            !geometry->latest[index].rtk_fixed) {
            return false;
        }
        raw_east += geometry->latest[index].east_m;
        raw_north += geometry->latest[index].north_m;
        fixed_x += geometry->fixed[index].x_m;
        fixed_y += geometry->fixed[index].y_m;
    }
    const double count = geometry->anchor_count;
    raw_east /= count;
    raw_north /= count;
    fixed_x /= count;
    fixed_y /= count;

    double dot = 0.0;
    double cross = 0.0;
    double raw_energy = 0.0;
    double fixed_energy = 0.0;
    for (uint8_t index = 0U; index < geometry->anchor_count; ++index) {
        const double east = geometry->latest[index].east_m - raw_east;
        const double north = geometry->latest[index].north_m - raw_north;
        const double x = geometry->fixed[index].x_m - fixed_x;
        const double y = geometry->fixed[index].y_m - fixed_y;
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
    const double translation_x = fixed_x -
        (rotation_cos * raw_east - rotation_sin * raw_north);
    const double translation_y = fixed_y -
        (rotation_sin * raw_east + rotation_cos * raw_north);
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
    const double fit_rms = sqrt(squared_error / count);
    if (!isfinite(fit_rms) || fit_rms > UWB_MOBILE_MAX_INITIAL_FIT_RMS_M) {
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

void uwb_mobile_geometry_init(struct uwb_mobile_geometry *geometry,
                              const struct uwb_mobile_anchor *fixed,
                              size_t anchor_count)
{
    if (geometry == NULL) {
        return;
    }
    memset(geometry, 0, sizeof(*geometry));
    if (fixed == NULL || anchor_count < 3U ||
        anchor_count > UWB_MOBILE_MAX_ANCHORS) {
        return;
    }
    geometry->anchor_count = (uint8_t)anchor_count;
    memcpy(geometry->fixed, fixed,
           anchor_count * sizeof(geometry->fixed[0]));
    for (uint8_t index = 0U; index < geometry->anchor_count; ++index) {
        geometry->latest[index].id = fixed[index].id;
    }
}

bool uwb_mobile_geometry_ingest(struct uwb_mobile_geometry *geometry,
                                uint16_t anchor_id,
                                const struct uwb_mobile_position *position)
{
    const int index = anchor_index(geometry, anchor_id);
    if (index < 0 || !coordinate_valid(position) ||
        (!geometry->active && !rtk_fixed(position))) {
        return false;
    }
    if (!geometry->origin_valid) {
        geometry->origin_latitude_rad =
            position->latitude_e7 * 1.0e-7 * UWB_MOBILE_DEG_TO_RAD;
        geometry->origin_longitude_rad =
            position->longitude_e7 * 1.0e-7 * UWB_MOBILE_DEG_TO_RAD;
        geometry->origin_valid = true;
    }
    struct uwb_mobile_anchor_state *latest = &geometry->latest[index];
    if (geometry->active && !rtk_fixed(position)) {
        latest->rtk_fixed = false;
        return false;
    }
    double east_m = 0.0;
    double north_m = 0.0;
    raw_position(geometry, position, &east_m, &north_m);
    const double velocity_east_mps =
        (position->flags & UWB_MOBILE_POSITION_VELOCITY_VALID) != 0U
            ? position->velocity_east_mmps / 1000.0
            : 0.0;
    const double velocity_north_mps =
        (position->flags & UWB_MOBILE_POSITION_VELOCITY_VALID) != 0U
            ? position->velocity_north_mmps / 1000.0
            : 0.0;
    if (geometry->active && latest->valid) {
        const double displacement_m = hypot(
            east_m - latest->east_m, north_m - latest->north_m);
        const double previous_speed_mps = hypot(
            latest->velocity_east_mps, latest->velocity_north_mps);
        const double candidate_speed_mps = hypot(
            velocity_east_mps, velocity_north_mps);
        const double allowed_displacement_m =
            UWB_MOBILE_BASE_JUMP_M +
            fmax(previous_speed_mps, candidate_speed_mps) *
                UWB_MOBILE_MOTION_HORIZON_S;
        if (!isfinite(displacement_m) ||
            displacement_m > allowed_displacement_m) {
            /*
             * Some receivers briefly label an ambiguity jump as RTK Fixed.
             * Preserve the last plausible coordinate instead of injecting a
             * metre-scale, zero-speed anchor jump into every solver equation.
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
    latest->rtk_fixed = true;
    const bool was_active = geometry->active;
    if (!was_active) {
        (void)try_activate(geometry);
    }
    return !was_active && geometry->active;
}

bool uwb_mobile_geometry_latest(const struct uwb_mobile_geometry *geometry,
                                uint16_t anchor_id, double *x_m,
                                double *y_m)
{
    const int index = anchor_index(geometry, anchor_id);
    if (index < 0 || !geometry->active ||
        !geometry->latest[index].valid || x_m == NULL || y_m == NULL) {
        return false;
    }
    const struct uwb_mobile_anchor_state *latest = &geometry->latest[index];
    *x_m = geometry->rotation_cos * latest->east_m -
           geometry->rotation_sin * latest->north_m +
           geometry->translation_x_m;
    *y_m = geometry->rotation_sin * latest->east_m +
           geometry->rotation_cos * latest->north_m +
           geometry->translation_y_m;
    return isfinite(*x_m) && isfinite(*y_m);
}

bool uwb_mobile_geometry_project_anchor(
    const struct uwb_mobile_geometry *geometry, uint16_t anchor_id,
    const struct uwb_mobile_position *position, double *x_m, double *y_m)
{
    (void)position;
    return uwb_mobile_geometry_latest(geometry, anchor_id, x_m, y_m);
}

bool uwb_mobile_geometry_all_rtk_fixed(
    const struct uwb_mobile_geometry *geometry)
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

uint32_t uwb_mobile_geometry_version(uint32_t sequence)
{
    uint32_t value = sequence & 0x7fffffffU;
    if (value == 0U) {
        value = 1U;
    }
    return 0x80000000U | value;
}
