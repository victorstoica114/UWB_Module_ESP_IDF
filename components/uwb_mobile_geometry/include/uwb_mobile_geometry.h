#ifndef UWB_MOBILE_GEOMETRY_H
#define UWB_MOBILE_GEOMETRY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UWB_MOBILE_MAX_ANCHORS 16U
#define UWB_MOBILE_POSITION_WIRE_SIZE 15U

#define UWB_MOBILE_POSITION_VALID (1U << 0U)
#define UWB_MOBILE_POSITION_RTK_FIXED (1U << 1U)
#define UWB_MOBILE_POSITION_VELOCITY_VALID (1U << 2U)
#define UWB_MOBILE_POSITION_KNOWN_FLAGS                              \
    (UWB_MOBILE_POSITION_VALID | UWB_MOBILE_POSITION_RTK_FIXED |     \
     UWB_MOBILE_POSITION_VELOCITY_VALID)

struct uwb_mobile_position {
    uint8_t flags;
    uint16_t age_ms;
    int32_t latitude_e7;
    int32_t longitude_e7;
    int16_t velocity_east_mmps;
    int16_t velocity_north_mmps;
};

struct uwb_mobile_anchor {
    uint16_t id;
    double x_m;
    double y_m;
};

struct uwb_mobile_anchor_state {
    bool valid;
    bool rtk_fixed;
    uint16_t id;
    double east_m;
    double north_m;
    double velocity_east_mps;
    double velocity_north_mps;
};

struct uwb_mobile_geometry {
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
    struct uwb_mobile_anchor fixed[UWB_MOBILE_MAX_ANCHORS];
    struct uwb_mobile_anchor_state latest[UWB_MOBILE_MAX_ANCHORS];
};

bool uwb_mobile_position_valid(const struct uwb_mobile_position *position);
bool uwb_mobile_position_encode(const struct uwb_mobile_position *position,
                                uint8_t *payload, size_t capacity);
bool uwb_mobile_position_decode(const uint8_t *payload, size_t payload_len,
                                struct uwb_mobile_position *position);

void uwb_mobile_geometry_init(struct uwb_mobile_geometry *geometry,
                              const struct uwb_mobile_anchor *fixed,
                              size_t anchor_count);

/* Returns true only on the transition from surveyed to mobile geometry. */
bool uwb_mobile_geometry_ingest(struct uwb_mobile_geometry *geometry,
                                uint16_t anchor_id,
                                const struct uwb_mobile_position *position);

/*
 * Returns the last coordinate accepted by ingest().  Float/SPS and implausible
 * RTK-Fixed jumps therefore hold the last plausible Fixed fix.
 */
bool uwb_mobile_geometry_project_anchor(
    const struct uwb_mobile_geometry *geometry, uint16_t anchor_id,
    const struct uwb_mobile_position *position, double *x_m, double *y_m);

bool uwb_mobile_geometry_latest(const struct uwb_mobile_geometry *geometry,
                                uint16_t anchor_id, double *x_m,
                                double *y_m);
bool uwb_mobile_geometry_all_rtk_fixed(
    const struct uwb_mobile_geometry *geometry);
uint32_t uwb_mobile_geometry_version(uint32_t sequence);

#ifdef __cplusplus
}
#endif

#endif /* UWB_MOBILE_GEOMETRY_H */
