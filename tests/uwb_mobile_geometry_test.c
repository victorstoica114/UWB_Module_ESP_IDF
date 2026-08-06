#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "uwb_mobile_geometry.h"

#define TEST_LATITUDE_E7 444000000
#define TEST_LONGITUDE_E7 260000000
#define TEST_E7_LATITUDE_METERS 0.011131949079327358
#define TEST_E7_LONGITUDE_METERS 0.007966

static struct uwb_mobile_position sample_at(double east_m, double north_m)
{
    return (struct uwb_mobile_position){
        .flags = UWB_MOBILE_POSITION_VALID |
                 UWB_MOBILE_POSITION_RTK_FIXED,
        .latitude_e7 = TEST_LATITUDE_E7 +
            (int32_t)llround(north_m / TEST_E7_LATITUDE_METERS),
        .longitude_e7 = TEST_LONGITUDE_E7 +
            (int32_t)llround(east_m / TEST_E7_LONGITUDE_METERS),
    };
}

int main(void)
{
    const struct uwb_mobile_anchor fixed[] = {
        {.id = 2U, .x_m = 0.0, .y_m = 0.0},
        {.id = 3U, .x_m = 5.0, .y_m = 0.0},
        {.id = 4U, .x_m = 0.0, .y_m = 5.0},
        {.id = 5U, .x_m = 5.0, .y_m = 5.0},
    };
    struct uwb_mobile_geometry geometry = {0};
    uwb_mobile_geometry_init(&geometry, fixed, 4U);
    struct uwb_mobile_position samples[] = {
        sample_at(0.0, 0.0), sample_at(5.0, 0.0),
        sample_at(0.0, 5.0), sample_at(5.0, 5.0),
    };
    for (size_t index = 0U; index < 3U; ++index) {
        assert(!uwb_mobile_geometry_ingest(
            &geometry, fixed[index].id, &samples[index]));
    }
    assert(uwb_mobile_geometry_ingest(
        &geometry, fixed[3].id, &samples[3]));
    assert(geometry.active);
    assert(geometry.fit_rms_m < 0.03);
    assert(uwb_mobile_geometry_all_rtk_fixed(&geometry));

    struct uwb_mobile_position false_fixed = sample_at(7.0, 0.0);
    assert(!uwb_mobile_geometry_ingest(&geometry, 3U, &false_fixed));
    assert(!uwb_mobile_geometry_all_rtk_fixed(&geometry));
    double x_m = 0.0;
    double y_m = 0.0;
    assert(uwb_mobile_geometry_project_anchor(
        &geometry, 3U, &false_fixed, &x_m, &y_m));
    assert(fabs(x_m - 5.0) < 0.03);
    assert(fabs(y_m) < 0.03);

    assert(!uwb_mobile_geometry_ingest(&geometry, 3U, &samples[1]));
    assert(uwb_mobile_geometry_all_rtk_fixed(&geometry));

    struct uwb_mobile_position moved = sample_at(6.0, 0.0);
    moved.flags |= UWB_MOBILE_POSITION_VELOCITY_VALID;
    moved.velocity_east_mmps = 2000;
    assert(!uwb_mobile_geometry_ingest(&geometry, 3U, &moved));
    assert(uwb_mobile_geometry_project_anchor(
        &geometry, 3U, &moved, &x_m, &y_m));
    assert(fabs(x_m - 6.0) < 0.03);

    moved.flags &= (uint8_t)~UWB_MOBILE_POSITION_RTK_FIXED;
    assert(!uwb_mobile_geometry_ingest(&geometry, 3U, &moved));
    assert(!uwb_mobile_geometry_all_rtk_fixed(&geometry));
    assert(uwb_mobile_geometry_project_anchor(
        &geometry, 3U, &moved, &x_m, &y_m));
    assert(fabs(x_m - 6.0) < 0.03);

    uint8_t wire[UWB_MOBILE_POSITION_WIRE_SIZE] = {0};
    assert(uwb_mobile_position_encode(&moved, wire, sizeof(wire)));
    struct uwb_mobile_position decoded = {0};
    assert(uwb_mobile_position_decode(wire, sizeof(wire), &decoded));
    assert(decoded.flags == moved.flags);
    assert(decoded.age_ms == moved.age_ms);
    assert(decoded.latitude_e7 == moved.latitude_e7);
    assert(decoded.longitude_e7 == moved.longitude_e7);
    assert(decoded.velocity_east_mmps == moved.velocity_east_mmps);
    assert(decoded.velocity_north_mmps == moved.velocity_north_mmps);
    assert(uwb_mobile_geometry_version(103U) == 0x80000067U);

    puts("uwb_mobile_geometry_test: PASS");
    return 0;
}
