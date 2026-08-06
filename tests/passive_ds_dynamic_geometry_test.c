#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "passive_ds_dynamic_geometry.h"

#define TEST_LATITUDE_E7 444000000
#define TEST_LONGITUDE_E7 260000000
#define TEST_E7_LATITUDE_METERS 0.011131949079327358
#define TEST_E7_LONGITUDE_METERS 0.007966

static struct passive_ds_dynamic_position_sample sample_at(
    double east_m, double north_m)
{
    return (struct passive_ds_dynamic_position_sample){
        .flags = PASSIVE_DS_DYNAMIC_POSITION_VALID |
                 PASSIVE_DS_DYNAMIC_POSITION_RTK_FIXED,
        .latitude_e7 = TEST_LATITUDE_E7 +
            (int32_t)llround(north_m / TEST_E7_LATITUDE_METERS),
        .longitude_e7 = TEST_LONGITUDE_E7 +
            (int32_t)llround(east_m / TEST_E7_LONGITUDE_METERS),
    };
}

int main(void)
{
    const struct passive_ds_position_anchor fixed[] = {
        {.id = 2U, .x_m = 0.0, .y_m = 0.0},
        {.id = 3U, .x_m = 5.0, .y_m = 0.0},
        {.id = 4U, .x_m = 0.0, .y_m = 5.0},
        {.id = 5U, .x_m = 5.0, .y_m = 5.0},
    };
    struct passive_ds_dynamic_geometry geometry = {0};
    passive_ds_dynamic_geometry_init(&geometry, fixed, 4U);
    struct passive_ds_dynamic_position_sample samples[] = {
        sample_at(0.0, 0.0),
        sample_at(5.0, 0.0),
        sample_at(0.0, 5.0),
        sample_at(5.0, 5.0),
    };
    struct passive_ds_dynamic_position_sample non_fixed = samples[0];
    non_fixed.flags &=
        (uint8_t)~PASSIVE_DS_DYNAMIC_POSITION_RTK_FIXED;
    assert(!passive_ds_dynamic_geometry_ingest(
        &geometry, 2U, &non_fixed));
    assert(!geometry.active);
    assert(!passive_ds_dynamic_geometry_ingest(
        &geometry, 2U, &samples[0]));
    assert(!passive_ds_dynamic_geometry_ingest(
        &geometry, 3U, &samples[1]));
    assert(!passive_ds_dynamic_geometry_ingest(
        &geometry, 4U, &samples[2]));
    assert(passive_ds_dynamic_geometry_ingest(
        &geometry, 5U, &samples[3]));
    assert(geometry.active);
    assert(geometry.fit_rms_m < 0.03);
    assert(passive_ds_dynamic_geometry_all_rtk_fixed(&geometry));

    struct passive_ds_dynamic_position_sample moved =
        sample_at(6.0, 0.0);
    double x_m = 0.0;
    double y_m = 0.0;
    assert(passive_ds_dynamic_geometry_project(
        &geometry, &moved, &x_m, &y_m));
    assert(fabs(x_m - 6.0) < 0.03);
    assert(fabs(y_m) < 0.03);

    moved.flags |= PASSIVE_DS_DYNAMIC_POSITION_VELOCITY_VALID;
    moved.age_ms = 100U;
    moved.velocity_east_mmps = 1000;
    assert(passive_ds_dynamic_geometry_project(
        &geometry, &moved, &x_m, &y_m));
    assert(fabs(x_m - 6.1) < 0.03);
    assert(fabs(y_m) < 0.03);

    moved.flags &=
        (uint8_t)~PASSIVE_DS_DYNAMIC_POSITION_RTK_FIXED;
    assert(!passive_ds_dynamic_geometry_ingest(
        &geometry, 3U, &moved));
    assert(!passive_ds_dynamic_geometry_all_rtk_fixed(&geometry));
    assert(passive_ds_dynamic_geometry_project(
        &geometry, &moved, &x_m, &y_m));

    puts("passive DS dynamic geometry: OK");
    return 0;
}
