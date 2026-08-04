#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "flextdoa_cfo_estimator.h"

static void test_warmup_and_responder_isolation(void)
{
    struct flextdoa_cfo_estimator estimator;
    struct flextdoa_cfo_result result = {0};
    flextdoa_cfo_estimator_reset(&estimator);
    flextdoa_cfo_estimator_configure(&estimator, 7U);

    for (uint16_t sample = 1U; sample < FLEXTDOA_CFO_MIN_SAMPLES;
         ++sample) {
        flextdoa_cfo_estimator_note_slot(&estimator, sample);
        const double raw = (sample & 1U) ? 2.1e-6 : 1.9e-6;
        assert(flextdoa_cfo_estimator_update(
            &estimator, 3U, raw, &result));
        assert(result.sample_count == sample);
        assert((result.flags & FLEXTDOA_CFO_RESULT_READY) == 0U);
        assert(result.applied_fraction == raw);
    }

    flextdoa_cfo_estimator_note_slot(
        &estimator, FLEXTDOA_CFO_MIN_SAMPLES);
    assert(flextdoa_cfo_estimator_update(
        &estimator, 3U, 2.1e-6, &result));
    assert(result.sample_count == FLEXTDOA_CFO_MIN_SAMPLES);
    assert((result.flags & FLEXTDOA_CFO_RESULT_READY) != 0U);
    assert((result.flags & FLEXTDOA_CFO_RESULT_ESTIMATE_APPLIED) != 0U);
    assert(result.applied_fraction == result.estimated_fraction);

    assert(flextdoa_cfo_estimator_update(
        &estimator, 4U, -3.0e-6, &result));
    assert(result.sample_count == 1U);
    assert(result.estimated_fraction == -3.0e-6);
    assert(result.applied_fraction == -3.0e-6);
}

static void test_reset_conditions_and_invalid_input(void)
{
    struct flextdoa_cfo_estimator estimator;
    struct flextdoa_cfo_result result = {0};
    flextdoa_cfo_estimator_reset(&estimator);
    flextdoa_cfo_estimator_configure(&estimator, 11U);
    flextdoa_cfo_estimator_note_slot(&estimator, 100U);
    assert(flextdoa_cfo_estimator_update(
        &estimator, 2U, 1.0e-6, &result));
    assert((result.flags & FLEXTDOA_CFO_RESULT_RESET) != 0U);

    flextdoa_cfo_estimator_note_slot(&estimator, 100U);
    assert(flextdoa_cfo_estimator_update(
        &estimator, 2U, 1.0e-6, &result));
    assert(result.sample_count == 2U);

    flextdoa_cfo_estimator_note_slot(
        &estimator, 100U + FLEXTDOA_CFO_MAX_SLOT_GAP + 1U);
    assert(flextdoa_cfo_estimator_update(
        &estimator, 2U, 1.0e-6, &result));
    assert(result.sample_count == 1U);
    assert((result.flags & FLEXTDOA_CFO_RESULT_RESET) != 0U);

    flextdoa_cfo_estimator_note_slot(&estimator, 110U);
    assert(flextdoa_cfo_estimator_update(
        &estimator, 2U, 1.0e-6, &result));
    assert(result.sample_count == 1U);
    assert((result.flags & FLEXTDOA_CFO_RESULT_RESET) != 0U);

    flextdoa_cfo_estimator_configure(&estimator, 12U);
    flextdoa_cfo_estimator_note_slot(&estimator, 200U);
    assert(flextdoa_cfo_estimator_update(
        &estimator, 2U, 1.0e-6, &result));
    assert(result.sample_count == 1U);
    assert((result.flags & FLEXTDOA_CFO_RESULT_RESET) != 0U);

    assert(!flextdoa_cfo_estimator_update(
        &estimator, 2U, NAN, &result));
    assert(!flextdoa_cfo_estimator_update(
        &estimator, 2U, FLEXTDOA_CFO_MAX_ABS_FRACTION * 1.01,
        &result));
}

static void test_uint32_slot_wrap_is_continuous(void)
{
    struct flextdoa_cfo_estimator estimator;
    struct flextdoa_cfo_result result = {0};
    flextdoa_cfo_estimator_reset(&estimator);
    flextdoa_cfo_estimator_configure(&estimator, 1U);
    flextdoa_cfo_estimator_note_slot(&estimator, UINT32_MAX - 1U);
    assert(flextdoa_cfo_estimator_update(
        &estimator, 5U, 4.0e-6, &result));
    flextdoa_cfo_estimator_note_slot(&estimator, 1U);
    assert(flextdoa_cfo_estimator_update(
        &estimator, 5U, 4.0e-6, &result));
    assert(result.sample_count == 2U);
    assert((result.flags & FLEXTDOA_CFO_RESULT_RESET) == 0U);
}

static void test_synthetic_drift_and_noise(void)
{
    static const double noise_ppm[] = {
        0.18, -0.18, 0.12, -0.12, 0.16, -0.16, 0.10, -0.10,
    };
    static const double processing_us[] = {2250.0, 2750.0, 3250.0};
    const double dtu_seconds = 15.650040064102564e-12;
    const double speed_of_light_mps = 299702547.0;
    const double pi = 3.14159265358979323846;
    struct flextdoa_cfo_estimator estimator;
    struct flextdoa_cfo_result result = {0};
    double raw_squared_error = 0.0;
    double filtered_squared_error = 0.0;
    double filtered_signed_error = 0.0;
    uint32_t evaluated = 0U;

    flextdoa_cfo_estimator_reset(&estimator);
    flextdoa_cfo_estimator_configure(&estimator, 42U);
    for (uint32_t sample = 0U; sample < 1200U; ++sample) {
        const uint16_t responder_id = (uint16_t)(2U + sample % 4U);
        const uint32_t responder_sample = sample / 4U;
        const double phase =
            2.0 * pi * (double)responder_sample / 150.0;
        const double true_ppm =
            -3.0 + 1.25 * (double)(responder_id - 2U) +
            0.025 * sin(phase);
        const double raw_ppm =
            true_ppm + noise_ppm[responder_sample % 8U];

        flextdoa_cfo_estimator_note_slot(&estimator, sample);
        assert(flextdoa_cfo_estimator_update(
            &estimator, responder_id, raw_ppm * 1.0e-6, &result));
        if ((result.flags & FLEXTDOA_CFO_RESULT_READY) == 0U) {
            continue;
        }

        const double processing_dtu =
            processing_us[sample % 3U] * 1.0e-6 / dtu_seconds;
        const double raw_range_error =
            processing_dtu * (result.raw_fraction - true_ppm * 1.0e-6) *
            dtu_seconds * speed_of_light_mps;
        const double filtered_range_error =
            processing_dtu *
            (result.applied_fraction - true_ppm * 1.0e-6) *
            dtu_seconds * speed_of_light_mps;
        raw_squared_error += raw_range_error * raw_range_error;
        filtered_squared_error +=
            filtered_range_error * filtered_range_error;
        filtered_signed_error += filtered_range_error;
        evaluated++;
    }

    assert(evaluated > 1000U);
    const double raw_rmse = sqrt(raw_squared_error / (double)evaluated);
    const double filtered_rmse =
        sqrt(filtered_squared_error / (double)evaluated);
    const double filtered_bias =
        filtered_signed_error / (double)evaluated;
    printf("synthetic CFO range error raw=%.3f cm ema=%.3f cm bias=%.3f cm\n",
           raw_rmse * 100.0, filtered_rmse * 100.0,
           filtered_bias * 100.0);
    fflush(stdout);
    assert(filtered_rmse < raw_rmse * 0.45);
    assert(fabs(filtered_bias) < 0.002);
}

int main(void)
{
    test_warmup_and_responder_isolation();
    test_reset_conditions_and_invalid_input();
    test_uint32_slot_wrap_is_continuous();
    test_synthetic_drift_and_noise();
    puts("flextdoa_cfo_estimator_test: PASS");
    return 0;
}
