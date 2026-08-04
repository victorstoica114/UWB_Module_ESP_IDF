#ifndef FLEXTDOA_CFO_ESTIMATOR_H
#define FLEXTDOA_CFO_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

#include "flextdoa_protocol.h"

/*
 * The EMA's equivalent noise bandwidth is approximately that of this many
 * independent samples. Keep these constants visible so field tests can tune
 * the noise/drift trade-off without changing estimator semantics.
 */
#define FLEXTDOA_CFO_EMA_WINDOW 48U
#define FLEXTDOA_CFO_MIN_SAMPLES 24U
#define FLEXTDOA_CFO_MAX_SLOT_GAP 16U
#define FLEXTDOA_CFO_MAX_ABS_FRACTION 0.0001

enum flextdoa_cfo_result_flags {
    FLEXTDOA_CFO_RESULT_READY = 1U << 0,
    FLEXTDOA_CFO_RESULT_ESTIMATE_APPLIED = 1U << 1,
    FLEXTDOA_CFO_RESULT_RESET = 1U << 2,
};

struct flextdoa_cfo_responder_state {
    bool in_use;
    uint16_t responder_id;
    uint16_t sample_count;
    double ema_fraction;
};

struct flextdoa_cfo_estimator {
    struct flextdoa_cfo_responder_state responders[FLEXTDOA_MAX_ANCHORS];
    uint32_t config_generation;
    uint32_t last_slot_id;
    bool configured;
    bool have_last_slot;
    bool reset_pending;
};

struct flextdoa_cfo_result {
    double raw_fraction;
    double estimated_fraction;
    double applied_fraction;
    uint16_t sample_count;
    uint8_t flags;
};

/* Reset on protocol entry/reboot. The first accepted sample reports RESET. */
void flextdoa_cfo_estimator_reset(
    struct flextdoa_cfo_estimator *estimator);

/* A changed generation invalidates all accumulated responder state. */
void flextdoa_cfo_estimator_configure(
    struct flextdoa_cfo_estimator *estimator,
    uint32_t config_generation);

/*
 * Call for each received FlexTDOA observation. Repeated observations from the
 * same slot are valid. A rollback or a forward gap greater than the bound
 * resets all responder state; uint32_t slot wrap is handled naturally.
 */
void flextdoa_cfo_estimator_note_slot(
    struct flextdoa_cfo_estimator *estimator, uint32_t slot_id);

/*
 * Update one responder's causal EMA. During warm-up applied_fraction is the
 * current raw sample, providing the same safe correction as the old path.
 * Returns false for invalid/out-of-range samples or exhausted state storage.
 */
bool flextdoa_cfo_estimator_update(
    struct flextdoa_cfo_estimator *estimator, uint16_t responder_id,
    double raw_fraction, struct flextdoa_cfo_result *result);

#endif
