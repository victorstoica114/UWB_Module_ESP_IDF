#include "flextdoa_cfo_estimator.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

_Static_assert(FLEXTDOA_CFO_MIN_SAMPLES <= FLEXTDOA_CFO_EMA_WINDOW,
               "CFO warm-up cannot exceed the EMA window");

static void flextdoa_cfo_clear_responders(
    struct flextdoa_cfo_estimator *estimator)
{
    memset(estimator->responders, 0, sizeof(estimator->responders));
}

void flextdoa_cfo_estimator_reset(
    struct flextdoa_cfo_estimator *estimator)
{
    if (estimator == NULL) {
        return;
    }
    memset(estimator, 0, sizeof(*estimator));
    estimator->reset_pending = true;
}

void flextdoa_cfo_estimator_configure(
    struct flextdoa_cfo_estimator *estimator,
    uint32_t config_generation)
{
    if (estimator == NULL) {
        return;
    }
    if (estimator->configured &&
        estimator->config_generation == config_generation) {
        return;
    }

    flextdoa_cfo_clear_responders(estimator);
    estimator->config_generation = config_generation;
    estimator->configured = true;
    estimator->have_last_slot = false;
    estimator->reset_pending = true;
}

void flextdoa_cfo_estimator_note_slot(
    struct flextdoa_cfo_estimator *estimator, uint32_t slot_id)
{
    if (estimator == NULL) {
        return;
    }
    if (estimator->have_last_slot) {
        const uint32_t delta = slot_id - estimator->last_slot_id;
        if (delta > FLEXTDOA_CFO_MAX_SLOT_GAP) {
            flextdoa_cfo_clear_responders(estimator);
            estimator->reset_pending = true;
        }
    }
    estimator->last_slot_id = slot_id;
    estimator->have_last_slot = true;
}

static struct flextdoa_cfo_responder_state *flextdoa_cfo_find_state(
    struct flextdoa_cfo_estimator *estimator, uint16_t responder_id)
{
    struct flextdoa_cfo_responder_state *free_state = NULL;
    for (size_t index = 0U; index < FLEXTDOA_MAX_ANCHORS; ++index) {
        struct flextdoa_cfo_responder_state *state =
            &estimator->responders[index];
        if (state->in_use && state->responder_id == responder_id) {
            return state;
        }
        if (!state->in_use && free_state == NULL) {
            free_state = state;
        }
    }
    if (free_state != NULL) {
        free_state->in_use = true;
        free_state->responder_id = responder_id;
    }
    return free_state;
}

bool flextdoa_cfo_estimator_update(
    struct flextdoa_cfo_estimator *estimator, uint16_t responder_id,
    double raw_fraction, struct flextdoa_cfo_result *result)
{
    if (estimator == NULL || result == NULL || !isfinite(raw_fraction) ||
        fabs(raw_fraction) > FLEXTDOA_CFO_MAX_ABS_FRACTION) {
        return false;
    }

    struct flextdoa_cfo_responder_state *state =
        flextdoa_cfo_find_state(estimator, responder_id);
    if (state == NULL) {
        return false;
    }

    const uint16_t previous_count = state->sample_count;
    if (previous_count == 0U) {
        state->ema_fraction = raw_fraction;
    } else if (previous_count < FLEXTDOA_CFO_EMA_WINDOW) {
        /* Unbiased start-up mean; avoid giving sample zero an EMA-sized
         * influence throughout the warm-up interval. */
        state->ema_fraction +=
            (raw_fraction - state->ema_fraction) /
            (double)(previous_count + 1U);
    } else {
        const double alpha =
            2.0 / ((double)FLEXTDOA_CFO_EMA_WINDOW + 1.0);
        state->ema_fraction += alpha * (raw_fraction - state->ema_fraction);
    }
    if (state->sample_count < UINT16_MAX) {
        state->sample_count++;
    }

    const bool ready = state->sample_count >= FLEXTDOA_CFO_MIN_SAMPLES;
    result->raw_fraction = raw_fraction;
    result->estimated_fraction = state->ema_fraction;
    result->applied_fraction = ready ? state->ema_fraction : raw_fraction;
    result->sample_count = state->sample_count;
    result->flags = ready
                        ? FLEXTDOA_CFO_RESULT_READY |
                              FLEXTDOA_CFO_RESULT_ESTIMATE_APPLIED
                        : 0U;
    if (estimator->reset_pending) {
        result->flags |= FLEXTDOA_CFO_RESULT_RESET;
        estimator->reset_pending = false;
    }
    return true;
}
