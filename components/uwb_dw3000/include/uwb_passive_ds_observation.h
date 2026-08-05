#ifndef UWB_PASSIVE_DS_OBSERVATION_H
#define UWB_PASSIVE_DS_OBSERVATION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct uwb_passive_ds_observation_input {
    uint64_t listener_poll_rx;
    uint64_t listener_response_rx;
    uint64_t listener_final_rx;

    uint64_t initiator_poll_tx;
    uint64_t initiator_response_rx;
    uint64_t initiator_final_tx;

    uint32_t responder_reply_dtu;
    uint32_t responder_exchange_dtu;
};

struct uwb_passive_ds_observation_result {
    /* distance(tag, responder) - distance(tag, initiator) */
    double difference_m;
    double listener_to_initiator_clock_ratio;
    double listener_to_responder_clock_ratio;
    double responder_delay_ratio;
    /* Exact tag timestamp interpolation coefficient used by GLS. */
    double listener_interpolation_ratio;
};

bool uwb_passive_ds_compute_observation(
    const struct uwb_passive_ds_observation_input *input,
    struct uwb_passive_ds_observation_result *result);

/*
 * Two-packet FlexTDOA-equivalent estimator retained only for same-frame A/B
 * diagnostics.  The production Passive DS position path uses the three-packet
 * estimator above and does not depend on CFO.
 */
struct uwb_passive_ds_cfo_observation_input {
    uint64_t listener_poll_rx;
    uint64_t listener_response_rx;
    uint32_t responder_reply_dtu;
    double responder_to_listener_cfo_fraction;
    double initiator_responder_distance_m;
};

struct uwb_passive_ds_cfo_observation_result {
    double difference_m;
    double raw_difference_m;
    double cfo_correction_m;
};

bool uwb_passive_ds_compute_cfo_observation(
    const struct uwb_passive_ds_cfo_observation_input *input,
    struct uwb_passive_ds_cfo_observation_result *result);

#ifdef __cplusplus
}
#endif

#endif
