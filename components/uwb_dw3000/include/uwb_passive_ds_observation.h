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
    uint32_t responder_reply_dtu;
    /* Eq. (12) epsilon: responder interval becomes reply * (1 - epsilon). */
    double responder_to_listener_cfo_fraction;
    double initiator_responder_distance_m;
};

struct uwb_passive_ds_observation_result {
    /* distance(tag, responder) - distance(tag, initiator) */
    double difference_m;
    double raw_difference_m;
    double cfo_correction_m;
};

bool uwb_passive_ds_compute_observation(
    const struct uwb_passive_ds_observation_input *input,
    struct uwb_passive_ds_observation_result *result);

#ifdef __cplusplus
}
#endif

#endif
