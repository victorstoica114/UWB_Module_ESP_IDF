#include "uwb_passive_ds_observation.h"

#include <math.h>

#include "uwb_passive_ds_protocol.h"

#define PASSIVE_DS_TIME_UNIT_SECONDS 15.650040064102564e-12
#define PASSIVE_DS_SPEED_OF_LIGHT_MPS 299702547.0
#define PASSIVE_DS_MAX_CFO_FRACTION 0.001
#define PASSIVE_DS_MAX_LISTENER_INTERVAL_DTU 1277952000ULL /* 20 ms */
#define PASSIVE_DS_MAX_ABS_DIFFERENCE_M 100.0

bool uwb_passive_ds_compute_observation(
    const struct uwb_passive_ds_observation_input *input,
    struct uwb_passive_ds_observation_result *result)
{
    if (input == NULL || result == NULL ||
        input->responder_reply_dtu == 0U ||
        !isfinite(input->responder_to_listener_cfo_fraction) ||
        fabs(input->responder_to_listener_cfo_fraction) >
            PASSIVE_DS_MAX_CFO_FRACTION ||
        !isfinite(input->initiator_responder_distance_m) ||
        input->initiator_responder_distance_m <= 0.0) {
        return false;
    }

    const uint64_t listener_interval = uwb_passive_ds_timestamp_delta(
        input->listener_response_rx, input->listener_poll_rx);
    if (listener_interval == 0U ||
        listener_interval > PASSIVE_DS_MAX_LISTENER_INTERVAL_DTU) {
        return false;
    }
    const double anchor_tof_dtu =
        input->initiator_responder_distance_m /
        (PASSIVE_DS_TIME_UNIT_SECONDS * PASSIVE_DS_SPEED_OF_LIGHT_MPS);
    const double corrected_reply_dtu =
        (double)input->responder_reply_dtu *
        (1.0 - input->responder_to_listener_cfo_fraction);
    const double corrected_difference_dtu =
        (double)listener_interval - corrected_reply_dtu - anchor_tof_dtu;
    const double raw_difference_dtu =
        (double)listener_interval - input->responder_reply_dtu -
        anchor_tof_dtu;
    const double scale =
        PASSIVE_DS_TIME_UNIT_SECONDS * PASSIVE_DS_SPEED_OF_LIGHT_MPS;
    const double corrected_m = corrected_difference_dtu * scale;
    const double raw_m = raw_difference_dtu * scale;
    if (!isfinite(corrected_m) || !isfinite(raw_m) ||
        fabs(corrected_m) > PASSIVE_DS_MAX_ABS_DIFFERENCE_M ||
        fabs(raw_m) > PASSIVE_DS_MAX_ABS_DIFFERENCE_M) {
        return false;
    }
    result->difference_m = corrected_m;
    result->raw_difference_m = raw_m;
    result->cfo_correction_m = corrected_m - raw_m;
    return true;
}
