#include "uwb_passive_ds_observation.h"

#include <math.h>

#include "uwb_passive_ds_protocol.h"

#define PASSIVE_DS_TIME_UNIT_SECONDS 15.650040064102564e-12
#define PASSIVE_DS_SPEED_OF_LIGHT_MPS 299702547.0
#define PASSIVE_DS_MAX_LISTENER_INTERVAL_DTU 1277952000ULL /* 20 ms */
#define PASSIVE_DS_MAX_ABS_DIFFERENCE_M 100.0
#define PASSIVE_DS_MIN_CLOCK_RATIO 0.9995
#define PASSIVE_DS_MAX_CLOCK_RATIO 1.0005
#define PASSIVE_DS_MAX_CFO_FRACTION 0.001

bool uwb_passive_ds_compute_observation(
    const struct uwb_passive_ds_observation_input *input,
    struct uwb_passive_ds_observation_result *result)
{
    if (input == NULL || result == NULL ||
        input->responder_reply_dtu == 0U ||
        input->responder_exchange_dtu <= input->responder_reply_dtu) {
        return false;
    }

    const double listener_first = (double)uwb_passive_ds_timestamp_delta(
        input->listener_response_rx, input->listener_poll_rx);
    const double listener_second = (double)uwb_passive_ds_timestamp_delta(
        input->listener_final_rx, input->listener_response_rx);
    const double listener_exchange = listener_first + listener_second;
    const double initiator_round = (double)uwb_passive_ds_timestamp_delta(
        input->initiator_response_rx, input->initiator_poll_tx);
    const double initiator_reply = (double)uwb_passive_ds_timestamp_delta(
        input->initiator_final_tx, input->initiator_response_rx);
    const double initiator_exchange = initiator_round + initiator_reply;
    const double responder_reply = (double)input->responder_reply_dtu;
    const double responder_exchange =
        (double)input->responder_exchange_dtu;
    if (listener_first <= 0.0 || listener_second <= 0.0 ||
        listener_exchange <= 0.0 || initiator_round <= 0.0 ||
        initiator_reply <= 0.0 || initiator_exchange <= 0.0 ||
        responder_exchange <= 0.0 ||
        listener_exchange > PASSIVE_DS_MAX_LISTENER_INTERVAL_DTU ||
        initiator_exchange > PASSIVE_DS_MAX_LISTENER_INTERVAL_DTU ||
        responder_exchange > PASSIVE_DS_MAX_LISTENER_INTERVAL_DTU) {
        return false;
    }

    /*
     * Rathje & Landsiedel Eq. (19), sign-adjusted to the solver convention:
     *
     *   d(T,R)-d(T,I) = M_T
     *                   - 0.5*(k_T/k_I)*R_I
     *                   - 0.5*(k_T/k_R)*D_R
     *
     * The complete POLL..FINAL intervals provide both clock ratios.  No CIA
     * carrier-integrator estimate and no measured anchor baseline enters the
     * tag observation.
     */
    const double listener_to_initiator_clock_ratio =
        listener_exchange / initiator_exchange;
    const double listener_to_responder_clock_ratio =
        listener_exchange / responder_exchange;
    if (!isfinite(listener_to_initiator_clock_ratio) ||
        !isfinite(listener_to_responder_clock_ratio) ||
        listener_to_initiator_clock_ratio < PASSIVE_DS_MIN_CLOCK_RATIO ||
        listener_to_initiator_clock_ratio > PASSIVE_DS_MAX_CLOCK_RATIO ||
        listener_to_responder_clock_ratio < PASSIVE_DS_MIN_CLOCK_RATIO ||
        listener_to_responder_clock_ratio > PASSIVE_DS_MAX_CLOCK_RATIO) {
        return false;
    }
    const double difference_dtu =
        listener_first -
        0.5 * listener_to_initiator_clock_ratio * initiator_round -
        0.5 * listener_to_responder_clock_ratio * responder_reply;
    const double scale =
        PASSIVE_DS_TIME_UNIT_SECONDS * PASSIVE_DS_SPEED_OF_LIGHT_MPS;
    const double difference_m = difference_dtu * scale;
    if (!isfinite(difference_m) ||
        fabs(difference_m) > PASSIVE_DS_MAX_ABS_DIFFERENCE_M) {
        return false;
    }
    result->difference_m = difference_m;
    result->listener_to_initiator_clock_ratio =
        listener_to_initiator_clock_ratio;
    result->listener_to_responder_clock_ratio =
        listener_to_responder_clock_ratio;
    result->responder_delay_ratio = responder_reply / responder_exchange;
    result->listener_interpolation_ratio = 0.5 *
        (initiator_round / initiator_exchange +
         responder_reply / responder_exchange);
    if (!isfinite(result->listener_interpolation_ratio) ||
        result->listener_interpolation_ratio <= 0.0 ||
        result->listener_interpolation_ratio >= 1.0) {
        return false;
    }
    return true;
}

bool uwb_passive_ds_compute_cfo_observation(
    const struct uwb_passive_ds_cfo_observation_input *input,
    struct uwb_passive_ds_cfo_observation_result *result)
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
        (double)listener_interval - (double)input->responder_reply_dtu -
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
