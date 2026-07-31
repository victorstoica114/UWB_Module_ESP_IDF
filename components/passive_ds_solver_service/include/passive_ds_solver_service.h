#ifndef PASSIVE_DS_SOLVER_SERVICE_H
#define PASSIVE_DS_SOLVER_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t passive_ds_solver_service_start(void);
bool passive_ds_solver_service_reset(void);
bool passive_ds_solver_service_submit_anchor_range(
    uint8_t anchor_a_id, uint8_t anchor_b_id, uint32_t slot_id,
    int32_t distance_mm);
bool passive_ds_solver_service_submit_observation(
    uint8_t tag_id, uint8_t initiator_id, uint8_t responder_id,
    uint32_t slot_id, int32_t difference_mm);

#ifdef __cplusplus
}
#endif

#endif /* PASSIVE_DS_SOLVER_SERVICE_H */
