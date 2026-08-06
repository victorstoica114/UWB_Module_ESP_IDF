#ifndef FLEXTDOA_SOLVER_SERVICE_H
#define FLEXTDOA_SOLVER_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t flextdoa_solver_service_start(void);
bool flextdoa_solver_service_reload_geometry(void);
bool flextdoa_solver_service_reset(void);
bool flextdoa_solver_service_submit_observation(
    uint8_t tag_id, uint8_t initiator_id, uint8_t responder_id,
    uint32_t slot_id, int32_t difference_mm, bool dynamic_geometry,
    int32_t initiator_x_mm, int32_t initiator_y_mm,
    int32_t responder_x_mm, int32_t responder_y_mm,
    uint32_t geometry_version, int32_t geometry_fit_rms_mm,
    bool geometry_all_rtk_fixed);

#ifdef __cplusplus
}
#endif

#endif /* FLEXTDOA_SOLVER_SERVICE_H */
