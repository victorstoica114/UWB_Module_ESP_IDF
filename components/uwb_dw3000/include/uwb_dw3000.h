#ifndef UWB_DW3000_H
#define UWB_DW3000_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

enum uwb_dw3000_status {
    UWB_DW3000_STATUS_IDLE = 0,
    UWB_DW3000_STATUS_INITIALIZING,
    UWB_DW3000_STATUS_PROBING,
    UWB_DW3000_STATUS_READY,
    UWB_DW3000_STATUS_FAILED,
};

esp_err_t uwb_dw3000_start(void);
esp_err_t uwb_dw3000_start_distance_test(void);
esp_err_t uwb_dw3000_start_calibration(void);
esp_err_t uwb_dw3000_start_anchor_survey(void);
esp_err_t uwb_dw3000_start_ranging(void);
esp_err_t uwb_dw3000_hold_in_reset(void);
bool uwb_dw3000_is_ready(void);
enum uwb_dw3000_status uwb_dw3000_get_status(void);
const char *uwb_dw3000_status_to_string(enum uwb_dw3000_status status);
uint32_t uwb_dw3000_get_device_id(void);
uint8_t uwb_dw3000_get_source_id(void);
uint32_t uwb_dw3000_get_tx_count(void);
uint32_t uwb_dw3000_get_tx_error_count(void);
uint32_t uwb_dw3000_get_rx_count(void);
uint32_t uwb_dw3000_get_rx_error_count(void);
uint32_t uwb_dw3000_get_rx_ignored_count(void);
uint8_t uwb_dw3000_get_last_rx_source_id(void);
uint32_t uwb_dw3000_get_last_rx_sequence(void);
uint16_t uwb_dw3000_get_antenna_delay(void);

#ifdef __cplusplus
}
#endif

#endif
