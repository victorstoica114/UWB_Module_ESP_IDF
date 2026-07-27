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
esp_err_t uwb_dw3000_start_flex_tdoa(void);
esp_err_t uwb_dw3000_start_passive_ds_twr(void);
bool uwb_dw3000_hot_switch_mode_supported(uint8_t app_runtime_mode);
esp_err_t uwb_dw3000_request_runtime_switch(uint8_t app_runtime_mode);
bool uwb_dw3000_runtime_switch_in_progress(void);
uint32_t uwb_dw3000_get_runtime_switch_count(void);
uint32_t uwb_dw3000_get_last_runtime_switch_ms(void);
esp_err_t uwb_dw3000_hold_in_reset(void);
bool uwb_dw3000_is_ready(void);
enum uwb_dw3000_status uwb_dw3000_get_status(void);
const char *uwb_dw3000_status_to_string(enum uwb_dw3000_status status);
uint32_t uwb_dw3000_get_device_id(void);
uint32_t uwb_dw3000_get_spi_clock_hz(void);
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
