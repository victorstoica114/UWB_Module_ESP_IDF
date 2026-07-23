#ifndef MAX77958_SERVICE_H
#define MAX77958_SERVICE_H

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX77958_SERVICE_REGISTER_MAP_SIZE 0xE1U
#define MAX77958_SERVICE_AP_DATA_BYTES 33U
#define MAX77958_SERVICE_MAX_SOURCE_PDOS 7U
#define MAX77958_SERVICE_MAX_SINK_PDOS 5U

typedef struct {
    bool ok;
    int err;
    uint8_t opcode;
    uint8_t response_opcode;
    uint8_t result_code;
    uint8_t response[MAX77958_SERVICE_AP_DATA_BYTES];
} max77958_service_ap_result_t;

typedef struct {
    bool monitor_enabled;
    bool present;
    bool read_ok;
    bool raw_valid;
    bool config_write_supported;
    bool config_writes_enabled;
    int last_error;
    uint32_t read_count;
    uint32_t error_count;
    uint32_t last_read_duration_ms;
    uint32_t last_update_age_ms;
    uint32_t i2c_clock_hz;
    uint32_t i2c_hs_direct_clock_hz;
    bool i2c_hs_ext_requested;
    bool i2c_hs_ext_active;
    bool i2c_hs_direct_reads_enabled;
    bool i2c_hs_direct_writes_enabled;
    uint32_t i2c_hs_direct_read_count;
    uint32_t i2c_hs_direct_write_count;
    uint32_t i2c_hs_direct_error_count;
    uint32_t i2c_hs_direct_last_elapsed_us;
    int i2c_hs_direct_scl_measure_error;
    uint32_t i2c_hs_direct_scl_edges;
    uint32_t i2c_hs_direct_scl_elapsed_us;
    uint32_t i2c_hs_direct_scl_measured_hz;

    uint8_t device_id;
    uint8_t device_rev;
    uint8_t fw_rev;
    uint8_t fw_sub_ver;
    uint8_t uic_int;
    uint8_t cc_int;
    uint8_t pd_int;
    uint8_t action_int;
    uint8_t usbc_status1;
    uint8_t usbc_status2;
    uint8_t bc_status;
    uint8_t dp_status;
    uint8_t cc_status0;
    uint8_t cc_status1;
    uint8_t pd_status0;
    uint8_t pd_status1;
    uint8_t uic_int_mask;
    uint8_t cc_int_mask;
    uint8_t pd_int_mask;
    uint8_t action_int_mask;
    uint8_t sw_reset;
    uint8_t i2c_cnfg;

    uint8_t vbadc_code;
    uint16_t vbus_min_mv;
    uint16_t vbus_max_mv;
    uint16_t vbus_mid_mv;
    bool vbus_above_range;
    bool vbus_detected;
    uint8_t chg_typ;
    uint8_t pr_chg_typ;
    bool dcd_timeout;
    uint8_t cc_pin;
    uint8_t cci;
    bool vconn_enabled;
    uint8_t cc_stat;
    bool det_abrt;
    bool data_role_dfp;
    bool power_role_source;
    bool vconn_source;
    bool psrdy_as_sink;

    bool ctrl1_valid;
    uint8_t ctrl1_raw;
    uint8_t ctrl1_comp2_sw;
    uint8_t ctrl1_comn1_sw;
    bool usb2_switch_closed;

    bool source_caps_valid;
    uint8_t source_pdo_count;
    uint8_t selected_source_pdo_pos;
    uint32_t source_pdos[MAX77958_SERVICE_MAX_SOURCE_PDOS];
    bool sink_pdos_valid;
    uint8_t sink_pdo_count;
    bool sink_pdos_from_mtp;
    uint32_t sink_pdos[MAX77958_SERVICE_MAX_SINK_PDOS];
    bool pps_default_valid;
    bool pps_default_enabled;
    uint16_t pps_default_mv;
    uint16_t pps_default_ma;

    uint32_t operation_count;
    uint32_t operation_error_count;
    uint32_t last_operation_age_ms;
    uint8_t last_opcode;
    uint8_t last_response_opcode;
    uint8_t last_result_code;
    int last_operation_error;
    uint8_t last_response[MAX77958_SERVICE_AP_DATA_BYTES];

    uint8_t raw[MAX77958_SERVICE_REGISTER_MAP_SIZE];
} max77958_service_snapshot_t;

esp_err_t max77958_service_start(void);
void max77958_service_get_snapshot(max77958_service_snapshot_t *snapshot);
void max77958_service_format_raw_hex(const max77958_service_snapshot_t *snapshot,
                                     char *buffer, size_t buffer_size);
esp_err_t max77958_service_read_register(uint8_t reg, uint8_t *value);
esp_err_t max77958_service_write_register(uint8_t reg, uint8_t value);
void max77958_service_request_refresh(void);

esp_err_t max77958_service_trigger_bc_detection(
    max77958_service_ap_result_t *result);
esp_err_t max77958_service_read_ctrl1(max77958_service_ap_result_t *result);
esp_err_t max77958_service_set_usb2_switch(
    bool closed, max77958_service_ap_result_t *result);
esp_err_t max77958_service_read_current_source_caps(
    max77958_service_ap_result_t *result);
esp_err_t max77958_service_request_source_pdo(
    uint8_t position, max77958_service_ap_result_t *result);
esp_err_t max77958_service_read_sink_pdos(
    bool mtp, max77958_service_ap_result_t *result);
esp_err_t max77958_service_set_sink_fixed_pdos(
    const uint16_t *voltages_mv, const uint16_t *currents_ma, size_t count,
    bool mtp, max77958_service_ap_result_t *result);
esp_err_t max77958_service_set_pps_default(
    bool enabled, uint16_t voltage_mv, uint16_t current_ma,
    max77958_service_ap_result_t *result);
esp_err_t max77958_service_request_apdo(
    uint8_t position, uint16_t voltage_mv, uint16_t current_ma,
    max77958_service_ap_result_t *result);

const char *max77958_service_sys_msg_to_string(uint8_t value);
const char *max77958_service_chg_typ_to_string(uint8_t value);
const char *max77958_service_cc_pin_to_string(uint8_t value);
const char *max77958_service_cci_to_string(uint8_t value);
const char *max77958_service_cc_stat_to_string(uint8_t value);
const char *max77958_service_apdo_result_to_string(uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* MAX77958_SERVICE_H */
