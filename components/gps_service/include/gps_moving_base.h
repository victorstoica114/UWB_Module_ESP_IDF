#ifndef GPS_MOVING_BASE_H
#define GPS_MOVING_BASE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GPS_MB_ROLE_NONE = 0,
    GPS_MB_ROLE_PRECISE_BASE = 1,
    GPS_MB_ROLE_MOVING_ROVER = 2,
    GPS_MB_ROLE_LOCAL_BASE = 3,
    GPS_MB_ROLE_RTK_ROVER = 4,
} gps_moving_base_role_t;

typedef struct {
    gps_moving_base_role_t role;
    bool active;
    bool correction_uart_ready;
    bool receiver_config_sent;
    uint32_t receiver_ack_count;
    uint32_t receiver_nack_count;
    uint8_t receiver_last_ack_id;
    uint8_t receiver_last_nack_id;
    uint32_t uplink_packet_count;
    uint32_t uplink_byte_count;
    uint32_t uplink_error_count;
    uint32_t downlink_packet_count;
    uint32_t downlink_byte_count;
    uint32_t downlink_error_count;
    uint32_t downlink_gap_count;
    uint32_t last_uplink_age_ms;
    uint32_t last_downlink_age_ms;
    uint8_t last_downlink_source_id;
    uint32_t skytraq_binary_frame_count;
    bool software_version_valid;
    uint8_t software_type;
    uint32_t software_kernel_version;
    uint32_t software_odm_version;
    uint32_t software_revision;
    bool binary_output_status_valid;
    uint8_t binary_output_rate_code;
    bool binary_meas_time_enabled;
    bool binary_raw_meas_enabled;
    uint32_t binary_meas_time_count;
    uint32_t binary_raw_meas_count;
    uint32_t rtcm_preamble_count;
    bool ntrip_configured;
    bool ntrip_running;
    bool ntrip_tls_connected;
    bool ntrip_stream_active;
    uint16_t ntrip_http_status;
    uint32_t ntrip_connect_count;
    uint32_t ntrip_reconnect_count;
    uint32_t ntrip_error_count;
    uint32_t ntrip_rtcm_frame_count;
    uint32_t ntrip_rtcm_byte_count;
    uint32_t ntrip_last_data_age_ms;
    char ntrip_state[24];
} gps_moving_base_snapshot_t;

esp_err_t gps_moving_base_start(uart_port_t primary_uart, uint8_t module_id);
void gps_moving_base_stop(void);
void gps_moving_base_process_primary_bytes(const uint8_t *data, size_t length);
void gps_moving_base_set_gga(const char *sentence);
void gps_moving_base_poll(void);
void gps_moving_base_get_snapshot(gps_moving_base_snapshot_t *snapshot);
const char *gps_moving_base_role_to_string(gps_moving_base_role_t role);

#ifdef __cplusplus
}
#endif

#endif /* GPS_MOVING_BASE_H */
