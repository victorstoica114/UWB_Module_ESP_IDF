#ifndef GPS_SERVICE_H
#define GPS_SERVICE_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool runtime_enabled;
    bool powered;
    bool task_running;
    bool uart_ready;
    int last_error;
    uint32_t last_rx_age_ms;
    uint32_t last_fix_age_ms;
    uint32_t byte_count;
    uint32_t sentence_count;
    uint32_t gga_count;
    uint32_t rmc_count;
    uint32_t gsa_count;
    uint32_t gsv_count;
    uint32_t psti030_count;
    uint32_t psti032_count;
    uint32_t psti035_count;
    uint32_t ths_count;
    uint32_t checksum_error_count;
    uint32_t parse_error_count;
    bool fix_valid;
    int fix_quality;
    uint8_t fix_type;
    uint8_t satellites;
    uint8_t satellites_in_view;
    double hdop;
    double latitude_deg;
    double longitude_deg;
    double altitude_m;
    double speed_mps;
    double course_deg;
    double rtk_age_s;
    double rtk_ratio;
    bool baseline_valid;
    uint8_t baseline_source;
    double baseline_east_m;
    double baseline_north_m;
    double baseline_up_m;
    double baseline_length_m;
    double baseline_course_deg;
    char baseline_status;
    char baseline_mode;
    bool true_heading_valid;
    double true_heading_deg;
    char true_heading_mode;
    bool moving_base_active;
    bool moving_base_correction_uart_ready;
    bool moving_base_receiver_config_sent;
    uint32_t moving_base_receiver_ack_count;
    uint32_t moving_base_receiver_nack_count;
    uint8_t moving_base_receiver_last_ack_id;
    uint8_t moving_base_receiver_last_nack_id;
    uint32_t moving_base_uplink_packet_count;
    uint32_t moving_base_uplink_byte_count;
    uint32_t moving_base_uplink_error_count;
    uint32_t moving_base_downlink_packet_count;
    uint32_t moving_base_downlink_byte_count;
    uint32_t moving_base_downlink_error_count;
    uint32_t moving_base_downlink_gap_count;
    uint32_t moving_base_last_uplink_age_ms;
    uint32_t moving_base_last_downlink_age_ms;
    uint32_t moving_base_skytraq_frame_count;
    bool moving_base_software_version_valid;
    uint8_t moving_base_software_type;
    uint32_t moving_base_software_kernel_version;
    uint32_t moving_base_software_odm_version;
    uint32_t moving_base_software_revision;
    bool moving_base_binary_output_status_valid;
    uint8_t moving_base_binary_output_rate_code;
    bool moving_base_binary_meas_time_enabled;
    bool moving_base_binary_raw_meas_enabled;
    uint32_t moving_base_binary_meas_time_count;
    uint32_t moving_base_binary_raw_meas_count;
    uint32_t moving_base_rtcm_preamble_count;
    uint8_t moving_base_last_downlink_source_id;
    char moving_base_role[20];
    char rmc_status;
    char rmc_mode;
    char utc_time[16];
    char utc_date[8];
    char last_sentence_id[8];
} gps_service_snapshot_t;

esp_err_t gps_service_start(void);
esp_err_t gps_service_apply_runtime_config(void);
/* Temporarily release the primary GNSS UART without changing the persisted
 * runtime configuration or powering the receiver down.  These calls are used
 * by the firmware updater, which must have exclusive access to UART1. */
esp_err_t gps_service_suspend_primary_uart(void);
/* Power-cycle only the GNSS receiver while the primary UART is suspended.
 * This guarantees that a previous interrupted download cannot leave the
 * receiver in its loader before a new update starts. */
esp_err_t gps_service_reset_receiver_for_update(void);
esp_err_t gps_service_resume_primary_uart(void);
void gps_service_get_snapshot(gps_service_snapshot_t *snapshot);
const char *gps_service_fix_quality_to_string(int quality);

#ifdef __cplusplus
}
#endif

#endif /* GPS_SERVICE_H */
