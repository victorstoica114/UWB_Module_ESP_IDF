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
    char rmc_status;
    char rmc_mode;
    char utc_time[16];
    char utc_date[8];
    char last_sentence_id[8];
} gps_service_snapshot_t;

esp_err_t gps_service_start(void);
esp_err_t gps_service_apply_runtime_config(void);
void gps_service_get_snapshot(gps_service_snapshot_t *snapshot);
const char *gps_service_fix_quality_to_string(int quality);

#ifdef __cplusplus
}
#endif

#endif /* GPS_SERVICE_H */
