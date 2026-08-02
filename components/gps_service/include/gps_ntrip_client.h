#ifndef GPS_NTRIP_CLIENT_H
#define GPS_NTRIP_CLIENT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*gps_ntrip_write_fn_t)(const uint8_t *data, size_t length,
                                    void *context);

typedef struct {
    bool configured;
    bool running;
    bool tls_connected;
    bool stream_active;
    uint16_t http_status;
    uint32_t connect_count;
    uint32_t reconnect_count;
    uint32_t error_count;
    uint32_t rtcm_frame_count;
    uint32_t rtcm_byte_count;
    uint32_t last_data_age_ms;
    char state[24];
} gps_ntrip_snapshot_t;

bool gps_ntrip_client_is_enabled(void);
bool gps_ntrip_client_is_enabled_for_module(uint8_t module_id);
esp_err_t gps_ntrip_client_start(uint8_t module_id,
                                 gps_ntrip_write_fn_t write_fn,
                                 void *write_context);
void gps_ntrip_client_stop(void);
void gps_ntrip_client_set_gga(const char *sentence);
void gps_ntrip_client_get_snapshot(gps_ntrip_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* GPS_NTRIP_CLIENT_H */
