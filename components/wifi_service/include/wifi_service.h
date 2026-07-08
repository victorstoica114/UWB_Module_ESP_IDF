#ifndef WIFI_SERVICE_H
#define WIFI_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

enum wifi_service_status {
    WIFI_SERVICE_STATUS_IDLE = 0,
    WIFI_SERVICE_STATUS_CONNECTING,
    WIFI_SERVICE_STATUS_CONNECTED,
    WIFI_SERVICE_STATUS_FAILED,
};

esp_err_t wifi_service_start(void);
bool wifi_service_is_connected(void);
enum wifi_service_status wifi_service_get_status(void);
const char *wifi_service_get_ip_address(void);
uint32_t wifi_service_get_disconnect_count(void);
uint16_t wifi_service_get_last_disconnect_reason(void);
const char *wifi_service_get_last_disconnect_reason_name(void);
const char *wifi_service_get_connected_bssid(void);
uint8_t wifi_service_get_connected_channel(void);
int wifi_service_get_connected_rssi(void);
uint16_t wifi_service_get_last_scan_ap_count(void);
const char *wifi_service_get_last_scan_best_bssid(void);
uint8_t wifi_service_get_last_scan_best_channel(void);
int wifi_service_get_last_scan_best_rssi(void);

#ifdef __cplusplus
}
#endif

#endif
