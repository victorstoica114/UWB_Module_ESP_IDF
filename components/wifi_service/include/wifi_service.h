#ifndef WIFI_SERVICE_H
#define WIFI_SERVICE_H

#include <stdbool.h>

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

#ifdef __cplusplus
}
#endif

#endif
