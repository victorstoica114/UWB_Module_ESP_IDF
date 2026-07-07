#ifndef OTA_SERVICE_H
#define OTA_SERVICE_H

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

enum ota_service_status {
    OTA_SERVICE_STATUS_IDLE = 0,
    OTA_SERVICE_STATUS_WAITING_FOR_WIFI,
    OTA_SERVICE_STATUS_RUNNING,
    OTA_SERVICE_STATUS_UPDATING,
    OTA_SERVICE_STATUS_REBOOTING,
    OTA_SERVICE_STATUS_FAILED,
};

esp_err_t ota_service_start(void);
bool ota_service_is_running(void);
enum ota_service_status ota_service_get_status(void);

#ifdef __cplusplus
}
#endif

#endif
