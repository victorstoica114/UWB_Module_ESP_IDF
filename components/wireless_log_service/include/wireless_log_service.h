#ifndef WIRELESS_LOG_SERVICE_H
#define WIRELESS_LOG_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

enum wireless_log_status {
    WIRELESS_LOG_STATUS_DISABLED = 0,
    WIRELESS_LOG_STATUS_IDLE,
    WIRELESS_LOG_STATUS_WAITING_FOR_WIFI,
    WIRELESS_LOG_STATUS_CONNECTING,
    WIRELESS_LOG_STATUS_CONNECTED,
    WIRELESS_LOG_STATUS_FAILED,
};

esp_err_t wireless_log_service_start(void);
enum wireless_log_status wireless_log_service_get_status(void);
const char *wireless_log_service_status_to_string(
    enum wireless_log_status status);
bool wireless_log_service_is_connected(void);
const char *wireless_log_service_get_target(void);
uint16_t wireless_log_service_get_port(void);
uint32_t wireless_log_service_get_dropped_count(void);
bool wireless_log_service_submit(char level, const char *tag,
                                 const char *format, ...)
    __attribute__((format(printf, 3, 4)));

#ifdef __cplusplus
}
#endif

#endif
