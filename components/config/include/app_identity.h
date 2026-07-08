#ifndef APP_IDENTITY_H
#define APP_IDENTITY_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t app_identity_init(void);
const char *app_identity_get_hostname(void);
uint8_t app_identity_get_module_id(void);
uint8_t app_identity_get_uwb_role(void);
uint16_t app_identity_get_uwb_antenna_delay(void);
const char *app_identity_uwb_role_to_string(uint8_t role);
bool app_identity_module_id_from_nvs(void);
bool app_identity_module_id_provisioned_this_boot(void);
bool app_identity_uwb_role_from_nvs(void);
bool app_identity_uwb_role_provisioned_this_boot(void);
bool app_identity_uwb_antenna_delay_from_nvs(void);
esp_err_t app_identity_set_uwb_antenna_delay(uint16_t delay);
esp_err_t app_identity_clear_uwb_antenna_delay(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_IDENTITY_H */
