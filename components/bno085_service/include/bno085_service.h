#ifndef BNO085_SERVICE_H
#define BNO085_SERVICE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bno085_service_start(void);
esp_err_t bno085_service_apply_runtime_config(void);

#ifdef __cplusplus
}
#endif

#endif /* BNO085_SERVICE_H */
