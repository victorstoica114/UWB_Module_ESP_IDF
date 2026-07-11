#ifndef I2C_BUS_SERVICE_H
#define I2C_BUS_SERVICE_H

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t i2c_bus_service_get(i2c_master_bus_handle_t *bus);
bool i2c_bus_service_lock(TickType_t timeout);
bool i2c_bus_service_lock_realtime(TickType_t timeout);
bool i2c_bus_service_lock_background(TickType_t timeout);
void i2c_bus_service_unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* I2C_BUS_SERVICE_H */
