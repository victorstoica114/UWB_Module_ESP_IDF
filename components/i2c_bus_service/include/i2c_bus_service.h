#ifndef I2C_BUS_SERVICE_H
#define I2C_BUS_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t realtime_waiters;
    uint32_t realtime_period_us;
    int32_t realtime_time_to_next_us;
    uint32_t realtime_lock_count;
    uint32_t background_lock_count;
    uint32_t background_deferred_count;
} i2c_bus_service_stats_t;

esp_err_t i2c_bus_service_get(i2c_master_bus_handle_t *bus);
bool i2c_bus_service_lock(TickType_t timeout);
bool i2c_bus_service_lock_realtime(TickType_t timeout);
bool i2c_bus_service_lock_background(TickType_t timeout);
void i2c_bus_service_unlock(void);
void i2c_bus_service_set_realtime_period_us(uint32_t period_us);
void i2c_bus_service_note_realtime_activity(void);
void i2c_bus_service_get_stats(i2c_bus_service_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* I2C_BUS_SERVICE_H */
