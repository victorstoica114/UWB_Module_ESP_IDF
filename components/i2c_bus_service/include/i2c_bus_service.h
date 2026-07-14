#ifndef I2C_BUS_SERVICE_H
#define I2C_BUS_SERVICE_H

#include <stdbool.h>
#include <stddef.h>
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
    int32_t background_window_us;
    uint32_t realtime_lock_count;
    uint32_t background_lock_count;
    uint32_t background_deferred_count;
} i2c_bus_service_stats_t;

typedef struct {
    uint32_t clock_hz;
    uint32_t timeout_us;
    uint32_t estimated_transfer_us;
    bool manual_timing;
    uint32_t scl_low_period;
    uint32_t scl_high_period;
    uint32_t scl_wait_high_period;
    bool hs_master_code;
    bool hs_master_stop;
    uint32_t hs_entry_clock_hz;
} i2c_bus_service_direct_config_t;

typedef struct {
    uint32_t elapsed_us;
    uint32_t int_raw;
    uint32_t command_done_mask;
    uint32_t hs_master_elapsed_us;
    uint32_t hs_master_int_raw;
    uint32_t hs_master_command_done_mask;
} i2c_bus_service_direct_result_t;

esp_err_t i2c_bus_service_get(i2c_master_bus_handle_t *bus);
bool i2c_bus_service_lock(TickType_t timeout);
bool i2c_bus_service_lock_realtime(TickType_t timeout);
bool i2c_bus_service_lock_background(TickType_t timeout);
bool i2c_bus_service_lock_background_for(TickType_t timeout,
                                         uint32_t estimated_transfer_us);
void i2c_bus_service_unlock(void);
void i2c_bus_service_set_realtime_period_us(uint32_t period_us);
void i2c_bus_service_note_realtime_activity(void);
int32_t i2c_bus_service_background_window_us(void);
void i2c_bus_service_get_stats(i2c_bus_service_stats_t *stats);
esp_err_t i2c_bus_service_direct_read_reg(
    uint8_t address, uint8_t start_reg, uint8_t *data, size_t data_len,
    const i2c_bus_service_direct_config_t *config,
    i2c_bus_service_direct_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* I2C_BUS_SERVICE_H */
