#ifndef BNO085_SERVICE_H
#define BNO085_SERVICE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool service_started;
    bool int_irq_enabled;
    uint32_t i2c_clock_hz;
    uint32_t accel_interval_ms;
    uint32_t report_count;
    uint32_t packet_count;
    uint32_t input_packet_count;
    uint32_t timebase_count;
    uint32_t max_reports_per_packet;
    uint32_t continuation_packet_count;
    uint32_t continuation_transfer_count;
    uint32_t continuation_header_error_count;
    uint32_t high_rate_poll_count;
    uint32_t wait_immediate_count;
    uint32_t wait_notify_count;
    uint32_t wait_late_active_count;
    uint32_t null_header_count;
    uint32_t read_error_count;
    uint32_t parse_error_count;
    uint32_t int_irq_count;
    uint32_t int_wait_timeout_count;
    size_t last_packet_len;
    size_t last_input_payload_len;
    float last_x_mps2;
    float last_y_mps2;
    float last_z_mps2;
    uint8_t last_accuracy;
    int i2c_scl_measure_error;
    uint32_t i2c_scl_edges;
    uint32_t i2c_scl_elapsed_us;
    uint32_t i2c_scl_measured_hz;
} bno085_service_snapshot_t;

esp_err_t bno085_service_start(void);
esp_err_t bno085_service_apply_runtime_config(void);
void bno085_service_get_snapshot(bno085_service_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* BNO085_SERVICE_H */
