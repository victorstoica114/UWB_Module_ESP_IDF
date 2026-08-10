#ifndef BNO085_SERVICE_H
#define BNO085_SERVICE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    BNO085_ACCEL_TIME_SH2_VALID = 1U << 0,
    BNO085_ACCEL_TIME_MONOTONIC_REPAIRED = 1U << 1,
    BNO085_ACCEL_TIME_HINT_EXACT = 1U << 2,
    BNO085_ACCEL_TIME_HINT_ESTIMATED = 1U << 3,
};

typedef struct {
    uint64_t fusion_time_ticks;
    uint32_t sequence;
    int32_t x_milli_mps2;
    int32_t y_milli_mps2;
    int32_t z_milli_mps2;
    /* Native BNO085 Q8 values; retained so telemetry compression is lossless
     * relative to the sensor report. */
    int16_t x_q8;
    int16_t y_q8;
    int16_t z_q8;
    uint16_t sensor_delay_100us;
    uint8_t accuracy;
    uint8_t time_flags;
} bno085_accel_sample_t;

typedef struct {
    uint64_t fusion_time_ticks;
    uint32_t sequence;
    int16_t quat_i_q14;
    int16_t quat_j_q14;
    int16_t quat_k_q14;
    int16_t quat_real_q14;
    int16_t gyro_x_q10;
    int16_t gyro_y_q10;
    int16_t gyro_z_q10;
    uint8_t time_flags;
} bno085_gyro_rv_sample_t;

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
    uint32_t sample_ring_capacity;
    uint32_t sample_ring_count;
    uint32_t sample_overwrite_count;
    uint32_t telemetry_submit_count;
    uint32_t telemetry_drop_count;
    uint32_t telemetry_decimated_count;
    uint32_t monotonic_repair_count;
    uint32_t gyro_rv_report_count;
    uint32_t gyro_rv_ring_capacity;
    uint32_t gyro_rv_ring_count;
    uint32_t gyro_rv_overwrite_count;
    uint32_t gyro_rv_rate_hz;
    uint32_t telemetry_rate_hz;
    uint32_t fusion_timer_hz;
    uint64_t last_fusion_time_ticks;
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
uint64_t bno085_service_fusion_time_ticks(void);
size_t bno085_service_copy_accel_samples(uint32_t after_sequence,
                                         bno085_accel_sample_t *samples,
                                         size_t max_samples);
size_t bno085_service_copy_gyro_rv_samples(uint32_t after_sequence,
                                           bno085_gyro_rv_sample_t *samples,
                                           size_t max_samples);

#ifdef __cplusplus
}
#endif

#endif /* BNO085_SERVICE_H */
