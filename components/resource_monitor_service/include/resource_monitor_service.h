#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool running;
    bool cpu_load_valid;
    bool temperature_valid;
    uint32_t update_count;
    uint32_t last_update_age_ms;

    size_t heap_free_bytes;
    size_t heap_total_bytes;
    size_t heap_min_free_bytes;
    size_t heap_largest_free_block_bytes;
    size_t internal_free_bytes;
    size_t internal_total_bytes;
    size_t internal_min_free_bytes;
    size_t internal_largest_free_block_bytes;
    size_t psram_free_bytes;
    size_t psram_total_bytes;
    size_t psram_min_free_bytes;
    size_t psram_largest_free_block_bytes;

    float core0_load_percent;
    float core1_load_percent;
    float temperature_c;
    esp_err_t temperature_error;
} resource_monitor_snapshot_t;

esp_err_t resource_monitor_service_start(void);
void resource_monitor_service_get_snapshot(resource_monitor_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif
