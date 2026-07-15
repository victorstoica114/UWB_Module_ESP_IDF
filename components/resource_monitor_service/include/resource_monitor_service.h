#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RESOURCE_MONITOR_TOP_TASK_COUNT 6
#define RESOURCE_MONITOR_TASK_NAME_LEN 24

typedef struct {
    char name[RESOURCE_MONITOR_TASK_NAME_LEN];
    int32_t core_id;
    uint64_t runtime_delta_us;
    float load_percent;
} resource_monitor_task_load_t;

typedef struct {
    bool running;
    bool cpu_load_valid;
    bool task_load_valid;
    bool task_list_overflow;
    bool temperature_valid;
    bool flash_valid;
    uint32_t update_count;
    uint32_t last_update_age_ms;
    uint32_t top_task_count;

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
    size_t flash_total_bytes;
    size_t flash_reserved_bytes;
    size_t flash_free_bytes;
    uint32_t flash_partition_count;

    float core0_load_percent;
    float core1_load_percent;
    float temperature_c;
    esp_err_t temperature_error;
    esp_err_t flash_error;
    resource_monitor_task_load_t top_tasks[RESOURCE_MONITOR_TOP_TASK_COUNT];
} resource_monitor_snapshot_t;

esp_err_t resource_monitor_service_start(void);
void resource_monitor_service_get_snapshot(resource_monitor_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif
