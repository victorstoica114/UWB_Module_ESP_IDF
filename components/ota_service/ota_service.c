#include "ota_service.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "flextdoa_solver_service.h"
#include "app_identity.h"
#include "app_runtime_config.h"
#include "bno085_service.h"
#include "boot_guard.h"
#include "charger_service.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps_service.h"
#include "gnss_firmware_updater.h"
#include "i2c_bus_service.h"
#include "max77958_service.h"
#include "resource_monitor_service.h"
#include "uwb_config.h"
#include "uwb_dw3000.h"
#include "wifi_service.h"
#include "wireless_log_service.h"
#include "wireless_telemetry_service.h"

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef APP_OTA_PASSWORD
#define APP_OTA_PASSWORD ""
#endif

static const char *TAG = "ota_service";

enum {
    OTA_SERVICE_TASK_STACK_WORDS = 4096,
    OTA_SERVICE_TASK_PRIORITY = 5,
    OTA_SERVICE_RESTART_TASK_STACK_WORDS = 2048,
    OTA_SERVICE_RESTART_TASK_PRIORITY = 5,
    OTA_SERVICE_CHUNK_SIZE = 4096,
    OTA_SERVICE_WIFI_WAIT_MS = 500,
    OTA_SERVICE_REBOOT_DELAY_MS = 1200,
    OTA_SERVICE_MAX_TOKEN_LEN = 128,
    OTA_SERVICE_MAX_QUERY_LEN = 768,
    OTA_SERVICE_STATUS_RESPONSE_SIZE = 32000,
    OTA_SERVICE_RUNTIME_RESPONSE_SIZE = 3072,
    GNSS_PX1105R_LOADER_SOURCE_SIZE = 67064,
    GNSS_PX1105R_PACKED_IMAGE_SIZE = 471243,
    GNSS_PX1105R_RAW_IMAGE_SIZE = 1116336,
    GNSS_PX1105R_PACKED_SUM8 = 189,
};

#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
#define OTA_SERVICE_HTTPD_TASK_CORE 0
#else
#define OTA_SERVICE_HTTPD_TASK_CORE 0
#endif

#define OTA_SERVICE_TOKEN_HEADER "X-OTA-Token"

static httpd_handle_t s_http_server;
static bool s_started;
static bool s_running;
static volatile bool s_ota_in_progress;
static volatile enum ota_service_status s_status = OTA_SERVICE_STATUS_IDLE;

typedef struct {
    uint8_t *data;
    size_t size;
} gnss_cached_firmware_t;

typedef struct {
    uint8_t *data;
    size_t size;
} gnss_cached_loader_t;

typedef struct {
    const uint8_t *data;
    size_t size;
    size_t offset;
} gnss_memory_reader_context_t;

static gnss_cached_firmware_t s_gnss_cached_firmware;
static gnss_cached_loader_t s_gnss_cached_loader;

static void gnss_release_update_cache(void)
{
    if (s_gnss_cached_firmware.data != NULL) {
        heap_caps_free(s_gnss_cached_firmware.data);
    }
    if (s_gnss_cached_loader.data != NULL) {
        heap_caps_free(s_gnss_cached_loader.data);
    }
    memset(&s_gnss_cached_firmware, 0, sizeof(s_gnss_cached_firmware));
    memset(&s_gnss_cached_loader, 0, sizeof(s_gnss_cached_loader));
}

typedef struct {
    i2c_bus_service_stats_t i2c_stats;
    bno085_service_snapshot_t bno_snapshot;
    gps_service_snapshot_t gps_snapshot;
    charger_service_snapshot_t charger_snapshot;
    max77958_service_snapshot_t pd_snapshot;
    resource_monitor_snapshot_t resource_snapshot;
    char charger_raw_hex[(CHARGER_SERVICE_REGISTER_MAP_SIZE * 2U) + 1U];
    char resource_top_tasks_json[1024];
    char pd_raw_hex[(MAX77958_SERVICE_REGISTER_MAP_SIZE * 2U) + 1U];
    char pd_last_response_hex[(MAX77958_SERVICE_AP_DATA_BYTES * 2U) + 1U];
    char pd_source_pdos_json[128];
    char pd_sink_pdos_json[96];
    char runtime_anchor_ids_json[48];
    char runtime_flex_slots_json[48];
    char runtime_flex_masks_json[72];
    char runtime_flex_anchor_x_json[128];
    char runtime_flex_anchor_y_json[128];
    char runtime_flex_anchor_correction_json[128];
    char runtime_passive_ds_anchor_bias_json[128];
    char runtime_passive_ds_range_bias_json[512];
    struct uwb_passive_ds_pipeline_stats passive_ds_pipeline_stats;
    char passive_ds_pipeline_stats_json[1536];
    struct uwb_native_ds_pipeline_stats native_ds_pipeline_stats;
    char native_ds_pipeline_stats_json[2048];
    char response[OTA_SERVICE_STATUS_RESPONSE_SIZE];
} ota_status_context_t;

static void reboot_task(void *arg);

static ota_status_context_t *alloc_status_context(void)
{
    return heap_caps_calloc(
        1, sizeof(ota_status_context_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

static uint8_t *alloc_ota_buffer(void)
{
    uint8_t *buffer = heap_caps_malloc(
        OTA_SERVICE_CHUNK_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (buffer == NULL) {
        buffer = heap_caps_malloc(OTA_SERVICE_CHUNK_SIZE, MALLOC_CAP_8BIT);
    }
    return buffer;
}

static const char *ota_status_to_string(enum ota_service_status status)
{
    switch (status) {
    case OTA_SERVICE_STATUS_IDLE:
        return "idle";
    case OTA_SERVICE_STATUS_WAITING_FOR_WIFI:
        return "waiting_for_wifi";
    case OTA_SERVICE_STATUS_RUNNING:
        return "running";
    case OTA_SERVICE_STATUS_UPDATING:
        return "updating";
    case OTA_SERVICE_STATUS_REBOOTING:
        return "rebooting";
    case OTA_SERVICE_STATUS_FAILED:
        return "failed";
    default:
        return "unknown";
    }
}

static const char *partition_label_or_unknown(const esp_partition_t *partition)
{
    return partition != NULL ? partition->label : "unknown";
}

static void format_u32_array_json(const uint32_t *values, size_t count,
                                  char *buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0) {
        return;
    }
    size_t offset = 0;
    int written = snprintf(buffer, buffer_size, "[");
    if (written < 0 || (size_t)written >= buffer_size) {
        buffer[0] = '\0';
        return;
    }
    offset = (size_t)written;
    for (size_t i = 0; i < count; ++i) {
        written = snprintf(&buffer[offset], buffer_size - offset,
                           "%s%lu", i == 0 ? "" : ",",
                           (unsigned long)values[i]);
        if (written < 0 || (size_t)written >= buffer_size - offset) {
            buffer[0] = '\0';
            return;
        }
        offset += (size_t)written;
    }
    if (offset + 2U <= buffer_size) {
        (void)snprintf(&buffer[offset], buffer_size - offset, "]");
    }
}

static void format_u8_array_json(const uint8_t *values, size_t count,
                                 char *buffer, size_t buffer_size)
{
    uint32_t expanded[APP_RUNTIME_CONFIG_MAX_ANCHORS] = {0};
    const size_t limit = count < APP_RUNTIME_CONFIG_MAX_ANCHORS
                             ? count
                             : APP_RUNTIME_CONFIG_MAX_ANCHORS;
    for (size_t i = 0; i < limit; ++i) {
        expanded[i] = values[i];
    }
    format_u32_array_json(expanded, limit, buffer, buffer_size);
}

static void format_u16_array_json(const uint16_t *values, size_t count,
                                  char *buffer, size_t buffer_size)
{
    uint32_t expanded[APP_RUNTIME_CONFIG_FLEX_MAX_SLOTS] = {0};
    const size_t limit = count < APP_RUNTIME_CONFIG_FLEX_MAX_SLOTS
                             ? count
                             : APP_RUNTIME_CONFIG_FLEX_MAX_SLOTS;
    for (size_t i = 0; i < limit; ++i) {
        expanded[i] = values[i];
    }
    format_u32_array_json(expanded, limit, buffer, buffer_size);
}

static void format_i32_array_json(const int32_t *values, size_t count,
                                  char *buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0) {
        return;
    }
    const size_t limit = count < APP_RUNTIME_CONFIG_MAX_ANCHORS
                             ? count
                             : APP_RUNTIME_CONFIG_MAX_ANCHORS;
    size_t offset = 0;
    int written = snprintf(buffer, buffer_size, "[");
    if (written < 0 || (size_t)written >= buffer_size) {
        buffer[0] = '\0';
        return;
    }
    offset = (size_t)written;
    for (size_t i = 0; i < limit; ++i) {
        written = snprintf(&buffer[offset], buffer_size - offset,
                           "%s%ld", i == 0 ? "" : ",",
                           (long)values[i]);
        if (written < 0 || (size_t)written >= buffer_size - offset) {
            buffer[0] = '\0';
            return;
        }
        offset += (size_t)written;
    }
    if (offset + 2U <= buffer_size) {
        (void)snprintf(&buffer[offset], buffer_size - offset, "]");
    }
}

static void format_json_string(const char *value, char *buffer,
                               size_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0) {
        return;
    }
    size_t offset = 0;
    buffer[offset++] = '"';
    if (value != NULL) {
        for (const char *cursor = value;
             *cursor != '\0' && offset + 2U < buffer_size; ++cursor) {
            const char ch = *cursor;
            if (ch == '"' || ch == '\\') {
                if (offset + 3U >= buffer_size) {
                    break;
                }
                buffer[offset++] = '\\';
                buffer[offset++] = ch;
            } else if ((unsigned char)ch < 0x20U) {
                buffer[offset++] = '?';
            } else {
                buffer[offset++] = ch;
            }
        }
    }
    if (offset < buffer_size) {
        buffer[offset++] = '"';
    }
    if (offset < buffer_size) {
        buffer[offset] = '\0';
    } else {
        buffer[buffer_size - 1U] = '\0';
    }
}

static void format_resource_top_tasks_json(
    const resource_monitor_snapshot_t *snapshot, char *buffer,
    size_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0) {
        return;
    }
    size_t offset = 0;
    int written = snprintf(buffer, buffer_size, "[");
    if (written < 0 || (size_t)written >= buffer_size) {
        buffer[0] = '\0';
        return;
    }
    offset = (size_t)written;
    if (snapshot == NULL) {
        (void)snprintf(&buffer[offset], buffer_size - offset, "]");
        return;
    }

    const uint32_t count =
        snapshot->top_task_count > RESOURCE_MONITOR_TOP_TASK_COUNT
            ? RESOURCE_MONITOR_TOP_TASK_COUNT
            : snapshot->top_task_count;
    for (uint32_t i = 0; i < count; ++i) {
        const resource_monitor_task_load_t *task = &snapshot->top_tasks[i];
        char name_json[(RESOURCE_MONITOR_TASK_NAME_LEN * 2U) + 3U] = {0};
        format_json_string(task->name, name_json, sizeof(name_json));
        written = snprintf(&buffer[offset], buffer_size - offset,
                           "%s{\"name\":%s,\"core\":%ld,"
                           "\"load_percent\":%.1f,"
                           "\"runtime_delta_us\":%llu}",
                           i == 0 ? "" : ",", name_json,
                           (long)task->core_id, task->load_percent,
                           (unsigned long long)task->runtime_delta_us);
        if (written < 0 || (size_t)written >= buffer_size - offset) {
            buffer[0] = '\0';
            return;
        }
        offset += (size_t)written;
    }
    if (offset + 2U <= buffer_size) {
        (void)snprintf(&buffer[offset], buffer_size - offset, "]");
    }
}

static void format_bytes_hex(const uint8_t *data, size_t data_len,
                             char *buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0) {
        return;
    }
    buffer[0] = '\0';
    if (data == NULL) {
        return;
    }
    size_t offset = 0;
    for (size_t i = 0; i < data_len; ++i) {
        if (offset + 3U > buffer_size) {
            break;
        }
        const int written =
            snprintf(&buffer[offset], buffer_size - offset, "%02X", data[i]);
        if (written < 0) {
            buffer[0] = '\0';
            return;
        }
        offset += (size_t)written;
    }
}

static bool ota_token_configured(void)
{
    return strlen(APP_OTA_PASSWORD) > 0;
}

static bool constant_time_string_equal(const char *a, const char *b)
{
    const size_t a_len = strlen(a);
    const size_t b_len = strlen(b);
    const size_t max_len = a_len > b_len ? a_len : b_len;
    unsigned char diff = (unsigned char)(a_len ^ b_len);

    for (size_t i = 0; i < max_len; ++i) {
        const unsigned char ca = i < a_len ? (unsigned char)a[i] : 0;
        const unsigned char cb = i < b_len ? (unsigned char)b[i] : 0;
        diff |= (unsigned char)(ca ^ cb);
    }

    return diff == 0;
}

static bool ota_request_authorized(httpd_req_t *req)
{
    if (!ota_token_configured()) {
        return false;
    }

    const size_t token_len =
        httpd_req_get_hdr_value_len(req, OTA_SERVICE_TOKEN_HEADER);
    if (token_len == 0 || token_len >= OTA_SERVICE_MAX_TOKEN_LEN) {
        return false;
    }

    char token[OTA_SERVICE_MAX_TOKEN_LEN];
    if (httpd_req_get_hdr_value_str(req, OTA_SERVICE_TOKEN_HEADER, token,
                                    sizeof(token)) != ESP_OK) {
        return false;
    }

    return constant_time_string_equal(token, APP_OTA_PASSWORD);
}

static bool ota_parse_u16(const char *text, uint16_t *value)
{
    if (text == NULL || text[0] == '\0' || value == NULL) {
        return false;
    }

    errno = 0;
    char *end = NULL;
    const unsigned long parsed = strtoul(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || parsed > 0xFFFFUL) {
        return false;
    }

    *value = (uint16_t)parsed;
    return true;
}

static bool ota_parse_u8(const char *text, uint8_t *value)
{
    if (text == NULL || text[0] == '\0' || value == NULL) {
        return false;
    }

    errno = 0;
    char *end = NULL;
    const unsigned long parsed = strtoul(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || parsed > 0xFFUL) {
        return false;
    }

    *value = (uint8_t)parsed;
    return true;
}

static bool ota_parse_u32(const char *text, uint32_t *value)
{
    if (text == NULL || text[0] == '\0' || value == NULL) {
        return false;
    }

    errno = 0;
    char *end = NULL;
    const unsigned long parsed = strtoul(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' ||
        parsed > 0xFFFFFFFFUL) {
        return false;
    }

    *value = (uint32_t)parsed;
    return true;
}

static bool ota_parse_bool_text(const char *text, bool *value)
{
    if (text == NULL || value == NULL) {
        return false;
    }
    if (strcmp(text, "1") == 0 || strcmp(text, "true") == 0 ||
        strcmp(text, "on") == 0 || strcmp(text, "yes") == 0) {
        *value = true;
        return true;
    }
    if (strcmp(text, "0") == 0 || strcmp(text, "false") == 0 ||
        strcmp(text, "off") == 0 || strcmp(text, "no") == 0) {
        *value = false;
        return true;
    }
    return false;
}

static bool ota_parse_pd_fixed_pdo_list(char *text, uint16_t *voltages_mv,
                                        uint16_t *currents_ma,
                                        size_t max_count, size_t *count)
{
    if (text == NULL || voltages_mv == NULL || currents_ma == NULL ||
        count == NULL || max_count == 0) {
        return false;
    }

    size_t local_count = 0;
    char *save = NULL;
    for (char *token = strtok_r(text, ",", &save); token != NULL;
         token = strtok_r(NULL, ",", &save)) {
        if (local_count >= max_count) {
            return false;
        }

        char *separator = strchr(token, ':');
        if (separator == NULL) {
            separator = strchr(token, '/');
        }
        if (separator == NULL) {
            return false;
        }
        *separator = '\0';

        uint16_t mv = 0;
        uint16_t ma = 0;
        if (!ota_parse_u16(token, &mv) ||
            !ota_parse_u16(separator + 1, &ma)) {
            return false;
        }

        voltages_mv[local_count] = mv;
        currents_ma[local_count] = ma;
        local_count++;
    }

    if (local_count == 0) {
        return false;
    }
    *count = local_count;
    return true;
}

static bool ota_parse_bno085_sample_hz(const char *text, uint32_t *interval_ms)
{
    uint32_t hz = 0;
    if (!ota_parse_u32(text, &hz) || hz == 0 || hz > 500U ||
        interval_ms == NULL) {
        return false;
    }

    uint32_t ms = (1000U + (hz / 2U)) / hz;
    if (ms < 2U) {
        ms = 2U;
    }
    *interval_ms = ms;
    return true;
}

static bool ota_parse_u8_list(const char *text, uint8_t *values,
                              size_t max_count, uint8_t *count)
{
    if (text == NULL || text[0] == '\0' || values == NULL ||
        count == NULL || max_count == 0) {
        return false;
    }

    const char *cursor = text;
    uint8_t parsed_count = 0;

    while (*cursor != '\0') {
        while (*cursor == ' ') {
            cursor++;
        }

        errno = 0;
        char *end = NULL;
        const unsigned long parsed = strtoul(cursor, &end, 0);
        if (errno != 0 || end == cursor || parsed > 0xFFUL ||
            parsed_count >= max_count) {
            return false;
        }

        values[parsed_count++] = (uint8_t)parsed;
        cursor = end;
        while (*cursor == ' ') {
            cursor++;
        }

        if (*cursor == '\0') {
            break;
        }
        if (*cursor != ',' && *cursor != ';' && *cursor != ':') {
            return false;
        }
        cursor++;
    }

    if (parsed_count == 0) {
        return false;
    }

    *count = parsed_count;
    return true;
}

static bool ota_parse_u16_list(const char *text, uint16_t *values,
                               size_t max_count, uint8_t *count)
{
    if (text == NULL || text[0] == '\0' || values == NULL || count == NULL ||
        max_count == 0U) {
        return false;
    }

    const char *cursor = text;
    uint8_t parsed_count = 0;
    while (*cursor != '\0') {
        while (*cursor == ' ') {
            cursor++;
        }
        errno = 0;
        char *end = NULL;
        const unsigned long parsed = strtoul(cursor, &end, 0);
        if (errno != 0 || end == cursor || parsed > UINT16_MAX ||
            parsed_count >= max_count) {
            return false;
        }
        values[parsed_count++] = (uint16_t)parsed;
        cursor = end;
        while (*cursor == ' ') {
            cursor++;
        }
        if (*cursor == '\0') {
            break;
        }
        if (*cursor != ',' && *cursor != ';') {
            return false;
        }
        cursor++;
    }
    *count = parsed_count;
    return parsed_count > 0U;
}

static bool ota_parse_i32_list(const char *text, int32_t *values,
                               size_t max_count, uint8_t *count)
{
    if (text == NULL || text[0] == '\0' || values == NULL ||
        count == NULL || max_count == 0U || max_count > UINT8_MAX) {
        return false;
    }

    const char *cursor = text;
    uint8_t parsed_count = 0;
    while (*cursor != '\0') {
        while (*cursor == ' ') {
            cursor++;
        }
        errno = 0;
        char *end = NULL;
        const long parsed = strtol(cursor, &end, 0);
        if (errno != 0 || end == cursor || parsed < INT32_MIN ||
            parsed > INT32_MAX || parsed_count >= max_count) {
            return false;
        }
        values[parsed_count++] = (int32_t)parsed;
        cursor = end;
        while (*cursor == ' ') {
            cursor++;
        }
        if (*cursor == '\0') {
            break;
        }
        if (*cursor != ',' && *cursor != ';') {
            return false;
        }
        cursor++;
    }
    *count = parsed_count;
    return parsed_count > 0U;
}

static void format_passive_ds_range_bias_json(
    const app_runtime_config_t *config, char *buffer, size_t buffer_size)
{
    if (config == NULL || buffer == NULL || buffer_size == 0U) {
        return;
    }
    size_t used = 0;
    int written = snprintf(buffer, buffer_size, "[");
    if (written < 0 || (size_t)written >= buffer_size) {
        buffer[0] = '\0';
        return;
    }
    used = (size_t)written;
    bool first = true;
    for (size_t a = 0; a < config->anchor_count; ++a) {
        for (size_t b = a + 1U; b < config->anchor_count; ++b) {
            const size_t pair =
                app_runtime_config_anchor_pair_index(a, b);
            if (pair == SIZE_MAX) {
                buffer[0] = '\0';
                return;
            }
            written = snprintf(
                buffer + used, buffer_size - used, "%s%ld",
                first ? "" : ",",
                (long)config->passive_ds_range_bias_mm[pair]);
            if (written < 0 || (size_t)written >= buffer_size - used) {
                buffer[0] = '\0';
                return;
            }
            used += (size_t)written;
            first = false;
        }
    }
    if (used + 2U > buffer_size) {
        buffer[0] = '\0';
        return;
    }
    buffer[used++] = ']';
    buffer[used] = '\0';
}

static void format_passive_ds_stage_stats_json(
    const struct uwb_passive_ds_stage_stats *stats, char *buffer,
    size_t buffer_size)
{
    if (stats == NULL || buffer == NULL || buffer_size == 0U) {
        return;
    }
    const uint64_t average_us =
        stats->count != 0U ? stats->total_duration_us / stats->count : 0U;
    (void)snprintf(
        buffer, buffer_size,
        "{\"count\":%lu,\"fail\":%lu,\"last_us\":%lu,"
        "\"avg_us\":%llu,\"max_us\":%lu,\"last_event_us\":%lld}",
        (unsigned long)stats->count,
        (unsigned long)stats->failure_count,
        (unsigned long)stats->last_duration_us,
        (unsigned long long)average_us,
        (unsigned long)stats->max_duration_us,
        (long long)stats->last_event_host_us);
}

static void format_passive_ds_pipeline_stats_json(
    const struct uwb_passive_ds_pipeline_stats *stats, char *buffer,
    size_t buffer_size)
{
    if (stats == NULL || buffer == NULL || buffer_size == 0U) {
        return;
    }
    char poll[192] = {0};
    char response[192] = {0};
    char final_tx[192] = {0};
    char final_rx[192] = {0};
    char cia[192] = {0};
    char rearm[192] = {0};
    format_passive_ds_stage_stats_json(
        &stats->poll_tx, poll, sizeof(poll));
    format_passive_ds_stage_stats_json(
        &stats->response_tx, response, sizeof(response));
    format_passive_ds_stage_stats_json(
        &stats->final_tx, final_tx, sizeof(final_tx));
    format_passive_ds_stage_stats_json(
        &stats->final_rx, final_rx, sizeof(final_rx));
    format_passive_ds_stage_stats_json(
        &stats->cia_read, cia, sizeof(cia));
    format_passive_ds_stage_stats_json(
        &stats->rx_rearm, rearm, sizeof(rearm));
    (void)snprintf(
        buffer, buffer_size,
        "{\"deadline_active\":%s,\"completed\":%lu,"
        "\"response_timeouts\":%lu,\"final_timeouts\":%lu,"
        "\"invalid_frames\":%lu,\"state_collisions\":%lu,"
        "\"schedule_alarms\":%lu,\"schedule_overruns\":%lu,"
        "\"rx_rearm_failures\":%lu,\"stages\":{"
        "\"poll_tx\":%s,\"response_tx\":%s,\"final_tx\":%s,"
        "\"final_rx\":%s,\"cia_read\":%s,\"rx_rearm\":%s}}",
        stats->deadline_pipeline_active ? "true" : "false",
        (unsigned long)stats->completed_exchange_count,
        (unsigned long)stats->response_timeout_count,
        (unsigned long)stats->final_timeout_count,
        (unsigned long)stats->invalid_frame_count,
        (unsigned long)stats->state_collision_count,
        (unsigned long)stats->schedule_alarm_count,
        (unsigned long)stats->schedule_overrun_count,
        (unsigned long)stats->rx_rearm_failure_count,
        poll, response, final_tx, final_rx, cia, rearm);
}

static void format_native_ds_pipeline_stats_json(
    const struct uwb_native_ds_pipeline_stats *stats, char *buffer,
    size_t buffer_size)
{
    if (stats == NULL || buffer == NULL || buffer_size == 0U) {
        return;
    }
    (void)snprintf(
        buffer, buffer_size,
        "{\"poll_tx\":%lu,\"poll_rx\":%lu,"
        "\"response_tx\":%lu,\"response_rx\":%lu,"
        "\"final_tx\":%lu,\"final_rx\":%lu,"
        "\"completed_ranges\":%lu,\"rx_timeouts\":%lu,"
        "\"invalid_frames\":%lu,\"delayed_tx_errors\":%lu,"
        "\"rejected_ranges\":%lu,\"slot_overruns\":%lu,"
        "\"last_distance_mm\":%ld}",
        (unsigned long)stats->poll_tx_count,
        (unsigned long)stats->poll_rx_count,
        (unsigned long)stats->response_tx_count,
        (unsigned long)stats->response_rx_count,
        (unsigned long)stats->final_tx_count,
        (unsigned long)stats->final_rx_count,
        (unsigned long)stats->completed_range_count,
        (unsigned long)stats->rx_timeout_count,
        (unsigned long)stats->invalid_frame_count,
        (unsigned long)stats->delayed_tx_error_count,
        (unsigned long)stats->rejected_range_count,
        (unsigned long)stats->slot_overrun_count,
        (long)stats->last_distance_mm);
}

static bool ota_parse_flex_geometry(
    const char *text, const app_runtime_config_t *config,
    int32_t x_mm[APP_RUNTIME_CONFIG_MAX_ANCHORS],
    int32_t y_mm[APP_RUNTIME_CONFIG_MAX_ANCHORS])
{
    if (text == NULL || config == NULL || x_mm == NULL || y_mm == NULL) {
        return false;
    }
    bool seen[APP_RUNTIME_CONFIG_MAX_ANCHORS] = {0};
    const char *cursor = text;
    size_t parsed_count = 0;
    while (*cursor != '\0') {
        errno = 0;
        char *end = NULL;
        const unsigned long id = strtoul(cursor, &end, 10);
        if (errno != 0 || end == cursor || id > UINT8_MAX || *end != ':') {
            return false;
        }
        cursor = end + 1;
        errno = 0;
        const long x = strtol(cursor, &end, 10);
        if (errno != 0 || end == cursor || x < INT32_MIN || x > INT32_MAX ||
            *end != ':') {
            return false;
        }
        cursor = end + 1;
        errno = 0;
        const long y = strtol(cursor, &end, 10);
        if (errno != 0 || end == cursor || y < INT32_MIN || y > INT32_MAX) {
            return false;
        }
        size_t index = SIZE_MAX;
        for (size_t i = 0; i < config->anchor_count; ++i) {
            if (config->anchor_ids[i] == (uint8_t)id) {
                index = i;
                break;
            }
        }
        if (index == SIZE_MAX || seen[index]) {
            return false;
        }
        seen[index] = true;
        x_mm[index] = (int32_t)x;
        y_mm[index] = (int32_t)y;
        parsed_count++;
        cursor = end;
        if (*cursor == '\0') {
            break;
        }
        if (*cursor != ',') {
            return false;
        }
        cursor++;
    }
    return parsed_count == config->anchor_count;
}

static bool ota_parse_calibration_method(const char *text, uint8_t *value)
{
    uint8_t parsed = 0;
    if (ota_parse_u8(text, &parsed) &&
        (parsed == APP_UWB_CALIBRATION_METHOD_TWO_MODULE ||
         parsed == APP_UWB_CALIBRATION_METHOD_THREE_MODULE)) {
        *value = parsed;
        return true;
    }

    if (strcmp(text, "two") == 0 || strcmp(text, "two_module") == 0) {
        *value = APP_UWB_CALIBRATION_METHOD_TWO_MODULE;
        return true;
    }
    if (strcmp(text, "three") == 0 || strcmp(text, "three_module") == 0 ||
        strcmp(text, "three_module_edm") == 0 ||
        strcmp(text, "edm") == 0) {
        *value = APP_UWB_CALIBRATION_METHOD_THREE_MODULE;
        return true;
    }

    return false;
}

static bool ota_query_option_enabled(const char *query, const char *key)
{
    char value[8] = {0};
    return httpd_query_key_value(query, key, value, sizeof(value)) == ESP_OK &&
           strcmp(value, "1") == 0;
}

static bool ota_query_has_key(const char *query, const char *key)
{
    char value[8] = {0};
    return httpd_query_key_value(query, key, value, sizeof(value)) == ESP_OK;
}

static bool runtime_config_reboot_recommended(
    const app_runtime_config_t *before, const app_runtime_config_t *after)
{
    if (before == NULL || after == NULL) {
        return true;
    }

    return before->runtime_mode != after->runtime_mode ||
           before->tag_id != after->tag_id ||
           before->anchor_count != after->anchor_count ||
           memcmp(before->anchor_ids, after->anchor_ids,
                  sizeof(before->anchor_ids)) != 0 ||
           before->flex_tdoa_responder_count !=
               after->flex_tdoa_responder_count ||
           before->flex_tdoa_slot_count != after->flex_tdoa_slot_count ||
           memcmp(before->flex_tdoa_slot_initiator_ids,
                  after->flex_tdoa_slot_initiator_ids,
                  sizeof(before->flex_tdoa_slot_initiator_ids)) != 0 ||
           memcmp(before->flex_tdoa_slot_responder_masks,
                  after->flex_tdoa_slot_responder_masks,
                  sizeof(before->flex_tdoa_slot_responder_masks)) != 0 ||
           before->flex_tdoa_guard_us != after->flex_tdoa_guard_us ||
           before->flex_tdoa_request_subslot_us !=
               after->flex_tdoa_request_subslot_us ||
           before->flex_tdoa_request_process_us !=
               after->flex_tdoa_request_process_us ||
           before->flex_tdoa_response_subslot_us !=
               after->flex_tdoa_response_subslot_us ||
           before->flex_tdoa_response_process_us !=
               after->flex_tdoa_response_process_us ||
           before->passive_ds_schedule != after->passive_ds_schedule ||
           before->passive_ds_slot_ms != after->passive_ds_slot_ms ||
           before->passive_ds_round_gap_ms !=
               after->passive_ds_round_gap_ms ||
           before->passive_ds_rx_slice_ms !=
               after->passive_ds_rx_slice_ms ||
           before->passive_ds_rx_timeout_ms !=
               after->passive_ds_rx_timeout_ms ||
           before->passive_ds_resp_delay_us !=
               after->passive_ds_resp_delay_us ||
           before->passive_ds_final_delay_us !=
               after->passive_ds_final_delay_us ||
           before->passive_ds_auto_rx_delay_uus !=
               after->passive_ds_auto_rx_delay_uus ||
           before->passive_ds_pipeline_mode !=
               after->passive_ds_pipeline_mode ||
           before->passive_ds_solve_mode !=
               after->passive_ds_solve_mode ||
           before->passive_ds_rolling_max_hz !=
               after->passive_ds_rolling_max_hz ||
           before->anchor_survey_coordinator_id !=
               after->anchor_survey_coordinator_id ||
           before->uwb_enabled != after->uwb_enabled ||
           before->radio_channel != after->radio_channel ||
           before->radio_phy_mode != after->radio_phy_mode;
}

static bool runtime_config_hot_switch_eligible(
    const app_runtime_config_t *before, const app_runtime_config_t *after)
{
    if (before == NULL || after == NULL ||
        !before->uwb_enabled || !after->uwb_enabled ||
        !uwb_dw3000_hot_switch_mode_supported(before->runtime_mode) ||
        !uwb_dw3000_hot_switch_mode_supported(after->runtime_mode)) {
        return false;
    }

    bool passive_mode_valid = false;
    const uint8_t passive_mode =
        app_runtime_config_runtime_mode_from_string(
            "passive_ds_twr", &passive_mode_valid);
    if (passive_mode_valid &&
        before->runtime_mode == passive_mode &&
        after->runtime_mode == passive_mode) {
        app_runtime_config_t allowed = *before;
        allowed.passive_ds_schedule = after->passive_ds_schedule;
        allowed.passive_ds_slot_ms = after->passive_ds_slot_ms;
        allowed.passive_ds_round_gap_ms =
            after->passive_ds_round_gap_ms;
        allowed.passive_ds_rx_slice_ms =
            after->passive_ds_rx_slice_ms;
        allowed.passive_ds_rx_timeout_ms =
            after->passive_ds_rx_timeout_ms;
        allowed.passive_ds_resp_delay_us =
            after->passive_ds_resp_delay_us;
        allowed.passive_ds_final_delay_us =
            after->passive_ds_final_delay_us;
        allowed.passive_ds_auto_rx_delay_uus =
            after->passive_ds_auto_rx_delay_uus;
        allowed.passive_ds_pipeline_mode =
            after->passive_ds_pipeline_mode;
        allowed.passive_ds_solve_mode =
            after->passive_ds_solve_mode;
        allowed.passive_ds_rolling_max_hz =
            after->passive_ds_rolling_max_hz;
        allowed.from_nvs = after->from_nvs;
        return memcmp(&allowed, after, sizeof(allowed)) == 0;
    }

    const uint8_t *before_bytes = (const uint8_t *)before;
    const uint8_t *after_bytes = (const uint8_t *)after;
    const size_t mode_begin =
        offsetof(app_runtime_config_t, runtime_mode);
    const size_t mode_end =
        mode_begin + sizeof(before->runtime_mode);
    const size_t source_begin =
        offsetof(app_runtime_config_t, from_nvs);
    const size_t source_end =
        source_begin + sizeof(before->from_nvs);
    for (size_t index = 0; index < sizeof(*before); ++index) {
        if ((index >= mode_begin && index < mode_end) ||
            (index >= source_begin && index < source_end)) {
            continue;
        }
        if (before_bytes[index] != after_bytes[index]) {
            return false;
        }
    }
    return true;
}

static uint8_t runtime_radio_channel(const app_runtime_config_t *config)
{
    return config != NULL && config->radio_channel == 9U ? 9U : 5U;
}

static uint8_t runtime_radio_rf_channel_bit(const app_runtime_config_t *config)
{
    return runtime_radio_channel(config) == 9U ? 1U : 0U;
}

static uint8_t runtime_radio_phy_mode(const app_runtime_config_t *config)
{
    if (config != NULL) {
        switch (config->radio_phy_mode) {
        case APP_UWB_RADIO_PHY_LONG_RANGE:
        case APP_UWB_RADIO_PHY_FAST_PLEN256:
        case APP_UWB_RADIO_PHY_FAST_PLEN512:
        case APP_UWB_RADIO_PHY_850K_PLEN512:
        case APP_UWB_RADIO_PHY_850K_PLEN512_STD_SFD:
            return config->radio_phy_mode;
        default:
            break;
        }
    }
    return APP_UWB_RADIO_PHY_FAST;
}

static bool runtime_radio_is_850k(const app_runtime_config_t *config)
{
    const uint8_t mode = runtime_radio_phy_mode(config);
    return mode == APP_UWB_RADIO_PHY_LONG_RANGE ||
           mode == APP_UWB_RADIO_PHY_850K_PLEN512 ||
           mode == APP_UWB_RADIO_PHY_850K_PLEN512_STD_SFD;
}

static uint8_t runtime_radio_profile(const app_runtime_config_t *config)
{
    const uint8_t mode = runtime_radio_phy_mode(config);
    if (mode == APP_UWB_RADIO_PHY_LONG_RANGE) {
        return runtime_radio_channel(config) == 9U
                   ? APP_UWB_RADIO_PROFILE_LONG_RANGE_CH9_850K_PLEN1024
                   : APP_UWB_RADIO_PROFILE_LONG_RANGE_CH5_850K_PLEN1024;
    }
    if (mode == APP_UWB_RADIO_PHY_FAST_PLEN256) {
        return runtime_radio_channel(config) == 9U
                   ? APP_UWB_RADIO_PROFILE_CH9_6M8_PLEN256
                   : APP_UWB_RADIO_PROFILE_CH5_6M8_PLEN256;
    }
    if (mode == APP_UWB_RADIO_PHY_FAST_PLEN512) {
        return runtime_radio_channel(config) == 9U
                   ? APP_UWB_RADIO_PROFILE_CH9_6M8_PLEN512
                   : APP_UWB_RADIO_PROFILE_CH5_6M8_PLEN512;
    }
    if (mode == APP_UWB_RADIO_PHY_850K_PLEN512) {
        return runtime_radio_channel(config) == 9U
                   ? APP_UWB_RADIO_PROFILE_CH9_850K_PLEN512
                   : APP_UWB_RADIO_PROFILE_CH5_850K_PLEN512;
    }
    if (mode == APP_UWB_RADIO_PHY_850K_PLEN512_STD_SFD) {
        return runtime_radio_channel(config) == 9U
                   ? APP_UWB_RADIO_PROFILE_CH9_850K_PLEN512_STD_SFD
                   : APP_UWB_RADIO_PROFILE_CH5_850K_PLEN512_STD_SFD;
    }
    return runtime_radio_channel(config) == 9U
               ? APP_UWB_RADIO_PROFILE_LEGACY_CH9_6M8_PLEN128
               : APP_UWB_RADIO_PROFILE_LEGACY_CH5_6M8_PLEN128;
}

static uint8_t runtime_radio_preamble_len_code(
    const app_runtime_config_t *config)
{
    switch (runtime_radio_phy_mode(config)) {
    case APP_UWB_RADIO_PHY_LONG_RANGE:
        return APP_UWB_RADIO_PLEN_1024;
    case APP_UWB_RADIO_PHY_FAST_PLEN256:
        return APP_UWB_RADIO_PLEN_256;
    case APP_UWB_RADIO_PHY_FAST_PLEN512:
    case APP_UWB_RADIO_PHY_850K_PLEN512:
    case APP_UWB_RADIO_PHY_850K_PLEN512_STD_SFD:
        return APP_UWB_RADIO_PLEN_512;
    default:
        return APP_UWB_RADIO_PREAMBLE_LEN_CODE;
    }
}

static uint8_t runtime_radio_pac(const app_runtime_config_t *config)
{
    const uint8_t mode = runtime_radio_phy_mode(config);
    if (mode == APP_UWB_RADIO_PHY_FAST_PLEN256) {
        return 1U;
    }
    return mode == APP_UWB_RADIO_PHY_FAST ? APP_UWB_RADIO_PAC : 2U;
}

static uint8_t runtime_radio_data_rate(const app_runtime_config_t *config)
{
    return runtime_radio_is_850k(config)
               ? APP_UWB_RADIO_BR_850K
               : APP_UWB_RADIO_DATA_RATE;
}

static uint8_t runtime_radio_phr_rate(const app_runtime_config_t *config)
{
    return runtime_radio_is_850k(config) ? 0U
                                                : APP_UWB_RADIO_PHR_RATE;
}

static uint16_t runtime_radio_sfd_timeout(
    const app_runtime_config_t *config)
{
    switch (runtime_radio_phy_mode(config)) {
    case APP_UWB_RADIO_PHY_LONG_RANGE:
        return 1001U;
    case APP_UWB_RADIO_PHY_FAST_PLEN256:
        return 249U;
    case APP_UWB_RADIO_PHY_FAST_PLEN512:
    case APP_UWB_RADIO_PHY_850K_PLEN512:
    case APP_UWB_RADIO_PHY_850K_PLEN512_STD_SFD:
        return 489U;
    default:
        return 129U;
    }
}

static uint8_t runtime_radio_preamble_code(
    const app_runtime_config_t *config)
{
    return runtime_radio_phy_mode(config) ==
                   APP_UWB_RADIO_PHY_850K_PLEN512_STD_SFD
               ? 9U
               : APP_UWB_RADIO_PREAMBLE_CODE;
}

static uint8_t runtime_radio_sfd_type(const app_runtime_config_t *config)
{
    return runtime_radio_phy_mode(config) ==
                   APP_UWB_RADIO_PHY_850K_PLEN512_STD_SFD
               ? 0U
               : APP_UWB_RADIO_SFD_TYPE;
}

static uint32_t runtime_radio_rf_tx_ctrl_2(const app_runtime_config_t *config)
{
    return runtime_radio_channel(config) == 9U
               ? APP_UWB_RADIO_RF_TX_CTRL_2_CH9
               : APP_UWB_RADIO_RF_TX_CTRL_2_CH5;
}

static uint32_t runtime_radio_pll_cfg_final(const app_runtime_config_t *config)
{
    return runtime_radio_channel(config) == 9U
               ? APP_UWB_RADIO_PLL_CFG_FINAL_CH9
               : APP_UWB_RADIO_PLL_CFG_FINAL_CH5;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    static const char response[] =
        "uwb_esp_idf\n"
        "GET  /status\n"
        "POST /ota    raw firmware image, requires X-OTA-Token header\n"
        "POST /gnss/loader, /gnss/firmware, /gnss/firmware/run\n"
        "             fixed PX1105R 01.07.33 update via PSRAM\n"
        "POST /config/antenna-delay?value=0x4018[&reboot=1]\n"
        "POST /config/antenna-delay?clear=1[&reboot=1]\n"
        "POST /config/runtime?mode=ranging&tag=1&anchors=2,3,4,5[&reboot=1]\n"
        "POST /config/runtime?clear=1[&reboot=1]\n"
        "POST /config/recovery?clear=1[&reboot=1]\n"
        "POST /config/charger?adc=1&adc_sample=2\n"
        "POST /config/charger?charge_current_ma=500&charge_voltage_mv=4200\n"
        "POST /config/charger?fast_charge_timer_enabled=1&fast_charge_timer_hours=12\n"
        "POST /config/max77958?refresh=1\n"
        "POST /config/max77958?sink_pdos=5000:3000,9000:3000,15000:3000\n"
        "POST /config/max77958?source_pdo_pos=2\n"
        "POST /config/max77958?apdo_pos=1&apdo_voltage_mv=9000&apdo_current_ma=2000\n";

    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_get_handler(httpd_req_t *req)
{
    ota_status_context_t *ctx = alloc_status_context();
    if (ctx == NULL) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "status allocation failed");
    }

#define i2c_stats (ctx->i2c_stats)
#define bno_snapshot (ctx->bno_snapshot)
#define gps_snapshot (ctx->gps_snapshot)
#define charger_snapshot (ctx->charger_snapshot)
#define pd_snapshot (ctx->pd_snapshot)
#define resource_snapshot (ctx->resource_snapshot)
#define charger_raw_hex (ctx->charger_raw_hex)
#define resource_top_tasks_json (ctx->resource_top_tasks_json)
#define pd_raw_hex (ctx->pd_raw_hex)
#define pd_last_response_hex (ctx->pd_last_response_hex)
#define pd_source_pdos_json (ctx->pd_source_pdos_json)
#define pd_sink_pdos_json (ctx->pd_sink_pdos_json)
#define runtime_anchor_ids_json (ctx->runtime_anchor_ids_json)
#define runtime_flex_slots_json (ctx->runtime_flex_slots_json)
#define runtime_flex_masks_json (ctx->runtime_flex_masks_json)
#define runtime_flex_anchor_x_json (ctx->runtime_flex_anchor_x_json)
#define runtime_flex_anchor_y_json (ctx->runtime_flex_anchor_y_json)
#define runtime_flex_anchor_correction_json \
    (ctx->runtime_flex_anchor_correction_json)
#define runtime_passive_ds_anchor_bias_json \
    (ctx->runtime_passive_ds_anchor_bias_json)
#define runtime_passive_ds_range_bias_json \
    (ctx->runtime_passive_ds_range_bias_json)
#define passive_ds_pipeline_stats (ctx->passive_ds_pipeline_stats)
#define passive_ds_pipeline_stats_json \
    (ctx->passive_ds_pipeline_stats_json)
#define native_ds_pipeline_stats (ctx->native_ds_pipeline_stats)
#define native_ds_pipeline_stats_json (ctx->native_ds_pipeline_stats_json)

    const esp_app_desc_t *app = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *boot = esp_ota_get_boot_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    const uint16_t active_antenna_delay = uwb_dw3000_get_antenna_delay();
    const uint16_t configured_antenna_delay =
        app_identity_get_uwb_antenna_delay();
    const app_runtime_config_t *runtime_config = app_runtime_config_get();
    i2c_bus_service_get_stats(&i2c_stats);
    bno085_service_get_snapshot(&bno_snapshot);
    gps_service_get_snapshot(&gps_snapshot);
    charger_service_get_snapshot(&charger_snapshot);
    charger_service_format_raw_hex(&charger_snapshot, charger_raw_hex,
                                   sizeof(charger_raw_hex));
    max77958_service_get_snapshot(&pd_snapshot);
    resource_monitor_service_get_snapshot(&resource_snapshot);
    format_resource_top_tasks_json(&resource_snapshot, resource_top_tasks_json,
                                   sizeof(resource_top_tasks_json));
    max77958_service_format_raw_hex(&pd_snapshot, pd_raw_hex,
                                    sizeof(pd_raw_hex));
    format_bytes_hex(pd_snapshot.last_response,
                     sizeof(pd_snapshot.last_response),
                     pd_last_response_hex, sizeof(pd_last_response_hex));
    format_u32_array_json(pd_snapshot.source_pdos,
                          pd_snapshot.source_pdo_count,
                          pd_source_pdos_json,
                          sizeof(pd_source_pdos_json));
    format_u32_array_json(pd_snapshot.sink_pdos,
                          pd_snapshot.sink_pdo_count,
                          pd_sink_pdos_json, sizeof(pd_sink_pdos_json));
    format_u8_array_json(runtime_config->anchor_ids,
                         runtime_config->anchor_count,
                         runtime_anchor_ids_json,
                         sizeof(runtime_anchor_ids_json));
    format_u8_array_json(runtime_config->flex_tdoa_slot_initiator_ids,
                         runtime_config->flex_tdoa_slot_count,
                         runtime_flex_slots_json,
                         sizeof(runtime_flex_slots_json));
    format_u16_array_json(runtime_config->flex_tdoa_slot_responder_masks,
                          runtime_config->flex_tdoa_slot_count,
                          runtime_flex_masks_json,
                          sizeof(runtime_flex_masks_json));
    format_i32_array_json(runtime_config->flex_tdoa_anchor_x_mm,
                          runtime_config->anchor_count,
                          runtime_flex_anchor_x_json,
                          sizeof(runtime_flex_anchor_x_json));
    format_i32_array_json(runtime_config->flex_tdoa_anchor_y_mm,
                          runtime_config->anchor_count,
                          runtime_flex_anchor_y_json,
                          sizeof(runtime_flex_anchor_y_json));
    format_i32_array_json(runtime_config->flex_tdoa_anchor_correction_mm,
                          runtime_config->anchor_count,
                          runtime_flex_anchor_correction_json,
                          sizeof(runtime_flex_anchor_correction_json));
    format_i32_array_json(runtime_config->passive_ds_anchor_bias_mm,
                          runtime_config->anchor_count,
                          runtime_passive_ds_anchor_bias_json,
                          sizeof(runtime_passive_ds_anchor_bias_json));
    format_passive_ds_range_bias_json(
        runtime_config, runtime_passive_ds_range_bias_json,
        sizeof(runtime_passive_ds_range_bias_json));
    uwb_dw3000_get_passive_ds_pipeline_stats(
        &passive_ds_pipeline_stats);
    format_passive_ds_pipeline_stats_json(
        &passive_ds_pipeline_stats, passive_ds_pipeline_stats_json,
        sizeof(passive_ds_pipeline_stats_json));
    uwb_dw3000_get_native_ds_pipeline_stats(
        &native_ds_pipeline_stats);
    format_native_ds_pipeline_stats_json(
        &native_ds_pipeline_stats, native_ds_pipeline_stats_json,
        sizeof(native_ds_pipeline_stats_json));

    char *response = ctx->response;

    const int len = snprintf(
        response, OTA_SERVICE_STATUS_RESPONSE_SIZE,
        "{"
        "\"project\":\"%s\","
        "\"version\":\"%s\","
        "\"idf\":\"%s\","
        "\"hostname\":\"%s\","
        "\"module_id\":%u,"
        "\"module_id_from_nvs\":%s,"
        "\"module_id_provisioned_this_boot\":%s,"
        "\"uwb_role\":%u,"
        "\"uwb_role_name\":\"%s\","
        "\"uwb_role_from_nvs\":%s,"
        "\"uwb_role_provisioned_this_boot\":%s,"
        "\"ota_status\":\"%s\","
        "\"wifi_connected\":%s,"
        "\"ip\":\"%s\","
        "\"wifi_disconnect_count\":%lu,"
        "\"wifi_last_disconnect_reason\":%u,"
        "\"wifi_last_disconnect_reason_name\":\"%s\","
        "\"wifi_connected_bssid\":\"%s\","
        "\"wifi_connected_channel\":%u,"
        "\"wifi_connected_rssi\":%d,"
        "\"wifi_scan_ap_count\":%u,"
        "\"wifi_scan_best_bssid\":\"%s\","
        "\"wifi_scan_best_channel\":%u,"
        "\"wifi_scan_best_rssi\":%d,"
        "\"running_partition\":\"%s\","
        "\"boot_partition\":\"%s\","
        "\"next_update_partition\":\"%s\","
        "\"ota_in_progress\":%s,"
        "\"ota_auth_configured\":%s,"
        "\"boot_recovery_mode\":%s,"
        "\"boot_guard_boot_count\":%lu,"
        "\"boot_guard_failure_count\":%lu,"
        "\"boot_guard_recovery_threshold\":%lu,"
        "\"boot_guard_stable_delay_ms\":%lu,"
        "\"boot_guard_last_reset_reason\":%u,"
        "\"boot_guard_last_reset_reason_name\":\"%s\","
        "\"boot_guard_ota_state\":\"%s\","
        "\"boot_guard_pending_verify\":%s,"
        "\"boot_guard_validated\":%s,"
        "\"boot_guard_rollback_possible\":%s,"
        "\"runtime_config_from_nvs\":%s,"
        "\"runtime_mode\":%u,"
        "\"runtime_mode_name\":\"%s\","
        "\"runtime_tag_id\":%u,"
        "\"runtime_anchor_count\":%u,"
        "\"runtime_anchor_ids\":%s,"
        "\"runtime_flex_tdoa_responder_count\":%u,"
        "\"runtime_flex_tdoa_slot_count\":%u,"
        "\"runtime_flex_tdoa_slot_initiator_ids\":%s,"
        "\"runtime_flex_tdoa_slot_responder_masks\":%s,"
        "\"runtime_flex_tdoa_config_generation\":%lu,"
        "\"runtime_flex_tdoa_guard_us\":%lu,"
        "\"runtime_flex_tdoa_request_subslot_us\":%lu,"
        "\"runtime_flex_tdoa_request_process_us\":%lu,"
        "\"runtime_flex_tdoa_response_subslot_us\":%lu,"
        "\"runtime_flex_tdoa_response_process_us\":%lu,"
        "\"runtime_flex_tdoa_geometry_fixed\":%s,"
        "\"runtime_flex_tdoa_geometry_generation\":%lu,"
        "\"runtime_flex_tdoa_anchor_x_mm\":%s,"
        "\"runtime_flex_tdoa_anchor_y_mm\":%s,"
        "\"runtime_flex_tdoa_anchor_correction_mm\":%s,"
        "\"runtime_anchor_survey_coordinator_id\":%u,"
        "\"runtime_anchor_survey_rx_slice_ms\":%lu,"
        "\"runtime_anchor_survey_command_delay_ms\":%lu,"
        "\"runtime_anchor_survey_slot_ms\":%lu,"
        "\"runtime_anchor_survey_round_gap_ms\":%lu,"
        "\"runtime_anchor_survey_passive_tag_log_every\":%lu,"
        "\"runtime_ranging_slot_ms\":%lu,"
        "\"runtime_ranging_round_gap_ms\":%lu,"
        "\"runtime_ranging_rx_slice_ms\":%lu,"
        "\"runtime_ranging_rx_timeout_ms\":%lu,"
        "\"runtime_ranging_resp_delay_ms\":%lu,"
        "\"runtime_ranging_final_delay_ms\":%lu,"
        "\"runtime_ranging_auto_rx_delay_uus\":%lu,"
        "\"native_ds_pipeline_stats\":%s,"
        "\"runtime_passive_ds_schedule\":%u,"
        "\"runtime_passive_ds_slot_ms\":%lu,"
        "\"runtime_passive_ds_round_gap_ms\":%lu,"
        "\"runtime_passive_ds_rx_slice_ms\":%lu,"
        "\"runtime_passive_ds_rx_timeout_ms\":%lu,"
        "\"runtime_passive_ds_resp_delay_us\":%lu,"
        "\"runtime_passive_ds_final_delay_us\":%lu,"
        "\"runtime_passive_ds_auto_rx_delay_uus\":%lu,"
        "\"runtime_passive_ds_pipeline_mode\":%u,"
        "\"runtime_passive_ds_solve_mode\":%u,"
        "\"runtime_passive_ds_rolling_max_hz\":%lu,"
        "\"passive_ds_pipeline_stats\":%s,"
        "\"runtime_passive_ds_calibration_enabled\":%s,"
        "\"runtime_passive_ds_calibration_generation\":%lu,"
        "\"runtime_passive_ds_anchor_bias_mm\":%s,"
        "\"runtime_passive_ds_range_bias_mm\":%s,"
        "\"runtime_distance_test_peer_id\":%u,"
        "\"runtime_distance_test_initiator_id\":%u,"
        "\"runtime_distance_test_responder_id\":%u,"
        "\"runtime_distance_test_interval_ms\":%lu,"
        "\"runtime_distance_test_rx_timeout_ms\":%lu,"
        "\"runtime_distance_test_resp_delay_ms\":%lu,"
        "\"runtime_distance_test_final_delay_ms\":%lu,"
        "\"runtime_distance_test_report_delay_ms\":%lu,"
        "\"runtime_distance_test_auto_rx_delay_uus\":%lu,"
        "\"runtime_calibration_method\":%u,"
        "\"runtime_calibration_reference_id\":%u,"
        "\"runtime_calibration_dut_id\":%u,"
        "\"runtime_calibration_three_ids\":[%u,%u,%u],"
        "\"runtime_calibration_known_distance_mm\":%lu,"
        "\"runtime_calibration_three_distance_0_1_mm\":%lu,"
        "\"runtime_calibration_three_distance_0_2_mm\":%lu,"
        "\"runtime_calibration_three_distance_1_2_mm\":%lu,"
        "\"runtime_calibration_sample_count\":%lu,"
        "\"runtime_calibration_summary_every\":%lu,"
        "\"runtime_calibration_min_interval_ms\":%lu,"
        "\"runtime_calibration_max_interval_ms\":%lu,"
        "\"runtime_calibration_rx_slice_ms\":%lu,"
        "\"runtime_calibration_slot_guard_us\":%lu,"
        "\"runtime_uwb_enabled\":%s,"
        "\"runtime_bno085_accel_enabled\":%s,"
        "\"runtime_bno085_accel_interval_ms\":%lu,"
        "\"runtime_bno085_log_interval_ms\":%lu,"
        "\"bno085_service_started\":%s,"
        "\"bno085_int_irq_enabled\":%s,"
        "\"bno085_i2c_clock_hz\":%lu,"
        "\"bno085_i2c_scl_measure_error\":%d,"
        "\"bno085_i2c_scl_measure_error_name\":\"%s\","
        "\"bno085_i2c_scl_edges\":%lu,"
        "\"bno085_i2c_scl_elapsed_us\":%lu,"
        "\"bno085_i2c_scl_measured_hz\":%lu,"
        "\"bno085_report_count\":%lu,"
        "\"bno085_packet_count\":%lu,"
        "\"bno085_input_packet_count\":%lu,"
        "\"bno085_timebase_count\":%lu,"
        "\"bno085_max_reports_per_packet\":%lu,"
        "\"bno085_continuation_packet_count\":%lu,"
        "\"bno085_continuation_transfer_count\":%lu,"
        "\"bno085_continuation_header_error_count\":%lu,"
        "\"bno085_high_rate_poll_count\":%lu,"
        "\"bno085_wait_immediate_count\":%lu,"
        "\"bno085_wait_notify_count\":%lu,"
        "\"bno085_wait_late_active_count\":%lu,"
        "\"bno085_null_header_count\":%lu,"
        "\"bno085_read_error_count\":%lu,"
        "\"bno085_parse_error_count\":%lu,"
        "\"bno085_int_irq_count\":%lu,"
        "\"bno085_int_wait_timeout_count\":%lu,"
        "\"bno085_last_packet_len\":%lu,"
        "\"bno085_last_input_payload_len\":%lu,"
        "\"bno085_last_x_mps2\":%.3f,"
        "\"bno085_last_y_mps2\":%.3f,"
        "\"bno085_last_z_mps2\":%.3f,"
        "\"bno085_last_accuracy\":%u,"
        "\"i2c_realtime_period_us\":%lu,"
        "\"i2c_realtime_time_to_next_us\":%ld,"
        "\"i2c_background_window_us\":%ld,"
        "\"i2c_realtime_waiters\":%lu,"
        "\"i2c_realtime_lock_count\":%lu,"
        "\"i2c_background_lock_count\":%lu,"
        "\"i2c_background_deferred_count\":%lu,"
        "\"resource_monitor_running\":%s,"
        "\"resource_update_count\":%lu,"
        "\"resource_last_update_age_ms\":%lu,"
        "\"resource_cpu_load_valid\":%s,"
        "\"resource_task_load_valid\":%s,"
        "\"resource_task_list_overflow\":%s,"
        "\"resource_top_task_count\":%lu,"
        "\"resource_top_tasks\":%s,"
        "\"resource_core0_load_percent\":%.1f,"
        "\"resource_core1_load_percent\":%.1f,"
        "\"resource_heap_free_bytes\":%lu,"
        "\"resource_heap_total_bytes\":%lu,"
        "\"resource_heap_min_free_bytes\":%lu,"
        "\"resource_heap_largest_free_block_bytes\":%lu,"
        "\"resource_internal_free_bytes\":%lu,"
        "\"resource_internal_total_bytes\":%lu,"
        "\"resource_internal_min_free_bytes\":%lu,"
        "\"resource_internal_largest_free_block_bytes\":%lu,"
        "\"resource_psram_free_bytes\":%lu,"
        "\"resource_psram_total_bytes\":%lu,"
        "\"resource_psram_min_free_bytes\":%lu,"
        "\"resource_psram_largest_free_block_bytes\":%lu,"
        "\"resource_flash_valid\":%s,"
        "\"resource_flash_total_bytes\":%lu,"
        "\"resource_flash_reserved_bytes\":%lu,"
        "\"resource_flash_free_bytes\":%lu,"
        "\"resource_flash_partition_count\":%lu,"
        "\"resource_flash_error\":%d,"
        "\"resource_flash_error_name\":\"%s\","
        "\"resource_temperature_valid\":%s,"
        "\"resource_temperature_c\":%.1f,"
        "\"resource_temperature_error\":%d,"
        "\"resource_temperature_error_name\":\"%s\","
        "\"runtime_gps_enabled\":%s,"
        "\"gps_powered\":%s,"
        "\"gps_task_running\":%s,"
        "\"gps_uart_ready\":%s,"
        "\"gps_last_error\":%d,"
        "\"gps_last_error_name\":\"%s\","
        "\"gps_fix_valid\":%s,"
        "\"gps_fix_quality\":%d,"
        "\"gps_fix_quality_text\":\"%s\","
        "\"gps_fix_type\":%u,"
        "\"gps_satellites\":%u,"
        "\"gps_satellites_in_view\":%u,"
        "\"gps_hdop\":%.2f,"
        "\"gps_latitude_deg\":%.8f,"
        "\"gps_longitude_deg\":%.8f,"
        "\"gps_altitude_m\":%.2f,"
        "\"gps_speed_mps\":%.2f,"
        "\"gps_course_deg\":%.1f,"
        "\"gps_rmc_status\":\"%c\","
        "\"gps_rmc_mode\":\"%c\","
        "\"gps_utc_time\":\"%s\","
        "\"gps_utc_date\":\"%s\","
        "\"gps_last_sentence\":\"%s\","
        "\"gps_last_rx_age_ms\":%lu,"
        "\"gps_last_fix_age_ms\":%lu,"
        "\"gps_byte_count\":%lu,"
        "\"gps_sentence_count\":%lu,"
        "\"gps_gga_count\":%lu,"
        "\"gps_rmc_count\":%lu,"
        "\"gps_gsa_count\":%lu,"
        "\"gps_gsv_count\":%lu,"
        "\"gps_psti030_count\":%lu,"
        "\"gps_psti032_count\":%lu,"
        "\"gps_psti035_count\":%lu,"
        "\"gps_ths_count\":%lu,"
        "\"gps_rtk_age_s\":%.2f,"
        "\"gps_rtk_ratio\":%.2f,"
        "\"gps_baseline_valid\":%s,"
        "\"gps_baseline_source\":%u,"
        "\"gps_baseline_status\":\"%c\","
        "\"gps_baseline_mode\":\"%c\","
        "\"gps_baseline_east_m\":%.3f,"
        "\"gps_baseline_north_m\":%.3f,"
        "\"gps_baseline_up_m\":%.3f,"
        "\"gps_baseline_length_m\":%.3f,"
        "\"gps_baseline_course_deg\":%.2f,"
        "\"gps_true_heading_valid\":%s,"
        "\"gps_true_heading_deg\":%.2f,"
        "\"gps_true_heading_mode\":\"%c\","
        "\"gps_moving_base_role\":\"%s\","
        "\"gps_moving_base_active\":%s,"
        "\"gps_moving_base_correction_uart_ready\":%s,"
        "\"gps_moving_base_receiver_config_sent\":%s,"
        "\"gps_moving_base_receiver_ack_count\":%lu,"
        "\"gps_moving_base_receiver_nack_count\":%lu,"
        "\"gps_moving_base_receiver_last_ack_id\":%u,"
        "\"gps_moving_base_receiver_last_nack_id\":%u,"
        "\"gps_moving_base_uplink_packets\":%lu,"
        "\"gps_moving_base_uplink_bytes\":%lu,"
        "\"gps_moving_base_uplink_errors\":%lu,"
        "\"gps_moving_base_downlink_packets\":%lu,"
        "\"gps_moving_base_downlink_bytes\":%lu,"
        "\"gps_moving_base_downlink_errors\":%lu,"
        "\"gps_moving_base_downlink_gaps\":%lu,"
        "\"gps_moving_base_last_uplink_age_ms\":%lu,"
        "\"gps_moving_base_last_downlink_age_ms\":%lu,"
        "\"gps_moving_base_last_downlink_source_id\":%u,"
        "\"gps_moving_base_skytraq_frames\":%lu,"
        "\"gps_moving_base_software_version_valid\":%s,"
        "\"gps_moving_base_software_type\":%u,"
        "\"gps_moving_base_software_kernel_version\":\"%08lx\","
        "\"gps_moving_base_software_odm_version\":\"%08lx\","
        "\"gps_moving_base_software_revision\":\"%08lx\","
        "\"gps_moving_base_binary_output_status_valid\":%s,"
        "\"gps_moving_base_binary_output_rate_code\":%u,"
        "\"gps_moving_base_binary_meas_time_enabled\":%s,"
        "\"gps_moving_base_binary_raw_meas_enabled\":%s,"
        "\"gps_moving_base_binary_meas_time_count\":%lu,"
        "\"gps_moving_base_binary_raw_meas_count\":%lu,"
        "\"gps_moving_base_rtcm_preambles\":%lu,"
        "\"gps_ntrip_configured\":%s,"
        "\"gps_ntrip_running\":%s,"
        "\"gps_ntrip_tls_connected\":%s,"
        "\"gps_ntrip_stream_active\":%s,"
        "\"gps_ntrip_http_status\":%u,"
        "\"gps_ntrip_connect_count\":%lu,"
        "\"gps_ntrip_reconnect_count\":%lu,"
        "\"gps_ntrip_error_count\":%lu,"
        "\"gps_ntrip_rtcm_frames\":%lu,"
        "\"gps_ntrip_rtcm_bytes\":%lu,"
        "\"gps_ntrip_last_data_age_ms\":%lu,"
        "\"gps_ntrip_state\":\"%s\","
        "\"gps_checksum_errors\":%lu,"
        "\"gps_parse_errors\":%lu,"
        "\"charger_monitor_enabled\":%s,"
        "\"charger_present\":%s,"
        "\"charger_read_ok\":%s,"
        "\"charger_raw_valid\":%s,"
        "\"charger_config_write_supported\":%s,"
        "\"charger_config_writes_enabled\":%s,"
        "\"charger_last_error\":%d,"
        "\"charger_last_error_name\":\"%s\","
        "\"charger_read_count\":%lu,"
        "\"charger_full_read_count\":%lu,"
        "\"charger_quick_read_count\":%lu,"
        "\"charger_error_count\":%lu,"
        "\"charger_last_read_duration_ms\":%lu,"
        "\"charger_last_read_full\":%s,"
        "\"charger_last_update_age_ms\":%lu,"
        "\"charger_i2c_clock_hz\":%lu,"
        "\"charger_i2c_scl_measure_error\":%d,"
        "\"charger_i2c_scl_measure_error_name\":\"%s\","
        "\"charger_i2c_scl_edges\":%lu,"
        "\"charger_i2c_scl_elapsed_us\":%lu,"
        "\"charger_i2c_scl_measured_hz\":%lu,"
        "\"charger_int_gpio_level\":%d,"
        "\"charger_int_irq_count\":%lu,"
        "\"charger_int_last_irq_age_ms\":%lu,"
        "\"charger_pg_gpio_level\":%d,"
        "\"charger_pg_asserted\":%s,"
        "\"charger_pg_stat\":%s,"
        "\"charger_qon_gpio_level\":%d,"
        "\"charger_qon_asserted\":%s,"
        "\"charger_part_info\":\"0x%02x\","
        "\"charger_part_number\":%u,"
        "\"charger_device_revision\":%u,"
        "\"charger_adc_enabled\":%s,"
        "\"charger_reg09_termination_control\":\"0x%02x\","
        "\"charger_reg0a_recharge_control\":\"0x%02x\","
        "\"charger_reg0d_iotg_regulation\":\"0x%02x\","
        "\"charger_reg0e_timer_control\":\"0x%02x\","
        "\"charger_reg16_temperature_control\":\"0x%02x\","
        "\"charger_reg17_ntc_control_0\":\"0x%02x\","
        "\"charger_reg18_ntc_control_1\":\"0x%02x\","
        "\"charger_reg0f_charger_control_0\":\"0x%02x\","
        "\"charger_reg10_charger_control_1\":\"0x%02x\","
        "\"charger_reg14_charger_control_5\":\"0x%02x\","
        "\"charger_reg2e_adc_control\":\"0x%02x\","
        "\"charger_reg2f_adc_disable_0\":\"0x%02x\","
        "\"charger_reg30_adc_disable_1\":\"0x%02x\","
        "\"charger_minimal_system_voltage_mv\":%u,"
        "\"charger_charge_voltage_limit_mv\":%u,"
        "\"charger_charge_current_limit_ma\":%u,"
        "\"charger_input_voltage_limit_mv\":%u,"
        "\"charger_input_current_limit_ma\":%u,"
        "\"charger_external_input_current_limit_enabled\":%s,"
        "\"charger_charge_enabled\":%s,"
        "\"charger_termination_enabled\":%s,"
        "\"charger_termination_current_ma\":%u,"
        "\"charger_recharge_threshold_offset_mv\":%u,"
        "\"charger_recharge_threshold_mv\":%u,"
        "\"charger_recharge_deglitch_ms\":%u,"
        "\"charger_charge_status_code\":%u,"
        "\"charger_vbus_status_code\":%u,"
        "\"charger_iindpm_active\":%s,"
        "\"charger_vindpm_active\":%s,"
        "\"charger_vsys_regulation_active\":%s,"
        "\"charger_battery_overvoltage_active\":%s,"
        "\"charger_charge_safety_timer_expired\":%s,"
        "\"charger_topoff_timer_flag\":%s,"
        "\"charger_trickle_timer_flag\":%s,"
        "\"charger_precharge_timer_flag\":%s,"
        "\"charger_fast_charge_timer_flag\":%s,"
        "\"charger_watchdog_setting\":%u,"
        "\"charger_watchdog_disabled\":%s,"
        "\"charger_topoff_timer_minutes\":%u,"
        "\"charger_trickle_timer_enabled\":%s,"
        "\"charger_precharge_timer_enabled\":%s,"
        "\"charger_fast_charge_timer_enabled\":%s,"
        "\"charger_fast_charge_timer_hours\":%u,"
        "\"charger_timer_2x_enabled\":%s,"
        "\"charger_precharge_timer_minutes\":%u,"
        "\"charger_adc_sample\":%u,"
        "\"charger_adc_continuous\":%s,"
        "\"charger_adc_running_average\":%s,"
        "\"charger_ibat_discharge_sense_enabled\":%s,"
        "\"charger_status\":[%u,%u,%u,%u,%u],"
        "\"charger_fault_status\":[%u,%u],"
        "\"charger_flag\":[%u,%u,%u,%u],"
        "\"charger_fault_flag\":[%u,%u],"
        "\"charger_ibus_ma\":%d,"
        "\"charger_ibat_ma\":%d,"
        "\"charger_vbus_mv\":%u,"
        "\"charger_vac1_mv\":%u,"
        "\"charger_vac2_mv\":%u,"
        "\"charger_vbat_mv\":%u,"
        "\"charger_vsys_mv\":%u,"
        "\"charger_battery_soc_valid\":%s,"
        "\"charger_battery_soc_percent\":%u,"
        "\"charger_ts_percent\":%.4f,"
        "\"charger_ts_ignore\":%s,"
        "\"charger_ts_cold_active\":%s,"
        "\"charger_ts_cool_active\":%s,"
        "\"charger_ts_warm_active\":%s,"
        "\"charger_ts_hot_active\":%s,"
        "\"charger_ts_cold_flag\":%s,"
        "\"charger_ts_cool_flag\":%s,"
        "\"charger_ts_warm_flag\":%s,"
        "\"charger_ts_hot_flag\":%s,"
        "\"charger_tdie_c\":%.1f,"
        "\"charger_dp_mv\":%u,"
        "\"charger_dm_mv\":%u,"
        "\"charger_write_count\":%lu,"
        "\"charger_write_error_count\":%lu,"
        "\"charger_last_write_age_ms\":%lu,"
        "\"charger_last_write_reg\":\"0x%02x\","
        "\"charger_last_write_requested_value\":\"0x%02x\","
        "\"charger_last_write_mask\":\"0x%02x\","
        "\"charger_last_write_before\":\"0x%02x\","
        "\"charger_last_write_after\":\"0x%02x\","
        "\"charger_last_write_mask_used\":%s,"
        "\"charger_last_write_changed\":%s,"
        "\"charger_last_write_error\":%d,"
        "\"charger_last_write_error_name\":\"%s\","
        "\"charger_raw_hex\":\"%s\","
        "\"pd_monitor_enabled\":%s,"
        "\"pd_present\":%s,"
        "\"pd_read_ok\":%s,"
        "\"pd_raw_valid\":%s,"
        "\"pd_config_write_supported\":%s,"
        "\"pd_config_writes_enabled\":%s,"
        "\"pd_last_error\":%d,"
        "\"pd_last_error_name\":\"%s\","
        "\"pd_read_count\":%lu,"
        "\"pd_error_count\":%lu,"
        "\"pd_last_read_duration_ms\":%lu,"
        "\"pd_last_update_age_ms\":%lu,"
        "\"pd_i2c_clock_hz\":%lu,"
        "\"pd_i2c_hs_direct_clock_hz\":%lu,"
        "\"pd_i2c_hs_ext_requested\":%s,"
        "\"pd_i2c_hs_ext_active\":%s,"
        "\"pd_i2c_hs_direct_reads_enabled\":%s,"
        "\"pd_i2c_hs_direct_writes_enabled\":%s,"
        "\"pd_i2c_hs_direct_read_count\":%lu,"
        "\"pd_i2c_hs_direct_write_count\":%lu,"
        "\"pd_i2c_hs_direct_error_count\":%lu,"
        "\"pd_i2c_hs_direct_last_elapsed_us\":%lu,"
        "\"pd_i2c_hs_direct_scl_measure_error\":%d,"
        "\"pd_i2c_hs_direct_scl_measure_error_name\":\"%s\","
        "\"pd_i2c_hs_direct_scl_edges\":%lu,"
        "\"pd_i2c_hs_direct_scl_elapsed_us\":%lu,"
        "\"pd_i2c_hs_direct_scl_measured_hz\":%lu,"
        "\"pd_device_id\":\"0x%02x\","
        "\"pd_device_rev\":\"0x%02x\","
        "\"pd_fw_rev\":%u,"
        "\"pd_fw_sub_ver\":%u,"
        "\"pd_uic_int\":\"0x%02x\","
        "\"pd_cc_int\":\"0x%02x\","
        "\"pd_pd_int\":\"0x%02x\","
        "\"pd_action_int\":\"0x%02x\","
        "\"pd_usbc_status1\":\"0x%02x\","
        "\"pd_usbc_status2\":\"0x%02x\","
        "\"pd_sys_msg_name\":\"%s\","
        "\"pd_bc_status\":\"0x%02x\","
        "\"pd_dp_status\":\"0x%02x\","
        "\"pd_cc_status0\":\"0x%02x\","
        "\"pd_cc_status1\":\"0x%02x\","
        "\"pd_pd_status0\":\"0x%02x\","
        "\"pd_pd_status1\":\"0x%02x\","
        "\"pd_uic_int_mask\":\"0x%02x\","
        "\"pd_cc_int_mask\":\"0x%02x\","
        "\"pd_pd_int_mask\":\"0x%02x\","
        "\"pd_action_int_mask\":\"0x%02x\","
        "\"pd_sw_reset\":\"0x%02x\","
        "\"pd_i2c_cnfg\":\"0x%02x\","
        "\"pd_vbadc_code\":%u,"
        "\"pd_vbus_min_mv\":%u,"
        "\"pd_vbus_max_mv\":%u,"
        "\"pd_vbus_mid_mv\":%u,"
        "\"pd_vbus_above_range\":%s,"
        "\"pd_vbus_detected\":%s,"
        "\"pd_chg_typ\":%u,"
        "\"pd_chg_typ_name\":\"%s\","
        "\"pd_pr_chg_typ\":%u,"
        "\"pd_dcd_timeout\":%s,"
        "\"pd_cc_pin\":%u,"
        "\"pd_cc_pin_name\":\"%s\","
        "\"pd_cci\":%u,"
        "\"pd_cci_name\":\"%s\","
        "\"pd_vconn_enabled\":%s,"
        "\"pd_cc_stat\":%u,"
        "\"pd_cc_stat_name\":\"%s\","
        "\"pd_det_abrt\":%s,"
        "\"pd_data_role_dfp\":%s,"
        "\"pd_power_role_source\":%s,"
        "\"pd_vconn_source\":%s,"
        "\"pd_psrdy_as_sink\":%s,"
        "\"pd_ctrl1_valid\":%s,"
        "\"pd_ctrl1_raw\":\"0x%02x\","
        "\"pd_ctrl1_comp2_sw\":%u,"
        "\"pd_ctrl1_comn1_sw\":%u,"
        "\"pd_usb2_switch_closed\":%s,"
        "\"pd_source_caps_valid\":%s,"
        "\"pd_source_pdo_count\":%u,"
        "\"pd_selected_source_pdo_pos\":%u,"
        "\"pd_source_pdos\":%s,"
        "\"pd_sink_pdos_valid\":%s,"
        "\"pd_sink_pdo_count\":%u,"
        "\"pd_sink_pdos_from_mtp\":%s,"
        "\"pd_sink_pdos\":%s,"
        "\"pd_pps_default_valid\":%s,"
        "\"pd_pps_default_enabled\":%s,"
        "\"pd_pps_default_mv\":%u,"
        "\"pd_pps_default_ma\":%u,"
        "\"pd_operation_count\":%lu,"
        "\"pd_operation_error_count\":%lu,"
        "\"pd_last_operation_age_ms\":%lu,"
        "\"pd_last_opcode\":\"0x%02x\","
        "\"pd_last_response_opcode\":\"0x%02x\","
        "\"pd_last_result_code\":%u,"
        "\"pd_last_result_name\":\"%s\","
        "\"pd_last_operation_error\":%d,"
        "\"pd_last_operation_error_name\":\"%s\","
        "\"pd_last_response_hex\":\"%s\","
        "\"pd_raw_hex\":\"%s\","
        "\"runtime_radio_channel\":%u,"
        "\"runtime_radio_phy_mode\":%u,"
        "\"runtime_wireless_telemetry_port\":%lu,"
        "\"uwb_status\":\"%s\","
        "\"uwb_runtime_switching\":%s,"
        "\"uwb_runtime_switch_count\":%lu,"
        "\"uwb_last_runtime_switch_ms\":%lu,"
        "\"uwb_radio_profile\":%u,"
        "\"uwb_radio_channel\":%u,"
        "\"uwb_radio_rf_channel_bit\":%u,"
        "\"uwb_radio_preamble_len_code\":%u,"
        "\"uwb_radio_preamble_code\":%u,"
        "\"uwb_radio_pac\":%u,"
        "\"uwb_radio_data_rate\":%u,"
        "\"uwb_radio_phr_mode\":%u,"
        "\"uwb_radio_phr_rate\":%u,"
        "\"uwb_radio_sfd_type\":%u,"
        "\"uwb_radio_sfd_timeout\":%u,"
        "\"uwb_radio_tx_pg_delay\":\"0x%02x\","
        "\"uwb_radio_tx_power\":\"0x%08lx\","
        "\"uwb_radio_rf_tx_ctrl_2\":\"0x%08lx\","
        "\"uwb_radio_pll_cfg_final\":\"0x%04lx\","
        "\"uwb_sts_mode\":%u,"
        "\"uwb_sts_length_symbols\":%u,"
        "\"uwb_diagnostics_enabled\":%s,"
        "\"uwb_diagnostics_log_every\":%u,"
        "\"uwb_event_counters_enabled\":%s,"
        "\"uwb_event_counters_log_every\":%u,"
        "\"uwb_device_id\":\"0x%08lx\","
        "\"uwb_spi_clock_hz\":%lu,"
        "\"uwb_source_id\":%u,"
        "\"uwb_active_antenna_delay\":%u,"
        "\"uwb_active_antenna_delay_hex\":\"0x%04x\","
        "\"uwb_configured_antenna_delay\":%u,"
        "\"uwb_configured_antenna_delay_hex\":\"0x%04x\","
        "\"uwb_antenna_delay_from_nvs\":%s,"
        "\"uwb_antenna_delay_reboot_required\":%s,"
        "\"uwb_tx_count\":%lu,"
        "\"uwb_tx_error_count\":%lu,"
        "\"uwb_rx_count\":%lu,"
        "\"uwb_rx_error_count\":%lu,"
        "\"uwb_rx_ignored_count\":%lu,"
        "\"uwb_last_rx_source_id\":%u,"
        "\"uwb_last_rx_sequence\":%lu,"
        "\"wireless_log_status\":\"%s\","
        "\"wireless_log_connected\":%s,"
        "\"wireless_log_target\":\"%s\","
        "\"wireless_log_port\":%u,"
        "\"wireless_log_dropped\":%lu,"
        "\"wireless_telemetry_status\":\"%s\","
        "\"wireless_telemetry_connected\":%s,"
        "\"wireless_telemetry_target\":\"%s\","
        "\"wireless_telemetry_port\":%u,"
        "\"wireless_telemetry_dropped\":%lu,"
        "\"wireless_telemetry_drop_full\":%lu,"
        "\"wireless_telemetry_drop_mutex\":%lu,"
        "\"wireless_telemetry_drop_format\":%lu,"
        "\"wireless_telemetry_queue_depth\":%lu,"
        "\"wireless_telemetry_queue_high_water\":%lu,"
        "\"wireless_telemetry_binary_frames\":%lu,"
        "\"wireless_telemetry_binary_samples\":%lu,"
        "\"wireless_telemetry_text_frames\":%lu,"
        "\"wireless_telemetry_connect_count\":%lu,"
        "\"wireless_telemetry_send_failures\":%lu,"
        "\"wireless_telemetry_send_timeouts\":%lu,"
        "\"wireless_telemetry_socket_closes\":%lu,"
        "\"wireless_telemetry_last_send_ms\":%lu,"
        "\"wireless_telemetry_max_send_ms\":%lu,"
        "\"wireless_telemetry_last_error\":%d"
        "}\n",
        app->project_name, app->version, app->idf_ver,
        app_identity_get_hostname(), (unsigned)app_identity_get_module_id(),
        app_identity_module_id_from_nvs() ? "true" : "false",
        app_identity_module_id_provisioned_this_boot() ? "true" : "false",
        (unsigned)app_identity_get_uwb_role(),
        app_identity_uwb_role_to_string(app_identity_get_uwb_role()),
        app_identity_uwb_role_from_nvs() ? "true" : "false",
        app_identity_uwb_role_provisioned_this_boot() ? "true" : "false",
        ota_status_to_string(s_status),
        wifi_service_is_connected() ? "true" : "false",
        wifi_service_get_ip_address(),
        (unsigned long)wifi_service_get_disconnect_count(),
        (unsigned)wifi_service_get_last_disconnect_reason(),
        wifi_service_get_last_disconnect_reason_name(),
        wifi_service_get_connected_bssid(),
        (unsigned)wifi_service_get_connected_channel(),
        wifi_service_get_connected_rssi(),
        (unsigned)wifi_service_get_last_scan_ap_count(),
        wifi_service_get_last_scan_best_bssid(),
        (unsigned)wifi_service_get_last_scan_best_channel(),
        wifi_service_get_last_scan_best_rssi(),
        partition_label_or_unknown(running),
        partition_label_or_unknown(boot), partition_label_or_unknown(next),
        s_ota_in_progress ? "true" : "false",
        ota_token_configured() ? "true" : "false",
        boot_guard_recovery_mode() ? "true" : "false",
        (unsigned long)boot_guard_boot_count(),
        (unsigned long)boot_guard_failure_count(),
        (unsigned long)boot_guard_recovery_threshold(),
        (unsigned long)boot_guard_stable_delay_ms(),
        (unsigned)boot_guard_last_reset_reason(),
        boot_guard_last_reset_reason_name(),
        boot_guard_ota_state_name(boot_guard_running_ota_state()),
        boot_guard_new_app_pending_verify() ? "true" : "false",
        boot_guard_new_app_validated() ? "true" : "false",
        boot_guard_rollback_possible() ? "true" : "false",
        runtime_config->from_nvs ? "true" : "false",
        (unsigned)runtime_config->runtime_mode,
        app_runtime_config_runtime_mode_to_string(
            runtime_config->runtime_mode),
        (unsigned)runtime_config->tag_id,
        (unsigned)runtime_config->anchor_count,
        runtime_anchor_ids_json,
        (unsigned)runtime_config->flex_tdoa_responder_count,
        (unsigned)runtime_config->flex_tdoa_slot_count,
        runtime_flex_slots_json,
        runtime_flex_masks_json,
        (unsigned long)runtime_config->flex_tdoa_config_generation,
        (unsigned long)runtime_config->flex_tdoa_guard_us,
        (unsigned long)runtime_config->flex_tdoa_request_subslot_us,
        (unsigned long)runtime_config->flex_tdoa_request_process_us,
        (unsigned long)runtime_config->flex_tdoa_response_subslot_us,
        (unsigned long)runtime_config->flex_tdoa_response_process_us,
        runtime_config->flex_tdoa_geometry_fixed ? "true" : "false",
        (unsigned long)runtime_config->flex_tdoa_geometry_generation,
        runtime_flex_anchor_x_json,
        runtime_flex_anchor_y_json,
        runtime_flex_anchor_correction_json,
        (unsigned)runtime_config->anchor_survey_coordinator_id,
        (unsigned long)runtime_config->anchor_survey_rx_slice_ms,
        (unsigned long)runtime_config->anchor_survey_command_delay_ms,
        (unsigned long)runtime_config->anchor_survey_slot_ms,
        (unsigned long)runtime_config->anchor_survey_round_gap_ms,
        (unsigned long)runtime_config->anchor_survey_passive_tag_log_every,
        (unsigned long)runtime_config->ranging_slot_ms,
        (unsigned long)runtime_config->ranging_round_gap_ms,
        (unsigned long)runtime_config->ranging_rx_slice_ms,
        (unsigned long)runtime_config->ranging_rx_timeout_ms,
        (unsigned long)runtime_config->ranging_resp_delay_ms,
        (unsigned long)runtime_config->ranging_final_delay_ms,
        (unsigned long)runtime_config->ranging_auto_rx_delay_uus,
        native_ds_pipeline_stats_json,
        (unsigned)runtime_config->passive_ds_schedule,
        (unsigned long)runtime_config->passive_ds_slot_ms,
        (unsigned long)runtime_config->passive_ds_round_gap_ms,
        (unsigned long)runtime_config->passive_ds_rx_slice_ms,
        (unsigned long)runtime_config->passive_ds_rx_timeout_ms,
        (unsigned long)runtime_config->passive_ds_resp_delay_us,
        (unsigned long)runtime_config->passive_ds_final_delay_us,
        (unsigned long)runtime_config->passive_ds_auto_rx_delay_uus,
        (unsigned)runtime_config->passive_ds_pipeline_mode,
        (unsigned)runtime_config->passive_ds_solve_mode,
        (unsigned long)runtime_config->passive_ds_rolling_max_hz,
        passive_ds_pipeline_stats_json,
        runtime_config->passive_ds_calibration_enabled ? "true" : "false",
        (unsigned long)runtime_config->passive_ds_calibration_generation,
        runtime_passive_ds_anchor_bias_json,
        runtime_passive_ds_range_bias_json,
        (unsigned)runtime_config->distance_test_peer_id,
        (unsigned)runtime_config->distance_test_initiator_id,
        (unsigned)runtime_config->distance_test_responder_id,
        (unsigned long)runtime_config->distance_test_interval_ms,
        (unsigned long)runtime_config->distance_test_rx_timeout_ms,
        (unsigned long)runtime_config->distance_test_resp_delay_ms,
        (unsigned long)runtime_config->distance_test_final_delay_ms,
        (unsigned long)runtime_config->distance_test_report_delay_ms,
        (unsigned long)runtime_config->distance_test_auto_rx_delay_uus,
        (unsigned)runtime_config->calibration_method,
        (unsigned)runtime_config->calibration_reference_id,
        (unsigned)runtime_config->calibration_dut_id,
        (unsigned)runtime_config->calibration_three_ids[0],
        (unsigned)runtime_config->calibration_three_ids[1],
        (unsigned)runtime_config->calibration_three_ids[2],
        (unsigned long)runtime_config->calibration_known_distance_mm,
        (unsigned long)runtime_config->calibration_three_distance_0_1_mm,
        (unsigned long)runtime_config->calibration_three_distance_0_2_mm,
        (unsigned long)runtime_config->calibration_three_distance_1_2_mm,
        (unsigned long)runtime_config->calibration_sample_count,
        (unsigned long)runtime_config->calibration_summary_every,
        (unsigned long)runtime_config->calibration_min_interval_ms,
        (unsigned long)runtime_config->calibration_max_interval_ms,
        (unsigned long)runtime_config->calibration_rx_slice_ms,
        (unsigned long)runtime_config->calibration_slot_guard_us,
        runtime_config->uwb_enabled ? "true" : "false",
        runtime_config->bno085_accel_enabled ? "true" : "false",
        (unsigned long)runtime_config->bno085_accel_interval_ms,
        (unsigned long)runtime_config->bno085_log_interval_ms,
        bno_snapshot.service_started ? "true" : "false",
        bno_snapshot.int_irq_enabled ? "true" : "false",
        (unsigned long)bno_snapshot.i2c_clock_hz,
        bno_snapshot.i2c_scl_measure_error,
        esp_err_to_name((esp_err_t)bno_snapshot.i2c_scl_measure_error),
        (unsigned long)bno_snapshot.i2c_scl_edges,
        (unsigned long)bno_snapshot.i2c_scl_elapsed_us,
        (unsigned long)bno_snapshot.i2c_scl_measured_hz,
        (unsigned long)bno_snapshot.report_count,
        (unsigned long)bno_snapshot.packet_count,
        (unsigned long)bno_snapshot.input_packet_count,
        (unsigned long)bno_snapshot.timebase_count,
        (unsigned long)bno_snapshot.max_reports_per_packet,
        (unsigned long)bno_snapshot.continuation_packet_count,
        (unsigned long)bno_snapshot.continuation_transfer_count,
        (unsigned long)bno_snapshot.continuation_header_error_count,
        (unsigned long)bno_snapshot.high_rate_poll_count,
        (unsigned long)bno_snapshot.wait_immediate_count,
        (unsigned long)bno_snapshot.wait_notify_count,
        (unsigned long)bno_snapshot.wait_late_active_count,
        (unsigned long)bno_snapshot.null_header_count,
        (unsigned long)bno_snapshot.read_error_count,
        (unsigned long)bno_snapshot.parse_error_count,
        (unsigned long)bno_snapshot.int_irq_count,
        (unsigned long)bno_snapshot.int_wait_timeout_count,
        (unsigned long)bno_snapshot.last_packet_len,
        (unsigned long)bno_snapshot.last_input_payload_len,
        (double)bno_snapshot.last_x_mps2,
        (double)bno_snapshot.last_y_mps2,
        (double)bno_snapshot.last_z_mps2,
        (unsigned)bno_snapshot.last_accuracy,
        (unsigned long)i2c_stats.realtime_period_us,
        (long)i2c_stats.realtime_time_to_next_us,
        (long)i2c_stats.background_window_us,
        (unsigned long)i2c_stats.realtime_waiters,
        (unsigned long)i2c_stats.realtime_lock_count,
        (unsigned long)i2c_stats.background_lock_count,
        (unsigned long)i2c_stats.background_deferred_count,
        resource_snapshot.running ? "true" : "false",
        (unsigned long)resource_snapshot.update_count,
        (unsigned long)resource_snapshot.last_update_age_ms,
        resource_snapshot.cpu_load_valid ? "true" : "false",
        resource_snapshot.task_load_valid ? "true" : "false",
        resource_snapshot.task_list_overflow ? "true" : "false",
        (unsigned long)resource_snapshot.top_task_count,
        resource_top_tasks_json[0] != '\0' ? resource_top_tasks_json : "[]",
        resource_snapshot.core0_load_percent,
        resource_snapshot.core1_load_percent,
        (unsigned long)resource_snapshot.heap_free_bytes,
        (unsigned long)resource_snapshot.heap_total_bytes,
        (unsigned long)resource_snapshot.heap_min_free_bytes,
        (unsigned long)resource_snapshot.heap_largest_free_block_bytes,
        (unsigned long)resource_snapshot.internal_free_bytes,
        (unsigned long)resource_snapshot.internal_total_bytes,
        (unsigned long)resource_snapshot.internal_min_free_bytes,
        (unsigned long)resource_snapshot.internal_largest_free_block_bytes,
        (unsigned long)resource_snapshot.psram_free_bytes,
        (unsigned long)resource_snapshot.psram_total_bytes,
        (unsigned long)resource_snapshot.psram_min_free_bytes,
        (unsigned long)resource_snapshot.psram_largest_free_block_bytes,
        resource_snapshot.flash_valid ? "true" : "false",
        (unsigned long)resource_snapshot.flash_total_bytes,
        (unsigned long)resource_snapshot.flash_reserved_bytes,
        (unsigned long)resource_snapshot.flash_free_bytes,
        (unsigned long)resource_snapshot.flash_partition_count,
        resource_snapshot.flash_error,
        esp_err_to_name(resource_snapshot.flash_error),
        resource_snapshot.temperature_valid ? "true" : "false",
        resource_snapshot.temperature_c,
        resource_snapshot.temperature_error,
        esp_err_to_name(resource_snapshot.temperature_error),
        runtime_config->gps_enabled ? "true" : "false",
        gps_snapshot.powered ? "true" : "false",
        gps_snapshot.task_running ? "true" : "false",
        gps_snapshot.uart_ready ? "true" : "false",
        gps_snapshot.last_error,
        esp_err_to_name((esp_err_t)gps_snapshot.last_error),
        gps_snapshot.fix_valid ? "true" : "false",
        gps_snapshot.fix_quality,
        gps_service_fix_quality_to_string(gps_snapshot.fix_quality),
        (unsigned)gps_snapshot.fix_type,
        (unsigned)gps_snapshot.satellites,
        (unsigned)gps_snapshot.satellites_in_view,
        gps_snapshot.hdop,
        gps_snapshot.latitude_deg,
        gps_snapshot.longitude_deg,
        gps_snapshot.altitude_m,
        gps_snapshot.speed_mps,
        gps_snapshot.course_deg,
        gps_snapshot.rmc_status != '\0' ? gps_snapshot.rmc_status : '-',
        gps_snapshot.rmc_mode != '\0' ? gps_snapshot.rmc_mode : '-',
        gps_snapshot.utc_time,
        gps_snapshot.utc_date,
        gps_snapshot.last_sentence_id,
        (unsigned long)gps_snapshot.last_rx_age_ms,
        (unsigned long)gps_snapshot.last_fix_age_ms,
        (unsigned long)gps_snapshot.byte_count,
        (unsigned long)gps_snapshot.sentence_count,
        (unsigned long)gps_snapshot.gga_count,
        (unsigned long)gps_snapshot.rmc_count,
        (unsigned long)gps_snapshot.gsa_count,
        (unsigned long)gps_snapshot.gsv_count,
        (unsigned long)gps_snapshot.psti030_count,
        (unsigned long)gps_snapshot.psti032_count,
        (unsigned long)gps_snapshot.psti035_count,
        (unsigned long)gps_snapshot.ths_count,
        gps_snapshot.rtk_age_s,
        gps_snapshot.rtk_ratio,
        gps_snapshot.baseline_valid ? "true" : "false",
        (unsigned)gps_snapshot.baseline_source,
        gps_snapshot.baseline_status != '\0' ? gps_snapshot.baseline_status : '-',
        gps_snapshot.baseline_mode != '\0' ? gps_snapshot.baseline_mode : '-',
        gps_snapshot.baseline_east_m,
        gps_snapshot.baseline_north_m,
        gps_snapshot.baseline_up_m,
        gps_snapshot.baseline_length_m,
        gps_snapshot.baseline_course_deg,
        gps_snapshot.true_heading_valid ? "true" : "false",
        gps_snapshot.true_heading_deg,
        gps_snapshot.true_heading_mode != '\0' ? gps_snapshot.true_heading_mode : '-',
        gps_snapshot.moving_base_role,
        gps_snapshot.moving_base_active ? "true" : "false",
        gps_snapshot.moving_base_correction_uart_ready ? "true" : "false",
        gps_snapshot.moving_base_receiver_config_sent ? "true" : "false",
        (unsigned long)gps_snapshot.moving_base_receiver_ack_count,
        (unsigned long)gps_snapshot.moving_base_receiver_nack_count,
        (unsigned)gps_snapshot.moving_base_receiver_last_ack_id,
        (unsigned)gps_snapshot.moving_base_receiver_last_nack_id,
        (unsigned long)gps_snapshot.moving_base_uplink_packet_count,
        (unsigned long)gps_snapshot.moving_base_uplink_byte_count,
        (unsigned long)gps_snapshot.moving_base_uplink_error_count,
        (unsigned long)gps_snapshot.moving_base_downlink_packet_count,
        (unsigned long)gps_snapshot.moving_base_downlink_byte_count,
        (unsigned long)gps_snapshot.moving_base_downlink_error_count,
        (unsigned long)gps_snapshot.moving_base_downlink_gap_count,
        (unsigned long)gps_snapshot.moving_base_last_uplink_age_ms,
        (unsigned long)gps_snapshot.moving_base_last_downlink_age_ms,
        (unsigned)gps_snapshot.moving_base_last_downlink_source_id,
        (unsigned long)gps_snapshot.moving_base_skytraq_frame_count,
        gps_snapshot.moving_base_software_version_valid ? "true" : "false",
        (unsigned)gps_snapshot.moving_base_software_type,
        (unsigned long)gps_snapshot.moving_base_software_kernel_version,
        (unsigned long)gps_snapshot.moving_base_software_odm_version,
        (unsigned long)gps_snapshot.moving_base_software_revision,
        gps_snapshot.moving_base_binary_output_status_valid ? "true" : "false",
        (unsigned)gps_snapshot.moving_base_binary_output_rate_code,
        gps_snapshot.moving_base_binary_meas_time_enabled ? "true" : "false",
        gps_snapshot.moving_base_binary_raw_meas_enabled ? "true" : "false",
        (unsigned long)gps_snapshot.moving_base_binary_meas_time_count,
        (unsigned long)gps_snapshot.moving_base_binary_raw_meas_count,
        (unsigned long)gps_snapshot.moving_base_rtcm_preamble_count,
        gps_snapshot.ntrip_configured ? "true" : "false",
        gps_snapshot.ntrip_running ? "true" : "false",
        gps_snapshot.ntrip_tls_connected ? "true" : "false",
        gps_snapshot.ntrip_stream_active ? "true" : "false",
        (unsigned)gps_snapshot.ntrip_http_status,
        (unsigned long)gps_snapshot.ntrip_connect_count,
        (unsigned long)gps_snapshot.ntrip_reconnect_count,
        (unsigned long)gps_snapshot.ntrip_error_count,
        (unsigned long)gps_snapshot.ntrip_rtcm_frame_count,
        (unsigned long)gps_snapshot.ntrip_rtcm_byte_count,
        (unsigned long)gps_snapshot.ntrip_last_data_age_ms,
        gps_snapshot.ntrip_state,
        (unsigned long)gps_snapshot.checksum_error_count,
        (unsigned long)gps_snapshot.parse_error_count,
        charger_snapshot.monitor_enabled ? "true" : "false",
        charger_snapshot.present ? "true" : "false",
        charger_snapshot.read_ok ? "true" : "false",
        charger_snapshot.raw_valid ? "true" : "false",
        charger_snapshot.config_write_supported ? "true" : "false",
        charger_snapshot.config_writes_enabled ? "true" : "false",
        charger_snapshot.last_error,
        esp_err_to_name((esp_err_t)charger_snapshot.last_error),
        (unsigned long)charger_snapshot.read_count,
        (unsigned long)charger_snapshot.full_read_count,
        (unsigned long)charger_snapshot.quick_read_count,
        (unsigned long)charger_snapshot.error_count,
        (unsigned long)charger_snapshot.last_read_duration_ms,
        charger_snapshot.last_read_full ? "true" : "false",
        (unsigned long)charger_snapshot.last_update_age_ms,
        (unsigned long)charger_snapshot.i2c_clock_hz,
        charger_snapshot.i2c_scl_measure_error,
        esp_err_to_name((esp_err_t)charger_snapshot.i2c_scl_measure_error),
        (unsigned long)charger_snapshot.i2c_scl_edges,
        (unsigned long)charger_snapshot.i2c_scl_elapsed_us,
        (unsigned long)charger_snapshot.i2c_scl_measured_hz,
        charger_snapshot.int_gpio_level,
        (unsigned long)charger_snapshot.int_irq_count,
        (unsigned long)charger_snapshot.int_last_irq_age_ms,
        charger_snapshot.pg_gpio_level,
        charger_snapshot.pg_asserted ? "true" : "false",
        charger_snapshot.pg_stat ? "true" : "false",
        charger_snapshot.qon_gpio_level,
        charger_snapshot.qon_asserted ? "true" : "false",
        (unsigned)charger_snapshot.part_info,
        (unsigned)charger_snapshot.part_number,
        (unsigned)charger_snapshot.device_revision,
        charger_snapshot.adc_enabled ? "true" : "false",
        (unsigned)charger_snapshot.reg09_termination_control,
        (unsigned)charger_snapshot.reg0a_recharge_control,
        (unsigned)charger_snapshot.reg0d_iotg_regulation,
        (unsigned)charger_snapshot.reg0e_timer_control,
        (unsigned)charger_snapshot.reg16_temperature_control,
        (unsigned)charger_snapshot.reg17_ntc_control_0,
        (unsigned)charger_snapshot.reg18_ntc_control_1,
        (unsigned)charger_snapshot.reg0f_charger_control_0,
        (unsigned)charger_snapshot.reg10_charger_control_1,
        (unsigned)charger_snapshot.reg14_charger_control_5,
        (unsigned)charger_snapshot.reg2e_adc_control,
        (unsigned)charger_snapshot.reg2f_adc_disable_0,
        (unsigned)charger_snapshot.reg30_adc_disable_1,
        (unsigned)charger_snapshot.minimal_system_voltage_mv,
        (unsigned)charger_snapshot.charge_voltage_limit_mv,
        (unsigned)charger_snapshot.charge_current_limit_ma,
        (unsigned)charger_snapshot.input_voltage_limit_mv,
        (unsigned)charger_snapshot.input_current_limit_ma,
        charger_snapshot.external_input_current_limit_enabled ? "true"
                                                              : "false",
        charger_snapshot.charge_enabled ? "true" : "false",
        charger_snapshot.termination_enabled ? "true" : "false",
        (unsigned)charger_snapshot.termination_current_ma,
        (unsigned)charger_snapshot.recharge_threshold_offset_mv,
        (unsigned)charger_snapshot.recharge_threshold_mv,
        (unsigned)charger_snapshot.recharge_deglitch_ms,
        (unsigned)charger_snapshot.charge_status_code,
        (unsigned)charger_snapshot.vbus_status_code,
        charger_snapshot.iindpm_active ? "true" : "false",
        charger_snapshot.vindpm_active ? "true" : "false",
        charger_snapshot.vsys_regulation_active ? "true" : "false",
        charger_snapshot.battery_overvoltage_active ? "true" : "false",
        charger_snapshot.charge_safety_timer_expired ? "true" : "false",
        charger_snapshot.topoff_timer_flag ? "true" : "false",
        charger_snapshot.trickle_timer_flag ? "true" : "false",
        charger_snapshot.precharge_timer_flag ? "true" : "false",
        charger_snapshot.fast_charge_timer_flag ? "true" : "false",
        (unsigned)charger_snapshot.watchdog_setting,
        charger_snapshot.watchdog_disabled ? "true" : "false",
        (unsigned)charger_snapshot.topoff_timer_minutes,
        charger_snapshot.trickle_timer_enabled ? "true" : "false",
        charger_snapshot.precharge_timer_enabled ? "true" : "false",
        charger_snapshot.fast_charge_timer_enabled ? "true" : "false",
        (unsigned)charger_snapshot.fast_charge_timer_hours,
        charger_snapshot.timer_2x_enabled ? "true" : "false",
        (unsigned)charger_snapshot.precharge_timer_minutes,
        (unsigned)charger_snapshot.adc_sample,
        charger_snapshot.adc_continuous ? "true" : "false",
        charger_snapshot.adc_running_average ? "true" : "false",
        charger_snapshot.ibat_discharge_sense_enabled ? "true" : "false",
        (unsigned)charger_snapshot.charger_status[0],
        (unsigned)charger_snapshot.charger_status[1],
        (unsigned)charger_snapshot.charger_status[2],
        (unsigned)charger_snapshot.charger_status[3],
        (unsigned)charger_snapshot.charger_status[4],
        (unsigned)charger_snapshot.fault_status[0],
        (unsigned)charger_snapshot.fault_status[1],
        (unsigned)charger_snapshot.charger_flag[0],
        (unsigned)charger_snapshot.charger_flag[1],
        (unsigned)charger_snapshot.charger_flag[2],
        (unsigned)charger_snapshot.charger_flag[3],
        (unsigned)charger_snapshot.fault_flag[0],
        (unsigned)charger_snapshot.fault_flag[1],
        (int)charger_snapshot.ibus_ma,
        (int)charger_snapshot.ibat_ma,
        (unsigned)charger_snapshot.vbus_mv,
        (unsigned)charger_snapshot.vac1_mv,
        (unsigned)charger_snapshot.vac2_mv,
        (unsigned)charger_snapshot.vbat_mv,
        (unsigned)charger_snapshot.vsys_mv,
        charger_snapshot.battery_soc_valid ? "true" : "false",
        (unsigned)charger_snapshot.battery_soc_percent,
        charger_snapshot.ts_percent,
        charger_snapshot.ts_ignore ? "true" : "false",
        charger_snapshot.ts_cold_active ? "true" : "false",
        charger_snapshot.ts_cool_active ? "true" : "false",
        charger_snapshot.ts_warm_active ? "true" : "false",
        charger_snapshot.ts_hot_active ? "true" : "false",
        charger_snapshot.ts_cold_flag ? "true" : "false",
        charger_snapshot.ts_cool_flag ? "true" : "false",
        charger_snapshot.ts_warm_flag ? "true" : "false",
        charger_snapshot.ts_hot_flag ? "true" : "false",
        charger_snapshot.tdie_c,
        (unsigned)charger_snapshot.dp_mv,
        (unsigned)charger_snapshot.dm_mv,
        (unsigned long)charger_snapshot.write_count,
        (unsigned long)charger_snapshot.write_error_count,
        (unsigned long)charger_snapshot.last_write_age_ms,
        (unsigned)charger_snapshot.last_write_reg,
        (unsigned)charger_snapshot.last_write_requested_value,
        (unsigned)charger_snapshot.last_write_mask,
        (unsigned)charger_snapshot.last_write_before,
        (unsigned)charger_snapshot.last_write_after,
        charger_snapshot.last_write_mask_used ? "true" : "false",
        charger_snapshot.last_write_changed ? "true" : "false",
        charger_snapshot.last_write_error,
        esp_err_to_name((esp_err_t)charger_snapshot.last_write_error),
        charger_raw_hex,
        pd_snapshot.monitor_enabled ? "true" : "false",
        pd_snapshot.present ? "true" : "false",
        pd_snapshot.read_ok ? "true" : "false",
        pd_snapshot.raw_valid ? "true" : "false",
        pd_snapshot.config_write_supported ? "true" : "false",
        pd_snapshot.config_writes_enabled ? "true" : "false",
        pd_snapshot.last_error,
        esp_err_to_name((esp_err_t)pd_snapshot.last_error),
        (unsigned long)pd_snapshot.read_count,
        (unsigned long)pd_snapshot.error_count,
        (unsigned long)pd_snapshot.last_read_duration_ms,
        (unsigned long)pd_snapshot.last_update_age_ms,
        (unsigned long)pd_snapshot.i2c_clock_hz,
        (unsigned long)pd_snapshot.i2c_hs_direct_clock_hz,
        pd_snapshot.i2c_hs_ext_requested ? "true" : "false",
        pd_snapshot.i2c_hs_ext_active ? "true" : "false",
        pd_snapshot.i2c_hs_direct_reads_enabled ? "true" : "false",
        pd_snapshot.i2c_hs_direct_writes_enabled ? "true" : "false",
        (unsigned long)pd_snapshot.i2c_hs_direct_read_count,
        (unsigned long)pd_snapshot.i2c_hs_direct_write_count,
        (unsigned long)pd_snapshot.i2c_hs_direct_error_count,
        (unsigned long)pd_snapshot.i2c_hs_direct_last_elapsed_us,
        pd_snapshot.i2c_hs_direct_scl_measure_error,
        esp_err_to_name((esp_err_t)pd_snapshot.i2c_hs_direct_scl_measure_error),
        (unsigned long)pd_snapshot.i2c_hs_direct_scl_edges,
        (unsigned long)pd_snapshot.i2c_hs_direct_scl_elapsed_us,
        (unsigned long)pd_snapshot.i2c_hs_direct_scl_measured_hz,
        (unsigned)pd_snapshot.device_id,
        (unsigned)pd_snapshot.device_rev,
        (unsigned)pd_snapshot.fw_rev,
        (unsigned)pd_snapshot.fw_sub_ver,
        (unsigned)pd_snapshot.uic_int,
        (unsigned)pd_snapshot.cc_int,
        (unsigned)pd_snapshot.pd_int,
        (unsigned)pd_snapshot.action_int,
        (unsigned)pd_snapshot.usbc_status1,
        (unsigned)pd_snapshot.usbc_status2,
        max77958_service_sys_msg_to_string(pd_snapshot.usbc_status2),
        (unsigned)pd_snapshot.bc_status,
        (unsigned)pd_snapshot.dp_status,
        (unsigned)pd_snapshot.cc_status0,
        (unsigned)pd_snapshot.cc_status1,
        (unsigned)pd_snapshot.pd_status0,
        (unsigned)pd_snapshot.pd_status1,
        (unsigned)pd_snapshot.uic_int_mask,
        (unsigned)pd_snapshot.cc_int_mask,
        (unsigned)pd_snapshot.pd_int_mask,
        (unsigned)pd_snapshot.action_int_mask,
        (unsigned)pd_snapshot.sw_reset,
        (unsigned)pd_snapshot.i2c_cnfg,
        (unsigned)pd_snapshot.vbadc_code,
        (unsigned)pd_snapshot.vbus_min_mv,
        (unsigned)pd_snapshot.vbus_max_mv,
        (unsigned)pd_snapshot.vbus_mid_mv,
        pd_snapshot.vbus_above_range ? "true" : "false",
        pd_snapshot.vbus_detected ? "true" : "false",
        (unsigned)pd_snapshot.chg_typ,
        max77958_service_chg_typ_to_string(pd_snapshot.chg_typ),
        (unsigned)pd_snapshot.pr_chg_typ,
        pd_snapshot.dcd_timeout ? "true" : "false",
        (unsigned)pd_snapshot.cc_pin,
        max77958_service_cc_pin_to_string(pd_snapshot.cc_pin),
        (unsigned)pd_snapshot.cci,
        max77958_service_cci_to_string(pd_snapshot.cci),
        pd_snapshot.vconn_enabled ? "true" : "false",
        (unsigned)pd_snapshot.cc_stat,
        max77958_service_cc_stat_to_string(pd_snapshot.cc_stat),
        pd_snapshot.det_abrt ? "true" : "false",
        pd_snapshot.data_role_dfp ? "true" : "false",
        pd_snapshot.power_role_source ? "true" : "false",
        pd_snapshot.vconn_source ? "true" : "false",
        pd_snapshot.psrdy_as_sink ? "true" : "false",
        pd_snapshot.ctrl1_valid ? "true" : "false",
        (unsigned)pd_snapshot.ctrl1_raw,
        (unsigned)pd_snapshot.ctrl1_comp2_sw,
        (unsigned)pd_snapshot.ctrl1_comn1_sw,
        pd_snapshot.usb2_switch_closed ? "true" : "false",
        pd_snapshot.source_caps_valid ? "true" : "false",
        (unsigned)pd_snapshot.source_pdo_count,
        (unsigned)pd_snapshot.selected_source_pdo_pos,
        pd_source_pdos_json[0] != '\0' ? pd_source_pdos_json : "[]",
        pd_snapshot.sink_pdos_valid ? "true" : "false",
        (unsigned)pd_snapshot.sink_pdo_count,
        pd_snapshot.sink_pdos_from_mtp ? "true" : "false",
        pd_sink_pdos_json[0] != '\0' ? pd_sink_pdos_json : "[]",
        pd_snapshot.pps_default_valid ? "true" : "false",
        pd_snapshot.pps_default_enabled ? "true" : "false",
        (unsigned)pd_snapshot.pps_default_mv,
        (unsigned)pd_snapshot.pps_default_ma,
        (unsigned long)pd_snapshot.operation_count,
        (unsigned long)pd_snapshot.operation_error_count,
        (unsigned long)pd_snapshot.last_operation_age_ms,
        (unsigned)pd_snapshot.last_opcode,
        (unsigned)pd_snapshot.last_response_opcode,
        (unsigned)pd_snapshot.last_result_code,
        max77958_service_apdo_result_to_string(pd_snapshot.last_result_code),
        pd_snapshot.last_operation_error,
        esp_err_to_name((esp_err_t)pd_snapshot.last_operation_error),
        pd_last_response_hex,
        pd_raw_hex,
        (unsigned)runtime_radio_channel(runtime_config),
        (unsigned)runtime_radio_phy_mode(runtime_config),
        (unsigned long)runtime_config->wireless_telemetry_port,
        uwb_dw3000_status_to_string(uwb_dw3000_get_status()),
        uwb_dw3000_runtime_switch_in_progress() ? "true" : "false",
        (unsigned long)uwb_dw3000_get_runtime_switch_count(),
        (unsigned long)uwb_dw3000_get_last_runtime_switch_ms(),
        (unsigned)runtime_radio_profile(runtime_config),
        (unsigned)runtime_radio_channel(runtime_config),
        (unsigned)runtime_radio_rf_channel_bit(runtime_config),
        (unsigned)runtime_radio_preamble_len_code(runtime_config),
        (unsigned)runtime_radio_preamble_code(runtime_config),
        (unsigned)runtime_radio_pac(runtime_config),
        (unsigned)runtime_radio_data_rate(runtime_config),
        (unsigned)APP_UWB_RADIO_PHR_MODE,
        (unsigned)runtime_radio_phr_rate(runtime_config),
        (unsigned)runtime_radio_sfd_type(runtime_config),
        (unsigned)runtime_radio_sfd_timeout(runtime_config),
        (unsigned)APP_UWB_RADIO_TX_PG_DELAY,
        (unsigned long)APP_UWB_RADIO_TX_POWER,
        (unsigned long)runtime_radio_rf_tx_ctrl_2(runtime_config),
        (unsigned long)runtime_radio_pll_cfg_final(runtime_config),
        (unsigned)APP_UWB_STS_MODE,
        (unsigned)APP_UWB_STS_LENGTH_SYMBOLS,
        APP_UWB_DIAGNOSTICS_ENABLED ? "true" : "false",
        (unsigned)APP_UWB_DIAGNOSTICS_LOG_EVERY,
        APP_UWB_EVENT_COUNTERS_ENABLED ? "true" : "false",
        (unsigned)APP_UWB_EVENT_COUNTERS_LOG_EVERY,
        (unsigned long)uwb_dw3000_get_device_id(),
        (unsigned long)uwb_dw3000_get_spi_clock_hz(),
        (unsigned)uwb_dw3000_get_source_id(),
        (unsigned)active_antenna_delay, (unsigned)active_antenna_delay,
        (unsigned)configured_antenna_delay,
        (unsigned)configured_antenna_delay,
        app_identity_uwb_antenna_delay_from_nvs() ? "true" : "false",
        active_antenna_delay != configured_antenna_delay ? "true" : "false",
        (unsigned long)uwb_dw3000_get_tx_count(),
        (unsigned long)uwb_dw3000_get_tx_error_count(),
        (unsigned long)uwb_dw3000_get_rx_count(),
        (unsigned long)uwb_dw3000_get_rx_error_count(),
        (unsigned long)uwb_dw3000_get_rx_ignored_count(),
        (unsigned)uwb_dw3000_get_last_rx_source_id(),
        (unsigned long)uwb_dw3000_get_last_rx_sequence(),
        wireless_log_service_status_to_string(
            wireless_log_service_get_status()),
        wireless_log_service_is_connected() ? "true" : "false",
        wireless_log_service_get_target(),
        (unsigned)wireless_log_service_get_port(),
        (unsigned long)wireless_log_service_get_dropped_count(),
        wireless_telemetry_service_status_to_string(
            wireless_telemetry_service_get_status()),
        wireless_telemetry_service_is_connected() ? "true" : "false",
        wireless_telemetry_service_get_target(),
        (unsigned)wireless_telemetry_service_get_port(),
        (unsigned long)wireless_telemetry_service_get_dropped_count(),
        (unsigned long)wireless_telemetry_service_get_drop_full_count(),
        (unsigned long)wireless_telemetry_service_get_drop_mutex_count(),
        (unsigned long)wireless_telemetry_service_get_drop_format_count(),
        (unsigned long)wireless_telemetry_service_get_queue_depth(),
        (unsigned long)wireless_telemetry_service_get_queue_high_water(),
        (unsigned long)wireless_telemetry_service_get_binary_frame_count(),
        (unsigned long)wireless_telemetry_service_get_binary_sample_count(),
        (unsigned long)wireless_telemetry_service_get_text_frame_count(),
        (unsigned long)wireless_telemetry_service_get_connect_count(),
        (unsigned long)wireless_telemetry_service_get_send_failure_count(),
        (unsigned long)wireless_telemetry_service_get_send_timeout_count(),
        (unsigned long)wireless_telemetry_service_get_socket_close_count(),
        (unsigned long)wireless_telemetry_service_get_last_send_ms(),
        (unsigned long)wireless_telemetry_service_get_max_send_ms(),
        wireless_telemetry_service_get_last_error());

    if (len < 0 || len >= OTA_SERVICE_STATUS_RESPONSE_SIZE) {
        free(ctx);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "status too long");
    }

    httpd_resp_set_type(req, "application/json");
    const esp_err_t response_err = httpd_resp_send(req, response, (ssize_t)len);
    free(ctx);
#undef i2c_stats
#undef bno_snapshot
#undef gps_snapshot
#undef charger_snapshot
#undef pd_snapshot
#undef resource_snapshot
#undef charger_raw_hex
#undef resource_top_tasks_json
#undef pd_raw_hex
#undef pd_last_response_hex
#undef pd_source_pdos_json
#undef pd_sink_pdos_json
#undef runtime_anchor_ids_json
#undef runtime_flex_slots_json
#undef runtime_flex_masks_json
#undef runtime_flex_anchor_x_json
#undef runtime_flex_anchor_y_json
#undef runtime_flex_anchor_correction_json
#undef runtime_passive_ds_anchor_bias_json
#undef runtime_passive_ds_range_bias_json
#undef passive_ds_pipeline_stats
#undef passive_ds_pipeline_stats_json
#undef native_ds_pipeline_stats
#undef native_ds_pipeline_stats_json
    return response_err;
}

static esp_err_t antenna_delay_post_handler(httpd_req_t *req)
{
    if (!ota_request_authorized(req)) {
        ESP_LOGW(TAG,
                 "Rejected antenna delay config: missing or invalid token");
        return httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED,
                                   "Missing or invalid OTA token");
    }

    if (s_ota_in_progress) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "OTA already in progress");
    }

    const size_t query_len = httpd_req_get_url_query_len(req);
    if (query_len == 0 || query_len >= OTA_SERVICE_MAX_QUERY_LEN) {
        return httpd_resp_send_err(
            req, HTTPD_400_BAD_REQUEST,
            "Use ?value=0x4018 or ?clear=1");
    }

    char query[OTA_SERVICE_MAX_QUERY_LEN] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid query string");
    }

    const bool clear_requested = ota_query_option_enabled(query, "clear");
    const bool reboot_requested = ota_query_option_enabled(query, "reboot");
    esp_err_t err = ESP_OK;

    if (clear_requested) {
        err = app_identity_clear_uwb_antenna_delay();
    } else {
        char value_text[24] = {0};
        if (httpd_query_key_value(query, "value", value_text,
                                  sizeof(value_text)) != ESP_OK) {
            return httpd_resp_send_err(
                req, HTTPD_400_BAD_REQUEST,
                "Missing antenna delay value; use ?value=0x4018");
        }

        uint16_t delay = 0;
        if (!ota_parse_u16(value_text, &delay)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid antenna delay value");
        }

        err = app_identity_set_uwb_antenna_delay(delay);
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Antenna delay config failed: %s",
                 esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Antenna delay config failed");
    }

    const uint16_t active_delay = uwb_dw3000_get_antenna_delay();
    const uint16_t configured_delay = app_identity_get_uwb_antenna_delay();
    const bool reboot_required = active_delay != configured_delay;
    ESP_LOGW(TAG,
             "Antenna delay config: active=0x%04x configured=0x%04x source=%s reboot_required=%s reboot_requested=%s",
             (unsigned)active_delay, (unsigned)configured_delay,
             app_identity_uwb_antenna_delay_from_nvs() ? "nvs" : "fallback",
             reboot_required ? "true" : "false",
             reboot_requested ? "true" : "false");

    char response[420];
    const int len = snprintf(
        response, sizeof(response),
        "{"
        "\"ok\":true,"
        "\"uwb_active_antenna_delay\":%u,"
        "\"uwb_active_antenna_delay_hex\":\"0x%04x\","
        "\"uwb_configured_antenna_delay\":%u,"
        "\"uwb_configured_antenna_delay_hex\":\"0x%04x\","
        "\"uwb_antenna_delay_from_nvs\":%s,"
        "\"reboot_required\":%s,"
        "\"rebooting\":%s"
        "}\n",
        (unsigned)active_delay, (unsigned)active_delay,
        (unsigned)configured_delay, (unsigned)configured_delay,
        app_identity_uwb_antenna_delay_from_nvs() ? "true" : "false",
        reboot_required ? "true" : "false",
        reboot_requested ? "true" : "false");

    if (len < 0 || len >= (int)sizeof(response)) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "response too long");
    }

    httpd_resp_set_type(req, "application/json");
    const esp_err_t response_err = httpd_resp_send(req, response, len);

    if (reboot_requested) {
        xTaskCreate(reboot_task, "cfg_reboot",
                    OTA_SERVICE_RESTART_TASK_STACK_WORDS, NULL,
                    OTA_SERVICE_RESTART_TASK_PRIORITY, NULL);
    }

    return response_err;
}

static esp_err_t charger_config_apply_u16(
    const char *query, const char *key, esp_err_t (*setter)(
                                        uint16_t,
                                        charger_service_write_result_t *,
                                        size_t, size_t *),
    bool *handled, uint32_t *operation_count, esp_err_t *first_error)
{
    char value_text[32] = {0};
    const esp_err_t query_err =
        httpd_query_key_value(query, key, value_text, sizeof(value_text));
    if (query_err == ESP_ERR_NOT_FOUND) {
        return ESP_OK;
    }
    if (query_err != ESP_OK) {
        return query_err;
    }

    uint16_t value = 0;
    if (!ota_parse_u16(value_text, &value)) {
        return ESP_ERR_INVALID_ARG;
    }

    charger_service_write_result_t results[4] = {0};
    size_t written_count = 0;
    const esp_err_t err = setter(value, results, 4, &written_count);
    *handled = true;
    *operation_count += (uint32_t)written_count;
    if (err != ESP_OK && *first_error == ESP_OK) {
        *first_error = err;
    }
    return ESP_OK;
}

static esp_err_t charger_config_post_handler(httpd_req_t *req)
{
    if (!ota_request_authorized(req)) {
        ESP_LOGW(TAG, "Rejected charger config: missing or invalid token");
        return httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED,
                                   "Missing or invalid OTA token");
    }

    if (s_ota_in_progress) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "OTA already in progress");
    }

    const size_t query_len = httpd_req_get_url_query_len(req);
    if (query_len == 0 || query_len >= OTA_SERVICE_MAX_QUERY_LEN) {
        return httpd_resp_send_err(
            req, HTTPD_400_BAD_REQUEST,
            "Use charger config query parameters");
    }

    char query[OTA_SERVICE_MAX_QUERY_LEN] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid query string");
    }

    bool handled = false;
    uint32_t operation_count = 0;
    esp_err_t first_error = ESP_OK;
    esp_err_t query_err = ESP_OK;

    if (ota_query_option_enabled(query, "refresh")) {
        charger_service_request_refresh();
        handled = true;
    }

    if (ota_query_option_enabled(query, "disable_watchdog") ||
        ota_query_option_enabled(query, "watchdog_disable")) {
        charger_service_write_result_t result = {0};
        const esp_err_t err = charger_service_set_watchdog_disabled(&result);
        operation_count++;
        handled = true;
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    }

    char charge_enabled_text[16] = {0};
    query_err = httpd_query_key_value(query, "charge_enabled",
                                      charge_enabled_text,
                                      sizeof(charge_enabled_text));
    if (query_err == ESP_ERR_NOT_FOUND) {
        query_err = httpd_query_key_value(query, "charging",
                                          charge_enabled_text,
                                          sizeof(charge_enabled_text));
    }
    if (query_err == ESP_ERR_NOT_FOUND) {
        query_err = httpd_query_key_value(query, "en_chg", charge_enabled_text,
                                          sizeof(charge_enabled_text));
    }
    if (query_err == ESP_OK) {
        bool charge_enabled = false;
        if (!ota_parse_bool_text(charge_enabled_text, &charge_enabled)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid charge_enabled value");
        }
        charger_service_write_result_t result = {0};
        const esp_err_t err =
            charger_service_set_charge_enabled(charge_enabled, &result);
        operation_count += 2U;
        handled = true;
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    } else if (query_err != ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid charge_enabled value");
    }

    char adc_text[16] = {0};
    query_err = httpd_query_key_value(query, "adc", adc_text, sizeof(adc_text));
    if (query_err == ESP_OK) {
        bool adc_enabled = false;
        if (!ota_parse_bool_text(adc_text, &adc_enabled)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid adc value");
        }

        uint8_t adc_sample = 2;
        char sample_text[16] = {0};
        query_err = httpd_query_key_value(query, "adc_sample", sample_text,
                                          sizeof(sample_text));
        if (query_err == ESP_OK) {
            if (!ota_parse_u8(sample_text, &adc_sample) || adc_sample > 3U) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid adc_sample");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid adc_sample");
        }

        bool continuous = true;
        char rate_text[16] = {0};
        query_err = httpd_query_key_value(query, "adc_rate", rate_text,
                                          sizeof(rate_text));
        if (query_err == ESP_OK) {
            if (strcmp(rate_text, "continuous") == 0 ||
                strcmp(rate_text, "cont") == 0 || strcmp(rate_text, "0") == 0) {
                continuous = true;
            } else if (strcmp(rate_text, "oneshot") == 0 ||
                       strcmp(rate_text, "one_shot") == 0 ||
                       strcmp(rate_text, "1") == 0) {
                continuous = false;
            } else {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid adc_rate");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid adc_rate");
        }

        bool running_average = false;
        char avg_text[16] = {0};
        query_err = httpd_query_key_value(query, "adc_avg", avg_text,
                                          sizeof(avg_text));
        if (query_err == ESP_OK) {
            if (!ota_parse_bool_text(avg_text, &running_average)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid adc_avg");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid adc_avg");
        }

        charger_service_write_result_t results[6] = {0};
        size_t written_count = 0;
        const esp_err_t err =
            charger_service_set_adc(adc_enabled, continuous, adc_sample,
                                    running_average, results, 6,
                                    &written_count);
        operation_count += (uint32_t)written_count;
        handled = true;
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    } else if (query_err != ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid adc value");
    }

    esp_err_t apply_err = charger_config_apply_u16(
        query, "minimal_system_voltage_mv",
        charger_service_set_minimal_system_voltage_mv, &handled,
        &operation_count, &first_error);
    if (apply_err != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid minimal_system_voltage_mv");
    }

    apply_err = charger_config_apply_u16(
        query, "charge_voltage_mv",
        charger_service_set_charge_voltage_limit_mv, &handled,
        &operation_count, &first_error);
    if (apply_err != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid charge_voltage_mv");
    }

    apply_err = charger_config_apply_u16(
        query, "charge_current_ma",
        charger_service_set_charge_current_limit_ma, &handled,
        &operation_count, &first_error);
    if (apply_err != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid charge_current_ma");
    }

    char input_voltage_text[32] = {0};
    query_err = httpd_query_key_value(query, "input_voltage_mv",
                                      input_voltage_text,
                                      sizeof(input_voltage_text));
    if (query_err == ESP_OK) {
        uint16_t mv = 0;
        if (!ota_parse_u16(input_voltage_text, &mv)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid input_voltage_mv");
        }
        charger_service_write_result_t result = {0};
        const esp_err_t err =
            charger_service_set_input_voltage_limit_mv(mv, &result);
        operation_count += 2U;
        handled = true;
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    } else if (query_err != ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid input_voltage_mv");
    }

    char ext_ilim_text[16] = {0};
    query_err = httpd_query_key_value(
        query, "external_input_current_limit_enabled", ext_ilim_text,
        sizeof(ext_ilim_text));
    if (query_err == ESP_ERR_NOT_FOUND) {
        query_err = httpd_query_key_value(query, "external_ilim_enabled",
                                          ext_ilim_text,
                                          sizeof(ext_ilim_text));
    }
    if (query_err == ESP_ERR_NOT_FOUND) {
        query_err = httpd_query_key_value(query, "en_extilim", ext_ilim_text,
                                          sizeof(ext_ilim_text));
    }
    if (query_err == ESP_OK) {
        bool enabled = false;
        if (!ota_parse_bool_text(ext_ilim_text, &enabled)) {
            return httpd_resp_send_err(
                req, HTTPD_400_BAD_REQUEST,
                "Invalid external_input_current_limit_enabled");
        }
        charger_service_write_result_t result = {0};
        const esp_err_t err =
            charger_service_set_external_input_current_limit_enabled(enabled,
                                                                     &result);
        operation_count += 2U;
        handled = true;
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    } else if (query_err != ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(
            req, HTTPD_400_BAD_REQUEST,
            "Invalid external_input_current_limit_enabled");
    }

    apply_err = charger_config_apply_u16(
        query, "input_current_ma",
        charger_service_set_input_current_limit_ma, &handled,
        &operation_count, &first_error);
    if (apply_err != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid input_current_ma");
    }

    apply_err = charger_config_apply_u16(
        query, "iindpm_ma",
        charger_service_set_input_current_limit_ma, &handled,
        &operation_count, &first_error);
    if (apply_err != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid iindpm_ma");
    }

    const bool termination_requested =
        ota_query_has_key(query, "termination_enabled") ||
        ota_query_has_key(query, "termination_current_ma") ||
        ota_query_has_key(query, "recharge_threshold_offset_mv") ||
        ota_query_has_key(query, "recharge_deglitch_ms");
    if (termination_requested) {
        charger_service_snapshot_t current = {0};
        charger_service_get_snapshot(&current);

        bool termination_enabled =
            current.raw_valid ? current.termination_enabled : true;
        uint16_t termination_current_ma =
            current.termination_current_ma != 0U
                ? current.termination_current_ma
                : 200U;
        uint16_t recharge_threshold_offset_mv =
            current.recharge_threshold_offset_mv != 0U
                ? current.recharge_threshold_offset_mv
                : 200U;
        uint16_t recharge_deglitch_ms =
            current.recharge_deglitch_ms != 0U ? current.recharge_deglitch_ms
                                               : 1024U;

        char text[32] = {0};
        query_err = httpd_query_key_value(query, "termination_enabled", text,
                                          sizeof(text));
        if (query_err == ESP_OK) {
            if (!ota_parse_bool_text(text, &termination_enabled)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid termination_enabled");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid termination_enabled");
        }

        query_err = httpd_query_key_value(query, "termination_current_ma",
                                          text, sizeof(text));
        if (query_err == ESP_OK) {
            if (!ota_parse_u16(text, &termination_current_ma)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid termination_current_ma");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid termination_current_ma");
        }

        query_err =
            httpd_query_key_value(query, "recharge_threshold_offset_mv", text,
                                  sizeof(text));
        if (query_err == ESP_OK) {
            if (!ota_parse_u16(text, &recharge_threshold_offset_mv)) {
                return httpd_resp_send_err(
                    req, HTTPD_400_BAD_REQUEST,
                    "Invalid recharge_threshold_offset_mv");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(
                req, HTTPD_400_BAD_REQUEST,
                "Invalid recharge_threshold_offset_mv");
        }

        query_err = httpd_query_key_value(query, "recharge_deglitch_ms", text,
                                          sizeof(text));
        if (query_err == ESP_OK) {
            if (!ota_parse_u16(text, &recharge_deglitch_ms)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid recharge_deglitch_ms");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid recharge_deglitch_ms");
        }

        charger_service_write_result_t results[4] = {0};
        size_t written_count = 0;
        const esp_err_t err = charger_service_set_termination_recharge(
            termination_enabled, termination_current_ma,
            recharge_threshold_offset_mv, recharge_deglitch_ms, results, 4,
            &written_count);
        operation_count += (uint32_t)written_count;
        handled = true;
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    }

    const bool timer_requested =
        ota_query_has_key(query, "topoff_timer_minutes") ||
        ota_query_has_key(query, "trickle_timer_enabled") ||
        ota_query_has_key(query, "precharge_timer_enabled") ||
        ota_query_has_key(query, "fast_charge_timer_enabled") ||
        ota_query_has_key(query, "fast_charge_timer_hours") ||
        ota_query_has_key(query, "timer_2x_enabled") ||
        ota_query_has_key(query, "precharge_timer_minutes");
    if (timer_requested) {
        charger_service_snapshot_t current = {0};
        charger_service_get_snapshot(&current);

        uint16_t topoff_timer_minutes = current.topoff_timer_minutes;
        bool trickle_timer_enabled = current.trickle_timer_enabled;
        bool precharge_timer_enabled = current.precharge_timer_enabled;
        bool fast_charge_timer_enabled = current.fast_charge_timer_enabled;
        uint8_t fast_charge_timer_hours = current.fast_charge_timer_hours;
        bool timer_2x_enabled = current.timer_2x_enabled;
        uint16_t precharge_timer_minutes = current.precharge_timer_minutes;

        char text[32] = {0};
        query_err = httpd_query_key_value(query, "topoff_timer_minutes", text,
                                          sizeof(text));
        if (query_err == ESP_OK) {
            if (!ota_parse_u16(text, &topoff_timer_minutes)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid topoff_timer_minutes");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid topoff_timer_minutes");
        }

        query_err = httpd_query_key_value(query, "trickle_timer_enabled", text,
                                          sizeof(text));
        if (query_err == ESP_OK) {
            if (!ota_parse_bool_text(text, &trickle_timer_enabled)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid trickle_timer_enabled");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid trickle_timer_enabled");
        }

        query_err = httpd_query_key_value(query, "precharge_timer_enabled",
                                          text, sizeof(text));
        if (query_err == ESP_OK) {
            if (!ota_parse_bool_text(text, &precharge_timer_enabled)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid precharge_timer_enabled");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid precharge_timer_enabled");
        }

        query_err = httpd_query_key_value(query, "fast_charge_timer_enabled",
                                          text, sizeof(text));
        if (query_err == ESP_OK) {
            if (!ota_parse_bool_text(text, &fast_charge_timer_enabled)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid fast_charge_timer_enabled");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid fast_charge_timer_enabled");
        }

        query_err = httpd_query_key_value(query, "fast_charge_timer_hours",
                                          text, sizeof(text));
        if (query_err == ESP_OK) {
            if (!ota_parse_u8(text, &fast_charge_timer_hours)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid fast_charge_timer_hours");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid fast_charge_timer_hours");
        }

        query_err = httpd_query_key_value(query, "timer_2x_enabled", text,
                                          sizeof(text));
        if (query_err == ESP_OK) {
            if (!ota_parse_bool_text(text, &timer_2x_enabled)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid timer_2x_enabled");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid timer_2x_enabled");
        }

        query_err = httpd_query_key_value(query, "precharge_timer_minutes",
                                          text, sizeof(text));
        if (query_err == ESP_OK) {
            if (!ota_parse_u16(text, &precharge_timer_minutes)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid precharge_timer_minutes");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid precharge_timer_minutes");
        }

        charger_service_write_result_t results[4] = {0};
        size_t written_count = 0;
        const esp_err_t err = charger_service_set_safety_timers(
            topoff_timer_minutes, trickle_timer_enabled,
            precharge_timer_enabled, fast_charge_timer_enabled,
            fast_charge_timer_hours, timer_2x_enabled, precharge_timer_minutes,
            results, 4, &written_count);
        operation_count += (uint32_t)written_count;
        handled = true;
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    }

    char reg_text[16] = {0};
    query_err = httpd_query_key_value(query, "reg", reg_text,
                                      sizeof(reg_text));
    if (query_err == ESP_OK) {
        if (!ota_query_option_enabled(query, "confirm")) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Raw register write needs confirm=1");
        }

        uint8_t reg = 0;
        if (!ota_parse_u8(reg_text, &reg) ||
            reg >= CHARGER_SERVICE_REGISTER_MAP_SIZE) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid reg");
        }

        charger_service_write_result_t result = {0};
        const esp_err_t watchdog_err =
            charger_service_set_watchdog_disabled(&result);
        operation_count++;
        if (watchdog_err != ESP_OK && first_error == ESP_OK) {
            first_error = watchdog_err;
        }

        char mask_text[16] = {0};
        char value_text[16] = {0};
        const esp_err_t mask_err =
            httpd_query_key_value(query, "mask", mask_text, sizeof(mask_text));
        if (mask_err == ESP_OK) {
            char bits_text[16] = {0};
            if (httpd_query_key_value(query, "bits", bits_text,
                                      sizeof(bits_text)) != ESP_OK &&
                httpd_query_key_value(query, "value", bits_text,
                                      sizeof(bits_text)) != ESP_OK) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Missing bits/value");
            }
            uint8_t mask = 0;
            uint8_t bits = 0;
            if (!ota_parse_u8(mask_text, &mask) ||
                !ota_parse_u8(bits_text, &bits)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid mask/bits");
            }
            const esp_err_t err =
                charger_service_update_register_bits(reg, mask, bits, &result);
            operation_count++;
            if (err != ESP_OK && first_error == ESP_OK) {
                first_error = err;
            }
        } else if (mask_err == ESP_ERR_NOT_FOUND) {
            if (httpd_query_key_value(query, "value", value_text,
                                      sizeof(value_text)) != ESP_OK) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Missing raw value");
            }
            uint8_t value = 0;
            if (!ota_parse_u8(value_text, &value)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid value");
            }
            const esp_err_t err =
                charger_service_write_register(reg, value, &result);
            operation_count++;
            if (err != ESP_OK && first_error == ESP_OK) {
                first_error = err;
            }
        } else {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid mask");
        }
        handled = true;
    } else if (query_err != ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid reg");
    }

    if (!handled) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "No charger operation requested");
    }

    charger_service_request_refresh();
    vTaskDelay(pdMS_TO_TICKS(40));

    charger_service_snapshot_t snapshot = {0};
    charger_service_get_snapshot(&snapshot);

    ESP_LOGW(TAG,
             "Charger config: ops=%lu err=%s ADC=%s CHG=%s TERM=%s ITERM=%umA VRECHG=%umV TRECHG=%ums REG0F=0x%02X VREG=%umV ICHG=%umA VINDPM=%umV IINDPM=%umA EXTILIM=%s fast_tmr=%s/%uh pre_tmr=%s/%umin WD=%u",
             (unsigned long)operation_count, esp_err_to_name(first_error),
             snapshot.adc_enabled ? "on" : "off",
             snapshot.charge_enabled ? "on" : "off",
             snapshot.termination_enabled ? "on" : "off",
             (unsigned)snapshot.termination_current_ma,
             (unsigned)snapshot.recharge_threshold_offset_mv,
             (unsigned)snapshot.recharge_deglitch_ms,
             (unsigned)snapshot.reg0f_charger_control_0,
             (unsigned)snapshot.charge_voltage_limit_mv,
             (unsigned)snapshot.charge_current_limit_ma,
             (unsigned)snapshot.input_voltage_limit_mv,
             (unsigned)snapshot.input_current_limit_ma,
             snapshot.external_input_current_limit_enabled ? "on" : "off",
             snapshot.fast_charge_timer_enabled ? "on" : "off",
             (unsigned)snapshot.fast_charge_timer_hours,
             snapshot.precharge_timer_enabled ? "on" : "off",
             (unsigned)snapshot.precharge_timer_minutes,
             (unsigned)snapshot.watchdog_setting);

    char response[3000];
    const int len = snprintf(
        response, sizeof(response),
        "{"
        "\"ok\":%s,"
        "\"operation_count\":%lu,"
        "\"error\":%d,"
        "\"error_name\":\"%s\","
        "\"charger_present\":%s,"
        "\"charger_adc_enabled\":%s,"
        "\"charger_adc_sample\":%u,"
        "\"charger_adc_continuous\":%s,"
        "\"charger_adc_running_average\":%s,"
        "\"charger_charge_enabled\":%s,"
        "\"charger_termination_enabled\":%s,"
        "\"charger_termination_current_ma\":%u,"
        "\"charger_recharge_threshold_offset_mv\":%u,"
        "\"charger_recharge_threshold_mv\":%u,"
        "\"charger_recharge_deglitch_ms\":%u,"
        "\"charger_reg0f_charger_control_0\":\"0x%02x\","
        "\"charger_reg09_termination_control\":\"0x%02x\","
        "\"charger_reg0a_recharge_control\":\"0x%02x\","
        "\"charger_reg0d_iotg_regulation\":\"0x%02x\","
        "\"charger_reg0e_timer_control\":\"0x%02x\","
        "\"charger_reg16_temperature_control\":\"0x%02x\","
        "\"charger_reg17_ntc_control_0\":\"0x%02x\","
        "\"charger_reg18_ntc_control_1\":\"0x%02x\","
        "\"charger_watchdog_setting\":%u,"
        "\"charger_watchdog_disabled\":%s,"
        "\"charger_topoff_timer_minutes\":%u,"
        "\"charger_trickle_timer_enabled\":%s,"
        "\"charger_precharge_timer_enabled\":%s,"
        "\"charger_fast_charge_timer_enabled\":%s,"
        "\"charger_fast_charge_timer_hours\":%u,"
        "\"charger_timer_2x_enabled\":%s,"
        "\"charger_precharge_timer_minutes\":%u,"
        "\"charger_charge_safety_timer_expired\":%s,"
        "\"charger_topoff_timer_flag\":%s,"
        "\"charger_trickle_timer_flag\":%s,"
        "\"charger_precharge_timer_flag\":%s,"
        "\"charger_fast_charge_timer_flag\":%s,"
        "\"charger_minimal_system_voltage_mv\":%u,"
        "\"charger_charge_voltage_limit_mv\":%u,"
        "\"charger_charge_current_limit_ma\":%u,"
        "\"charger_input_voltage_limit_mv\":%u,"
        "\"charger_input_current_limit_ma\":%u,"
        "\"charger_external_input_current_limit_enabled\":%s,"
        "\"charger_vbat_mv\":%u,"
        "\"charger_vsys_mv\":%u,"
        "\"charger_vbus_mv\":%u,"
        "\"charger_battery_soc_valid\":%s,"
        "\"charger_battery_soc_percent\":%u,"
        "\"charger_ibus_ma\":%d,"
        "\"charger_ibat_ma\":%d,"
        "\"charger_ts_percent\":%.4f,"
        "\"charger_ts_ignore\":%s,"
        "\"charger_ts_cold_active\":%s,"
        "\"charger_ts_cool_active\":%s,"
        "\"charger_ts_warm_active\":%s,"
        "\"charger_ts_hot_active\":%s,"
        "\"charger_tdie_c\":%.1f,"
        "\"charger_write_count\":%lu,"
        "\"charger_write_error_count\":%lu,"
        "\"charger_last_write_reg\":\"0x%02x\","
        "\"charger_last_write_before\":\"0x%02x\","
        "\"charger_last_write_after\":\"0x%02x\","
        "\"charger_last_write_error_name\":\"%s\""
        "}\n",
        first_error == ESP_OK ? "true" : "false",
        (unsigned long)operation_count, first_error,
        esp_err_to_name(first_error),
        snapshot.present ? "true" : "false",
        snapshot.adc_enabled ? "true" : "false",
        (unsigned)snapshot.adc_sample,
        snapshot.adc_continuous ? "true" : "false",
        snapshot.adc_running_average ? "true" : "false",
        snapshot.charge_enabled ? "true" : "false",
        snapshot.termination_enabled ? "true" : "false",
        (unsigned)snapshot.termination_current_ma,
        (unsigned)snapshot.recharge_threshold_offset_mv,
        (unsigned)snapshot.recharge_threshold_mv,
        (unsigned)snapshot.recharge_deglitch_ms,
        (unsigned)snapshot.reg0f_charger_control_0,
        (unsigned)snapshot.reg09_termination_control,
        (unsigned)snapshot.reg0a_recharge_control,
        (unsigned)snapshot.reg0d_iotg_regulation,
        (unsigned)snapshot.reg0e_timer_control,
        (unsigned)snapshot.reg16_temperature_control,
        (unsigned)snapshot.reg17_ntc_control_0,
        (unsigned)snapshot.reg18_ntc_control_1,
        (unsigned)snapshot.watchdog_setting,
        snapshot.watchdog_disabled ? "true" : "false",
        (unsigned)snapshot.topoff_timer_minutes,
        snapshot.trickle_timer_enabled ? "true" : "false",
        snapshot.precharge_timer_enabled ? "true" : "false",
        snapshot.fast_charge_timer_enabled ? "true" : "false",
        (unsigned)snapshot.fast_charge_timer_hours,
        snapshot.timer_2x_enabled ? "true" : "false",
        (unsigned)snapshot.precharge_timer_minutes,
        snapshot.charge_safety_timer_expired ? "true" : "false",
        snapshot.topoff_timer_flag ? "true" : "false",
        snapshot.trickle_timer_flag ? "true" : "false",
        snapshot.precharge_timer_flag ? "true" : "false",
        snapshot.fast_charge_timer_flag ? "true" : "false",
        (unsigned)snapshot.minimal_system_voltage_mv,
        (unsigned)snapshot.charge_voltage_limit_mv,
        (unsigned)snapshot.charge_current_limit_ma,
        (unsigned)snapshot.input_voltage_limit_mv,
        (unsigned)snapshot.input_current_limit_ma,
        snapshot.external_input_current_limit_enabled ? "true" : "false",
        (unsigned)snapshot.vbat_mv,
        (unsigned)snapshot.vsys_mv,
        (unsigned)snapshot.vbus_mv,
        snapshot.battery_soc_valid ? "true" : "false",
        (unsigned)snapshot.battery_soc_percent,
        (int)snapshot.ibus_ma,
        (int)snapshot.ibat_ma,
        snapshot.ts_percent,
        snapshot.ts_ignore ? "true" : "false",
        snapshot.ts_cold_active ? "true" : "false",
        snapshot.ts_cool_active ? "true" : "false",
        snapshot.ts_warm_active ? "true" : "false",
        snapshot.ts_hot_active ? "true" : "false",
        snapshot.tdie_c,
        (unsigned long)snapshot.write_count,
        (unsigned long)snapshot.write_error_count,
        (unsigned)snapshot.last_write_reg,
        (unsigned)snapshot.last_write_before,
        (unsigned)snapshot.last_write_after,
        esp_err_to_name((esp_err_t)snapshot.last_write_error));

    if (len < 0 || len >= (int)sizeof(response)) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "response too long");
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, response, len);
}

static esp_err_t max77958_config_post_handler(httpd_req_t *req)
{
    if (!ota_request_authorized(req)) {
        ESP_LOGW(TAG, "Rejected MAX77958 config: missing or invalid token");
        return httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED,
                                   "Missing or invalid OTA token");
    }

    if (s_ota_in_progress) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "OTA already in progress");
    }

    const size_t query_len = httpd_req_get_url_query_len(req);
    if (query_len == 0 || query_len >= OTA_SERVICE_MAX_QUERY_LEN) {
        return httpd_resp_send_err(
            req, HTTPD_400_BAD_REQUEST,
            "Use MAX77958 config query parameters");
    }

    char query[OTA_SERVICE_MAX_QUERY_LEN] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid query string");
    }

    bool handled = false;
    uint32_t operation_count = 0;
    esp_err_t first_error = ESP_OK;
    esp_err_t query_err = ESP_OK;
    max77958_service_ap_result_t result = {0};

    if (ota_query_option_enabled(query, "refresh")) {
        max77958_service_request_refresh();
        handled = true;
    }

    if (ota_query_option_enabled(query, "bc_trigger") ||
        ota_query_option_enabled(query, "trigger_bc")) {
        const esp_err_t err = max77958_service_trigger_bc_detection(&result);
        operation_count++;
        handled = true;
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    }

    char usb2_text[16] = {0};
    query_err = httpd_query_key_value(query, "usb2_closed", usb2_text,
                                      sizeof(usb2_text));
    if (query_err == ESP_ERR_NOT_FOUND) {
        query_err = httpd_query_key_value(query, "usb2_switch_closed",
                                          usb2_text, sizeof(usb2_text));
    }
    if (query_err == ESP_OK) {
        bool closed = false;
        if (!ota_parse_bool_text(usb2_text, &closed)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid usb2_closed");
        }
        result = (max77958_service_ap_result_t){0};
        const esp_err_t err = max77958_service_set_usb2_switch(closed,
                                                               &result);
        operation_count++;
        handled = true;
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    } else if (query_err != ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid usb2_closed");
    }

    char source_pos_text[16] = {0};
    query_err = httpd_query_key_value(query, "source_pdo_pos",
                                      source_pos_text,
                                      sizeof(source_pos_text));
    if (query_err == ESP_ERR_NOT_FOUND) {
        query_err = httpd_query_key_value(query, "source_pos",
                                          source_pos_text,
                                          sizeof(source_pos_text));
    }
    if (query_err == ESP_OK) {
        uint8_t pos = 0;
        if (!ota_parse_u8(source_pos_text, &pos)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid source_pdo_pos");
        }
        result = (max77958_service_ap_result_t){0};
        const esp_err_t err =
            max77958_service_request_source_pdo(pos, &result);
        operation_count++;
        handled = true;
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    } else if (query_err != ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid source_pdo_pos");
    }

    char sink_pdos_text[160] = {0};
    query_err = httpd_query_key_value(query, "sink_pdos", sink_pdos_text,
                                      sizeof(sink_pdos_text));
    if (query_err == ESP_OK) {
        uint16_t voltages[MAX77958_SERVICE_MAX_SINK_PDOS] = {0};
        uint16_t currents[MAX77958_SERVICE_MAX_SINK_PDOS] = {0};
        size_t pdo_count = 0;
        if (!ota_parse_pd_fixed_pdo_list(
                sink_pdos_text, voltages, currents,
                MAX77958_SERVICE_MAX_SINK_PDOS, &pdo_count)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid sink_pdos");
        }
        bool mtp = false;
        char mtp_text[16] = {0};
        const esp_err_t mtp_err = httpd_query_key_value(
            query, "sink_pdos_mtp", mtp_text, sizeof(mtp_text));
        if (mtp_err == ESP_OK) {
            if (!ota_parse_bool_text(mtp_text, &mtp)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid sink_pdos_mtp");
            }
        } else if (mtp_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid sink_pdos_mtp");
        }

        result = (max77958_service_ap_result_t){0};
        const esp_err_t err = max77958_service_set_sink_fixed_pdos(
            voltages, currents, pdo_count, mtp, &result);
        operation_count++;
        handled = true;
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    } else if (query_err != ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid sink_pdos");
    }

    const bool pps_requested =
        ota_query_has_key(query, "pps_enabled") ||
        ota_query_has_key(query, "pps_voltage_mv") ||
        ota_query_has_key(query, "pps_current_ma");
    if (pps_requested) {
        bool enabled = false;
        uint16_t voltage_mv = 5000;
        uint16_t current_ma = 3000;
        char text[32] = {0};

        query_err = httpd_query_key_value(query, "pps_enabled", text,
                                          sizeof(text));
        if (query_err == ESP_OK) {
            if (!ota_parse_bool_text(text, &enabled)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid pps_enabled");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid pps_enabled");
        }

        query_err = httpd_query_key_value(query, "pps_voltage_mv", text,
                                          sizeof(text));
        if (query_err == ESP_OK) {
            if (!ota_parse_u16(text, &voltage_mv)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid pps_voltage_mv");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid pps_voltage_mv");
        }

        query_err = httpd_query_key_value(query, "pps_current_ma", text,
                                          sizeof(text));
        if (query_err == ESP_OK) {
            if (!ota_parse_u16(text, &current_ma)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid pps_current_ma");
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid pps_current_ma");
        }

        result = (max77958_service_ap_result_t){0};
        const esp_err_t err = max77958_service_set_pps_default(
            enabled, voltage_mv, current_ma, &result);
        operation_count++;
        handled = true;
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    }

    const bool apdo_requested =
        ota_query_has_key(query, "apdo_pos") ||
        ota_query_has_key(query, "apdo_voltage_mv") ||
        ota_query_has_key(query, "apdo_current_ma");
    if (apdo_requested) {
        uint8_t pos = 0;
        uint16_t voltage_mv = 0;
        uint16_t current_ma = 0;
        char text[32] = {0};

        if (httpd_query_key_value(query, "apdo_pos", text,
                                  sizeof(text)) != ESP_OK ||
            !ota_parse_u8(text, &pos) ||
            httpd_query_key_value(query, "apdo_voltage_mv", text,
                                  sizeof(text)) != ESP_OK ||
            !ota_parse_u16(text, &voltage_mv) ||
            httpd_query_key_value(query, "apdo_current_ma", text,
                                  sizeof(text)) != ESP_OK ||
            !ota_parse_u16(text, &current_ma)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid APDO request");
        }

        result = (max77958_service_ap_result_t){0};
        const esp_err_t err =
            max77958_service_request_apdo(pos, voltage_mv, current_ma,
                                          &result);
        operation_count++;
        handled = true;
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    }

    if (ota_query_option_enabled(query, "read_sink") ||
        ota_query_has_key(query, "read_sink_mtp")) {
        bool mtp = false;
        char mtp_text[16] = {0};
        const esp_err_t mtp_err = httpd_query_key_value(
            query, "read_sink_mtp", mtp_text, sizeof(mtp_text));
        if (mtp_err == ESP_OK) {
            if (!ota_parse_bool_text(mtp_text, &mtp)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid read_sink_mtp");
            }
        } else if (mtp_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid read_sink_mtp");
        }
        result = (max77958_service_ap_result_t){0};
        const esp_err_t err = max77958_service_read_sink_pdos(mtp, &result);
        operation_count++;
        handled = true;
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    }

    char reg_text[16] = {0};
    query_err = httpd_query_key_value(query, "reg", reg_text,
                                      sizeof(reg_text));
    if (query_err == ESP_OK) {
        if (!ota_query_option_enabled(query, "confirm")) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Raw MAX77958 write needs confirm=1");
        }
        uint8_t reg = 0;
        uint8_t value = 0;
        char value_text[16] = {0};
        if (!ota_parse_u8(reg_text, &reg) ||
            httpd_query_key_value(query, "value", value_text,
                                  sizeof(value_text)) != ESP_OK ||
            !ota_parse_u8(value_text, &value)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid raw register write");
        }
        const esp_err_t err = max77958_service_write_register(reg, value);
        operation_count++;
        handled = true;
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    } else if (query_err != ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid reg");
    }

    if (!handled) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "No MAX77958 operation requested");
    }

    max77958_service_request_refresh();
    vTaskDelay(pdMS_TO_TICKS(80));

    max77958_service_snapshot_t snapshot = {0};
    max77958_service_get_snapshot(&snapshot);
    ESP_LOGW(TAG,
             "MAX77958 config: ops=%lu err=%s present=%s VBUS~%umV BC=%s CC=%s source_pos=%u src_pdos=%u sink_pdos=%u last_op=0x%02X result=%u",
             (unsigned long)operation_count, esp_err_to_name(first_error),
             snapshot.present ? "yes" : "no",
             (unsigned)snapshot.vbus_mid_mv,
             max77958_service_chg_typ_to_string(snapshot.chg_typ),
             max77958_service_cc_stat_to_string(snapshot.cc_stat),
             (unsigned)snapshot.selected_source_pdo_pos,
             (unsigned)snapshot.source_pdo_count,
             (unsigned)snapshot.sink_pdo_count,
             (unsigned)snapshot.last_opcode,
             (unsigned)snapshot.last_result_code);

    char response[1000];
    const int len = snprintf(
        response, sizeof(response),
        "{"
        "\"ok\":%s,"
        "\"operation_count\":%lu,"
        "\"error\":%d,"
        "\"error_name\":\"%s\","
        "\"pd_present\":%s,"
        "\"pd_vbus_mid_mv\":%u,"
        "\"pd_vbus_detected\":%s,"
        "\"pd_chg_typ_name\":\"%s\","
        "\"pd_cc_stat_name\":\"%s\","
        "\"pd_usb2_switch_closed\":%s,"
        "\"pd_source_pdo_count\":%u,"
        "\"pd_selected_source_pdo_pos\":%u,"
        "\"pd_sink_pdo_count\":%u,"
        "\"pd_last_opcode\":\"0x%02x\","
        "\"pd_last_response_opcode\":\"0x%02x\","
        "\"pd_last_result_code\":%u,"
        "\"pd_last_result_name\":\"%s\","
        "\"pd_last_operation_error_name\":\"%s\""
        "}\n",
        first_error == ESP_OK ? "true" : "false",
        (unsigned long)operation_count, first_error,
        esp_err_to_name(first_error),
        snapshot.present ? "true" : "false",
        (unsigned)snapshot.vbus_mid_mv,
        snapshot.vbus_detected ? "true" : "false",
        max77958_service_chg_typ_to_string(snapshot.chg_typ),
        max77958_service_cc_stat_to_string(snapshot.cc_stat),
        snapshot.usb2_switch_closed ? "true" : "false",
        (unsigned)snapshot.source_pdo_count,
        (unsigned)snapshot.selected_source_pdo_pos,
        (unsigned)snapshot.sink_pdo_count,
        (unsigned)snapshot.last_opcode,
        (unsigned)snapshot.last_response_opcode,
        (unsigned)snapshot.last_result_code,
        max77958_service_apdo_result_to_string(snapshot.last_result_code),
        esp_err_to_name((esp_err_t)snapshot.last_operation_error));

    if (len < 0 || len >= (int)sizeof(response)) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "response too long");
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, response, len);
}

static esp_err_t runtime_config_post_handler(httpd_req_t *req)
{
    if (!ota_request_authorized(req)) {
        ESP_LOGW(TAG, "Rejected runtime config: missing or invalid token");
        return httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED,
                                   "Missing or invalid OTA token");
    }

    if (s_ota_in_progress) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "OTA already in progress");
    }

    const size_t query_len = httpd_req_get_url_query_len(req);
    if (query_len == 0 || query_len >= OTA_SERVICE_MAX_QUERY_LEN) {
        return httpd_resp_send_err(
            req, HTTPD_400_BAD_REQUEST,
            "Use runtime config query parameters");
    }

    char query[OTA_SERVICE_MAX_QUERY_LEN] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid query string");
    }

    const app_runtime_config_t before_config = *app_runtime_config_get();
    const bool clear_requested = ota_query_option_enabled(query, "clear");
    const bool reboot_requested = ota_query_option_enabled(query, "reboot");
    const bool hot_switch_requested =
        ota_query_option_enabled(query, "hot_switch");
    if (reboot_requested && hot_switch_requested) {
        return httpd_resp_send_err(
            req, HTTPD_400_BAD_REQUEST,
            "Choose either reboot=1 or hot_switch=1");
    }
    bool changed = false;
    bool cleared = false;
    esp_err_t err = ESP_OK;

    if (clear_requested) {
        err = app_runtime_config_clear();
        changed = true;
        cleared = true;
    } else {
        app_runtime_config_t config = before_config;

        char mode_text[40] = {0};
        esp_err_t query_err = httpd_query_key_value(
            query, "mode", mode_text, sizeof(mode_text));
        if (query_err == ESP_OK) {
            bool mode_ok = false;
            config.runtime_mode =
                app_runtime_config_runtime_mode_from_string(mode_text,
                                                            &mode_ok);
            if (!mode_ok) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid mode");
            }
            changed = true;
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid mode");
        }

        char anchors_text[80] = {0};
        query_err = httpd_query_key_value(query, "anchors", anchors_text,
                                          sizeof(anchors_text));
        if (query_err == ESP_OK) {
            uint8_t ids[APP_RUNTIME_CONFIG_MAX_ANCHORS] = {0};
            uint8_t count = 0;
            if (!ota_parse_u8_list(anchors_text, ids,
                                   APP_RUNTIME_CONFIG_MAX_ANCHORS, &count)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid anchors");
            }
            const bool anchors_changed =
                count != config.anchor_count ||
                memcmp(config.anchor_ids, ids, sizeof(config.anchor_ids)) != 0;
            if (anchors_changed) {
                memcpy(config.anchor_ids, ids, sizeof(config.anchor_ids));
                config.anchor_count = count;
                app_runtime_config_reset_flex_tdoa(&config);
                app_runtime_config_reset_passive_ds_calibration(&config);
                changed = true;
            }
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid anchors");
        }

        char flex_slots_text[80] = {0};
        query_err = httpd_query_key_value(query, "flex_slots",
                                          flex_slots_text,
                                          sizeof(flex_slots_text));
        if (query_err == ESP_OK) {
            uint8_t count = 0;
            memset(config.flex_tdoa_slot_initiator_ids, 0,
                   sizeof(config.flex_tdoa_slot_initiator_ids));
            if (!ota_parse_u8_list(
                    flex_slots_text, config.flex_tdoa_slot_initiator_ids,
                    APP_RUNTIME_CONFIG_FLEX_MAX_SLOTS, &count)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid flex_slots");
            }
            config.flex_tdoa_slot_count = count;
            changed = true;
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid flex_slots");
        }

        char flex_masks_text[120] = {0};
        query_err = httpd_query_key_value(query, "flex_masks",
                                          flex_masks_text,
                                          sizeof(flex_masks_text));
        if (query_err == ESP_OK) {
            uint8_t count = 0;
            memset(config.flex_tdoa_slot_responder_masks, 0,
                   sizeof(config.flex_tdoa_slot_responder_masks));
            if (!ota_parse_u16_list(
                    flex_masks_text, config.flex_tdoa_slot_responder_masks,
                    APP_RUNTIME_CONFIG_FLEX_MAX_SLOTS, &count) ||
                count != config.flex_tdoa_slot_count) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid flex_masks");
            }
            changed = true;
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid flex_masks");
        }

        const bool flex_geometry_clear =
            ota_query_option_enabled(query, "flex_geometry_clear");
        char flex_geometry_text[512] = {0};
        query_err = httpd_query_key_value(
            query, "flex_geometry", flex_geometry_text,
            sizeof(flex_geometry_text));
        if (query_err == ESP_OK) {
            int32_t x_mm[APP_RUNTIME_CONFIG_MAX_ANCHORS] = {0};
            int32_t y_mm[APP_RUNTIME_CONFIG_MAX_ANCHORS] = {0};
            if (!ota_parse_flex_geometry(flex_geometry_text, &config,
                                         x_mm, y_mm)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid flex_geometry");
            }
            memcpy(config.flex_tdoa_anchor_x_mm, x_mm,
                   sizeof(config.flex_tdoa_anchor_x_mm));
            memcpy(config.flex_tdoa_anchor_y_mm, y_mm,
                   sizeof(config.flex_tdoa_anchor_y_mm));
            config.flex_tdoa_geometry_fixed = true;
            config.flex_tdoa_geometry_generation =
                before_config.flex_tdoa_geometry_generation == UINT32_MAX
                    ? 1U
                    : before_config.flex_tdoa_geometry_generation + 1U;
            changed = true;
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid flex_geometry");
        } else if (flex_geometry_clear) {
            config.flex_tdoa_geometry_fixed = false;
            memset(config.flex_tdoa_anchor_x_mm, 0,
                   sizeof(config.flex_tdoa_anchor_x_mm));
            memset(config.flex_tdoa_anchor_y_mm, 0,
                   sizeof(config.flex_tdoa_anchor_y_mm));
            config.flex_tdoa_geometry_generation =
                before_config.flex_tdoa_geometry_generation == UINT32_MAX
                    ? 1U
                    : before_config.flex_tdoa_geometry_generation + 1U;
            changed = true;
        }

        char flex_anchor_correction_text[256] = {0};
        query_err = httpd_query_key_value(
            query, "flex_tdoa_anchor_correction_mm",
            flex_anchor_correction_text,
            sizeof(flex_anchor_correction_text));
        if (query_err == ESP_OK) {
            int32_t correction[APP_RUNTIME_CONFIG_MAX_ANCHORS] = {0};
            uint8_t correction_count = 0;
            if (!ota_parse_i32_list(
                    flex_anchor_correction_text, correction,
                    APP_RUNTIME_CONFIG_MAX_ANCHORS,
                    &correction_count) ||
                correction_count != config.anchor_count ||
                correction[0] != 0) {
                return httpd_resp_send_err(
                    req, HTTPD_400_BAD_REQUEST,
                    "Invalid FlexTDOA anchor correction list");
            }
            memset(config.flex_tdoa_anchor_correction_mm, 0,
                   sizeof(config.flex_tdoa_anchor_correction_mm));
            memcpy(config.flex_tdoa_anchor_correction_mm, correction,
                   (size_t)correction_count * sizeof(correction[0]));
            changed = true;
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(
                req, HTTPD_400_BAD_REQUEST,
                "Invalid FlexTDOA anchor correction list");
        }

        const bool passive_ds_calibration_clear =
            ota_query_option_enabled(
                query, "passive_ds_calibration_clear");
        char passive_ds_anchor_bias_text[256] = {0};
        char passive_ds_range_bias_text[512] = {0};
        const esp_err_t passive_ds_anchor_bias_err =
            httpd_query_key_value(
                query, "passive_ds_anchor_bias_mm",
                passive_ds_anchor_bias_text,
                sizeof(passive_ds_anchor_bias_text));
        const esp_err_t passive_ds_range_bias_err =
            httpd_query_key_value(
                query, "passive_ds_range_bias_mm",
                passive_ds_range_bias_text,
                sizeof(passive_ds_range_bias_text));
        if (passive_ds_anchor_bias_err == ESP_OK ||
            passive_ds_range_bias_err == ESP_OK) {
            if (passive_ds_anchor_bias_err != ESP_OK ||
                passive_ds_range_bias_err != ESP_OK ||
                passive_ds_calibration_clear) {
                return httpd_resp_send_err(
                    req, HTTPD_400_BAD_REQUEST,
                    "Passive DS calibration requires both bias lists");
            }
            int32_t anchor_bias[
                APP_RUNTIME_CONFIG_MAX_ANCHORS] = {0};
            int32_t range_bias[
                APP_RUNTIME_CONFIG_MAX_ANCHOR_PAIRS] = {0};
            uint8_t anchor_bias_count = 0;
            uint8_t range_bias_count = 0;
            const size_t expected_pair_count =
                (size_t)config.anchor_count *
                ((size_t)config.anchor_count - 1U) / 2U;
            if (!ota_parse_i32_list(
                    passive_ds_anchor_bias_text, anchor_bias,
                    APP_RUNTIME_CONFIG_MAX_ANCHORS,
                    &anchor_bias_count) ||
                !ota_parse_i32_list(
                    passive_ds_range_bias_text, range_bias,
                    APP_RUNTIME_CONFIG_MAX_ANCHOR_PAIRS,
                    &range_bias_count) ||
                anchor_bias_count != config.anchor_count ||
                range_bias_count != expected_pair_count ||
                anchor_bias[0] != 0) {
                return httpd_resp_send_err(
                    req, HTTPD_400_BAD_REQUEST,
                    "Invalid Passive DS calibration lists");
            }
            memset(config.passive_ds_anchor_bias_mm, 0,
                   sizeof(config.passive_ds_anchor_bias_mm));
            memset(config.passive_ds_range_bias_mm, 0,
                   sizeof(config.passive_ds_range_bias_mm));
            memcpy(config.passive_ds_anchor_bias_mm, anchor_bias,
                   (size_t)anchor_bias_count * sizeof(anchor_bias[0]));
            size_t compact_pair = 0;
            for (size_t a = 0; a < config.anchor_count; ++a) {
                for (size_t b = a + 1U; b < config.anchor_count; ++b) {
                    const size_t stored_pair =
                        app_runtime_config_anchor_pair_index(a, b);
                    if (stored_pair == SIZE_MAX) {
                        return httpd_resp_send_err(
                            req, HTTPD_400_BAD_REQUEST,
                            "Invalid Passive DS pair index");
                    }
                    config.passive_ds_range_bias_mm[stored_pair] =
                        range_bias[compact_pair++];
                }
            }
            config.passive_ds_calibration_enabled = true;
            config.passive_ds_calibration_generation =
                before_config.passive_ds_calibration_generation ==
                        UINT32_MAX
                    ? 1U
                    : before_config.passive_ds_calibration_generation +
                          1U;
            changed = true;
        } else if (passive_ds_anchor_bias_err != ESP_ERR_NOT_FOUND ||
                   passive_ds_range_bias_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(
                req, HTTPD_400_BAD_REQUEST,
                "Invalid Passive DS calibration");
        } else if (passive_ds_calibration_clear) {
            app_runtime_config_reset_passive_ds_calibration(&config);
            changed = true;
        }

        char cal_three_text[64] = {0};
        query_err = httpd_query_key_value(query, "cal_three",
                                          cal_three_text,
                                          sizeof(cal_three_text));
        if (query_err == ESP_OK) {
            uint8_t ids[APP_RUNTIME_CONFIG_CAL_THREE_COUNT] = {0};
            uint8_t count = 0;
            if (!ota_parse_u8_list(cal_three_text, ids,
                                   APP_RUNTIME_CONFIG_CAL_THREE_COUNT,
                                   &count) ||
                count != APP_RUNTIME_CONFIG_CAL_THREE_COUNT) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid cal_three");
            }
            memcpy(config.calibration_three_ids, ids,
                   sizeof(config.calibration_three_ids));
            changed = true;
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid cal_three");
        }

        char cal_method_text[32] = {0};
        query_err = httpd_query_key_value(query, "cal_method",
                                          cal_method_text,
                                          sizeof(cal_method_text));
        if (query_err == ESP_OK) {
            if (!ota_parse_calibration_method(cal_method_text,
                                              &config.calibration_method)) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid cal_method");
            }
            changed = true;
        } else if (query_err != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid cal_method");
        }

#define APPLY_U8_PARAM(KEY, FIELD)                                      \
        do {                                                            \
            char value_text[32] = {0};                                  \
            esp_err_t key_err = httpd_query_key_value(                  \
                query, KEY, value_text, sizeof(value_text));            \
            if (key_err == ESP_OK) {                                    \
                uint8_t parsed = 0;                                     \
                if (!ota_parse_u8(value_text, &parsed)) {               \
                    return httpd_resp_send_err(req,                     \
                                               HTTPD_400_BAD_REQUEST,   \
                                               "Invalid " KEY);         \
                }                                                       \
                config.FIELD = parsed;                                  \
                changed = true;                                         \
            } else if (key_err != ESP_ERR_NOT_FOUND) {                  \
                return httpd_resp_send_err(req,                         \
                                           HTTPD_400_BAD_REQUEST,       \
                                           "Invalid " KEY);             \
            }                                                           \
        } while (0)

#define APPLY_U32_PARAM(KEY, FIELD)                                     \
        do {                                                            \
            char value_text[32] = {0};                                  \
            esp_err_t key_err = httpd_query_key_value(                  \
                query, KEY, value_text, sizeof(value_text));            \
            if (key_err == ESP_OK) {                                    \
                uint32_t parsed = 0;                                    \
                if (!ota_parse_u32(value_text, &parsed)) {              \
                    return httpd_resp_send_err(req,                     \
                                               HTTPD_400_BAD_REQUEST,   \
                                               "Invalid " KEY);         \
                }                                                       \
                config.FIELD = parsed;                                  \
                changed = true;                                         \
            } else if (key_err != ESP_ERR_NOT_FOUND) {                  \
                return httpd_resp_send_err(req,                         \
                                           HTTPD_400_BAD_REQUEST,       \
                                           "Invalid " KEY);             \
            }                                                           \
        } while (0)

#define APPLY_BOOL_PARAM(KEY, FIELD)                                    \
        do {                                                            \
            char value_text[8] = {0};                                   \
            esp_err_t key_err = httpd_query_key_value(                  \
                query, KEY, value_text, sizeof(value_text));            \
            if (key_err == ESP_OK) {                                    \
                if (strcmp(value_text, "1") == 0 ||                    \
                    strcmp(value_text, "true") == 0 ||                 \
                    strcmp(value_text, "on") == 0) {                   \
                    config.FIELD = true;                                \
                } else if (strcmp(value_text, "0") == 0 ||             \
                           strcmp(value_text, "false") == 0 ||         \
                           strcmp(value_text, "off") == 0) {           \
                    config.FIELD = false;                               \
                } else {                                                \
                    return httpd_resp_send_err(req,                     \
                                               HTTPD_400_BAD_REQUEST,   \
                                               "Invalid " KEY);         \
                }                                                       \
                changed = true;                                         \
            } else if (key_err != ESP_ERR_NOT_FOUND) {                  \
                return httpd_resp_send_err(req,                         \
                                           HTTPD_400_BAD_REQUEST,       \
                                           "Invalid " KEY);             \
            }                                                           \
        } while (0)

        APPLY_U8_PARAM("tag", tag_id);
        APPLY_U8_PARAM("anchor_count", anchor_count);
        APPLY_U8_PARAM("flex_k", flex_tdoa_responder_count);
        APPLY_U32_PARAM("flex_guard_us", flex_tdoa_guard_us);
        APPLY_U32_PARAM("flex_req_us", flex_tdoa_request_subslot_us);
        APPLY_U32_PARAM("flex_req_process_us", flex_tdoa_request_process_us);
        APPLY_U32_PARAM("flex_resp_us", flex_tdoa_response_subslot_us);
        APPLY_U32_PARAM("flex_resp_process_us",
                        flex_tdoa_response_process_us);
        APPLY_U8_PARAM("coordinator", anchor_survey_coordinator_id);
        APPLY_U8_PARAM("coord", anchor_survey_coordinator_id);
        APPLY_U32_PARAM("survey_rx_ms", anchor_survey_rx_slice_ms);
        APPLY_U32_PARAM("survey_delay_ms",
                        anchor_survey_command_delay_ms);
        APPLY_U32_PARAM("survey_slot_ms", anchor_survey_slot_ms);
        APPLY_U32_PARAM("survey_gap_ms", anchor_survey_round_gap_ms);
        APPLY_U32_PARAM("survey_log_every",
                        anchor_survey_passive_tag_log_every);
        APPLY_U32_PARAM("ranging_slot_ms", ranging_slot_ms);
        APPLY_U32_PARAM("ranging_gap_ms", ranging_round_gap_ms);
        APPLY_U32_PARAM("ranging_rx_ms", ranging_rx_slice_ms);
        APPLY_U32_PARAM("ranging_timeout_ms", ranging_rx_timeout_ms);
        APPLY_U32_PARAM("ranging_resp_delay_ms", ranging_resp_delay_ms);
        APPLY_U32_PARAM("ranging_final_delay_ms", ranging_final_delay_ms);
        APPLY_U32_PARAM("ranging_auto_rx_delay_uus",
                        ranging_auto_rx_delay_uus);
        APPLY_U8_PARAM("passive_ds_schedule", passive_ds_schedule);
        APPLY_U32_PARAM("passive_ds_slot_ms", passive_ds_slot_ms);
        APPLY_U32_PARAM("passive_ds_gap_ms", passive_ds_round_gap_ms);
        APPLY_U32_PARAM("passive_ds_rx_ms", passive_ds_rx_slice_ms);
        APPLY_U32_PARAM("passive_ds_timeout_ms", passive_ds_rx_timeout_ms);
        APPLY_U32_PARAM("passive_ds_resp_delay_us",
                        passive_ds_resp_delay_us);
        APPLY_U32_PARAM("passive_ds_final_delay_us",
                        passive_ds_final_delay_us);
        APPLY_U32_PARAM("passive_ds_auto_rx_delay_uus",
                        passive_ds_auto_rx_delay_uus);
        APPLY_U8_PARAM("passive_ds_pipeline_mode",
                       passive_ds_pipeline_mode);
        APPLY_U8_PARAM("passive_ds_solve_mode", passive_ds_solve_mode);
        APPLY_U32_PARAM("passive_ds_rolling_max_hz",
                        passive_ds_rolling_max_hz);
        APPLY_U8_PARAM("dt_peer", distance_test_peer_id);
        APPLY_U8_PARAM("dt_initiator", distance_test_initiator_id);
        APPLY_U8_PARAM("dt_responder", distance_test_responder_id);
        APPLY_U32_PARAM("dt_interval_ms", distance_test_interval_ms);
        APPLY_U32_PARAM("dt_rx_timeout_ms", distance_test_rx_timeout_ms);
        APPLY_U32_PARAM("dt_resp_delay_ms", distance_test_resp_delay_ms);
        APPLY_U32_PARAM("dt_final_delay_ms", distance_test_final_delay_ms);
        APPLY_U32_PARAM("dt_report_delay_ms", distance_test_report_delay_ms);
        APPLY_U32_PARAM("dt_auto_rx_delay_uus",
                        distance_test_auto_rx_delay_uus);
        APPLY_U8_PARAM("cal_ref", calibration_reference_id);
        APPLY_U8_PARAM("cal_dut", calibration_dut_id);
        APPLY_U32_PARAM("cal_known_mm", calibration_known_distance_mm);
        APPLY_U32_PARAM("cal_d01_mm", calibration_three_distance_0_1_mm);
        APPLY_U32_PARAM("cal_d02_mm", calibration_three_distance_0_2_mm);
        APPLY_U32_PARAM("cal_d12_mm", calibration_three_distance_1_2_mm);
        APPLY_U32_PARAM("cal_samples", calibration_sample_count);
        APPLY_U32_PARAM("cal_summary", calibration_summary_every);
        APPLY_U32_PARAM("cal_slot_ms", calibration_min_interval_ms);
        APPLY_U32_PARAM("cal_min_ms", calibration_min_interval_ms);
        APPLY_U32_PARAM("cal_round_gap_ms", calibration_max_interval_ms);
        APPLY_U32_PARAM("cal_max_ms", calibration_max_interval_ms);
        APPLY_U32_PARAM("cal_rx_ms", calibration_rx_slice_ms);
        APPLY_U32_PARAM("cal_guard", calibration_slot_guard_us);
        APPLY_U32_PARAM("cal_guard_us", calibration_slot_guard_us);
        APPLY_BOOL_PARAM("uwb", uwb_enabled);
        APPLY_BOOL_PARAM("bno085", bno085_accel_enabled);
        APPLY_BOOL_PARAM("bno085_accel", bno085_accel_enabled);
        APPLY_U32_PARAM("bno085_accel_interval_ms",
                        bno085_accel_interval_ms);
        APPLY_U32_PARAM("bno085_log_interval_ms", bno085_log_interval_ms);
        {
            char value_text[32] = {0};
            esp_err_t key_err = httpd_query_key_value(
                query, "bno085_sample_hz", value_text, sizeof(value_text));
            if (key_err == ESP_OK) {
                uint32_t interval_ms = 0;
                if (!ota_parse_bno085_sample_hz(value_text, &interval_ms)) {
                    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                               "Invalid bno085_sample_hz");
                }
                config.bno085_accel_interval_ms = interval_ms;
                changed = true;
            } else if (key_err != ESP_ERR_NOT_FOUND) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid bno085_sample_hz");
            }
        }
        {
            char value_text[32] = {0};
            esp_err_t key_err = httpd_query_key_value(
                query, "bno085_sample_ms", value_text, sizeof(value_text));
            if (key_err == ESP_OK) {
                uint32_t parsed = 0;
                if (!ota_parse_u32(value_text, &parsed)) {
                    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                               "Invalid bno085_sample_ms");
                }
                config.bno085_accel_interval_ms = parsed;
                changed = true;
            } else if (key_err != ESP_ERR_NOT_FOUND) {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "Invalid bno085_sample_ms");
            }
        }
        APPLY_BOOL_PARAM("gps", gps_enabled);
        APPLY_U8_PARAM("radio_channel", radio_channel);
        APPLY_U8_PARAM("uwb_channel", radio_channel);
        APPLY_U8_PARAM("radio_phy_mode", radio_phy_mode);
        APPLY_U8_PARAM("uwb_phy_mode", radio_phy_mode);
        APPLY_U32_PARAM("telemetry_port", wireless_telemetry_port);
        APPLY_U32_PARAM("tel_port", wireless_telemetry_port);

#undef APPLY_BOOL_PARAM
#undef APPLY_U32_PARAM
#undef APPLY_U8_PARAM

        if (config.flex_tdoa_responder_count !=
                before_config.flex_tdoa_responder_count ||
            config.flex_tdoa_slot_count != before_config.flex_tdoa_slot_count ||
            memcmp(config.flex_tdoa_slot_initiator_ids,
                   before_config.flex_tdoa_slot_initiator_ids,
                   sizeof(config.flex_tdoa_slot_initiator_ids)) != 0 ||
            memcmp(config.flex_tdoa_slot_responder_masks,
                   before_config.flex_tdoa_slot_responder_masks,
                   sizeof(config.flex_tdoa_slot_responder_masks)) != 0 ||
            config.flex_tdoa_guard_us != before_config.flex_tdoa_guard_us ||
            config.flex_tdoa_request_subslot_us !=
                before_config.flex_tdoa_request_subslot_us ||
            config.flex_tdoa_request_process_us !=
                before_config.flex_tdoa_request_process_us ||
            config.flex_tdoa_response_subslot_us !=
                before_config.flex_tdoa_response_subslot_us ||
            config.flex_tdoa_response_process_us !=
                before_config.flex_tdoa_response_process_us) {
            if (config.flex_tdoa_config_generation ==
                before_config.flex_tdoa_config_generation) {
                config.flex_tdoa_config_generation =
                    before_config.flex_tdoa_config_generation == UINT32_MAX
                        ? 1U
                        : before_config.flex_tdoa_config_generation + 1U;
            }
        }

        if (!app_runtime_config_validate(&config)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Invalid runtime config");
        }

        if (changed) {
            if (hot_switch_requested &&
                !runtime_config_hot_switch_eligible(
                    &before_config, &config)) {
                return httpd_resp_send_err(
                    req, HTTPD_400_BAD_REQUEST,
                    "Hot switch supports protocol transitions and in-place Passive DS-TWR timing reloads");
            }
            err = app_runtime_config_save(&config);
            if (err == ESP_OK &&
                config.flex_tdoa_geometry_generation !=
                    before_config.flex_tdoa_geometry_generation) {
                (void)flextdoa_solver_service_reload_geometry();
            }
        }
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Runtime config failed: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Runtime config failed");
    }

    const app_runtime_config_t *active_config = app_runtime_config_get();
    char active_anchor_ids_json[48] = {0};
    char active_flex_slots_json[48] = {0};
    char active_flex_masks_json[72] = {0};
    format_u8_array_json(active_config->anchor_ids,
                         active_config->anchor_count,
                         active_anchor_ids_json,
                         sizeof(active_anchor_ids_json));
    format_u8_array_json(active_config->flex_tdoa_slot_initiator_ids,
                         active_config->flex_tdoa_slot_count,
                         active_flex_slots_json,
                         sizeof(active_flex_slots_json));
    format_u16_array_json(active_config->flex_tdoa_slot_responder_masks,
                          active_config->flex_tdoa_slot_count,
                          active_flex_masks_json,
                          sizeof(active_flex_masks_json));
    if (changed && before_config.gps_enabled != active_config->gps_enabled) {
        const esp_err_t gps_err = gps_service_apply_runtime_config();
        if (gps_err != ESP_OK) {
            ESP_LOGW(TAG, "GPS runtime apply failed: %s",
                     esp_err_to_name(gps_err));
        }
    }

    if (changed &&
        (before_config.bno085_accel_enabled !=
             active_config->bno085_accel_enabled ||
         before_config.bno085_accel_interval_ms !=
             active_config->bno085_accel_interval_ms ||
         before_config.bno085_log_interval_ms !=
             active_config->bno085_log_interval_ms)) {
        const esp_err_t bno_err = bno085_service_apply_runtime_config();
        if (bno_err != ESP_OK) {
            ESP_LOGW(TAG, "BNO085 runtime apply failed: %s",
                     esp_err_to_name(bno_err));
        }
    }

    bool hot_switch_started = false;
    if (hot_switch_requested && changed && !cleared) {
        const esp_err_t switch_err =
            uwb_dw3000_request_runtime_switch(
                active_config->runtime_mode);
        if (switch_err != ESP_OK) {
            ESP_LOGE(TAG, "UWB hot switch request failed: %s",
                     esp_err_to_name(switch_err));
            return httpd_resp_send_err(
                req, HTTPD_500_INTERNAL_SERVER_ERROR,
                "UWB hot switch request failed");
        }
        hot_switch_started = true;
    }

    const bool reboot_recommended =
        runtime_config_reboot_recommended(&before_config, active_config);
    ESP_LOGW(TAG,
             "Runtime config: changed=%s cleared=%s mode=%s(%u) tag=%u anchors=%s count=%u K=%u M=%u generation=%lu coord=%u reboot_recommended=%s reboot_requested=%s hot_switch_requested=%s hot_switch_started=%s",
             changed ? "true" : "false", cleared ? "true" : "false",
             app_runtime_config_runtime_mode_to_string(
                 active_config->runtime_mode),
             (unsigned)active_config->runtime_mode,
             (unsigned)active_config->tag_id,
             active_anchor_ids_json,
             (unsigned)active_config->anchor_count,
             (unsigned)active_config->flex_tdoa_responder_count,
             (unsigned)active_config->flex_tdoa_slot_count,
             (unsigned long)active_config->flex_tdoa_config_generation,
             (unsigned)active_config->anchor_survey_coordinator_id,
             reboot_recommended ? "true" : "false",
             reboot_requested ? "true" : "false",
             hot_switch_requested ? "true" : "false",
             hot_switch_started ? "true" : "false");

    char *response = heap_caps_calloc(
        1, OTA_SERVICE_RUNTIME_RESPONSE_SIZE,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (response == NULL) {
        response = heap_caps_calloc(
            1, OTA_SERVICE_RUNTIME_RESPONSE_SIZE, MALLOC_CAP_8BIT);
    }
    if (response == NULL) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "runtime response allocation failed");
    }
    const int len = snprintf(
        response, OTA_SERVICE_RUNTIME_RESPONSE_SIZE,
        "{"
        "\"ok\":true,"
        "\"changed\":%s,"
        "\"cleared\":%s,"
        "\"runtime_config_from_nvs\":%s,"
        "\"runtime_mode\":%u,"
        "\"runtime_mode_name\":\"%s\","
        "\"runtime_tag_id\":%u,"
        "\"runtime_anchor_count\":%u,"
        "\"runtime_anchor_ids\":%s,"
        "\"runtime_flex_tdoa_responder_count\":%u,"
        "\"runtime_flex_tdoa_slot_count\":%u,"
        "\"runtime_flex_tdoa_slot_initiator_ids\":%s,"
        "\"runtime_flex_tdoa_slot_responder_masks\":%s,"
        "\"runtime_flex_tdoa_config_generation\":%lu,"
        "\"runtime_flex_tdoa_guard_us\":%lu,"
        "\"runtime_flex_tdoa_request_subslot_us\":%lu,"
        "\"runtime_flex_tdoa_request_process_us\":%lu,"
        "\"runtime_flex_tdoa_response_subslot_us\":%lu,"
        "\"runtime_flex_tdoa_response_process_us\":%lu,"
        "\"runtime_flex_tdoa_geometry_fixed\":%s,"
        "\"runtime_flex_tdoa_geometry_generation\":%lu,"
        "\"runtime_anchor_survey_coordinator_id\":%u,"
        "\"runtime_ranging_slot_ms\":%lu,"
        "\"runtime_ranging_round_gap_ms\":%lu,"
        "\"runtime_ranging_rx_slice_ms\":%lu,"
        "\"runtime_ranging_rx_timeout_ms\":%lu,"
        "\"runtime_ranging_resp_delay_ms\":%lu,"
        "\"runtime_ranging_final_delay_ms\":%lu,"
        "\"runtime_ranging_auto_rx_delay_uus\":%lu,"
        "\"runtime_passive_ds_schedule\":%u,"
        "\"runtime_passive_ds_slot_ms\":%lu,"
        "\"runtime_passive_ds_round_gap_ms\":%lu,"
        "\"runtime_passive_ds_rx_slice_ms\":%lu,"
        "\"runtime_passive_ds_rx_timeout_ms\":%lu,"
        "\"runtime_passive_ds_resp_delay_us\":%lu,"
        "\"runtime_passive_ds_final_delay_us\":%lu,"
        "\"runtime_passive_ds_auto_rx_delay_uus\":%lu,"
        "\"runtime_passive_ds_pipeline_mode\":%u,"
        "\"runtime_passive_ds_solve_mode\":%u,"
        "\"runtime_passive_ds_rolling_max_hz\":%lu,"
        "\"runtime_anchor_survey_slot_ms\":%lu,"
        "\"runtime_anchor_survey_round_gap_ms\":%lu,"
        "\"runtime_calibration_method\":%u,"
        "\"runtime_calibration_three_ids\":[%u,%u,%u],"
        "\"runtime_uwb_enabled\":%s,"
        "\"runtime_bno085_accel_enabled\":%s,"
        "\"runtime_bno085_accel_interval_ms\":%lu,"
        "\"runtime_bno085_log_interval_ms\":%lu,"
        "\"runtime_gps_enabled\":%s,"
        "\"runtime_radio_channel\":%u,"
        "\"runtime_radio_phy_mode\":%u,"
        "\"runtime_wireless_telemetry_port\":%lu,"
        "\"reboot_recommended\":%s,"
        "\"rebooting\":%s,"
        "\"hot_switch_requested\":%s,"
        "\"hot_switch_started\":%s,"
        "\"hot_switch_in_progress\":%s"
        "}\n",
        changed ? "true" : "false", cleared ? "true" : "false",
        active_config->from_nvs ? "true" : "false",
        (unsigned)active_config->runtime_mode,
        app_runtime_config_runtime_mode_to_string(
            active_config->runtime_mode),
        (unsigned)active_config->tag_id,
        (unsigned)active_config->anchor_count,
        active_anchor_ids_json,
        (unsigned)active_config->flex_tdoa_responder_count,
        (unsigned)active_config->flex_tdoa_slot_count,
        active_flex_slots_json,
        active_flex_masks_json,
        (unsigned long)active_config->flex_tdoa_config_generation,
        (unsigned long)active_config->flex_tdoa_guard_us,
        (unsigned long)active_config->flex_tdoa_request_subslot_us,
        (unsigned long)active_config->flex_tdoa_request_process_us,
        (unsigned long)active_config->flex_tdoa_response_subslot_us,
        (unsigned long)active_config->flex_tdoa_response_process_us,
        active_config->flex_tdoa_geometry_fixed ? "true" : "false",
        (unsigned long)active_config->flex_tdoa_geometry_generation,
        (unsigned)active_config->anchor_survey_coordinator_id,
        (unsigned long)active_config->ranging_slot_ms,
        (unsigned long)active_config->ranging_round_gap_ms,
        (unsigned long)active_config->ranging_rx_slice_ms,
        (unsigned long)active_config->ranging_rx_timeout_ms,
        (unsigned long)active_config->ranging_resp_delay_ms,
        (unsigned long)active_config->ranging_final_delay_ms,
        (unsigned long)active_config->ranging_auto_rx_delay_uus,
        (unsigned)active_config->passive_ds_schedule,
        (unsigned long)active_config->passive_ds_slot_ms,
        (unsigned long)active_config->passive_ds_round_gap_ms,
        (unsigned long)active_config->passive_ds_rx_slice_ms,
        (unsigned long)active_config->passive_ds_rx_timeout_ms,
        (unsigned long)active_config->passive_ds_resp_delay_us,
        (unsigned long)active_config->passive_ds_final_delay_us,
        (unsigned long)active_config->passive_ds_auto_rx_delay_uus,
        (unsigned)active_config->passive_ds_pipeline_mode,
        (unsigned)active_config->passive_ds_solve_mode,
        (unsigned long)active_config->passive_ds_rolling_max_hz,
        (unsigned long)active_config->anchor_survey_slot_ms,
        (unsigned long)active_config->anchor_survey_round_gap_ms,
        (unsigned)active_config->calibration_method,
        (unsigned)active_config->calibration_three_ids[0],
        (unsigned)active_config->calibration_three_ids[1],
        (unsigned)active_config->calibration_three_ids[2],
        active_config->uwb_enabled ? "true" : "false",
        active_config->bno085_accel_enabled ? "true" : "false",
        (unsigned long)active_config->bno085_accel_interval_ms,
        (unsigned long)active_config->bno085_log_interval_ms,
        active_config->gps_enabled ? "true" : "false",
        (unsigned)runtime_radio_channel(active_config),
        (unsigned)runtime_radio_phy_mode(active_config),
        (unsigned long)active_config->wireless_telemetry_port,
        reboot_recommended ? "true" : "false",
        reboot_requested ? "true" : "false",
        hot_switch_requested ? "true" : "false",
        hot_switch_started ? "true" : "false",
        uwb_dw3000_runtime_switch_in_progress() ? "true" : "false");

    if (len < 0 || len >= OTA_SERVICE_RUNTIME_RESPONSE_SIZE) {
        free(response);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "response too long");
    }

    httpd_resp_set_type(req, "application/json");
    const esp_err_t response_err = httpd_resp_send(req, response, len);
    free(response);

    if (reboot_requested) {
        s_status = OTA_SERVICE_STATUS_REBOOTING;
        xTaskCreate(reboot_task, "runtime_reboot",
                    OTA_SERVICE_RESTART_TASK_STACK_WORDS, NULL,
                    OTA_SERVICE_RESTART_TASK_PRIORITY, NULL);
    }

    return response_err;
}

static esp_err_t recovery_config_post_handler(httpd_req_t *req)
{
    if (!ota_request_authorized(req)) {
        ESP_LOGW(TAG, "Rejected recovery config: missing or invalid token");
        return httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED,
                                   "Missing or invalid OTA token");
    }

    if (s_ota_in_progress) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "OTA already in progress");
    }

    const size_t query_len = httpd_req_get_url_query_len(req);
    if (query_len == 0 || query_len >= OTA_SERVICE_MAX_QUERY_LEN) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Use ?clear=1[&reboot=1]");
    }

    char query[OTA_SERVICE_MAX_QUERY_LEN] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid query string");
    }

    const bool clear_requested = ota_query_option_enabled(query, "clear");
    const bool reboot_requested = ota_query_option_enabled(query, "reboot");
    if (!clear_requested) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Use ?clear=1[&reboot=1]");
    }

    const esp_err_t err = boot_guard_clear_recovery();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Recovery clear failed: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Recovery clear failed");
    }

    char response[320];
    const int len = snprintf(
        response, sizeof(response),
        "{"
        "\"ok\":true,"
        "\"cleared\":true,"
        "\"boot_recovery_mode\":%s,"
        "\"boot_guard_failure_count\":%lu,"
        "\"rebooting\":%s"
        "}\n",
        boot_guard_recovery_mode() ? "true" : "false",
        (unsigned long)boot_guard_failure_count(),
        reboot_requested ? "true" : "false");
    if (len < 0 || len >= (int)sizeof(response)) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "response too long");
    }

    httpd_resp_set_type(req, "application/json");
    const esp_err_t response_err = httpd_resp_send(req, response, len);

    if (reboot_requested) {
        s_status = OTA_SERVICE_STATUS_REBOOTING;
        xTaskCreate(reboot_task, "recovery_reboot",
                    OTA_SERVICE_RESTART_TASK_STACK_WORDS, NULL,
                    OTA_SERVICE_RESTART_TASK_PRIORITY, NULL);
    }

    return response_err;
}

static void reboot_task(void *arg)
{
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(OTA_SERVICE_REBOOT_DELAY_MS));
    (void)boot_guard_mark_stable();
    ESP_LOGI(TAG, "Restarting device");
    esp_restart();
}

static esp_err_t send_ota_error(httpd_req_t *req, httpd_err_code_t code,
                                const char *message)
{
    ESP_LOGE(TAG, "%s", message);
    return httpd_resp_send_err(req, code, message);
}

static esp_err_t ota_post_handler(httpd_req_t *req)
{
    if (!ota_request_authorized(req)) {
        ESP_LOGW(TAG, "Rejected OTA upload: missing or invalid token");
        return httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED,
                                   "Missing or invalid OTA token");
    }

    if (s_ota_in_progress) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "OTA already in progress");
    }

    const esp_partition_t *update_partition =
        esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        s_status = OTA_SERVICE_STATUS_FAILED;
        return send_ota_error(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                              "No OTA update partition available");
    }

    if (req->content_len <= 0) {
        s_status = OTA_SERVICE_STATUS_FAILED;
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Missing firmware body");
    }

    if ((size_t)req->content_len > update_partition->size) {
        s_status = OTA_SERVICE_STATUS_FAILED;
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Firmware image is larger than OTA partition");
    }

    s_ota_in_progress = true;
    s_status = OTA_SERVICE_STATUS_UPDATING;

    ESP_LOGI(TAG, "OTA upload start: %d bytes -> partition %s @ 0x%lx",
             req->content_len, update_partition->label,
             (unsigned long)update_partition->address);

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES,
                                  &ota_handle);
    if (err != ESP_OK) {
        s_ota_in_progress = false;
        s_status = OTA_SERVICE_STATUS_FAILED;
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "esp_ota_begin failed");
    }

    uint8_t *ota_buffer = alloc_ota_buffer();
    if (ota_buffer == NULL) {
        esp_ota_abort(ota_handle);
        s_ota_in_progress = false;
        s_status = OTA_SERVICE_STATUS_FAILED;
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "OTA buffer allocation failed");
    }

    int remaining = req->content_len;
    size_t written = 0;
    size_t next_progress_log = 256 * 1024;

    while (remaining > 0) {
        const size_t to_read = remaining < OTA_SERVICE_CHUNK_SIZE
                                   ? (size_t)remaining
                                   : OTA_SERVICE_CHUNK_SIZE;
        const int received =
            httpd_req_recv(req, (char *)ota_buffer, to_read);

        if (received == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }

        if (received <= 0) {
            esp_ota_abort(ota_handle);
            heap_caps_free(ota_buffer);
            s_ota_in_progress = false;
            s_status = OTA_SERVICE_STATUS_FAILED;
            return send_ota_error(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                  "Failed to receive OTA data");
        }

        err = esp_ota_write(ota_handle, ota_buffer, (size_t)received);
        if (err != ESP_OK) {
            esp_ota_abort(ota_handle);
            heap_caps_free(ota_buffer);
            s_ota_in_progress = false;
            s_status = OTA_SERVICE_STATUS_FAILED;
            ESP_LOGE(TAG, "esp_ota_write failed after %u bytes: %s",
                     (unsigned)written, esp_err_to_name(err));
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                       "esp_ota_write failed");
        }

        remaining -= received;
        written += (size_t)received;

        if (written >= next_progress_log || remaining == 0) {
            ESP_LOGI(TAG, "OTA received %u/%d bytes", (unsigned)written,
                     req->content_len);
            next_progress_log += 256 * 1024;
        }
    }

    heap_caps_free(ota_buffer);

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        s_ota_in_progress = false;
        s_status = OTA_SERVICE_STATUS_FAILED;
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "esp_ota_end failed");
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        s_ota_in_progress = false;
        s_status = OTA_SERVICE_STATUS_FAILED;
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s",
                 esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "esp_ota_set_boot_partition failed");
    }

    s_status = OTA_SERVICE_STATUS_REBOOTING;
    ESP_LOGI(TAG, "OTA upload complete, next boot partition: %s",
             update_partition->label);

    httpd_resp_set_type(req, "application/json");
    const esp_err_t response_err =
        httpd_resp_sendstr(req, "{\"ok\":true,\"rebooting\":true}\n");

    xTaskCreate(reboot_task, "ota_reboot", OTA_SERVICE_RESTART_TASK_STACK_WORDS,
                NULL, OTA_SERVICE_RESTART_TASK_PRIORITY, NULL);

    return response_err;
}

static esp_err_t gnss_memory_reader(void *context, uint8_t *buffer,
                                    size_t capacity, size_t *received)
{
    gnss_memory_reader_context_t *reader = context;
    if (reader == NULL || buffer == NULL || received == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (reader->offset > reader->size) {
        return ESP_ERR_INVALID_SIZE;
    }
    const size_t remaining = reader->size - reader->offset;
    const size_t length = remaining < capacity ? remaining : capacity;
    if (length > 0) {
        memcpy(buffer, &reader->data[reader->offset], length);
        reader->offset += length;
    }
    *received = length;
    return ESP_OK;
}

static esp_err_t gnss_receive_body(httpd_req_t *req, uint8_t *buffer,
                                   size_t length)
{
    size_t offset = 0;
    while (offset < length) {
        const int received =
            httpd_req_recv(req, (char *)&buffer[offset], length - offset);
        if (received == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (received <= 0 || (size_t)received > length - offset) {
            return ESP_FAIL;
        }
        offset += (size_t)received;
    }
    return ESP_OK;
}

static esp_err_t gnss_firmware_post_handler(httpd_req_t *req)
{
    if (!ota_request_authorized(req)) {
        return httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED,
                                   "Missing or invalid OTA token");
    }
    if (s_ota_in_progress) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Update already in progress");
    }
    if (req->content_len != GNSS_PX1105R_PACKED_IMAGE_SIZE) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Unexpected PX1105R firmware image");
    }

    const size_t image_size = (size_t)req->content_len;
    s_ota_in_progress = true;
    uint8_t *image = heap_caps_malloc(
        image_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (image == NULL) {
        s_ota_in_progress = false;
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Unable to allocate GNSS PSRAM cache");
    }
    if (gnss_receive_body(req, image, image_size) != ESP_OK) {
        heap_caps_free(image);
        s_ota_in_progress = false;
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Incomplete GNSS firmware upload");
    }

    const uint8_t *lzma_header = image;
    uint64_t unpacked_size = 0;
    for (size_t index = 0; index < 8; ++index) {
        unpacked_size |= ((uint64_t)lzma_header[5 + index]) << (8U * index);
    }
    const uint32_t dictionary_size =
        ((uint32_t)lzma_header[1]) |
        ((uint32_t)lzma_header[2] << 8) |
        ((uint32_t)lzma_header[3] << 16) |
        ((uint32_t)lzma_header[4] << 24);
    if (lzma_header[0] != 0x5D || dictionary_size != (8U * 1024U) ||
        unpacked_size != GNSS_PX1105R_RAW_IMAGE_SIZE) {
        heap_caps_free(image);
        s_ota_in_progress = false;
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid Phoenix LZMA header");
    }

    uint8_t calculated_packed_sum = 0;
    for (size_t index = GNSS_FIRMWARE_LZMA_HEADER_SIZE; index < image_size;
         ++index) {
        calculated_packed_sum += image[index];
    }
    if (calculated_packed_sum != GNSS_PX1105R_PACKED_SUM8) {
        heap_caps_free(image);
        s_ota_in_progress = false;
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "GNSS packed checksum mismatch");
    }

    if (s_gnss_cached_firmware.data != NULL) {
        heap_caps_free(s_gnss_cached_firmware.data);
    }
    s_gnss_cached_firmware = (gnss_cached_firmware_t){
        .data = image,
        .size = image_size,
    };
    s_ota_in_progress = false;

    char response[320] = {0};
    const int response_length = snprintf(
        response, sizeof(response),
        "{\"ok\":true,\"cached\":true,\"bytes\":%u,"
        "\"packed_bytes\":%u,\"packed_sum8\":%u,"
        "\"psram_free\":%u}\n",
        (unsigned)image_size,
        (unsigned)(image_size - GNSS_FIRMWARE_LZMA_HEADER_SIZE),
        (unsigned)calculated_packed_sum,
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, response, response_length);
}

static esp_err_t gnss_loader_post_handler(httpd_req_t *req)
{
    if (!ota_request_authorized(req)) {
        return httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED,
                                   "Missing or invalid OTA token");
    }
    if (s_ota_in_progress) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Update already in progress");
    }
    if (req->content_len != GNSS_PX1105R_LOADER_SOURCE_SIZE) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid GNSS S-record loader size");
    }

    const size_t loader_size = (size_t)req->content_len;
    s_ota_in_progress = true;
    uint8_t *loader = heap_caps_malloc(
        loader_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (loader == NULL) {
        s_ota_in_progress = false;
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Unable to allocate GNSS loader cache");
    }
    if (gnss_receive_body(req, loader, loader_size) != ESP_OK) {
        heap_caps_free(loader);
        s_ota_in_progress = false;
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Incomplete GNSS loader upload");
    }
    if (loader[0] != 'S' || loader[1] != '0' || loader[loader_size - 1] != '\n') {
        heap_caps_free(loader);
        s_ota_in_progress = false;
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Invalid GNSS S-record loader");
    }

    if (s_gnss_cached_loader.data != NULL) {
        heap_caps_free(s_gnss_cached_loader.data);
    }
    s_gnss_cached_loader = (gnss_cached_loader_t){
        .data = loader,
        .size = loader_size,
    };
    s_ota_in_progress = false;

    char response[192] = {0};
    const int response_length = snprintf(
        response, sizeof(response),
        "{\"ok\":true,\"cached\":true,\"loader_bytes\":%u,"
        "\"psram_free\":%u}\n",
        (unsigned)loader_size,
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, response, response_length);
}

static esp_err_t gnss_firmware_run_post_handler(httpd_req_t *req)
{
    if (!ota_request_authorized(req)) {
        return httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED,
                                   "Missing or invalid OTA token");
    }
    if (s_ota_in_progress) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Update already in progress");
    }
    if (s_gnss_cached_firmware.data == NULL ||
        s_gnss_cached_firmware.size <= GNSS_FIRMWARE_LZMA_HEADER_SIZE) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "No GNSS firmware cached in PSRAM");
    }
    if (s_gnss_cached_loader.data == NULL || s_gnss_cached_loader.size == 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "No GNSS S-record loader cached in PSRAM");
    }

    gnss_memory_reader_context_t loader_reader = {
        .data = s_gnss_cached_loader.data,
        .size = s_gnss_cached_loader.size,
        .offset = 0,
    };
    gnss_memory_reader_context_t reader = {
        .data = &s_gnss_cached_firmware.data[GNSS_FIRMWARE_LZMA_HEADER_SIZE],
        .size = s_gnss_cached_firmware.size -
                GNSS_FIRMWARE_LZMA_HEADER_SIZE,
        .offset = 0,
    };
    gnss_firmware_update_result_t result = {0};
    s_ota_in_progress = true;
    const esp_err_t err = gnss_firmware_update(
        loader_reader.size, gnss_memory_reader, &loader_reader, reader.size,
        s_gnss_cached_firmware.data, gnss_memory_reader, &reader, &result);
    s_ota_in_progress = false;
    gnss_release_update_cache();

    char response[384] = {0};
    const int response_length = snprintf(
        response, sizeof(response),
        "{\"ok\":%s,\"error\":\"%s\",\"stage\":\"%s\","
        "\"feedback\":\"%s\",\"profile\":\"PX1105R-01.07.33\","
        "\"loader_bytes\":%u,\"bytes\":%u,\"sum8\":%u,"
        "\"psram_cache_released\":true}\n",
        err == ESP_OK ? "true" : "false", esp_err_to_name(err),
        result.stage, result.feedback, (unsigned)result.loader_bytes_written,
        (unsigned)result.bytes_written, (unsigned)result.calculated_sum);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_status(req, err == ESP_OK ? "200 OK"
                                              : "500 Internal Server Error");
    return httpd_resp_send(req, response, response_length);
}

static esp_err_t register_uri_handler_checked(const httpd_uri_t *uri)
{
    esp_err_t err = httpd_register_uri_handler(s_http_server, uri);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP handler register failed for %s: %s", uri->uri,
                 esp_err_to_name(err));
    }
    return err;
}

static esp_err_t start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.stack_size = 8192;
    config.core_id = OTA_SERVICE_HTTPD_TASK_CORE;
    config.max_uri_handlers = 11;

    esp_err_t err = httpd_start(&s_http_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return err;
    }

    const httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = status_get_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t ota_uri = {
        .uri = "/ota",
        .method = HTTP_POST,
        .handler = ota_post_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t gnss_firmware_uri = {
        .uri = "/gnss/firmware",
        .method = HTTP_POST,
        .handler = gnss_firmware_post_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t gnss_loader_uri = {
        .uri = "/gnss/loader",
        .method = HTTP_POST,
        .handler = gnss_loader_post_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t gnss_firmware_run_uri = {
        .uri = "/gnss/firmware/run",
        .method = HTTP_POST,
        .handler = gnss_firmware_run_post_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t antenna_delay_uri = {
        .uri = "/config/antenna-delay",
        .method = HTTP_POST,
        .handler = antenna_delay_post_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t runtime_config_uri = {
        .uri = "/config/runtime",
        .method = HTTP_POST,
        .handler = runtime_config_post_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t recovery_config_uri = {
        .uri = "/config/recovery",
        .method = HTTP_POST,
        .handler = recovery_config_post_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t charger_config_uri = {
        .uri = "/config/charger",
        .method = HTTP_POST,
        .handler = charger_config_post_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t max77958_config_uri = {
        .uri = "/config/max77958",
        .method = HTTP_POST,
        .handler = max77958_config_post_handler,
        .user_ctx = NULL,
    };

    err = register_uri_handler_checked(&root_uri);
    if (err == ESP_OK) {
        err = register_uri_handler_checked(&status_uri);
    }
    if (err == ESP_OK) {
        err = register_uri_handler_checked(&ota_uri);
    }
    if (err == ESP_OK) {
        err = register_uri_handler_checked(&gnss_firmware_uri);
    }
    if (err == ESP_OK) {
        err = register_uri_handler_checked(&gnss_loader_uri);
    }
    if (err == ESP_OK) {
        err = register_uri_handler_checked(&gnss_firmware_run_uri);
    }
    if (err == ESP_OK) {
        err = register_uri_handler_checked(&antenna_delay_uri);
    }
    if (err == ESP_OK) {
        err = register_uri_handler_checked(&runtime_config_uri);
    }
    if (err == ESP_OK) {
        err = register_uri_handler_checked(&recovery_config_uri);
    }
    if (err == ESP_OK) {
        err = register_uri_handler_checked(&charger_config_uri);
    }
    if (err == ESP_OK) {
        err = register_uri_handler_checked(&max77958_config_uri);
    }
    if (err != ESP_OK) {
        httpd_stop(s_http_server);
        s_http_server = NULL;
        return err;
    }

    s_running = true;
    s_status = OTA_SERVICE_STATUS_RUNNING;

    ESP_LOGI(TAG, "HTTP OTA server ready: http://%s/status",
             wifi_service_get_ip_address());
    return ESP_OK;
}

static void ota_service_task(void *arg)
{
    (void)arg;

    s_status = OTA_SERVICE_STATUS_WAITING_FOR_WIFI;
    while (!wifi_service_is_connected()) {
        vTaskDelay(pdMS_TO_TICKS(OTA_SERVICE_WIFI_WAIT_MS));
    }

    if (start_http_server() != ESP_OK) {
        s_status = OTA_SERVICE_STATUS_FAILED;
    }

    vTaskDelete(NULL);
}

esp_err_t ota_service_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    const BaseType_t created = xTaskCreatePinnedToCore(
        ota_service_task, "ota_service", OTA_SERVICE_TASK_STACK_WORDS, NULL,
        OTA_SERVICE_TASK_PRIORITY, NULL, 0);
    if (created != pdPASS) {
        s_status = OTA_SERVICE_STATUS_FAILED;
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    return ESP_OK;
}

bool ota_service_is_running(void)
{
    return s_running;
}

enum ota_service_status ota_service_get_status(void)
{
    return s_status;
}
