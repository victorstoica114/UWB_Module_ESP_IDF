#include "wireless_telemetry_service.h"

#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sys/select.h>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "wifi_service.h"

#include "app_config.h"
#include "app_identity.h"
#include "app_runtime_config.h"
#include "sdkconfig.h"

static const char *TAG = "wireless_telemetry";

#ifndef APP_WIRELESS_TELEMETRY_TARGET
#define APP_WIRELESS_TELEMETRY_TARGET ""
#endif

enum {
    WIRELESS_TELEMETRY_TASK_STACK_BYTES = 4096,
    WIRELESS_TELEMETRY_TASK_PRIORITY = 4,
    WIRELESS_TELEMETRY_QUEUE_LEN = 1024,
    WIRELESS_TELEMETRY_LINE_MAX = 96,
    WIRELESS_TELEMETRY_BATCH_MAX = 4096,
    WIRELESS_TELEMETRY_FRAME_HEADER_LEN = 12,
    WIRELESS_TELEMETRY_ACCEL_SAMPLE_LEN = 21,
    WIRELESS_TELEMETRY_FLEX_OBSERVATION_SAMPLE_LEN = 26,
    WIRELESS_TELEMETRY_PASSIVE_DS_OBSERVATION_V2_SAMPLE_LEN = 41,
    WIRELESS_TELEMETRY_FLEX_ANCHOR_RANGE_SAMPLE_LEN = 20,
    WIRELESS_TELEMETRY_FLEX_POSITION_SAMPLE_LEN = 32,
    WIRELESS_TELEMETRY_PASSIVE_DS_POSITION_V2_SAMPLE_LEN = 40,
    WIRELESS_TELEMETRY_PASSIVE_DS_POSITION_V3_SAMPLE_LEN = 49,
    WIRELESS_TELEMETRY_PASSIVE_DS_POSITION_V4_SAMPLE_LEN = 63,
    WIRELESS_TELEMETRY_FRAME_VERSION = 1,
    WIRELESS_TELEMETRY_STREAM_BNO085_ACCEL = 1,
    WIRELESS_TELEMETRY_STREAM_FLEX_TDOA_OBSERVATION = 2,
    WIRELESS_TELEMETRY_STREAM_FLEX_ANCHOR_RANGE = 3,
    WIRELESS_TELEMETRY_STREAM_FLEX_POSITION = 4,
    WIRELESS_TELEMETRY_STREAM_PASSIVE_DS_OBSERVATION = 5,
    WIRELESS_TELEMETRY_STREAM_PASSIVE_DS_ANCHOR_RANGE = 6,
    WIRELESS_TELEMETRY_STREAM_PASSIVE_DS_POSITION = 7,
    WIRELESS_TELEMETRY_STREAM_PASSIVE_DS_OBSERVATION_V2 = 8,
    WIRELESS_TELEMETRY_STREAM_NATIVE_DS_ANCHOR_RANGE = 9,
    WIRELESS_TELEMETRY_STREAM_PASSIVE_DS_POSITION_V2 = 10,
    WIRELESS_TELEMETRY_STREAM_PASSIVE_DS_POSITION_V3 = 11,
    WIRELESS_TELEMETRY_STREAM_PASSIVE_DS_POSITION_V4 = 12,
    WIRELESS_TELEMETRY_STREAM_NATIVE_DS_TAG_RANGE = 13,
    WIRELESS_TELEMETRY_RECONNECT_MS = 2000,
    WIRELESS_TELEMETRY_WIFI_WAIT_MS = 500,
    WIRELESS_TELEMETRY_QUEUE_WAIT_MS = 20,
    WIRELESS_TELEMETRY_SEND_TIMEOUT_MS = 1000,
};

#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
#define WIRELESS_TELEMETRY_TASK_CORE 1
#else
#define WIRELESS_TELEMETRY_TASK_CORE 0
#endif

typedef enum {
    WIRELESS_TELEMETRY_ITEM_TEXT = 0,
    WIRELESS_TELEMETRY_ITEM_BNO085_ACCEL,
    WIRELESS_TELEMETRY_ITEM_FLEX_TDOA_OBSERVATION,
    WIRELESS_TELEMETRY_ITEM_FLEX_ANCHOR_RANGE,
    WIRELESS_TELEMETRY_ITEM_FLEX_POSITION,
    WIRELESS_TELEMETRY_ITEM_PASSIVE_DS_OBSERVATION,
    WIRELESS_TELEMETRY_ITEM_PASSIVE_DS_ANCHOR_RANGE,
    WIRELESS_TELEMETRY_ITEM_PASSIVE_DS_POSITION,
    WIRELESS_TELEMETRY_ITEM_NATIVE_DS_ANCHOR_RANGE,
    WIRELESS_TELEMETRY_ITEM_NATIVE_DS_TAG_RANGE,
} wireless_telemetry_item_type_t;

typedef struct {
    int32_t x_milli_mps2;
    int32_t y_milli_mps2;
    int32_t z_milli_mps2;
    uint32_t report_count;
    uint8_t accuracy;
} wireless_telemetry_accel_t;

typedef struct {
    uint32_t slot_id;
    int32_t diff_mm;
    int32_t raw_diff_mm;
    int32_t anchor_distance_mm;
    int32_t cfo_correction_mm;
    int32_t clock_offset_ppb;
    uint32_t reply_delay_us;
    uint16_t range_age_slots;
    uint16_t sequence;
    uint8_t tag_id;
    uint8_t initiator_id;
    uint8_t responder_id;
    uint8_t responder_index;
    uint8_t range_source;
} wireless_telemetry_flex_observation_t;

typedef struct {
    uint32_t slot_id;
    int32_t distance_mm;
    int32_t raw_distance_mm;
    uint16_t sequence;
    uint8_t initiator_id;
    uint8_t responder_id;
} wireless_telemetry_flex_anchor_range_t;

typedef struct {
    uint32_t slot_id;
    uint32_t geometry_version;
    int32_t x_mm;
    int32_t y_mm;
    int32_t raw_x_mm;
    int32_t raw_y_mm;
    int32_t sigma_mm;
    int32_t rms_mm;
    uint16_t observation_count;
    uint32_t solver_update_count;
    uint32_t independent_frame_count;
    uint32_t position_rejected_count;
    uint16_t batch_span_ms;
    uint16_t batch_max_age_ms;
    uint16_t observation_mask;
    uint16_t rejection_reason_mask;
    uint16_t rejected_since_last;
    uint8_t tag_id;
    uint8_t anchor_count;
    uint8_t solution_flags;
} wireless_telemetry_flex_position_t;

typedef struct {
    wireless_telemetry_item_type_t type;
    uint32_t uptime_ms;
    union {
        char line[WIRELESS_TELEMETRY_LINE_MAX];
        wireless_telemetry_accel_t accel;
        wireless_telemetry_flex_observation_t flex_observation;
        wireless_telemetry_flex_anchor_range_t flex_anchor_range;
        wireless_telemetry_flex_position_t flex_position;
    } data;
} wireless_telemetry_item_t;

typedef struct {
    uint32_t binary_frames;
    uint32_t binary_samples;
    uint32_t text_frames;
} wireless_telemetry_batch_stats_t;

static wireless_telemetry_item_t *s_ring_items;
static size_t s_ring_capacity;
static size_t s_ring_head;
static size_t s_ring_count;
static portMUX_TYPE s_ring_lock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t s_ring_items_ready;
static uint8_t *s_batch_buffer;
static TaskHandle_t s_task_handle;
static bool s_started;
static volatile enum wireless_telemetry_status s_status =
    WIRELESS_TELEMETRY_STATUS_DISABLED;
static volatile bool s_connected;
static volatile uint32_t s_dropped_count;
static volatile uint32_t s_drop_full_count;
static volatile uint32_t s_drop_mutex_count;
static volatile uint32_t s_drop_format_count;
static volatile uint32_t s_ring_high_water;
static volatile uint32_t s_binary_frame_count;
static volatile uint32_t s_binary_sample_count;
static volatile uint32_t s_text_frame_count;
static volatile uint32_t s_connect_count;
static volatile uint32_t s_send_failure_count;
static volatile uint32_t s_send_timeout_count;
static volatile uint32_t s_socket_close_count;
static volatile uint32_t s_last_send_ms;
static volatile uint32_t s_max_send_ms;
static volatile int s_last_error;

static bool wireless_telemetry_target_configured(void)
{
    return APP_WIRELESS_TELEMETRY_ENABLED &&
           strlen(APP_WIRELESS_TELEMETRY_TARGET) > 0;
}

static uint16_t wireless_telemetry_active_port(void)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    if (config != NULL && config->wireless_telemetry_port > 0U &&
        config->wireless_telemetry_port <= 65535U) {
        return (uint16_t)config->wireless_telemetry_port;
    }
    return APP_WIRELESS_TELEMETRY_PORT;
}

static bool wireless_telemetry_create_ring(size_t capacity, uint32_t caps)
{
    s_ring_items = (wireless_telemetry_item_t *)heap_caps_calloc(
        capacity, sizeof(wireless_telemetry_item_t), caps | MALLOC_CAP_8BIT);
    s_ring_items_ready = xSemaphoreCreateCounting(capacity, 0);

    if (s_ring_items == NULL || s_ring_items_ready == NULL) {
        if (s_ring_items_ready != NULL) {
            vSemaphoreDelete(s_ring_items_ready);
            s_ring_items_ready = NULL;
        }
        if (s_ring_items != NULL) {
            heap_caps_free(s_ring_items);
            s_ring_items = NULL;
        }
        return false;
    }

    s_ring_capacity = capacity;
    s_ring_head = 0;
    s_ring_count = 0;
    return true;
}

static void wireless_telemetry_delete_ring(void)
{
    if (s_ring_items_ready != NULL) {
        vSemaphoreDelete(s_ring_items_ready);
        s_ring_items_ready = NULL;
    }
    if (s_ring_items != NULL) {
        heap_caps_free(s_ring_items);
        s_ring_items = NULL;
    }
    s_ring_capacity = 0;
    s_ring_head = 0;
    s_ring_count = 0;
}

static void wireless_telemetry_delete_batch_buffer(void)
{
    if (s_batch_buffer != NULL) {
        heap_caps_free(s_batch_buffer);
        s_batch_buffer = NULL;
    }
}

static bool wireless_telemetry_enqueue(const wireless_telemetry_item_t *item)
{
    if (s_ring_items == NULL || s_ring_items_ready == NULL || item == NULL ||
        !wireless_telemetry_target_configured()) {
        return false;
    }

    taskENTER_CRITICAL(&s_ring_lock);
    const bool was_full = s_ring_count >= s_ring_capacity;
    if (was_full) {
        s_ring_head = (s_ring_head + 1U) % s_ring_capacity;
        s_ring_count--;
        s_dropped_count++;
        s_drop_full_count++;
    }

    const size_t tail = (s_ring_head + s_ring_count) % s_ring_capacity;
    s_ring_items[tail] = *item;
    s_ring_count++;
    if (s_ring_count > s_ring_high_water) {
        s_ring_high_water = (uint32_t)s_ring_count;
    }
    taskEXIT_CRITICAL(&s_ring_lock);

    if (!was_full) {
        (void)xSemaphoreGive(s_ring_items_ready);
    }
    return true;
}

static bool wireless_telemetry_dequeue(wireless_telemetry_item_t *item,
                                       TickType_t wait_ticks)
{
    if (s_ring_items == NULL || s_ring_items_ready == NULL || item == NULL) {
        return false;
    }

    if (xSemaphoreTake(s_ring_items_ready, wait_ticks) != pdTRUE) {
        return false;
    }

    taskENTER_CRITICAL(&s_ring_lock);
    if (s_ring_count == 0) {
        taskEXIT_CRITICAL(&s_ring_lock);
        return false;
    }

    *item = s_ring_items[s_ring_head];
    s_ring_head = (s_ring_head + 1U) % s_ring_capacity;
    s_ring_count--;
    taskEXIT_CRITICAL(&s_ring_lock);
    return true;
}

static int wireless_telemetry_connect_socket(void)
{
    char port[8];
    const uint16_t active_port = wireless_telemetry_active_port();
    snprintf(port, sizeof(port), "%u", active_port);

    struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *result = NULL;

    const int gai_err =
        getaddrinfo(APP_WIRELESS_TELEMETRY_TARGET, port, &hints, &result);
    if (gai_err != 0 || result == NULL) {
        s_last_error = -gai_err;
        return -1;
    }

    int sock = -1;
    for (struct addrinfo *it = result; it != NULL; it = it->ai_next) {
        sock = socket(it->ai_family, it->ai_socktype, it->ai_protocol);
        if (sock < 0) {
            s_last_error = errno;
            continue;
        }

        const int yes = 1;
        (void)setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
        const int flags = fcntl(sock, F_GETFL, 0);
        if (flags >= 0) {
            (void)fcntl(sock, F_SETFL, flags | O_NONBLOCK);
        }

        struct timeval timeout = {
            .tv_sec = WIRELESS_TELEMETRY_SEND_TIMEOUT_MS / 1000,
            .tv_usec = (WIRELESS_TELEMETRY_SEND_TIMEOUT_MS % 1000) * 1000,
        };
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

        const int connect_result = connect(sock, it->ai_addr, it->ai_addrlen);
        if (connect_result == 0) {
            s_last_error = 0;
            break;
        }

        if (errno == EINPROGRESS) {
            fd_set write_fds;
            FD_ZERO(&write_fds);
            FD_SET(sock, &write_fds);

            struct timeval connect_timeout = timeout;
            const int selected =
                select(sock + 1, NULL, &write_fds, NULL, &connect_timeout);
            if (selected > 0 && FD_ISSET(sock, &write_fds)) {
                int socket_error = 0;
                socklen_t socket_error_len = sizeof(socket_error);
                if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &socket_error,
                               &socket_error_len) == 0 &&
                    socket_error == 0) {
                    s_last_error = 0;
                    break;
                }

                s_last_error = socket_error != 0 ? socket_error : errno;
            } else if (selected == 0) {
                s_last_error = ETIMEDOUT;
            } else {
                s_last_error = errno;
            }

            closesocket(sock);
            sock = -1;
            continue;
        }

        s_last_error = errno;
        closesocket(sock);
        sock = -1;
    }

    freeaddrinfo(result);
    return sock;
}

static bool wireless_telemetry_send_all(int sock, const uint8_t *payload,
                                        size_t len)
{
    size_t sent_total = 0;
    const TickType_t start = xTaskGetTickCount();
    while (sent_total < len) {
        const int sent = send(sock, payload + sent_total, len - sent_total,
                              MSG_DONTWAIT);
        if (sent > 0) {
            sent_total += (size_t)sent;
            continue;
        }

        if (sent < 0 && (errno == EWOULDBLOCK || errno == EAGAIN)) {
            if ((xTaskGetTickCount() - start) >=
                pdMS_TO_TICKS(WIRELESS_TELEMETRY_SEND_TIMEOUT_MS)) {
                s_last_error = ETIMEDOUT;
                s_last_send_ms = (uint32_t)(
                    (xTaskGetTickCount() - start) * portTICK_PERIOD_MS);
                if (s_last_send_ms > s_max_send_ms) {
                    s_max_send_ms = s_last_send_ms;
                }
                return false;
            }
            vTaskDelay(1);
            continue;
        }

        s_last_error = errno;
        s_last_send_ms = (uint32_t)(
            (xTaskGetTickCount() - start) * portTICK_PERIOD_MS);
        if (s_last_send_ms > s_max_send_ms) {
            s_max_send_ms = s_last_send_ms;
        }
        return false;
    }

    s_last_send_ms = (uint32_t)(
        (xTaskGetTickCount() - start) * portTICK_PERIOD_MS);
    if (s_last_send_ms > s_max_send_ms) {
        s_max_send_ms = s_last_send_ms;
    }
    return true;
}

static bool wireless_telemetry_socket_alive(int sock)
{
    char byte;
    const int received = recv(sock, &byte, sizeof(byte), MSG_DONTWAIT);

    if (received > 0) {
        return true;
    }

    if (received == 0) {
        s_last_error = 0;
        return false;
    }

    if (errno == EWOULDBLOCK || errno == EAGAIN) {
        return true;
    }

    s_last_error = errno;
    return false;
}

static void wireless_telemetry_write_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xffU);
    dst[1] = (uint8_t)((value >> 8) & 0xffU);
}

static void wireless_telemetry_write_u32_le(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xffU);
    dst[1] = (uint8_t)((value >> 8) & 0xffU);
    dst[2] = (uint8_t)((value >> 16) & 0xffU);
    dst[3] = (uint8_t)((value >> 24) & 0xffU);
}

static void wireless_telemetry_write_i32_le(uint8_t *dst, int32_t value)
{
    wireless_telemetry_write_u32_le(dst, (uint32_t)value);
}

static void wireless_telemetry_close_socket(int *sock)
{
    if (*sock >= 0) {
        closesocket(*sock);
        *sock = -1;
    }
    s_connected = false;
}

static bool wireless_telemetry_format_text_item(
    const wireless_telemetry_item_t *item, char *line, size_t line_size)
{
    if (item == NULL || line == NULL || line_size == 0) {
        return false;
    }

    int written = 0;
    switch (item->type) {
    case WIRELESS_TELEMETRY_ITEM_TEXT:
        written = snprintf(line, line_size, "%s", item->data.line);
        break;
    default:
        return false;
    }

    return written > 0 && written < (int)line_size;
}

static bool wireless_telemetry_binary_item_info(
    wireless_telemetry_item_type_t type, uint8_t *stream_type,
    uint8_t *sample_len)
{
    if (stream_type == NULL || sample_len == NULL) {
        return false;
    }

    switch (type) {
    case WIRELESS_TELEMETRY_ITEM_BNO085_ACCEL:
        *stream_type = WIRELESS_TELEMETRY_STREAM_BNO085_ACCEL;
        *sample_len = WIRELESS_TELEMETRY_ACCEL_SAMPLE_LEN;
        return true;
    case WIRELESS_TELEMETRY_ITEM_FLEX_TDOA_OBSERVATION:
        *stream_type = WIRELESS_TELEMETRY_STREAM_FLEX_TDOA_OBSERVATION;
        *sample_len = WIRELESS_TELEMETRY_FLEX_OBSERVATION_SAMPLE_LEN;
        return true;
    case WIRELESS_TELEMETRY_ITEM_FLEX_ANCHOR_RANGE:
        *stream_type = WIRELESS_TELEMETRY_STREAM_FLEX_ANCHOR_RANGE;
        *sample_len = WIRELESS_TELEMETRY_FLEX_ANCHOR_RANGE_SAMPLE_LEN;
        return true;
    case WIRELESS_TELEMETRY_ITEM_FLEX_POSITION:
        *stream_type = WIRELESS_TELEMETRY_STREAM_FLEX_POSITION;
        *sample_len = WIRELESS_TELEMETRY_FLEX_POSITION_SAMPLE_LEN;
        return true;
    case WIRELESS_TELEMETRY_ITEM_PASSIVE_DS_OBSERVATION:
        *stream_type =
            WIRELESS_TELEMETRY_STREAM_PASSIVE_DS_OBSERVATION_V2;
        *sample_len =
            WIRELESS_TELEMETRY_PASSIVE_DS_OBSERVATION_V2_SAMPLE_LEN;
        return true;
    case WIRELESS_TELEMETRY_ITEM_PASSIVE_DS_ANCHOR_RANGE:
        *stream_type = WIRELESS_TELEMETRY_STREAM_PASSIVE_DS_ANCHOR_RANGE;
        *sample_len = WIRELESS_TELEMETRY_FLEX_ANCHOR_RANGE_SAMPLE_LEN;
        return true;
    case WIRELESS_TELEMETRY_ITEM_PASSIVE_DS_POSITION:
        *stream_type =
            WIRELESS_TELEMETRY_STREAM_PASSIVE_DS_POSITION_V4;
        *sample_len =
            WIRELESS_TELEMETRY_PASSIVE_DS_POSITION_V4_SAMPLE_LEN;
        return true;
    case WIRELESS_TELEMETRY_ITEM_NATIVE_DS_ANCHOR_RANGE:
        *stream_type = WIRELESS_TELEMETRY_STREAM_NATIVE_DS_ANCHOR_RANGE;
        *sample_len = WIRELESS_TELEMETRY_FLEX_ANCHOR_RANGE_SAMPLE_LEN;
        return true;
    case WIRELESS_TELEMETRY_ITEM_NATIVE_DS_TAG_RANGE:
        *stream_type = WIRELESS_TELEMETRY_STREAM_NATIVE_DS_TAG_RANGE;
        *sample_len = WIRELESS_TELEMETRY_FLEX_ANCHOR_RANGE_SAMPLE_LEN;
        return true;
    default:
        return false;
    }
}

static bool wireless_telemetry_start_binary_frame(
    uint8_t *batch, size_t batch_size, size_t *used, size_t *frame_pos,
    uint16_t *frame_count, uint8_t stream_type, uint8_t sample_len)
{
    if (batch == NULL || used == NULL || frame_pos == NULL ||
        frame_count == NULL ||
        *used + WIRELESS_TELEMETRY_FRAME_HEADER_LEN + sample_len >
            batch_size) {
        return false;
    }

    *frame_pos = *used;
    batch[*used + 0U] = 'U';
    batch[*used + 1U] = 'W';
    batch[*used + 2U] = 'T';
    batch[*used + 3U] = '1';
    batch[*used + 4U] = WIRELESS_TELEMETRY_FRAME_VERSION;
    batch[*used + 5U] = stream_type;
    batch[*used + 6U] = app_identity_get_module_id();
    batch[*used + 7U] = sample_len;
    wireless_telemetry_write_u16_le(&batch[*used + 8U], 0);
    wireless_telemetry_write_u16_le(&batch[*used + 10U], 0);

    *used += WIRELESS_TELEMETRY_FRAME_HEADER_LEN;
    *frame_count = 0;
    return true;
}

static void wireless_telemetry_finish_binary_frame(
    uint8_t *batch, size_t frame_pos, uint16_t frame_count,
    uint8_t sample_len,
    wireless_telemetry_batch_stats_t *stats)
{
    if (batch == NULL || frame_count == 0) {
        return;
    }

    wireless_telemetry_write_u16_le(&batch[frame_pos + 8U], frame_count);
    wireless_telemetry_write_u16_le(
        &batch[frame_pos + 10U],
        (uint16_t)(frame_count * sample_len));

    if (stats != NULL) {
        stats->binary_frames++;
        stats->binary_samples += frame_count;
    }
}

static bool wireless_telemetry_append_binary_sample(
    uint8_t *batch, size_t batch_size, size_t *used,
    const wireless_telemetry_item_t *item, uint16_t *frame_count,
    uint8_t sample_len)
{
    if (batch == NULL || used == NULL || item == NULL ||
        frame_count == NULL || *used + sample_len > batch_size ||
        *frame_count == UINT16_MAX) {
        return false;
    }

    uint8_t *sample = &batch[*used];
    wireless_telemetry_write_u32_le(&sample[0], item->uptime_ms);
    switch (item->type) {
    case WIRELESS_TELEMETRY_ITEM_BNO085_ACCEL:
        wireless_telemetry_write_u32_le(&sample[4],
                                        item->data.accel.report_count);
        wireless_telemetry_write_i32_le(&sample[8],
                                        item->data.accel.x_milli_mps2);
        wireless_telemetry_write_i32_le(&sample[12],
                                        item->data.accel.y_milli_mps2);
        wireless_telemetry_write_i32_le(&sample[16],
                                        item->data.accel.z_milli_mps2);
        sample[20] = item->data.accel.accuracy;
        break;
    case WIRELESS_TELEMETRY_ITEM_FLEX_TDOA_OBSERVATION:
        wireless_telemetry_write_u32_le(
            &sample[4], item->data.flex_observation.slot_id);
        wireless_telemetry_write_i32_le(
            &sample[8], item->data.flex_observation.diff_mm);
        wireless_telemetry_write_i32_le(
            &sample[12], item->data.flex_observation.raw_diff_mm);
        wireless_telemetry_write_i32_le(
            &sample[16], item->data.flex_observation.anchor_distance_mm);
        wireless_telemetry_write_u16_le(
            &sample[20], item->data.flex_observation.sequence);
        sample[22] = item->data.flex_observation.tag_id;
        sample[23] = item->data.flex_observation.initiator_id;
        sample[24] = item->data.flex_observation.responder_id;
        sample[25] = item->data.flex_observation.responder_index;
        break;
    case WIRELESS_TELEMETRY_ITEM_PASSIVE_DS_OBSERVATION:
        wireless_telemetry_write_u32_le(
            &sample[4], item->data.flex_observation.slot_id);
        wireless_telemetry_write_i32_le(
            &sample[8], item->data.flex_observation.diff_mm);
        wireless_telemetry_write_i32_le(
            &sample[12], item->data.flex_observation.raw_diff_mm);
        wireless_telemetry_write_i32_le(
            &sample[16],
            item->data.flex_observation.anchor_distance_mm);
        wireless_telemetry_write_i32_le(
            &sample[20],
            item->data.flex_observation.cfo_correction_mm);
        wireless_telemetry_write_i32_le(
            &sample[24],
            item->data.flex_observation.clock_offset_ppb);
        wireless_telemetry_write_u32_le(
            &sample[28], item->data.flex_observation.reply_delay_us);
        wireless_telemetry_write_u16_le(
            &sample[32], item->data.flex_observation.sequence);
        sample[34] = item->data.flex_observation.tag_id;
        sample[35] = item->data.flex_observation.initiator_id;
        sample[36] = item->data.flex_observation.responder_id;
        sample[37] = item->data.flex_observation.responder_index;
        sample[38] = item->data.flex_observation.range_source;
        wireless_telemetry_write_u16_le(
            &sample[39], item->data.flex_observation.range_age_slots);
        break;
    case WIRELESS_TELEMETRY_ITEM_FLEX_ANCHOR_RANGE:
    case WIRELESS_TELEMETRY_ITEM_PASSIVE_DS_ANCHOR_RANGE:
    case WIRELESS_TELEMETRY_ITEM_NATIVE_DS_ANCHOR_RANGE:
    case WIRELESS_TELEMETRY_ITEM_NATIVE_DS_TAG_RANGE:
        wireless_telemetry_write_i32_le(
            &sample[8], item->data.flex_anchor_range.distance_mm);
        wireless_telemetry_write_i32_le(
            &sample[12], item->data.flex_anchor_range.raw_distance_mm);
        wireless_telemetry_write_u32_le(
            &sample[4], item->data.flex_anchor_range.slot_id);
        wireless_telemetry_write_u16_le(
            &sample[16], item->data.flex_anchor_range.sequence);
        sample[18] = item->data.flex_anchor_range.initiator_id;
        sample[19] = item->data.flex_anchor_range.responder_id;
        break;
    case WIRELESS_TELEMETRY_ITEM_FLEX_POSITION:
        wireless_telemetry_write_u32_le(
            &sample[4], item->data.flex_position.slot_id);
        wireless_telemetry_write_i32_le(
            &sample[8], item->data.flex_position.x_mm);
        wireless_telemetry_write_i32_le(
            &sample[12], item->data.flex_position.y_mm);
        wireless_telemetry_write_i32_le(
            &sample[16], item->data.flex_position.sigma_mm);
        wireless_telemetry_write_i32_le(
            &sample[20], item->data.flex_position.rms_mm);
        wireless_telemetry_write_u32_le(
            &sample[24], item->data.flex_position.geometry_version);
        wireless_telemetry_write_u16_le(
            &sample[28], item->data.flex_position.observation_count);
        sample[30] = item->data.flex_position.tag_id;
        sample[31] = item->data.flex_position.anchor_count;
        break;
    case WIRELESS_TELEMETRY_ITEM_PASSIVE_DS_POSITION:
        wireless_telemetry_write_u32_le(
            &sample[4], item->data.flex_position.slot_id);
        wireless_telemetry_write_i32_le(
            &sample[8], item->data.flex_position.x_mm);
        wireless_telemetry_write_i32_le(
            &sample[12], item->data.flex_position.y_mm);
        wireless_telemetry_write_i32_le(
            &sample[16], item->data.flex_position.raw_x_mm);
        wireless_telemetry_write_i32_le(
            &sample[20], item->data.flex_position.raw_y_mm);
        wireless_telemetry_write_i32_le(
            &sample[24], item->data.flex_position.sigma_mm);
        wireless_telemetry_write_i32_le(
            &sample[28], item->data.flex_position.rms_mm);
        wireless_telemetry_write_u32_le(
            &sample[32], item->data.flex_position.geometry_version);
        wireless_telemetry_write_u16_le(
            &sample[36], item->data.flex_position.observation_count);
        sample[38] = item->data.flex_position.tag_id;
        sample[39] = item->data.flex_position.anchor_count;
        wireless_telemetry_write_u32_le(
            &sample[40], item->data.flex_position.solver_update_count);
        wireless_telemetry_write_u32_le(
            &sample[44],
            item->data.flex_position.independent_frame_count);
        sample[48] = item->data.flex_position.solution_flags;
        wireless_telemetry_write_u16_le(
            &sample[49], item->data.flex_position.batch_span_ms);
        wireless_telemetry_write_u16_le(
            &sample[51], item->data.flex_position.batch_max_age_ms);
        wireless_telemetry_write_u16_le(
            &sample[53], item->data.flex_position.observation_mask);
        wireless_telemetry_write_u16_le(
            &sample[55], item->data.flex_position.rejection_reason_mask);
        wireless_telemetry_write_u16_le(
            &sample[57], item->data.flex_position.rejected_since_last);
        wireless_telemetry_write_u32_le(
            &sample[59], item->data.flex_position.position_rejected_count);
        break;
    default:
        return false;
    }

    *used += sample_len;
    (*frame_count)++;
    return true;
}

static bool wireless_telemetry_append_text_item(
    uint8_t *batch, size_t batch_size, size_t *used,
    const wireless_telemetry_item_t *item,
    wireless_telemetry_batch_stats_t *stats)
{
    char line[WIRELESS_TELEMETRY_LINE_MAX] = {0};
    if (!wireless_telemetry_format_text_item(item, line, sizeof(line))) {
        s_dropped_count++;
        s_drop_format_count++;
        return false;
    }

    const int written = snprintf((char *)batch + *used, batch_size - *used,
                                 "%s\n", line);
    if (written <= 0 || written >= (int)(batch_size - *used)) {
        s_dropped_count++;
        s_drop_format_count++;
        return false;
    }

    *used += (size_t)written;
    if (stats != NULL) {
        stats->text_frames++;
    }
    return true;
}

static bool wireless_telemetry_take_batch(
    uint8_t *batch, size_t batch_size, size_t *batch_len,
    wireless_telemetry_batch_stats_t *stats)
{
    wireless_telemetry_item_t item = {0};
    if (!wireless_telemetry_dequeue(
            &item, pdMS_TO_TICKS(WIRELESS_TELEMETRY_QUEUE_WAIT_MS))) {
        return false;
    }

    if (stats != NULL) {
        memset(stats, 0, sizeof(*stats));
    }

    size_t used = 0;
    bool binary_frame_open = false;
    size_t binary_frame_pos = 0;
    uint16_t binary_frame_count = 0;
    uint8_t binary_stream_type = 0;
    uint8_t binary_sample_len = 0;

    while (true) {
        uint8_t item_stream_type = 0;
        uint8_t item_sample_len = 0;
        const bool binary_item = wireless_telemetry_binary_item_info(
            item.type, &item_stream_type, &item_sample_len);
        if (binary_item) {
            if (binary_frame_open &&
                item_stream_type != binary_stream_type) {
                wireless_telemetry_finish_binary_frame(
                    batch, binary_frame_pos, binary_frame_count,
                    binary_sample_len, stats);
                binary_frame_open = false;
            }
            if (!binary_frame_open &&
                !wireless_telemetry_start_binary_frame(
                    batch, batch_size, &used, &binary_frame_pos,
                    &binary_frame_count, item_stream_type,
                    item_sample_len)) {
                if (used == 0) {
                    s_dropped_count++;
                    s_drop_format_count++;
                }
                break;
            }

            binary_stream_type = item_stream_type;
            binary_sample_len = item_sample_len;
            if (!wireless_telemetry_append_binary_sample(
                    batch, batch_size, &used, &item, &binary_frame_count,
                    binary_sample_len)) {
                wireless_telemetry_finish_binary_frame(
                    batch, binary_frame_pos, binary_frame_count,
                    binary_sample_len, stats);
                binary_frame_open = false;
                if (used == 0) {
                    s_dropped_count++;
                    s_drop_format_count++;
                }
                break;
            }
            binary_frame_open = true;
        } else {
            if (binary_frame_open) {
                wireless_telemetry_finish_binary_frame(
                    batch, binary_frame_pos, binary_frame_count,
                    binary_sample_len, stats);
                binary_frame_open = false;
            }
            if (!wireless_telemetry_append_text_item(batch, batch_size, &used,
                                                     &item, stats)) {
                break;
            }
        }

        if (batch_size - used <
            WIRELESS_TELEMETRY_LINE_MAX + 2U +
                WIRELESS_TELEMETRY_FRAME_HEADER_LEN) {
            break;
        }

        if (!wireless_telemetry_dequeue(&item, 0)) {
            break;
        }
    }

    if (binary_frame_open) {
        wireless_telemetry_finish_binary_frame(
            batch, binary_frame_pos, binary_frame_count, binary_sample_len,
            stats);
    }

    *batch_len = used;
    return used > 0;
}

static void wireless_telemetry_task(void *arg)
{
    (void)arg;

    int sock = -1;
    TickType_t last_connect_attempt = 0;
    uint16_t connected_port = 0;

    while (true) {
        const uint16_t active_port = wireless_telemetry_active_port();

        if (!wireless_telemetry_target_configured()) {
            s_status = WIRELESS_TELEMETRY_STATUS_DISABLED;
            wireless_telemetry_close_socket(&sock);
            connected_port = 0;
            vTaskDelay(pdMS_TO_TICKS(WIRELESS_TELEMETRY_WIFI_WAIT_MS));
            continue;
        }

        if (!wifi_service_is_connected()) {
            s_status = WIRELESS_TELEMETRY_STATUS_WAITING_FOR_WIFI;
            wireless_telemetry_close_socket(&sock);
            connected_port = 0;
            vTaskDelay(pdMS_TO_TICKS(WIRELESS_TELEMETRY_WIFI_WAIT_MS));
            continue;
        }

        if (sock >= 0 && connected_port != active_port) {
            ESP_LOGI(TAG, "telemetry port changed %u -> %u, reconnecting",
                     (unsigned)connected_port, (unsigned)active_port);
            wireless_telemetry_close_socket(&sock);
            connected_port = 0;
            last_connect_attempt = 0;
        }

        if (sock < 0) {
            const TickType_t now = xTaskGetTickCount();
            if (now - last_connect_attempt <
                pdMS_TO_TICKS(WIRELESS_TELEMETRY_RECONNECT_MS)) {
                vTaskDelay(pdMS_TO_TICKS(WIRELESS_TELEMETRY_WIFI_WAIT_MS));
                continue;
            }

            last_connect_attempt = now;
            s_status = WIRELESS_TELEMETRY_STATUS_CONNECTING;
            sock = wireless_telemetry_connect_socket();
            if (sock < 0) {
                s_connected = false;
                s_status = WIRELESS_TELEMETRY_STATUS_FAILED;
                ESP_LOGW(TAG, "connect failed target=%s port=%u err=%d",
                         APP_WIRELESS_TELEMETRY_TARGET,
                         (unsigned)active_port, s_last_error);
                continue;
            }

            s_connected = true;
            connected_port = active_port;
            s_status = WIRELESS_TELEMETRY_STATUS_CONNECTED;
            s_connect_count++;
            ESP_LOGI(TAG, "connected target=%s port=%u count=%lu queue=%lu",
                     APP_WIRELESS_TELEMETRY_TARGET,
                     (unsigned)active_port,
                     (unsigned long)s_connect_count,
                     (unsigned long)wireless_telemetry_service_get_queue_depth());
        }

        size_t batch_len = 0;
        wireless_telemetry_batch_stats_t batch_stats = {0};
        if (!wireless_telemetry_take_batch(s_batch_buffer,
                                           WIRELESS_TELEMETRY_BATCH_MAX,
                                           &batch_len, &batch_stats)) {
            if (!wireless_telemetry_socket_alive(sock)) {
                s_socket_close_count++;
                ESP_LOGW(TAG,
                         "socket closed while idle err=%d closes=%lu queue=%lu",
                         s_last_error, (unsigned long)s_socket_close_count,
                         (unsigned long)wireless_telemetry_service_get_queue_depth());
                wireless_telemetry_close_socket(&sock);
                s_status = WIRELESS_TELEMETRY_STATUS_FAILED;
            }
            continue;
        }

        if (!wireless_telemetry_send_all(sock, s_batch_buffer, batch_len)) {
            s_send_failure_count++;
            if (s_last_error == ETIMEDOUT) {
                s_send_timeout_count++;
            }
            ESP_LOGW(TAG,
                     "send failed err=%d bytes=%u send_ms=%lu failures=%lu timeouts=%lu queue=%lu",
                     s_last_error, (unsigned)batch_len,
                     (unsigned long)s_last_send_ms,
                     (unsigned long)s_send_failure_count,
                     (unsigned long)s_send_timeout_count,
                     (unsigned long)wireless_telemetry_service_get_queue_depth());
            wireless_telemetry_close_socket(&sock);
            connected_port = 0;
            s_status = WIRELESS_TELEMETRY_STATUS_FAILED;
        } else {
            s_binary_frame_count += batch_stats.binary_frames;
            s_binary_sample_count += batch_stats.binary_samples;
            s_text_frame_count += batch_stats.text_frames;
        }
    }
}

esp_err_t wireless_telemetry_service_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    if (!wireless_telemetry_target_configured()) {
        s_status = WIRELESS_TELEMETRY_STATUS_DISABLED;
        s_started = true;
        return ESP_OK;
    }

    if (wireless_telemetry_create_ring(WIRELESS_TELEMETRY_QUEUE_LEN,
                                       MALLOC_CAP_SPIRAM)) {
        ESP_LOGI(TAG, "ring storage in PSRAM len=%u bytes=%u",
                 (unsigned)WIRELESS_TELEMETRY_QUEUE_LEN,
                 (unsigned)(WIRELESS_TELEMETRY_QUEUE_LEN *
                            sizeof(wireless_telemetry_item_t)));
    }
    if (s_ring_items == NULL) {
        wireless_telemetry_delete_ring();
        s_last_error = ENOMEM;
        s_status = WIRELESS_TELEMETRY_STATUS_FAILED;
        ESP_LOGE(TAG, "PSRAM ring allocation failed len=%u bytes=%u",
                 (unsigned)WIRELESS_TELEMETRY_QUEUE_LEN,
                 (unsigned)(WIRELESS_TELEMETRY_QUEUE_LEN *
                            sizeof(wireless_telemetry_item_t)));
        return ESP_ERR_NO_MEM;
    }

    s_batch_buffer = (uint8_t *)heap_caps_malloc(
        WIRELESS_TELEMETRY_BATCH_MAX, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (s_batch_buffer == NULL) {
        wireless_telemetry_delete_ring();
        s_last_error = ENOMEM;
        s_status = WIRELESS_TELEMETRY_STATUS_FAILED;
        ESP_LOGE(TAG, "batch buffer allocation failed bytes=%u",
                 (unsigned)WIRELESS_TELEMETRY_BATCH_MAX);
        return ESP_ERR_NO_MEM;
    }

    const BaseType_t created = xTaskCreatePinnedToCore(
        wireless_telemetry_task, "wireless_tel",
        WIRELESS_TELEMETRY_TASK_STACK_BYTES, NULL,
        WIRELESS_TELEMETRY_TASK_PRIORITY, &s_task_handle,
        WIRELESS_TELEMETRY_TASK_CORE);
    if (created != pdPASS) {
        wireless_telemetry_delete_ring();
        wireless_telemetry_delete_batch_buffer();
        s_last_error = ENOMEM;
        s_status = WIRELESS_TELEMETRY_STATUS_FAILED;
        return ESP_ERR_NO_MEM;
    }

    s_status = WIRELESS_TELEMETRY_STATUS_IDLE;
    s_started = true;
    return ESP_OK;
}

enum wireless_telemetry_status wireless_telemetry_service_get_status(void)
{
    return s_status;
}

const char *wireless_telemetry_service_status_to_string(
    enum wireless_telemetry_status status)
{
    switch (status) {
    case WIRELESS_TELEMETRY_STATUS_DISABLED:
        return "disabled";
    case WIRELESS_TELEMETRY_STATUS_IDLE:
        return "idle";
    case WIRELESS_TELEMETRY_STATUS_WAITING_FOR_WIFI:
        return "waiting_for_wifi";
    case WIRELESS_TELEMETRY_STATUS_CONNECTING:
        return "connecting";
    case WIRELESS_TELEMETRY_STATUS_CONNECTED:
        return "connected";
    case WIRELESS_TELEMETRY_STATUS_FAILED:
        return "failed";
    default:
        return "unknown";
    }
}

bool wireless_telemetry_service_is_connected(void)
{
    return s_connected;
}

const char *wireless_telemetry_service_get_target(void)
{
    return APP_WIRELESS_TELEMETRY_TARGET;
}

uint16_t wireless_telemetry_service_get_port(void)
{
    return wireless_telemetry_active_port();
}

uint32_t wireless_telemetry_service_get_dropped_count(void)
{
    return s_dropped_count;
}

uint32_t wireless_telemetry_service_get_drop_full_count(void)
{
    return s_drop_full_count;
}

uint32_t wireless_telemetry_service_get_drop_mutex_count(void)
{
    return s_drop_mutex_count;
}

uint32_t wireless_telemetry_service_get_drop_format_count(void)
{
    return s_drop_format_count;
}

uint32_t wireless_telemetry_service_get_queue_depth(void)
{
    uint32_t depth;
    taskENTER_CRITICAL(&s_ring_lock);
    depth = (uint32_t)s_ring_count;
    taskEXIT_CRITICAL(&s_ring_lock);
    return depth;
}

uint32_t wireless_telemetry_service_get_queue_high_water(void)
{
    return s_ring_high_water;
}

uint32_t wireless_telemetry_service_get_binary_frame_count(void)
{
    return s_binary_frame_count;
}

uint32_t wireless_telemetry_service_get_binary_sample_count(void)
{
    return s_binary_sample_count;
}

uint32_t wireless_telemetry_service_get_text_frame_count(void)
{
    return s_text_frame_count;
}

uint32_t wireless_telemetry_service_get_connect_count(void)
{
    return s_connect_count;
}

uint32_t wireless_telemetry_service_get_send_failure_count(void)
{
    return s_send_failure_count;
}

uint32_t wireless_telemetry_service_get_send_timeout_count(void)
{
    return s_send_timeout_count;
}

uint32_t wireless_telemetry_service_get_socket_close_count(void)
{
    return s_socket_close_count;
}

uint32_t wireless_telemetry_service_get_last_send_ms(void)
{
    return s_last_send_ms;
}

uint32_t wireless_telemetry_service_get_max_send_ms(void)
{
    return s_max_send_ms;
}

int wireless_telemetry_service_get_last_error(void)
{
    return s_last_error;
}

bool wireless_telemetry_service_submit(const char *topic, const char *format,
                                       ...)
{
    if (topic == NULL || topic[0] == '\0' || format == NULL) {
        return false;
    }

    char payload[80] = {0};
    va_list args;
    va_start(args, format);
    vsnprintf(payload, sizeof(payload), format, args);
    va_end(args);

    const uint32_t uptime_ms =
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    wireless_telemetry_item_t item = {0};
    item.type = WIRELESS_TELEMETRY_ITEM_TEXT;
    item.uptime_ms = uptime_ms;
    const int written =
        snprintf(item.data.line, sizeof(item.data.line), "T,%s,%lu,%s,%s",
                 app_identity_get_hostname(), (unsigned long)uptime_ms, topic,
                 payload);
    if (written <= 0 || written >= (int)sizeof(item.data.line)) {
        s_dropped_count++;
        return false;
    }

    return wireless_telemetry_enqueue(&item);
}

bool wireless_telemetry_service_submit_bno085_accel(
    int32_t x_milli_mps2, int32_t y_milli_mps2, int32_t z_milli_mps2,
    uint8_t accuracy, uint32_t report_count)
{
    if (!s_connected) {
        return false;
    }

    const uint32_t uptime_ms =
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    wireless_telemetry_item_t item = {
        .type = WIRELESS_TELEMETRY_ITEM_BNO085_ACCEL,
        .uptime_ms = uptime_ms,
        .data = {
            .accel = {
                .x_milli_mps2 = x_milli_mps2,
                .y_milli_mps2 = y_milli_mps2,
                .z_milli_mps2 = z_milli_mps2,
                .report_count = report_count,
                .accuracy = accuracy,
            },
        },
    };

    return wireless_telemetry_enqueue(&item);
}

bool wireless_telemetry_service_submit_flex_tdoa_observation(
    uint8_t tag_id, uint8_t initiator_id, uint8_t responder_id,
    uint8_t responder_index, uint16_t sequence, uint32_t slot_id,
    int32_t diff_mm, int32_t raw_diff_mm, int32_t anchor_distance_mm)
{
    if (!s_connected) {
        return false;
    }

    const uint32_t uptime_ms =
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    wireless_telemetry_item_t item = {
        .type = WIRELESS_TELEMETRY_ITEM_FLEX_TDOA_OBSERVATION,
        .uptime_ms = uptime_ms,
        .data = {
            .flex_observation = {
                .slot_id = slot_id,
                .diff_mm = diff_mm,
                .raw_diff_mm = raw_diff_mm,
                .anchor_distance_mm = anchor_distance_mm,
                .sequence = sequence,
                .tag_id = tag_id,
                .initiator_id = initiator_id,
                .responder_id = responder_id,
                .responder_index = responder_index,
            },
        },
    };
    return wireless_telemetry_enqueue(&item);
}

bool wireless_telemetry_service_submit_flex_anchor_range(
    uint8_t initiator_id, uint8_t responder_id, uint16_t sequence,
    uint32_t slot_id, int32_t distance_mm, int32_t raw_distance_mm)
{
    if (!s_connected) {
        return false;
    }

    const uint32_t uptime_ms =
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    wireless_telemetry_item_t item = {
        .type = WIRELESS_TELEMETRY_ITEM_FLEX_ANCHOR_RANGE,
        .uptime_ms = uptime_ms,
        .data = {
            .flex_anchor_range = {
                .slot_id = slot_id,
                .distance_mm = distance_mm,
                .raw_distance_mm = raw_distance_mm,
                .sequence = sequence,
                .initiator_id = initiator_id,
                .responder_id = responder_id,
            },
        },
    };
    return wireless_telemetry_enqueue(&item);
}

bool wireless_telemetry_service_submit_flex_position(
    uint8_t tag_id, uint32_t slot_id, int32_t x_mm, int32_t y_mm,
    int32_t sigma_mm, int32_t rms_mm, uint16_t observation_count,
    uint8_t anchor_count, uint32_t geometry_version)
{
    if (!s_connected) {
        return false;
    }

    const uint32_t uptime_ms =
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    const wireless_telemetry_item_t item = {
        .type = WIRELESS_TELEMETRY_ITEM_FLEX_POSITION,
        .uptime_ms = uptime_ms,
        .data = {
            .flex_position = {
                .slot_id = slot_id,
                .geometry_version = geometry_version,
                .x_mm = x_mm,
                .y_mm = y_mm,
                .sigma_mm = sigma_mm,
                .rms_mm = rms_mm,
                .observation_count = observation_count,
                .tag_id = tag_id,
                .anchor_count = anchor_count,
            },
        },
    };
    return wireless_telemetry_enqueue(&item);
}

bool wireless_telemetry_service_submit_passive_ds_observation(
    uint8_t tag_id, uint8_t initiator_id, uint8_t responder_id,
    uint8_t responder_index, uint16_t sequence, uint32_t slot_id,
    int32_t diff_mm, int32_t raw_diff_mm, int32_t anchor_distance_mm,
    int32_t cfo_correction_mm, int32_t clock_offset_ppb,
    uint32_t reply_delay_us, uint8_t range_source,
    uint16_t range_age_slots)
{
    if (!s_connected) {
        return false;
    }

    const wireless_telemetry_item_t item = {
        .type = WIRELESS_TELEMETRY_ITEM_PASSIVE_DS_OBSERVATION,
        .uptime_ms =
            (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
        .data = {
            .flex_observation = {
                .slot_id = slot_id,
                .diff_mm = diff_mm,
                .raw_diff_mm = raw_diff_mm,
                .anchor_distance_mm = anchor_distance_mm,
                .cfo_correction_mm = cfo_correction_mm,
                .clock_offset_ppb = clock_offset_ppb,
                .reply_delay_us = reply_delay_us,
                .range_age_slots = range_age_slots,
                .sequence = sequence,
                .tag_id = tag_id,
                .initiator_id = initiator_id,
                .responder_id = responder_id,
                .responder_index = responder_index,
                .range_source = range_source,
            },
        },
    };
    return wireless_telemetry_enqueue(&item);
}

bool wireless_telemetry_service_submit_passive_ds_anchor_range(
    uint8_t initiator_id, uint8_t responder_id, uint16_t sequence,
    uint32_t slot_id, int32_t distance_mm, int32_t raw_distance_mm)
{
    if (!s_connected) {
        return false;
    }

    const wireless_telemetry_item_t item = {
        .type = WIRELESS_TELEMETRY_ITEM_PASSIVE_DS_ANCHOR_RANGE,
        .uptime_ms =
            (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
        .data = {
            .flex_anchor_range = {
                .slot_id = slot_id,
                .distance_mm = distance_mm,
                .raw_distance_mm = raw_distance_mm,
                .sequence = sequence,
                .initiator_id = initiator_id,
                .responder_id = responder_id,
            },
        },
    };
    return wireless_telemetry_enqueue(&item);
}

bool wireless_telemetry_service_submit_native_ds_anchor_range(
    uint8_t initiator_id, uint8_t responder_id, uint16_t sequence,
    uint32_t slot_id, int32_t distance_mm, int32_t raw_distance_mm)
{
    if (!s_connected) {
        return false;
    }

    const wireless_telemetry_item_t item = {
        .type = WIRELESS_TELEMETRY_ITEM_NATIVE_DS_ANCHOR_RANGE,
        .uptime_ms =
            (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
        .data = {
            .flex_anchor_range = {
                .slot_id = slot_id,
                .distance_mm = distance_mm,
                .raw_distance_mm = raw_distance_mm,
                .sequence = sequence,
                .initiator_id = initiator_id,
                .responder_id = responder_id,
            },
        },
    };
    return wireless_telemetry_enqueue(&item);
}

bool wireless_telemetry_service_submit_native_ds_tag_range(
    uint8_t tag_id, uint8_t anchor_id, uint16_t sequence,
    uint32_t slot_id, int32_t distance_mm, int32_t raw_distance_mm)
{
    if (!s_connected) {
        return false;
    }

    const wireless_telemetry_item_t item = {
        .type = WIRELESS_TELEMETRY_ITEM_NATIVE_DS_TAG_RANGE,
        .uptime_ms =
            (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
        .data = {
            .flex_anchor_range = {
                .slot_id = slot_id,
                .distance_mm = distance_mm,
                .raw_distance_mm = raw_distance_mm,
                .sequence = sequence,
                .initiator_id = tag_id,
                .responder_id = anchor_id,
            },
        },
    };
    return wireless_telemetry_enqueue(&item);
}

bool wireless_telemetry_service_submit_passive_ds_position(
    uint8_t tag_id, uint32_t slot_id, int32_t filtered_x_mm,
    int32_t filtered_y_mm, int32_t raw_x_mm, int32_t raw_y_mm,
    int32_t sigma_mm, int32_t rms_mm, uint16_t observation_count,
    uint8_t anchor_count, uint32_t geometry_version,
    bool independent_frame, bool complete_superframe,
    bool filter_correction, uint32_t solver_update_count,
    uint32_t independent_frame_count, uint16_t batch_span_ms,
    uint16_t batch_max_age_ms, uint16_t observation_mask,
    uint16_t rejection_reason_mask, uint16_t rejected_since_last,
    uint32_t position_rejected_count)
{
    if (!s_connected) {
        return false;
    }

    const wireless_telemetry_item_t item = {
        .type = WIRELESS_TELEMETRY_ITEM_PASSIVE_DS_POSITION,
        .uptime_ms =
            (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
        .data = {
            .flex_position = {
                .slot_id = slot_id,
                .geometry_version = geometry_version,
                .x_mm = filtered_x_mm,
                .y_mm = filtered_y_mm,
                .raw_x_mm = raw_x_mm,
                .raw_y_mm = raw_y_mm,
                .sigma_mm = sigma_mm,
                .rms_mm = rms_mm,
                .observation_count = observation_count,
                .solver_update_count = solver_update_count,
                .independent_frame_count = independent_frame_count,
                .position_rejected_count = position_rejected_count,
                .batch_span_ms = batch_span_ms,
                .batch_max_age_ms = batch_max_age_ms,
                .observation_mask = observation_mask,
                .rejection_reason_mask = rejection_reason_mask,
                .rejected_since_last = rejected_since_last,
                .tag_id = tag_id,
                .anchor_count = anchor_count,
                .solution_flags =
                    8U |
                    (independent_frame ? 1U : 0U) |
                    (complete_superframe ? 2U : 0U) |
                    (filter_correction ? 4U : 0U),
            },
        },
    };
    return wireless_telemetry_enqueue(&item);
}
