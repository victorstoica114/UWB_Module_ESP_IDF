#include "gps_ntrip_client.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>

#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_tls.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mbedtls/base64.h"
#include "mbedtls/ssl.h"
#include "secrets.h"
#include "wifi_service.h"

#ifndef NTRIP_ENABLED
#define NTRIP_ENABLED 0
#endif
#ifndef NTRIP_MODULE_ID
#define NTRIP_MODULE_ID 1
#endif
#ifndef NTRIP_GGA_INTERVAL_MS
#define NTRIP_GGA_INTERVAL_MS 10000
#endif

static const char *TAG = "gps_ntrip";

enum {
    GPS_NTRIP_TASK_STACK_BYTES = 8192,
    GPS_NTRIP_TASK_PRIORITY = 4,
    GPS_NTRIP_CONNECT_TIMEOUT_MS = 10000,
    GPS_NTRIP_READ_TIMEOUT_MS = 1000,
    GPS_NTRIP_RECONNECT_DELAY_MS = 2000,
    GPS_NTRIP_STOP_TIMEOUT_MS = 3000,
    GPS_NTRIP_BUFFER_SIZE = 1024,
    GPS_NTRIP_HEADER_SIZE = 2048,
    GPS_NTRIP_GGA_SIZE = 192,
};

typedef struct {
    uint8_t header[3];
    uint8_t header_length;
    uint16_t remaining;
} rtcm_counter_t;

typedef struct {
    uint8_t buffer[GPS_NTRIP_BUFFER_SIZE];
    char header[GPS_NTRIP_HEADER_SIZE];
    char request[GPS_NTRIP_HEADER_SIZE];
    char credentials[768];
    char authorization[1024];
    char gga_line[GPS_NTRIP_GGA_SIZE + 2];
} ntrip_workspace_t;

static SemaphoreHandle_t s_mutex;
static TaskHandle_t s_task;
static volatile bool s_stop_requested;
static esp_tls_t *s_tls;
static int s_socket = -1;
static uint8_t s_module_id;
static gps_ntrip_write_fn_t s_write_fn;
static void *s_write_context;
static char s_gga[GPS_NTRIP_GGA_SIZE];
static uint32_t s_gga_updated_ms;
static uint32_t s_last_data_ms;
static gps_ntrip_snapshot_t s_snapshot;
static rtcm_counter_t s_rtcm_counter;

static uint32_t ticks_to_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static bool lock_state(TickType_t timeout)
{
    return s_mutex != NULL && xSemaphoreTake(s_mutex, timeout) == pdTRUE;
}

static void unlock_state(void)
{
    if (s_mutex != NULL) {
        xSemaphoreGive(s_mutex);
    }
}

static void set_state(const char *state)
{
    if (lock_state(pdMS_TO_TICKS(50))) {
        snprintf(s_snapshot.state, sizeof(s_snapshot.state), "%s", state);
        unlock_state();
    }
}

static void note_error(const char *state)
{
    if (lock_state(pdMS_TO_TICKS(50))) {
        s_snapshot.error_count++;
        snprintf(s_snapshot.state, sizeof(s_snapshot.state), "%s", state);
        unlock_state();
    }
}

static bool copy_gga(char *destination, size_t capacity)
{
    bool available = false;
    if (lock_state(pdMS_TO_TICKS(50))) {
        if (s_gga[0] != '\0') {
            snprintf(destination, capacity, "%s", s_gga);
            available = true;
        }
        unlock_state();
    }
    return available;
}

static void count_rtcm_bytes(const uint8_t *data, size_t length)
{
    for (size_t index = 0; index < length; ++index) {
        const uint8_t byte = data[index];
        if (s_rtcm_counter.header_length == 0) {
            if (byte == 0xD3) {
                s_rtcm_counter.header[0] = byte;
                s_rtcm_counter.header_length = 1;
            }
            continue;
        }
        if (s_rtcm_counter.header_length < 3) {
            s_rtcm_counter.header[s_rtcm_counter.header_length++] = byte;
            if (s_rtcm_counter.header_length == 3) {
                const uint16_t payload_length =
                    (uint16_t)(((s_rtcm_counter.header[1] & 0x03U) << 8) |
                               s_rtcm_counter.header[2]);
                s_rtcm_counter.remaining = (uint16_t)(payload_length + 3U);
                if (s_rtcm_counter.remaining == 0) {
                    s_rtcm_counter.header_length = 0;
                }
            }
            continue;
        }
        if (s_rtcm_counter.remaining > 0) {
            s_rtcm_counter.remaining--;
            if (s_rtcm_counter.remaining == 0) {
                if (lock_state(pdMS_TO_TICKS(10))) {
                    s_snapshot.rtcm_frame_count++;
                    unlock_state();
                }
                s_rtcm_counter.header_length = 0;
            }
        }
    }
}

static bool tls_write_all(const void *data, size_t length)
{
    const uint8_t *cursor = data;
    size_t remaining = length;
    while (!s_stop_requested && remaining > 0) {
        const ssize_t written = esp_tls_conn_write(s_tls, cursor, remaining);
        if (written > 0) {
            cursor += written;
            remaining -= (size_t)written;
            continue;
        }
        if (written == MBEDTLS_ERR_SSL_WANT_READ ||
            written == MBEDTLS_ERR_SSL_WANT_WRITE) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        return false;
    }
    return remaining == 0;
}

static bool build_request(ntrip_workspace_t *workspace, const char *gga)
{
    const int credential_length = snprintf(workspace->credentials,
                                           sizeof(workspace->credentials),
                                           "%s:%s", NTRIP_USERNAME,
                                           NTRIP_PASSWORD);
    if (credential_length <= 0 ||
        credential_length >= (int)sizeof(workspace->credentials)) {
        return false;
    }

    size_t authorization_length = 0;
    if (mbedtls_base64_encode((unsigned char *)workspace->authorization,
                              sizeof(workspace->authorization) - 1U,
                              &authorization_length,
                              (const unsigned char *)workspace->credentials,
                              (size_t)credential_length) != 0) {
        return false;
    }
    workspace->authorization[authorization_length] = '\0';
    memset(workspace->credentials, 0, sizeof(workspace->credentials));

    const int request_length = snprintf(
        workspace->request, sizeof(workspace->request),
        "GET /%s HTTP/1.1\r\n"
        "Host: %s:%u\r\n"
        "User-Agent: NTRIP UWB-ESP32/1.0\r\n"
        "Authorization: Basic %s\r\n"
        "Ntrip-Version: Ntrip/2.0\r\n"
        "Ntrip-GGA: %s\r\n"
        "Connection: keep-alive\r\n\r\n",
        NTRIP_MOUNTPOINT, NTRIP_SERVER, (unsigned)NTRIP_PORT,
        workspace->authorization, gga);
    memset(workspace->authorization, 0, sizeof(workspace->authorization));
    return request_length > 0 &&
           request_length < (int)sizeof(workspace->request);
}

static int parse_http_status(const char *header)
{
    int status = 0;
    if (sscanf(header, "HTTP/%*u.%*u %d", &status) == 1) {
        return status;
    }
    if (sscanf(header, "ICY %d", &status) == 1) {
        return status;
    }
    return 0;
}

static bool forward_stream_bytes(const uint8_t *data, size_t length)
{
    if (length == 0) {
        return true;
    }
    const int written = s_write_fn != NULL
                            ? s_write_fn(data, length, s_write_context)
                            : -1;
    if (written != (int)length) {
        return false;
    }
    count_rtcm_bytes(data, length);
    if (lock_state(pdMS_TO_TICKS(50))) {
        s_snapshot.rtcm_byte_count += (uint32_t)length;
        s_snapshot.stream_active = true;
        snprintf(s_snapshot.state, sizeof(s_snapshot.state), "streaming");
        s_last_data_ms = ticks_to_ms();
        unlock_state();
    }
    return true;
}

static bool open_stream(void)
{
    ntrip_workspace_t *workspace = heap_caps_calloc(
        1, sizeof(*workspace), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (workspace == NULL) {
        workspace = calloc(1, sizeof(*workspace));
    }
    if (workspace == NULL) {
        note_error("workspace_failed");
        return false;
    }

    char gga[GPS_NTRIP_GGA_SIZE] = {0};
    if (!copy_gga(gga, sizeof(gga))) {
        set_state("waiting_gga");
        goto done;
    }

    set_state("connecting");
    s_tls = esp_tls_init();
    if (s_tls == NULL) {
        note_error("tls_init_failed");
        goto done;
    }
    const esp_tls_cfg_t config = {
        .timeout_ms = GPS_NTRIP_CONNECT_TIMEOUT_MS,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .tls_version = ESP_TLS_VER_TLS_1_2,
    };
    if (esp_tls_conn_new_sync(NTRIP_SERVER, strlen(NTRIP_SERVER),
                              NTRIP_PORT, &config, s_tls) != 1) {
        note_error("tls_failed");
        goto done;
    }
    if (esp_tls_get_conn_sockfd(s_tls, &s_socket) == ESP_OK) {
        const struct timeval timeout = {
            .tv_sec = GPS_NTRIP_READ_TIMEOUT_MS / 1000,
            .tv_usec = (GPS_NTRIP_READ_TIMEOUT_MS % 1000) * 1000,
        };
        (void)setsockopt(s_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                         sizeof(timeout));
    }
    if (lock_state(pdMS_TO_TICKS(50))) {
        s_snapshot.tls_connected = true;
        s_snapshot.connect_count++;
        snprintf(s_snapshot.state, sizeof(s_snapshot.state), "requesting");
        unlock_state();
    }

    if (!build_request(workspace, gga) ||
        !tls_write_all(workspace->request, strlen(workspace->request))) {
        note_error("request_failed");
        goto done;
    }
    memset(workspace->request, 0, sizeof(workspace->request));

    size_t header_length = 0;
    while (!s_stop_requested &&
           header_length < sizeof(workspace->header) - 1U) {
        const ssize_t received = esp_tls_conn_read(
            s_tls, workspace->buffer, sizeof(workspace->buffer));
        if (received > 0) {
            if (header_length + (size_t)received >=
                sizeof(workspace->header)) {
                note_error("header_too_large");
                goto done;
            }
            memcpy(workspace->header + header_length, workspace->buffer,
                   (size_t)received);
            header_length += (size_t)received;
            workspace->header[header_length] = '\0';
            char *end = strstr(workspace->header, "\r\n\r\n");
            if (end == NULL) {
                continue;
            }
            const size_t body_offset =
                (size_t)((end + 4) - workspace->header);
            const int status = parse_http_status(workspace->header);
            if (lock_state(pdMS_TO_TICKS(50))) {
                s_snapshot.http_status = (uint16_t)status;
                unlock_state();
            }
            if (status != 200) {
                note_error("caster_rejected");
                goto done;
            }
            if (header_length > body_offset &&
                !forward_stream_bytes(
                    (const uint8_t *)workspace->header + body_offset,
                    header_length - body_offset)) {
                note_error("uart_failed");
                goto done;
            }
            goto stream_ready;
        }
        if (received == MBEDTLS_ERR_SSL_TIMEOUT ||
            received == MBEDTLS_ERR_SSL_WANT_READ ||
            received == MBEDTLS_ERR_SSL_WANT_WRITE ||
            errno == EAGAIN || errno == EWOULDBLOCK) {
            continue;
        }
        note_error("header_failed");
        goto done;
    }
    note_error("header_too_large");
    goto done;

stream_ready:
    uint32_t last_gga_sent_ms = ticks_to_ms();
    while (!s_stop_requested && wifi_service_is_connected()) {
        const ssize_t received = esp_tls_conn_read(
            s_tls, workspace->buffer, sizeof(workspace->buffer));
        if (received > 0) {
            if (!forward_stream_bytes(workspace->buffer,
                                      (size_t)received)) {
                note_error("uart_failed");
                goto done;
            }
        } else if (received == 0) {
            note_error("caster_closed");
            goto done;
        } else if (received != MBEDTLS_ERR_SSL_TIMEOUT &&
                   received != MBEDTLS_ERR_SSL_WANT_READ &&
                   received != MBEDTLS_ERR_SSL_WANT_WRITE &&
                   errno != EAGAIN && errno != EWOULDBLOCK) {
            note_error("stream_failed");
            goto done;
        }

        const uint32_t now_ms = ticks_to_ms();
        if ((uint32_t)(now_ms - last_gga_sent_ms) >=
            NTRIP_GGA_INTERVAL_MS && copy_gga(gga, sizeof(gga))) {
            const int line_length = snprintf(workspace->gga_line,
                                             sizeof(workspace->gga_line),
                                             "%s\r\n", gga);
            if (line_length <= 0 ||
                line_length >= (int)sizeof(workspace->gga_line) ||
                !tls_write_all(workspace->gga_line,
                               (size_t)line_length)) {
                note_error("gga_send_failed");
                goto done;
            }
            last_gga_sent_ms = now_ms;
        }
    }

done:
    memset(workspace, 0, sizeof(*workspace));
    heap_caps_free(workspace);
    return false;
}

static void close_stream(void)
{
    s_socket = -1;
    if (s_tls != NULL) {
        esp_tls_conn_destroy(s_tls);
        s_tls = NULL;
    }
    if (lock_state(pdMS_TO_TICKS(50))) {
        s_snapshot.tls_connected = false;
        s_snapshot.stream_active = false;
        if (!s_stop_requested) {
            s_snapshot.reconnect_count++;
        }
        unlock_state();
    }
}

static void ntrip_task(void *arg)
{
    (void)arg;
    if (lock_state(pdMS_TO_TICKS(50))) {
        s_snapshot.running = true;
        unlock_state();
    }
    while (!s_stop_requested) {
        if (!wifi_service_is_connected()) {
            set_state("waiting_wifi");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        (void)open_stream();
        close_stream();
        if (!s_stop_requested) {
            vTaskDelay(pdMS_TO_TICKS(GPS_NTRIP_RECONNECT_DELAY_MS));
        }
    }
    if (lock_state(pdMS_TO_TICKS(50))) {
        s_snapshot.running = false;
        s_snapshot.tls_connected = false;
        s_snapshot.stream_active = false;
        snprintf(s_snapshot.state, sizeof(s_snapshot.state), "stopped");
        unlock_state();
    }
    s_task = NULL;
    vTaskDelete(NULL);
}

bool gps_ntrip_client_is_enabled(void)
{
    return NTRIP_ENABLED != 0 && NTRIP_SERVER[0] != '\0' &&
           NTRIP_USERNAME[0] != '\0' && NTRIP_PASSWORD[0] != '\0' &&
           NTRIP_MOUNTPOINT[0] != '\0';
}

bool gps_ntrip_client_is_enabled_for_module(uint8_t module_id)
{
    return gps_ntrip_client_is_enabled() && module_id == NTRIP_MODULE_ID;
}

esp_err_t gps_ntrip_client_start(uint8_t module_id,
                                 gps_ntrip_write_fn_t write_fn,
                                 void *write_context)
{
    gps_ntrip_client_stop();
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
        if (s_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    memset(&s_rtcm_counter, 0, sizeof(s_rtcm_counter));
    s_snapshot.configured = gps_ntrip_client_is_enabled();
    snprintf(s_snapshot.state, sizeof(s_snapshot.state), "disabled");
    s_module_id = module_id;
    s_write_fn = write_fn;
    s_write_context = write_context;
    s_stop_requested = false;
    s_last_data_ms = 0;
    if (!gps_ntrip_client_is_enabled_for_module(module_id)) {
        return ESP_OK;
    }
    snprintf(s_snapshot.state, sizeof(s_snapshot.state), "starting");
    if (xTaskCreate(ntrip_task, "gps_ntrip", GPS_NTRIP_TASK_STACK_BYTES,
                    NULL, GPS_NTRIP_TASK_PRIORITY, &s_task) != pdPASS) {
        s_task = NULL;
        note_error("task_failed");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "NTRIP TLS client enabled on module %u", (unsigned)module_id);
    return ESP_OK;
}

void gps_ntrip_client_stop(void)
{
    s_stop_requested = true;
    if (s_socket >= 0) {
        (void)shutdown(s_socket, SHUT_RDWR);
    }
    const uint32_t started_ms = ticks_to_ms();
    while (s_task != NULL &&
           (uint32_t)(ticks_to_ms() - started_ms) <
               GPS_NTRIP_STOP_TIMEOUT_MS) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    s_write_fn = NULL;
    s_write_context = NULL;
    s_module_id = 0;
}

void gps_ntrip_client_set_gga(const char *sentence)
{
    if (sentence == NULL || sentence[0] != '$' ||
        strstr(sentence, "GGA") == NULL) {
        return;
    }
    if (lock_state(pdMS_TO_TICKS(20))) {
        snprintf(s_gga, sizeof(s_gga), "%s", sentence);
        s_gga_updated_ms = ticks_to_ms();
        unlock_state();
    }
}

void gps_ntrip_client_get_snapshot(gps_ntrip_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    if (!lock_state(pdMS_TO_TICKS(50))) {
        memset(snapshot, 0, sizeof(*snapshot));
        return;
    }
    *snapshot = s_snapshot;
    snapshot->last_data_age_ms =
        s_last_data_ms == 0 ? UINT32_MAX : ticks_to_ms() - s_last_data_ms;
    unlock_state();
}
