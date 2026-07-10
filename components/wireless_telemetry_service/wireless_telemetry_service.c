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
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "wifi_service.h"

#include "app_config.h"
#include "app_identity.h"

static const char *TAG = "wireless_telemetry";

#ifndef APP_WIRELESS_TELEMETRY_TARGET
#define APP_WIRELESS_TELEMETRY_TARGET ""
#endif

enum {
    WIRELESS_TELEMETRY_TASK_STACK_WORDS = 4096,
    WIRELESS_TELEMETRY_TASK_PRIORITY = 4,
    WIRELESS_TELEMETRY_QUEUE_LEN = 256,
    WIRELESS_TELEMETRY_LINE_MAX = 96,
    WIRELESS_TELEMETRY_BATCH_MAX = 1400,
    WIRELESS_TELEMETRY_RECONNECT_MS = 2000,
    WIRELESS_TELEMETRY_WIFI_WAIT_MS = 500,
    WIRELESS_TELEMETRY_QUEUE_WAIT_MS = 20,
    WIRELESS_TELEMETRY_SEND_TIMEOUT_MS = 1000,
};

typedef enum {
    WIRELESS_TELEMETRY_ITEM_TEXT = 0,
    WIRELESS_TELEMETRY_ITEM_BNO085_ACCEL,
} wireless_telemetry_item_type_t;

typedef struct {
    int32_t x_milli_mps2;
    int32_t y_milli_mps2;
    int32_t z_milli_mps2;
    uint32_t report_count;
    uint8_t accuracy;
} wireless_telemetry_accel_t;

typedef struct {
    wireless_telemetry_item_type_t type;
    uint32_t uptime_ms;
    union {
        char line[WIRELESS_TELEMETRY_LINE_MAX];
        wireless_telemetry_accel_t accel;
    } data;
} wireless_telemetry_item_t;

static QueueHandle_t s_queue;
static TaskHandle_t s_task_handle;
static bool s_started;
static volatile enum wireless_telemetry_status s_status =
    WIRELESS_TELEMETRY_STATUS_DISABLED;
static volatile bool s_connected;
static volatile uint32_t s_dropped_count;
static volatile int s_last_error;

static bool wireless_telemetry_target_configured(void)
{
    return APP_WIRELESS_TELEMETRY_ENABLED &&
           strlen(APP_WIRELESS_TELEMETRY_TARGET) > 0;
}

static bool wireless_telemetry_enqueue(const wireless_telemetry_item_t *item)
{
    if (s_queue == NULL || item == NULL ||
        !wireless_telemetry_target_configured()) {
        return false;
    }

    if (xQueueSend(s_queue, item, 0) == pdTRUE) {
        return true;
    }

    wireless_telemetry_item_t discarded = {0};
    if (xQueueReceive(s_queue, &discarded, 0) == pdTRUE) {
        s_dropped_count++;
    }

    if (xQueueSend(s_queue, item, 0) == pdTRUE) {
        return true;
    }

    s_dropped_count++;
    return false;
}

static int wireless_telemetry_connect_socket(void)
{
    char port[8];
    snprintf(port, sizeof(port), "%u", APP_WIRELESS_TELEMETRY_PORT);

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

static bool wireless_telemetry_send_all(int sock, const char *payload,
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
                return false;
            }
            vTaskDelay(1);
            continue;
        }

        s_last_error = errno;
        return false;
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
        return false;
    }

    return errno == EWOULDBLOCK || errno == EAGAIN;
}

static void wireless_telemetry_close_socket(int *sock)
{
    if (*sock >= 0) {
        closesocket(*sock);
        *sock = -1;
    }
    s_connected = false;
}

static bool wireless_telemetry_format_item(const wireless_telemetry_item_t *item,
                                           char *line, size_t line_size)
{
    if (item == NULL || line == NULL || line_size == 0) {
        return false;
    }

    int written = 0;
    switch (item->type) {
    case WIRELESS_TELEMETRY_ITEM_TEXT:
        written = snprintf(line, line_size, "%s", item->data.line);
        break;
    case WIRELESS_TELEMETRY_ITEM_BNO085_ACCEL:
        written = snprintf(
            line, line_size, "T,%s,%lu,bno085.accel,%ld,%ld,%ld,%u,%lu",
            app_identity_get_hostname(), (unsigned long)item->uptime_ms,
            (long)item->data.accel.x_milli_mps2,
            (long)item->data.accel.y_milli_mps2,
            (long)item->data.accel.z_milli_mps2,
            (unsigned)item->data.accel.accuracy,
            (unsigned long)item->data.accel.report_count);
        break;
    default:
        return false;
    }

    return written > 0 && written < (int)line_size;
}

static bool wireless_telemetry_take_batch(char *batch, size_t batch_size,
                                          size_t *batch_len)
{
    wireless_telemetry_item_t item = {0};
    if (xQueueReceive(s_queue, &item,
                      pdMS_TO_TICKS(WIRELESS_TELEMETRY_QUEUE_WAIT_MS)) !=
        pdTRUE) {
        return false;
    }

    size_t used = 0;
    while (true) {
        char line[WIRELESS_TELEMETRY_LINE_MAX] = {0};
        if (wireless_telemetry_format_item(&item, line, sizeof(line))) {
            const int written = snprintf(batch + used, batch_size - used,
                                         "%s\n", line);
            if (written <= 0 || written >= (int)(batch_size - used)) {
                if (used == 0) {
                    s_dropped_count++;
                }
                break;
            }
            used += (size_t)written;
        } else {
            s_dropped_count++;
        }

        if (batch_size - used < WIRELESS_TELEMETRY_LINE_MAX + 2U) {
            break;
        }

        if (xQueueReceive(s_queue, &item, 0) != pdTRUE) {
            break;
        }
    }

    *batch_len = used;
    return used > 0;
}

static void wireless_telemetry_task(void *arg)
{
    (void)arg;

    int sock = -1;
    TickType_t last_connect_attempt = 0;
    char batch[WIRELESS_TELEMETRY_BATCH_MAX] = {0};

    while (true) {
        if (!wireless_telemetry_target_configured()) {
            s_status = WIRELESS_TELEMETRY_STATUS_DISABLED;
            wireless_telemetry_close_socket(&sock);
            vTaskDelay(pdMS_TO_TICKS(WIRELESS_TELEMETRY_WIFI_WAIT_MS));
            continue;
        }

        if (!wifi_service_is_connected()) {
            s_status = WIRELESS_TELEMETRY_STATUS_WAITING_FOR_WIFI;
            wireless_telemetry_close_socket(&sock);
            vTaskDelay(pdMS_TO_TICKS(WIRELESS_TELEMETRY_WIFI_WAIT_MS));
            continue;
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
                         (unsigned)APP_WIRELESS_TELEMETRY_PORT,
                         s_last_error);
                continue;
            }

            s_connected = true;
            s_status = WIRELESS_TELEMETRY_STATUS_CONNECTED;
            ESP_LOGI(TAG, "connected target=%s port=%u",
                     APP_WIRELESS_TELEMETRY_TARGET,
                     (unsigned)APP_WIRELESS_TELEMETRY_PORT);
        }

        size_t batch_len = 0;
        if (!wireless_telemetry_take_batch(batch, sizeof(batch),
                                           &batch_len)) {
            if (!wireless_telemetry_socket_alive(sock)) {
                wireless_telemetry_close_socket(&sock);
                s_status = WIRELESS_TELEMETRY_STATUS_FAILED;
            }
            continue;
        }

        if (!wireless_telemetry_send_all(sock, batch, batch_len)) {
            wireless_telemetry_close_socket(&sock);
            s_status = WIRELESS_TELEMETRY_STATUS_FAILED;
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

    s_queue = xQueueCreate(WIRELESS_TELEMETRY_QUEUE_LEN,
                           sizeof(wireless_telemetry_item_t));
    if (s_queue == NULL) {
        s_last_error = ENOMEM;
        s_status = WIRELESS_TELEMETRY_STATUS_FAILED;
        return ESP_ERR_NO_MEM;
    }

    const BaseType_t created =
        xTaskCreate(wireless_telemetry_task, "wireless_tel",
                    WIRELESS_TELEMETRY_TASK_STACK_WORDS, NULL,
                    WIRELESS_TELEMETRY_TASK_PRIORITY, &s_task_handle);
    if (created != pdPASS) {
        vQueueDelete(s_queue);
        s_queue = NULL;
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
    return APP_WIRELESS_TELEMETRY_PORT;
}

uint32_t wireless_telemetry_service_get_dropped_count(void)
{
    return s_dropped_count;
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
