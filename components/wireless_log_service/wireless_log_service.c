#include "wireless_log_service.h"

#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "esp_log_write.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "wifi_service.h"

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#include "app_config.h"

#ifndef HOSTNAME
#define HOSTNAME "uwb-module"
#endif

#ifndef APP_WIRELESS_LOG_TARGET
#define APP_WIRELESS_LOG_TARGET ""
#endif

enum {
    WIRELESS_LOG_TASK_STACK_WORDS = 4096,
    WIRELESS_LOG_TASK_PRIORITY = 4,
    WIRELESS_LOG_QUEUE_LEN = 64,
    WIRELESS_LOG_LINE_MAX = 256,
    WIRELESS_LOG_RECONNECT_MS = 2000,
    WIRELESS_LOG_WIFI_WAIT_MS = 500,
    WIRELESS_LOG_QUEUE_WAIT_MS = 500,
    WIRELESS_LOG_SEND_TIMEOUT_MS = 1000,
};

typedef struct {
    char line[WIRELESS_LOG_LINE_MAX];
} wireless_log_line_t;

static QueueHandle_t s_log_queue;
static TaskHandle_t s_task_handle;
static vprintf_like_t s_previous_vprintf;
static bool s_started;
static bool s_hook_installed;
static volatile enum wireless_log_status s_status =
    WIRELESS_LOG_STATUS_DISABLED;
static volatile bool s_connected;
static volatile uint32_t s_dropped_count;

static bool wireless_log_target_configured(void)
{
    return APP_WIRELESS_LOG_ENABLED && strlen(APP_WIRELESS_LOG_TARGET) > 0;
}

static void wireless_log_strip_line(char *line)
{
    size_t len = strlen(line);
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
        line[len - 1] = '\0';
        len--;
    }
}

static void wireless_log_strip_ansi(char *line)
{
    char *read = line;
    char *write = line;

    while (*read != '\0') {
        if (*read == '\x1b' && read[1] == '[') {
            read += 2;
            while (*read != '\0' &&
                   !((*read >= 'A' && *read <= 'Z') ||
                     (*read >= 'a' && *read <= 'z'))) {
                read++;
            }
            if (*read != '\0') {
                read++;
            }
            continue;
        }

        *write = *read;
        write++;
        read++;
    }

    *write = '\0';
}

static bool wireless_log_parse_idf_line(const char *raw, char *out,
                                        size_t out_size)
{
    char level = '\0';
    unsigned long uptime_ms = 0;
    char tag[40] = {0};
    const char *message = NULL;

    if (sscanf(raw, "%c (%lu) %39[^:]:", &level, &uptime_ms, tag) != 3) {
        return false;
    }

    const char *colon = strchr(raw, ':');
    if (colon == NULL) {
        return false;
    }

    message = colon + 1;
    while (*message == ' ') {
        message++;
    }

    snprintf(out, out_size, "[%s] [%10lu ms] [%c][%s] %s", HOSTNAME,
             uptime_ms, level, tag, message);
    return true;
}

static bool wireless_log_should_skip_idf_tag(const char *raw)
{
    char level = '\0';
    unsigned long uptime_ms = 0;
    char tag[40] = {0};

    if (sscanf(raw, "%c (%lu) %39[^:]:", &level, &uptime_ms, tag) != 3) {
        return false;
    }

    (void)level;
    (void)uptime_ms;

    return strcmp(tag, "wifi") == 0 || strcmp(tag, "wifi_init") == 0 ||
           strcmp(tag, "phy_init") == 0;
}

static bool wireless_log_enqueue_raw(const char *raw)
{
    if (s_log_queue == NULL || !wireless_log_target_configured()) {
        return false;
    }

    wireless_log_line_t item = {0};
    char normalized[WIRELESS_LOG_LINE_MAX] = {0};

    snprintf(normalized, sizeof(normalized), "%s", raw);
    wireless_log_strip_line(normalized);
    wireless_log_strip_ansi(normalized);
    if (normalized[0] == '\0') {
        return false;
    }

    if (wireless_log_should_skip_idf_tag(normalized)) {
        return false;
    }

    if (!wireless_log_parse_idf_line(normalized, item.line,
                                     sizeof(item.line))) {
        return false;
    }

    if (xQueueSend(s_log_queue, &item, 0) != pdTRUE) {
        s_dropped_count++;
        return false;
    }

    return true;
}

static int wireless_log_vprintf(const char *format, va_list args)
{
    va_list copy;
    va_copy(copy, args);

    int written = 0;
    if (s_previous_vprintf != NULL) {
        written = s_previous_vprintf(format, args);
    } else {
        written = vprintf(format, args);
    }

    char raw[WIRELESS_LOG_LINE_MAX] = {0};
    vsnprintf(raw, sizeof(raw), format, copy);
    va_end(copy);

    (void)wireless_log_enqueue_raw(raw);
    return written;
}

static int wireless_log_connect_socket(void)
{
    char port[8];
    snprintf(port, sizeof(port), "%u", APP_WIRELESS_LOG_PORT);

    struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *result = NULL;

    const int gai_err =
        getaddrinfo(APP_WIRELESS_LOG_TARGET, port, &hints, &result);
    if (gai_err != 0 || result == NULL) {
        return -1;
    }

    int sock = -1;
    for (struct addrinfo *it = result; it != NULL; it = it->ai_next) {
        sock = socket(it->ai_family, it->ai_socktype, it->ai_protocol);
        if (sock < 0) {
            continue;
        }

        struct timeval timeout = {
            .tv_sec = WIRELESS_LOG_SEND_TIMEOUT_MS / 1000,
            .tv_usec = (WIRELESS_LOG_SEND_TIMEOUT_MS % 1000) * 1000,
        };
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

        if (connect(sock, it->ai_addr, it->ai_addrlen) == 0) {
            break;
        }

        closesocket(sock);
        sock = -1;
    }

    freeaddrinfo(result);
    return sock;
}

static bool wireless_log_send_all(int sock, const char *line)
{
    char payload[WIRELESS_LOG_LINE_MAX + 2];
    const int len = snprintf(payload, sizeof(payload), "%s\n", line);
    if (len <= 0 || len >= (int)sizeof(payload)) {
        return false;
    }

    int sent_total = 0;
    const TickType_t start = xTaskGetTickCount();
    while (sent_total < len) {
        const int sent = send(sock, payload + sent_total,
                              (size_t)(len - sent_total), MSG_DONTWAIT);
        if (sent > 0) {
            sent_total += sent;
            continue;
        }

        if (sent < 0 && (errno == EWOULDBLOCK || errno == EAGAIN)) {
            if ((xTaskGetTickCount() - start) >=
                pdMS_TO_TICKS(WIRELESS_LOG_SEND_TIMEOUT_MS)) {
                return false;
            }
            vTaskDelay(1);
            continue;
        }

        if (sent <= 0) {
            return false;
        }
    }

    return true;
}

static bool wireless_log_socket_alive(int sock)
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

static void wireless_log_close_socket(int *sock)
{
    if (*sock >= 0) {
        closesocket(*sock);
        *sock = -1;
    }
    s_connected = false;
}

static void wireless_log_task(void *arg)
{
    (void)arg;

    int sock = -1;
    TickType_t last_connect_attempt = 0;

    while (true) {
        if (!wireless_log_target_configured()) {
            s_status = WIRELESS_LOG_STATUS_DISABLED;
            wireless_log_close_socket(&sock);
            vTaskDelay(pdMS_TO_TICKS(WIRELESS_LOG_WIFI_WAIT_MS));
            continue;
        }

        if (!wifi_service_is_connected()) {
            s_status = WIRELESS_LOG_STATUS_WAITING_FOR_WIFI;
            wireless_log_close_socket(&sock);
            vTaskDelay(pdMS_TO_TICKS(WIRELESS_LOG_WIFI_WAIT_MS));
            continue;
        }

        if (sock < 0) {
            const TickType_t now = xTaskGetTickCount();
            if (now - last_connect_attempt <
                pdMS_TO_TICKS(WIRELESS_LOG_RECONNECT_MS)) {
                vTaskDelay(pdMS_TO_TICKS(WIRELESS_LOG_WIFI_WAIT_MS));
                continue;
            }

            last_connect_attempt = now;
            s_status = WIRELESS_LOG_STATUS_CONNECTING;
            sock = wireless_log_connect_socket();
            if (sock < 0) {
                s_connected = false;
                s_status = WIRELESS_LOG_STATUS_FAILED;
                continue;
            }

            s_connected = true;
            s_status = WIRELESS_LOG_STATUS_CONNECTED;
            (void)wireless_log_enqueue_raw(
                "I (0) wireless_log: TCP log stream connected");
        }

        wireless_log_line_t item;
        if (xQueueReceive(s_log_queue, &item,
                          pdMS_TO_TICKS(WIRELESS_LOG_QUEUE_WAIT_MS)) !=
            pdTRUE) {
            if (!wireless_log_socket_alive(sock)) {
                wireless_log_close_socket(&sock);
                s_status = WIRELESS_LOG_STATUS_FAILED;
            }
            continue;
        }

        if (!wireless_log_send_all(sock, item.line)) {
            wireless_log_close_socket(&sock);
            s_status = WIRELESS_LOG_STATUS_FAILED;
        }
    }
}

esp_err_t wireless_log_service_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    if (!wireless_log_target_configured()) {
        s_status = WIRELESS_LOG_STATUS_DISABLED;
        s_started = true;
        return ESP_OK;
    }

    s_log_queue = xQueueCreate(WIRELESS_LOG_QUEUE_LEN,
                               sizeof(wireless_log_line_t));
    if (s_log_queue == NULL) {
        s_status = WIRELESS_LOG_STATUS_FAILED;
        return ESP_ERR_NO_MEM;
    }

    s_previous_vprintf = esp_log_set_vprintf(wireless_log_vprintf);
    s_hook_installed = true;

    const BaseType_t created = xTaskCreate(wireless_log_task, "wireless_log",
                                           WIRELESS_LOG_TASK_STACK_WORDS, NULL,
                                           WIRELESS_LOG_TASK_PRIORITY,
                                           &s_task_handle);
    if (created != pdPASS) {
        if (s_hook_installed) {
            esp_log_set_vprintf(s_previous_vprintf);
            s_hook_installed = false;
        }
        vQueueDelete(s_log_queue);
        s_log_queue = NULL;
        s_status = WIRELESS_LOG_STATUS_FAILED;
        return ESP_ERR_NO_MEM;
    }

    s_status = WIRELESS_LOG_STATUS_IDLE;
    s_started = true;
    return ESP_OK;
}

enum wireless_log_status wireless_log_service_get_status(void)
{
    return s_status;
}

const char *wireless_log_service_status_to_string(
    enum wireless_log_status status)
{
    switch (status) {
    case WIRELESS_LOG_STATUS_DISABLED:
        return "disabled";
    case WIRELESS_LOG_STATUS_IDLE:
        return "idle";
    case WIRELESS_LOG_STATUS_WAITING_FOR_WIFI:
        return "waiting_for_wifi";
    case WIRELESS_LOG_STATUS_CONNECTING:
        return "connecting";
    case WIRELESS_LOG_STATUS_CONNECTED:
        return "connected";
    case WIRELESS_LOG_STATUS_FAILED:
        return "failed";
    default:
        return "unknown";
    }
}

bool wireless_log_service_is_connected(void)
{
    return s_connected;
}

const char *wireless_log_service_get_target(void)
{
    return APP_WIRELESS_LOG_TARGET;
}

uint16_t wireless_log_service_get_port(void)
{
    return APP_WIRELESS_LOG_PORT;
}

uint32_t wireless_log_service_get_dropped_count(void)
{
    return s_dropped_count;
}

bool wireless_log_service_submit(char level, const char *tag,
                                 const char *format, ...)
{
    if (tag == NULL || format == NULL) {
        return false;
    }

    char message[WIRELESS_LOG_LINE_MAX] = {0};
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);

    const uint32_t uptime_ms =
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    char raw[WIRELESS_LOG_LINE_MAX] = {0};
    const int prefix_len = snprintf(raw, sizeof(raw), "%c (%lu) %.39s: ",
                                    level, (unsigned long)uptime_ms, tag);
    if (prefix_len < 0 || prefix_len >= (int)sizeof(raw)) {
        return false;
    }

    const size_t remaining = sizeof(raw) - (size_t)prefix_len;
    strncpy(raw + prefix_len, message, remaining - 1U);
    raw[sizeof(raw) - 1U] = '\0';

    return wireless_log_enqueue_raw(raw);
}
