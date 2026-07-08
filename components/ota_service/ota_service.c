#include "ota_service.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include "esp_app_desc.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "app_identity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stability_test_service.h"
#include "uwb_dw3000.h"
#include "wifi_service.h"
#include "wireless_log_service.h"

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
    OTA_SERVICE_MAX_QUERY_LEN = 160,
};

#define OTA_SERVICE_TOKEN_HEADER "X-OTA-Token"

static httpd_handle_t s_http_server;
static bool s_started;
static bool s_running;
static volatile bool s_ota_in_progress;
static volatile enum ota_service_status s_status = OTA_SERVICE_STATUS_IDLE;
static uint8_t s_ota_buffer[OTA_SERVICE_CHUNK_SIZE];

static void reboot_task(void *arg);

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

static bool ota_query_option_enabled(const char *query, const char *key)
{
    char value[8] = {0};
    return httpd_query_key_value(query, key, value, sizeof(value)) == ESP_OK &&
           strcmp(value, "1") == 0;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    static const char response[] =
        "uwb_esp_idf\n"
        "GET  /status\n"
        "POST /ota    raw firmware image, requires X-OTA-Token header\n"
        "POST /config/antenna-delay?value=0x4018[&reboot=1]\n"
        "POST /config/antenna-delay?clear=1[&reboot=1]\n";

    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_get_handler(httpd_req_t *req)
{
    const esp_app_desc_t *app = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *boot = esp_ota_get_boot_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    const uint16_t active_antenna_delay = uwb_dw3000_get_antenna_delay();
    const uint16_t configured_antenna_delay =
        app_identity_get_uwb_antenna_delay();

    char response[2800];
    const int len = snprintf(
        response, sizeof(response),
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
        "\"uwb_status\":\"%s\","
        "\"uwb_device_id\":\"0x%08lx\","
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
        "\"stability_log_stress_enabled\":%s,"
        "\"stability_log_stress_status\":\"%s\","
        "\"stability_log_stress_generated\":%lu,"
        "\"stability_log_stress_enqueue_failed\":%lu"
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
        uwb_dw3000_status_to_string(uwb_dw3000_get_status()),
        (unsigned long)uwb_dw3000_get_device_id(),
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
        stability_test_service_is_enabled() ? "true" : "false",
        stability_test_service_status_to_string(
            stability_test_service_get_status()),
        (unsigned long)stability_test_service_get_log_generated_count(),
        (unsigned long)stability_test_service_get_log_enqueue_failed_count());

    if (len < 0 || len >= (int)sizeof(response)) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "status too long");
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, response, (ssize_t)len);
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

static void reboot_task(void *arg)
{
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(OTA_SERVICE_REBOOT_DELAY_MS));
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

    int remaining = req->content_len;
    size_t written = 0;
    size_t next_progress_log = 256 * 1024;

    while (remaining > 0) {
        const size_t to_read = remaining < OTA_SERVICE_CHUNK_SIZE
                                   ? (size_t)remaining
                                   : OTA_SERVICE_CHUNK_SIZE;
        const int received =
            httpd_req_recv(req, (char *)s_ota_buffer, to_read);

        if (received == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }

        if (received <= 0) {
            esp_ota_abort(ota_handle);
            s_ota_in_progress = false;
            s_status = OTA_SERVICE_STATUS_FAILED;
            return send_ota_error(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                  "Failed to receive OTA data");
        }

        err = esp_ota_write(ota_handle, s_ota_buffer, (size_t)received);
        if (err != ESP_OK) {
            esp_ota_abort(ota_handle);
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

static esp_err_t start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.stack_size = 8192;
    config.max_uri_handlers = 4;

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
    const httpd_uri_t antenna_delay_uri = {
        .uri = "/config/antenna-delay",
        .method = HTTP_POST,
        .handler = antenna_delay_post_handler,
        .user_ctx = NULL,
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &root_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &status_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &ota_uri));
    ESP_ERROR_CHECK(
        httpd_register_uri_handler(s_http_server, &antenna_delay_uri));

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
