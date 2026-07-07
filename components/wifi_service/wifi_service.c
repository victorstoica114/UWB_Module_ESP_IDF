#include "wifi_service.h"

#include <string.h>

#include "esp_event.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef APP_WIFI_SSID
#define APP_WIFI_SSID ""
#endif

#ifndef APP_WIFI_PASSWORD
#define APP_WIFI_PASSWORD ""
#endif

#ifndef HOSTNAME
#define HOSTNAME "uwb-module"
#endif

#ifndef APP_WIFI_USE_WPA3
#define APP_WIFI_USE_WPA3 0
#endif

static const char *TAG = "wifi_service";

enum {
    WIFI_SERVICE_TASK_STACK_WORDS = 4096,
    WIFI_SERVICE_TASK_PRIORITY = 6,
    WIFI_SERVICE_CONNECT_TIMEOUT_MS = 30000,
    WIFI_SERVICE_MAX_RETRIES = 10,
};

#define WIFI_SERVICE_CONNECTED_BIT BIT0
#define WIFI_SERVICE_FAIL_BIT BIT1

static EventGroupHandle_t s_wifi_event_group;
static esp_netif_t *s_sta_netif;
static bool s_started;
static volatile bool s_connected;
static volatile enum wifi_service_status s_status = WIFI_SERVICE_STATUS_IDLE;
static int s_retry_count;
static char s_ip_address[16] = "0.0.0.0";

static bool wifi_credentials_present(void)
{
    return strlen(APP_WIFI_SSID) > 0 && strlen(APP_WIFI_PASSWORD) > 0;
}

static void copy_wifi_string(uint8_t *destination, size_t destination_size,
                             const char *source)
{
    if (destination_size == 0) {
        return;
    }

    strncpy((char *)destination, source, destination_size - 1);
    destination[destination_size - 1] = '\0';
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        s_status = WIFI_SERVICE_STATUS_CONNECTING;
        ESP_LOGI(TAG, "Connecting to Wi-Fi SSID '%s'", APP_WIFI_SSID);
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *event =
            (const wifi_event_sta_disconnected_t *)event_data;

        s_connected = false;
        s_status = WIFI_SERVICE_STATUS_CONNECTING;
        strcpy(s_ip_address, "0.0.0.0");

        if (s_retry_count < WIFI_SERVICE_MAX_RETRIES) {
            s_retry_count++;
            ESP_LOGW(TAG, "Disconnected, reason=%u, retry %d/%d",
                     event != NULL ? event->reason : 0, s_retry_count,
                     WIFI_SERVICE_MAX_RETRIES);
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
        } else {
            s_status = WIFI_SERVICE_STATUS_FAILED;
            ESP_LOGE(TAG, "Wi-Fi connection failed after %d retries",
                     WIFI_SERVICE_MAX_RETRIES);
            xEventGroupSetBits(s_wifi_event_group, WIFI_SERVICE_FAIL_BIT);
        }
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;

        s_retry_count = 0;
        s_connected = true;
        s_status = WIFI_SERVICE_STATUS_CONNECTED;
        snprintf(s_ip_address, sizeof(s_ip_address), IPSTR,
                 IP2STR(&event->ip_info.ip));

        ESP_LOGI(TAG, "Wi-Fi connected, IP: %s", s_ip_address);
        xEventGroupSetBits(s_wifi_event_group, WIFI_SERVICE_CONNECTED_BIT);
    }
}

static esp_err_t wifi_service_init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

static esp_err_t wifi_service_init_driver(void)
{
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init failed");

    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    s_sta_netif = esp_netif_create_default_wifi_sta();
    if (s_sta_netif == NULL) {
        ESP_LOGE(TAG, "Failed to create default Wi-Fi STA netif");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_set_hostname(s_sta_netif,
                                                         HOSTNAME));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "esp_wifi_init failed");

    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
                            WIFI_EVENT, ESP_EVENT_ANY_ID,
                            &wifi_event_handler, NULL, NULL),
                        TAG, "register WIFI_EVENT handler failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
                            IP_EVENT, IP_EVENT_STA_GOT_IP,
                            &wifi_event_handler, NULL, NULL),
                        TAG, "register IP_EVENT handler failed");

    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG,
                        "esp_wifi_set_storage failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG,
                        "esp_wifi_set_mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), TAG,
                        "esp_wifi_set_ps failed");

    wifi_config_t wifi_config = {0};
    copy_wifi_string(wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid),
                     APP_WIFI_SSID);
    copy_wifi_string(wifi_config.sta.password,
                     sizeof(wifi_config.sta.password), APP_WIFI_PASSWORD);
    wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wifi_config.sta.threshold.authmode =
        APP_WIFI_USE_WPA3 ? WIFI_AUTH_WPA2_WPA3_PSK : WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
    wifi_config.sta.sae_pk_mode = WPA3_SAE_PK_MODE_DISABLED;
    wifi_config.sta.failure_retry_cnt = 3;

    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG,
                        "esp_wifi_set_config failed");

    return ESP_OK;
}

static void wifi_service_task(void *arg)
{
    (void)arg;

    esp_err_t err = wifi_service_init_nvs();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
        s_status = WIFI_SERVICE_STATUS_FAILED;
        vTaskDelete(NULL);
        return;
    }

    if (!wifi_credentials_present()) {
        ESP_LOGW(TAG, "Wi-Fi credentials missing; skipping connection");
        s_status = WIFI_SERVICE_STATUS_FAILED;
        vTaskDelete(NULL);
        return;
    }

    err = wifi_service_init_driver();
    if (err != ESP_OK) {
        s_status = WIFI_SERVICE_STATUS_FAILED;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Wi-Fi hostname: %s", HOSTNAME);
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_start());

    const EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group, WIFI_SERVICE_CONNECTED_BIT | WIFI_SERVICE_FAIL_BIT,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(WIFI_SERVICE_CONNECT_TIMEOUT_MS));

    if ((bits & WIFI_SERVICE_CONNECTED_BIT) != 0) {
        ESP_LOGI(TAG, "Wi-Fi ready");
    } else if ((bits & WIFI_SERVICE_FAIL_BIT) != 0) {
        ESP_LOGE(TAG, "Wi-Fi failed");
    } else {
        ESP_LOGW(TAG, "Wi-Fi still connecting after %d ms",
                 WIFI_SERVICE_CONNECT_TIMEOUT_MS);
    }

    vTaskDelete(NULL);
}

esp_err_t wifi_service_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_status = WIFI_SERVICE_STATUS_CONNECTING;
    const BaseType_t created = xTaskCreatePinnedToCore(
        wifi_service_task, "wifi_service", WIFI_SERVICE_TASK_STACK_WORDS, NULL,
        WIFI_SERVICE_TASK_PRIORITY, NULL, 0);
    if (created != pdPASS) {
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
        s_status = WIFI_SERVICE_STATUS_FAILED;
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    return ESP_OK;
}

bool wifi_service_is_connected(void)
{
    return s_connected;
}

enum wifi_service_status wifi_service_get_status(void)
{
    return s_status;
}

const char *wifi_service_get_ip_address(void)
{
    return s_ip_address;
}
