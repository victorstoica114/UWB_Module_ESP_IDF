#include "wifi_service.h"

#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#include "app_config.h"
#include "app_identity.h"

#ifndef APP_WIFI_SSID
#define APP_WIFI_SSID ""
#endif

#ifndef APP_WIFI_PASSWORD
#define APP_WIFI_PASSWORD ""
#endif

#if APP_WIFI_SCAN_MAX_APS < 1
#error APP_WIFI_SCAN_MAX_APS must be at least 1
#endif

static const char *TAG = "wifi_service";

enum {
    WIFI_SERVICE_TASK_STACK_WORDS = 5120,
    WIFI_SERVICE_TASK_PRIORITY = 6,
    WIFI_SERVICE_BSSID_TEXT_LEN = 18,
};

#define WIFI_SERVICE_CONNECTED_BIT BIT0
#define WIFI_SERVICE_FAIL_BIT BIT1
#define WIFI_SERVICE_STARTED_BIT BIT2
#define WIFI_SERVICE_RECONNECT_BIT BIT3

static EventGroupHandle_t s_wifi_event_group;
static esp_netif_t *s_sta_netif;
static bool s_started;
static volatile bool s_connected;
static volatile enum wifi_service_status s_status = WIFI_SERVICE_STATUS_IDLE;
static uint32_t s_retry_count;
static uint32_t s_connect_attempt_count;
static uint32_t s_disconnect_count;
static uint16_t s_last_disconnect_reason;
static char s_ip_address[16] = "0.0.0.0";
static char s_connected_bssid[WIFI_SERVICE_BSSID_TEXT_LEN] = "";
static uint8_t s_connected_channel;
static int s_connected_rssi;
static uint16_t s_last_scan_ap_count;
static char s_last_scan_best_bssid[WIFI_SERVICE_BSSID_TEXT_LEN] = "";
static uint8_t s_last_scan_best_channel;
static int s_last_scan_best_rssi;
static wifi_ap_record_t s_scan_records[APP_WIFI_SCAN_MAX_APS];

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

static void format_bssid(char *destination, size_t destination_size,
                         const uint8_t bssid[6])
{
    snprintf(destination, destination_size, "%02x:%02x:%02x:%02x:%02x:%02x",
             bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
}

static bool parse_bssid_string(const char *text, uint8_t bssid[6])
{
    unsigned int values[6];
    if (sscanf(text, "%2x:%2x:%2x:%2x:%2x:%2x", &values[0], &values[1],
               &values[2], &values[3], &values[4], &values[5]) != 6) {
        return false;
    }

    for (size_t i = 0; i < 6; ++i) {
        if (values[i] > 0xFFU) {
            return false;
        }
        bssid[i] = (uint8_t)values[i];
    }

    return true;
}

static const char *wifi_auth_mode_name(wifi_auth_mode_t authmode)
{
    switch (authmode) {
    case WIFI_AUTH_OPEN:
        return "OPEN";
    case WIFI_AUTH_WEP:
        return "WEP";
    case WIFI_AUTH_WPA_PSK:
        return "WPA_PSK";
    case WIFI_AUTH_WPA2_PSK:
        return "WPA2_PSK";
    case WIFI_AUTH_WPA_WPA2_PSK:
        return "WPA_WPA2_PSK";
    case WIFI_AUTH_ENTERPRISE:
        return "WPA2_ENTERPRISE";
    case WIFI_AUTH_WPA3_PSK:
        return "WPA3_PSK";
    case WIFI_AUTH_WPA2_WPA3_PSK:
        return "WPA2_WPA3_PSK";
    case WIFI_AUTH_WAPI_PSK:
        return "WAPI_PSK";
    case WIFI_AUTH_OWE:
        return "OWE";
    case WIFI_AUTH_WPA3_ENT_192:
        return "WPA3_ENT_192";
    case WIFI_AUTH_DPP:
        return "DPP";
    case WIFI_AUTH_WPA3_ENTERPRISE:
        return "WPA3_ENTERPRISE";
    case WIFI_AUTH_WPA2_WPA3_ENTERPRISE:
        return "WPA2_WPA3_ENTERPRISE";
    case WIFI_AUTH_WPA_ENTERPRISE:
        return "WPA_ENTERPRISE";
    default:
        return "UNKNOWN";
    }
}

static const char *wifi_cipher_name(wifi_cipher_type_t cipher)
{
    switch (cipher) {
    case WIFI_CIPHER_TYPE_NONE:
        return "NONE";
    case WIFI_CIPHER_TYPE_WEP40:
        return "WEP40";
    case WIFI_CIPHER_TYPE_WEP104:
        return "WEP104";
    case WIFI_CIPHER_TYPE_TKIP:
        return "TKIP";
    case WIFI_CIPHER_TYPE_CCMP:
        return "CCMP";
    case WIFI_CIPHER_TYPE_TKIP_CCMP:
        return "TKIP_CCMP";
    case WIFI_CIPHER_TYPE_AES_CMAC128:
        return "AES_CMAC128";
    case WIFI_CIPHER_TYPE_SMS4:
        return "SMS4";
    case WIFI_CIPHER_TYPE_GCMP:
        return "GCMP";
    case WIFI_CIPHER_TYPE_GCMP256:
        return "GCMP256";
    case WIFI_CIPHER_TYPE_AES_GMAC128:
        return "AES_GMAC128";
    case WIFI_CIPHER_TYPE_AES_GMAC256:
        return "AES_GMAC256";
    case WIFI_CIPHER_TYPE_UNKNOWN:
        return "UNKNOWN";
    default:
        return "UNKNOWN";
    }
}

static const char *wifi_second_channel_name(wifi_second_chan_t second)
{
    switch (second) {
    case WIFI_SECOND_CHAN_NONE:
        return "HT20";
    case WIFI_SECOND_CHAN_ABOVE:
        return "HT40_ABOVE";
    case WIFI_SECOND_CHAN_BELOW:
        return "HT40_BELOW";
    default:
        return "UNKNOWN";
    }
}

static const char *wifi_bandwidth_name(wifi_bandwidth_t bandwidth)
{
    switch (bandwidth) {
    case WIFI_BW20:
        return "20";
    case WIFI_BW40:
        return "40";
    case WIFI_BW80:
        return "80";
    case WIFI_BW160:
        return "160";
    case WIFI_BW80_BW80:
        return "80+80";
    default:
        return "unknown";
    }
}

static const char *wifi_reason_name(uint16_t reason)
{
    switch (reason) {
    case WIFI_REASON_UNSPECIFIED:
        return "UNSPECIFIED";
    case WIFI_REASON_AUTH_EXPIRE:
        return "AUTH_EXPIRE";
    case WIFI_REASON_AUTH_LEAVE:
        return "AUTH_LEAVE";
    case WIFI_REASON_DISASSOC_DUE_TO_INACTIVITY:
        return "DISASSOC_DUE_TO_INACTIVITY";
    case WIFI_REASON_ASSOC_TOOMANY:
        return "ASSOC_TOOMANY";
    case WIFI_REASON_CLASS2_FRAME_FROM_NONAUTH_STA:
        return "CLASS2_FRAME_FROM_NONAUTH_STA";
    case WIFI_REASON_CLASS3_FRAME_FROM_NONASSOC_STA:
        return "CLASS3_FRAME_FROM_NONASSOC_STA";
    case WIFI_REASON_ASSOC_LEAVE:
        return "ASSOC_LEAVE";
    case WIFI_REASON_ASSOC_NOT_AUTHED:
        return "ASSOC_NOT_AUTHED";
    case WIFI_REASON_DISASSOC_PWRCAP_BAD:
        return "DISASSOC_PWRCAP_BAD";
    case WIFI_REASON_DISASSOC_SUPCHAN_BAD:
        return "DISASSOC_SUPCHAN_BAD";
    case WIFI_REASON_BSS_TRANSITION_DISASSOC:
        return "BSS_TRANSITION_DISASSOC";
    case WIFI_REASON_IE_INVALID:
        return "IE_INVALID";
    case WIFI_REASON_MIC_FAILURE:
        return "MIC_FAILURE";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        return "4WAY_HANDSHAKE_TIMEOUT";
    case WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT:
        return "GROUP_KEY_UPDATE_TIMEOUT";
    case WIFI_REASON_IE_IN_4WAY_DIFFERS:
        return "IE_IN_4WAY_DIFFERS";
    case WIFI_REASON_GROUP_CIPHER_INVALID:
        return "GROUP_CIPHER_INVALID";
    case WIFI_REASON_PAIRWISE_CIPHER_INVALID:
        return "PAIRWISE_CIPHER_INVALID";
    case WIFI_REASON_AKMP_INVALID:
        return "AKMP_INVALID";
    case WIFI_REASON_UNSUPP_RSN_IE_VERSION:
        return "UNSUPP_RSN_IE_VERSION";
    case WIFI_REASON_INVALID_RSN_IE_CAP:
        return "INVALID_RSN_IE_CAP";
    case WIFI_REASON_802_1X_AUTH_FAILED:
        return "802_1X_AUTH_FAILED";
    case WIFI_REASON_CIPHER_SUITE_REJECTED:
        return "CIPHER_SUITE_REJECTED";
    case WIFI_REASON_TIMEOUT:
        return "TIMEOUT";
    case WIFI_REASON_PEER_INITIATED:
        return "PEER_INITIATED";
    case WIFI_REASON_AP_INITIATED:
        return "AP_INITIATED";
    case WIFI_REASON_INVALID_PMKID:
        return "INVALID_PMKID";
    case WIFI_REASON_BEACON_TIMEOUT:
        return "BEACON_TIMEOUT";
    case WIFI_REASON_NO_AP_FOUND:
        return "NO_AP_FOUND";
    case WIFI_REASON_AUTH_FAIL:
        return "AUTH_FAIL";
    case WIFI_REASON_ASSOC_FAIL:
        return "ASSOC_FAIL";
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
        return "HANDSHAKE_TIMEOUT";
    case WIFI_REASON_CONNECTION_FAIL:
        return "CONNECTION_FAIL";
    case WIFI_REASON_AP_TSF_RESET:
        return "AP_TSF_RESET";
    case WIFI_REASON_ROAMING:
        return "ROAMING";
    case WIFI_REASON_ASSOC_COMEBACK_TIME_TOO_LONG:
        return "ASSOC_COMEBACK_TIME_TOO_LONG";
    case WIFI_REASON_SA_QUERY_TIMEOUT:
        return "SA_QUERY_TIMEOUT";
    case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY:
        return "NO_AP_FOUND_W_COMPATIBLE_SECURITY";
    case WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD:
        return "NO_AP_FOUND_IN_AUTHMODE_THRESHOLD";
    case WIFI_REASON_NO_AP_FOUND_IN_RSSI_THRESHOLD:
        return "NO_AP_FOUND_IN_RSSI_THRESHOLD";
    default:
        return "UNKNOWN";
    }
}

static wifi_auth_mode_t wifi_auth_threshold(void)
{
    if (APP_WIFI_USE_WPA3) {
        return WIFI_AUTH_WPA2_WPA3_PSK;
    }

    return APP_WIFI_ALLOW_WPA_COMPAT ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_WPA2_PSK;
}

static uint32_t wifi_reconnect_delay_ms(void)
{
    uint32_t delay_ms = APP_WIFI_RECONNECT_BASE_DELAY_MS;
    const uint32_t max_delay_ms = APP_WIFI_RECONNECT_MAX_DELAY_MS;
    const uint32_t retry_index = s_retry_count > 0 ? s_retry_count - 1 : 0;

    for (uint32_t i = 0; i < retry_index && delay_ms < max_delay_ms; ++i) {
        if (delay_ms > max_delay_ms / 2U) {
            delay_ms = max_delay_ms;
            break;
        }
        delay_ms *= 2U;
    }

    return delay_ms > max_delay_ms ? max_delay_ms : delay_ms;
}

static void wifi_service_log_connected_ap(void)
{
    wifi_ap_record_t ap = {0};
    const esp_err_t err = esp_wifi_sta_get_ap_info(&ap);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not read connected AP info: %s",
                 esp_err_to_name(err));
        return;
    }

    format_bssid(s_connected_bssid, sizeof(s_connected_bssid), ap.bssid);
    s_connected_channel = ap.primary;
    s_connected_rssi = ap.rssi;

    ESP_LOGI(TAG,
             "Connected AP bssid=%s channel=%u rssi=%d auth=%s pairwise=%s "
             "group=%s second=%s",
             s_connected_bssid, (unsigned)ap.primary, (int)ap.rssi,
             wifi_auth_mode_name(ap.authmode),
             wifi_cipher_name(ap.pairwise_cipher),
             wifi_cipher_name(ap.group_cipher),
             wifi_second_channel_name(ap.second));
}

static void wifi_service_connect_once(const char *reason)
{
    s_status = WIFI_SERVICE_STATUS_CONNECTING;
    s_connect_attempt_count++;

    ESP_LOGI(TAG, "Connecting to Wi-Fi SSID '%s' (%s, attempt %lu)",
             APP_WIFI_SSID, reason, (unsigned long)s_connect_attempt_count);

    const esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(err));
        xEventGroupSetBits(s_wifi_event_group, WIFI_SERVICE_RECONNECT_BIT);
    }
}

static void wifi_service_configure_ap_lock(wifi_config_t *wifi_config)
{
    if (APP_WIFI_LOCK_CHANNEL > 0) {
        wifi_config->sta.channel = APP_WIFI_LOCK_CHANNEL;
        ESP_LOGI(TAG, "Wi-Fi channel hint enabled: %u",
                 (unsigned)APP_WIFI_LOCK_CHANNEL);
    }

    if (APP_WIFI_LOCK_BSSID) {
        uint8_t bssid[6] = {0};
        if (!parse_bssid_string(APP_WIFI_BSSID, bssid)) {
            ESP_LOGW(TAG,
                     "APP_WIFI_LOCK_BSSID is enabled but APP_WIFI_BSSID is "
                     "invalid: '%s'",
                     APP_WIFI_BSSID);
            return;
        }

        wifi_config->sta.bssid_set = true;
        memcpy(wifi_config->sta.bssid, bssid, sizeof(wifi_config->sta.bssid));

        char bssid_text[WIFI_SERVICE_BSSID_TEXT_LEN];
        format_bssid(bssid_text, sizeof(bssid_text), bssid);
        ESP_LOGI(TAG, "Wi-Fi BSSID lock enabled: %s", bssid_text);
    }
}

static esp_err_t wifi_service_scan_and_log(void)
{
    if (!APP_WIFI_SCAN_BEFORE_CONNECT) {
        return ESP_OK;
    }

    s_last_scan_ap_count = 0;
    s_last_scan_best_bssid[0] = '\0';
    s_last_scan_best_channel = 0;
    s_last_scan_best_rssi = 0;

    wifi_scan_config_t scan_config = {0};
    scan_config.ssid = (uint8_t *)APP_WIFI_SSID;
    scan_config.channel = APP_WIFI_SCAN_CHANNEL;
    scan_config.show_hidden = true;
    scan_config.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    scan_config.scan_time.active.min = APP_WIFI_SCAN_ACTIVE_MIN_MS;
    scan_config.scan_time.active.max = APP_WIFI_SCAN_ACTIVE_MAX_MS;

    ESP_LOGI(TAG, "Scanning for Wi-Fi SSID '%s' on channel %u", APP_WIFI_SSID,
             (unsigned)APP_WIFI_SCAN_CHANNEL);

    esp_err_t err = esp_wifi_scan_start(&scan_config, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Wi-Fi scan failed: %s", esp_err_to_name(err));
        return err;
    }

    uint16_t ap_count = 0;
    err = esp_wifi_scan_get_ap_num(&ap_count);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Wi-Fi scan count failed: %s", esp_err_to_name(err));
        return err;
    }

    s_last_scan_ap_count = ap_count;
    uint16_t record_count = ap_count;
    if (record_count > APP_WIFI_SCAN_MAX_APS) {
        record_count = APP_WIFI_SCAN_MAX_APS;
    }

    ESP_LOGI(TAG, "Wi-Fi scan found %u AP(s), logging %u", (unsigned)ap_count,
             (unsigned)record_count);

    if (record_count == 0) {
        return ESP_OK;
    }

    err = esp_wifi_scan_get_ap_records(&record_count, s_scan_records);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Wi-Fi scan record read failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    int best_index = -1;
    for (uint16_t i = 0; i < record_count; ++i) {
        if (best_index < 0 || s_scan_records[i].rssi >
                                  s_scan_records[best_index].rssi) {
            best_index = i;
        }
    }

    for (uint16_t i = 0; i < record_count; ++i) {
        char bssid_text[WIFI_SERVICE_BSSID_TEXT_LEN];
        const wifi_ap_record_t *ap = &s_scan_records[i];
        format_bssid(bssid_text, sizeof(bssid_text), ap->bssid);

        ESP_LOGI(TAG,
                 "AP[%u] bssid=%s channel=%u rssi=%d auth=%s pairwise=%s "
                 "group=%s second=%s phy=%s%s%s%s%s%s%s bw=%s country=%c%c%s",
                 (unsigned)i, bssid_text, (unsigned)ap->primary,
                 (int)ap->rssi, wifi_auth_mode_name(ap->authmode),
                 wifi_cipher_name(ap->pairwise_cipher),
                 wifi_cipher_name(ap->group_cipher),
                 wifi_second_channel_name(ap->second),
                 ap->phy_11b ? "b" : "",
                 ap->phy_11g ? "g" : "",
                 ap->phy_11n ? "n" : "",
                 ap->phy_lr ? "lr" : "",
                 ap->phy_11a ? "a" : "",
                 ap->phy_11ac ? "ac" : "",
                 ap->phy_11ax ? "ax" : "",
                 wifi_bandwidth_name(ap->bandwidth),
                 ap->country.cc[0] != '\0' ? ap->country.cc[0] : '-',
                 ap->country.cc[1] != '\0' ? ap->country.cc[1] : '-',
                 (int)i == best_index ? " best_rssi" : "");
    }

    if (best_index >= 0) {
        const wifi_ap_record_t *best = &s_scan_records[best_index];
        format_bssid(s_last_scan_best_bssid,
                     sizeof(s_last_scan_best_bssid), best->bssid);
        s_last_scan_best_channel = best->primary;
        s_last_scan_best_rssi = best->rssi;
    }

    if (ap_count > 1 && !APP_WIFI_LOCK_BSSID) {
        ESP_LOGW(TAG,
                 "Multiple BSSIDs found for this SSID. If connection stays "
                 "unstable, set APP_WIFI_LOCK_BSSID, APP_WIFI_BSSID, and "
                 "APP_WIFI_LOCK_CHANNEL in app_config.h");
    }

    return ESP_OK;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        s_status = WIFI_SERVICE_STATUS_CONNECTING;
        ESP_LOGI(TAG, "Wi-Fi driver started");
        xEventGroupSetBits(s_wifi_event_group, WIFI_SERVICE_STARTED_BIT);
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *event =
            (const wifi_event_sta_disconnected_t *)event_data;
        const uint16_t reason = event != NULL ? event->reason : 0;

        s_connected = false;
        s_status = WIFI_SERVICE_STATUS_CONNECTING;
        strcpy(s_ip_address, "0.0.0.0");
        s_connected_bssid[0] = '\0';
        s_connected_channel = 0;
        s_connected_rssi = 0;
        s_disconnect_count++;
        s_last_disconnect_reason = reason;
        xEventGroupClearBits(s_wifi_event_group, WIFI_SERVICE_CONNECTED_BIT);

        const bool can_retry =
            APP_WIFI_RECONNECT_FOREVER || s_retry_count < APP_WIFI_MAX_RETRIES;
        if (can_retry) {
            s_retry_count++;
            ESP_LOGW(TAG,
                     "Disconnected, reason=%u (%s), disconnects=%lu, next "
                     "retry in %lu ms",
                     (unsigned)reason, wifi_reason_name(reason),
                     (unsigned long)s_disconnect_count,
                     (unsigned long)wifi_reconnect_delay_ms());
            xEventGroupSetBits(s_wifi_event_group, WIFI_SERVICE_RECONNECT_BIT);
        } else {
            s_status = WIFI_SERVICE_STATUS_FAILED;
            ESP_LOGE(TAG, "Wi-Fi connection failed after %lu retries",
                     (unsigned long)s_retry_count);
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
        wifi_service_log_connected_ap();
        xEventGroupSetBits(s_wifi_event_group, WIFI_SERVICE_CONNECTED_BIT);
    }
}

static esp_err_t wifi_service_init_nvs(void)
{
    return app_identity_init();
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

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_set_hostname(
        s_sta_netif, app_identity_get_hostname()));

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

    if (APP_WIFI_FORCE_HT20) {
        ESP_RETURN_ON_ERROR(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW20),
                            TAG, "esp_wifi_set_bandwidth failed");
        ESP_LOGI(TAG, "Wi-Fi STA bandwidth forced to HT20");
    }

    wifi_config_t wifi_config = {0};
    copy_wifi_string(wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid),
                     APP_WIFI_SSID);
    copy_wifi_string(wifi_config.sta.password,
                     sizeof(wifi_config.sta.password), APP_WIFI_PASSWORD);
    wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wifi_config.sta.threshold.authmode = wifi_auth_threshold();
    wifi_config.sta.pmf_cfg.capable = APP_WIFI_PMF_CAPABLE != 0;
    wifi_config.sta.pmf_cfg.required = false;
    wifi_config.sta.disable_wpa3_compatible_mode =
        APP_WIFI_DISABLE_WPA3_COMPATIBLE_MODE != 0;
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
    wifi_config.sta.sae_pk_mode = WPA3_SAE_PK_MODE_DISABLED;
    wifi_config.sta.failure_retry_cnt = APP_WIFI_FAILURE_RETRY_COUNT;
    wifi_service_configure_ap_lock(&wifi_config);

    ESP_LOGI(TAG,
             "Wi-Fi config: threshold=%s pmf_capable=%d wpa3=%d "
             "wpa_compat=%d wpa3_compatible_mode=%d internal_retry=%u",
             wifi_auth_mode_name(wifi_config.sta.threshold.authmode),
             wifi_config.sta.pmf_cfg.capable ? 1 : 0, APP_WIFI_USE_WPA3,
             APP_WIFI_ALLOW_WPA_COMPAT,
             wifi_config.sta.disable_wpa3_compatible_mode ? 0 : 1,
             (unsigned)wifi_config.sta.failure_retry_cnt);

    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG,
                        "esp_wifi_set_config failed");

    if (APP_WIFI_SET_PROTOCOL) {
        uint8_t protocol = WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
        if (APP_WIFI_ENABLE_11B) {
            protocol |= WIFI_PROTOCOL_11B;
        }
        err = esp_wifi_set_protocol(WIFI_IF_STA, protocol);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Wi-Fi STA protocol: %s%s",
                     APP_WIFI_ENABLE_11B ? "11b/" : "", "11g/11n");
        } else {
            ESP_LOGW(TAG, "esp_wifi_set_protocol failed: %s",
                     esp_err_to_name(err));
        }
    }

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

    ESP_LOGI(TAG, "Wi-Fi hostname: %s", app_identity_get_hostname());
    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(err));
        s_status = WIFI_SERVICE_STATUS_FAILED;
        vTaskDelete(NULL);
        return;
    }

    const EventBits_t start_bits = xEventGroupWaitBits(
        s_wifi_event_group, WIFI_SERVICE_STARTED_BIT | WIFI_SERVICE_FAIL_BIT,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(3000));
    if ((start_bits & WIFI_SERVICE_STARTED_BIT) == 0) {
        ESP_LOGW(TAG, "Wi-Fi start event did not arrive before scan");
    }

    (void)wifi_service_scan_and_log();
    wifi_service_connect_once("initial");

    const EventBits_t initial_bits = xEventGroupWaitBits(
        s_wifi_event_group, WIFI_SERVICE_CONNECTED_BIT | WIFI_SERVICE_FAIL_BIT,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(APP_WIFI_CONNECT_TIMEOUT_MS));

    if ((initial_bits & WIFI_SERVICE_CONNECTED_BIT) != 0) {
        ESP_LOGI(TAG, "Wi-Fi ready");
    } else if ((initial_bits & WIFI_SERVICE_FAIL_BIT) != 0) {
        ESP_LOGE(TAG, "Wi-Fi failed");
    } else {
        ESP_LOGW(TAG, "Wi-Fi still connecting after %d ms",
                 APP_WIFI_CONNECT_TIMEOUT_MS);
    }

    while (true) {
        const EventBits_t bits = xEventGroupWaitBits(
            s_wifi_event_group, WIFI_SERVICE_RECONNECT_BIT |
                                    WIFI_SERVICE_FAIL_BIT,
            pdTRUE, pdFALSE, portMAX_DELAY);

        if ((bits & WIFI_SERVICE_FAIL_BIT) != 0) {
            break;
        }

        if ((bits & WIFI_SERVICE_RECONNECT_BIT) == 0) {
            continue;
        }

        const uint32_t delay_ms = wifi_reconnect_delay_ms();
        if (delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }

        if (s_connected) {
            continue;
        }

#if APP_WIFI_SCAN_ON_RECONNECT_EVERY > 0
        if (s_disconnect_count % APP_WIFI_SCAN_ON_RECONNECT_EVERY == 0) {
            (void)wifi_service_scan_and_log();
        }
#endif

        wifi_service_connect_once("reconnect");
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

uint32_t wifi_service_get_disconnect_count(void)
{
    return s_disconnect_count;
}

uint16_t wifi_service_get_last_disconnect_reason(void)
{
    return s_last_disconnect_reason;
}

const char *wifi_service_get_last_disconnect_reason_name(void)
{
    return wifi_reason_name(s_last_disconnect_reason);
}

const char *wifi_service_get_connected_bssid(void)
{
    return s_connected_bssid;
}

uint8_t wifi_service_get_connected_channel(void)
{
    return s_connected_channel;
}

int wifi_service_get_connected_rssi(void)
{
    return s_connected_rssi;
}

uint16_t wifi_service_get_last_scan_ap_count(void)
{
    return s_last_scan_ap_count;
}

const char *wifi_service_get_last_scan_best_bssid(void)
{
    return s_last_scan_best_bssid;
}

uint8_t wifi_service_get_last_scan_best_channel(void)
{
    return s_last_scan_best_channel;
}

int wifi_service_get_last_scan_best_rssi(void)
{
    return s_last_scan_best_rssi;
}
