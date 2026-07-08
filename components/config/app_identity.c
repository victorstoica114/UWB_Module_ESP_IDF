#include "app_identity.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "uwb_config.h"

#ifndef HOSTNAME
#define HOSTNAME "uwb-module"
#endif

static const char *TAG = "app_identity";

enum {
    APP_IDENTITY_HOSTNAME_MAX = 32,
};

#define APP_IDENTITY_NVS_NAMESPACE "identity"
#define APP_IDENTITY_NVS_KEY_MODULE_ID "module_id"
#define APP_IDENTITY_NVS_KEY_UWB_ROLE "uwb_role"
#define APP_IDENTITY_NVS_KEY_UWB_ANTENNA_DELAY "ant_delay"

static bool s_initialized;
static bool s_module_id_from_nvs;
static bool s_module_id_provisioned_this_boot;
static bool s_uwb_role_from_nvs;
static bool s_uwb_role_provisioned_this_boot;
static bool s_uwb_antenna_delay_from_nvs;
static uint8_t s_module_id = 1;
static uint8_t s_uwb_role = APP_UWB_ROLE_UNSET;
static uint16_t s_uwb_antenna_delay = APP_UWB_ANTENNA_DELAY_DEFAULT;
static char s_hostname[APP_IDENTITY_HOSTNAME_MAX] = "uwb-module-1";

static bool module_id_valid(uint32_t module_id)
{
    return module_id > 0 && module_id <= 255;
}

static bool uwb_role_valid(uint32_t role)
{
    return role == APP_UWB_ROLE_UNSET || role == APP_UWB_ROLE_ANCHOR ||
           role == APP_UWB_ROLE_TAG ||
           role == APP_UWB_ROLE_DISTANCE_TEST_NODE;
}

static bool uwb_role_valid_for_provisioning(uint32_t role)
{
    return role == APP_UWB_ROLE_ANCHOR || role == APP_UWB_ROLE_TAG ||
           role == APP_UWB_ROLE_DISTANCE_TEST_NODE;
}

static bool antenna_delay_valid(uint32_t delay)
{
    return delay <= 0xFFFFU;
}

static uint8_t parse_hostname_id(const char *hostname)
{
    uint16_t value = 0;
    bool has_digit = false;

    if (hostname == NULL) {
        return 0;
    }

    for (size_t i = 0; hostname[i] != '\0'; ++i) {
        const char c = hostname[i];
        if (c >= '0' && c <= '9') {
            has_digit = true;
            value = (uint16_t)(value * 10U + (uint16_t)(c - '0'));
            if (value > 255U) {
                return 0;
            }
        } else {
            has_digit = false;
            value = 0;
        }
    }

    return has_digit && value > 0 ? (uint8_t)value : 0;
}

static esp_err_t init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    return err;
}

static uint8_t fallback_module_id(void)
{
    if (module_id_valid(APP_IDENTITY_DEFAULT_MODULE_ID)) {
        return (uint8_t)APP_IDENTITY_DEFAULT_MODULE_ID;
    }

    const uint8_t hostname_id = parse_hostname_id(HOSTNAME);
    if (hostname_id != 0) {
        return hostname_id;
    }

    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK && mac[5] != 0) {
        return mac[5];
    }

    return 1;
}

static void update_hostname(void)
{
    snprintf(s_hostname, sizeof(s_hostname), "%s%u",
             APP_IDENTITY_HOSTNAME_PREFIX, (unsigned)s_module_id);
}

static esp_err_t read_u8_from_nvs(const char *key, uint8_t *value)
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(APP_IDENTITY_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_get_u8(handle, key, value);
    nvs_close(handle);
    return err;
}

static esp_err_t read_u16_from_nvs(const char *key, uint16_t *value)
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(APP_IDENTITY_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_get_u16(handle, key, value);
    nvs_close(handle);
    return err;
}

static esp_err_t write_u8_to_nvs(const char *key, uint8_t value)
{
    nvs_handle_t handle = 0;
    ESP_RETURN_ON_ERROR(
        nvs_open(APP_IDENTITY_NVS_NAMESPACE, NVS_READWRITE, &handle), TAG,
        "open identity NVS failed");

    esp_err_t err = nvs_set_u8(handle, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}

static esp_err_t write_u16_to_nvs(const char *key, uint16_t value)
{
    nvs_handle_t handle = 0;
    ESP_RETURN_ON_ERROR(
        nvs_open(APP_IDENTITY_NVS_NAMESPACE, NVS_READWRITE, &handle), TAG,
        "open identity NVS failed");

    esp_err_t err = nvs_set_u16(handle, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}

static esp_err_t erase_key_from_nvs(const char *key)
{
    nvs_handle_t handle = 0;
    ESP_RETURN_ON_ERROR(
        nvs_open(APP_IDENTITY_NVS_NAMESPACE, NVS_READWRITE, &handle), TAG,
        "open identity NVS failed");

    esp_err_t err = nvs_erase_key(handle, key);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}

static uint8_t fallback_uwb_role(void)
{
    return uwb_role_valid(APP_UWB_ROLE) ? (uint8_t)APP_UWB_ROLE
                                        : APP_UWB_ROLE_UNSET;
}

esp_err_t app_identity_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(init_nvs(), TAG, "NVS init failed");

    if (APP_IDENTITY_PROVISION_ENABLED) {
        if (!module_id_valid(APP_IDENTITY_PROVISION_MODULE_ID)) {
            ESP_LOGE(TAG,
                     "Invalid APP_IDENTITY_PROVISION_MODULE_ID=%u; valid range is 1..255",
                     (unsigned)APP_IDENTITY_PROVISION_MODULE_ID);
            return ESP_ERR_INVALID_ARG;
        }

        s_module_id = (uint8_t)APP_IDENTITY_PROVISION_MODULE_ID;
        ESP_RETURN_ON_ERROR(
            write_u8_to_nvs(APP_IDENTITY_NVS_KEY_MODULE_ID, s_module_id), TAG,
            "module ID provisioning write failed");
        s_module_id_from_nvs = true;
        s_module_id_provisioned_this_boot = true;
        ESP_LOGW(TAG, "Provisioned persistent module_id=%u",
                 (unsigned)s_module_id);
    } else {
        uint8_t stored_module_id = 0;
        const esp_err_t read_err =
            read_u8_from_nvs(APP_IDENTITY_NVS_KEY_MODULE_ID,
                             &stored_module_id);
        if (read_err == ESP_OK && module_id_valid(stored_module_id)) {
            s_module_id = stored_module_id;
            s_module_id_from_nvs = true;
        } else {
            s_module_id = fallback_module_id();
            s_module_id_from_nvs = false;
            ESP_LOGW(TAG,
                     "No persistent module_id in NVS; using fallback module_id=%u",
                     (unsigned)s_module_id);
        }
    }

    if (APP_IDENTITY_PROVISION_UWB_ROLE_ENABLED) {
        if (!uwb_role_valid_for_provisioning(
                APP_IDENTITY_PROVISION_UWB_ROLE)) {
            ESP_LOGE(TAG,
                     "Invalid APP_IDENTITY_PROVISION_UWB_ROLE=%u; use anchor=%u, tag=%u, or distance_test_node=%u",
                     (unsigned)APP_IDENTITY_PROVISION_UWB_ROLE,
                     (unsigned)APP_UWB_ROLE_ANCHOR,
                     (unsigned)APP_UWB_ROLE_TAG,
                     (unsigned)APP_UWB_ROLE_DISTANCE_TEST_NODE);
            return ESP_ERR_INVALID_ARG;
        }

        s_uwb_role = (uint8_t)APP_IDENTITY_PROVISION_UWB_ROLE;
        ESP_RETURN_ON_ERROR(
            write_u8_to_nvs(APP_IDENTITY_NVS_KEY_UWB_ROLE, s_uwb_role), TAG,
            "UWB role provisioning write failed");
        s_uwb_role_from_nvs = true;
        s_uwb_role_provisioned_this_boot = true;
        ESP_LOGW(TAG, "Provisioned persistent UWB role=%s(%u)",
                 app_identity_uwb_role_to_string(s_uwb_role),
                 (unsigned)s_uwb_role);
    } else {
        uint8_t stored_uwb_role = APP_UWB_ROLE_UNSET;
        const esp_err_t read_err =
            read_u8_from_nvs(APP_IDENTITY_NVS_KEY_UWB_ROLE,
                             &stored_uwb_role);
        if (read_err == ESP_OK && uwb_role_valid(stored_uwb_role)) {
            s_uwb_role = stored_uwb_role;
            s_uwb_role_from_nvs = true;
        } else {
            s_uwb_role = fallback_uwb_role();
            s_uwb_role_from_nvs = false;
            ESP_LOGW(TAG,
                     "No persistent UWB role in NVS; using fallback role=%s(%u)",
                     app_identity_uwb_role_to_string(s_uwb_role),
                     (unsigned)s_uwb_role);
        }
    }

    uint16_t stored_antenna_delay = 0;
    const esp_err_t delay_read_err = read_u16_from_nvs(
        APP_IDENTITY_NVS_KEY_UWB_ANTENNA_DELAY, &stored_antenna_delay);
    if (delay_read_err == ESP_OK &&
        antenna_delay_valid(stored_antenna_delay)) {
        s_uwb_antenna_delay = stored_antenna_delay;
        s_uwb_antenna_delay_from_nvs = true;
    } else {
        s_uwb_antenna_delay = (uint16_t)APP_UWB_ANTENNA_DELAY_DEFAULT;
        s_uwb_antenna_delay_from_nvs = false;
        ESP_LOGW(TAG,
                 "No persistent UWB antenna delay in NVS; using fallback delay=0x%04x",
                 (unsigned)s_uwb_antenna_delay);
    }

    update_hostname();
    s_initialized = true;

    ESP_LOGI(TAG,
             "Identity ready: hostname=%s module_id=%u module_source=%s uwb_role=%s(%u) role_source=%s antenna_delay=0x%04x antenna_delay_source=%s",
             s_hostname, (unsigned)s_module_id,
             s_module_id_from_nvs ? "nvs" : "fallback",
             app_identity_uwb_role_to_string(s_uwb_role),
             (unsigned)s_uwb_role, s_uwb_role_from_nvs ? "nvs" : "fallback",
             (unsigned)s_uwb_antenna_delay,
             s_uwb_antenna_delay_from_nvs ? "nvs" : "fallback");
    return ESP_OK;
}

const char *app_identity_get_hostname(void)
{
    if (!s_initialized) {
        (void)app_identity_init();
    }

    return s_hostname;
}

uint8_t app_identity_get_module_id(void)
{
    if (!s_initialized) {
        (void)app_identity_init();
    }

    return s_module_id;
}

uint8_t app_identity_get_uwb_role(void)
{
    if (!s_initialized) {
        (void)app_identity_init();
    }

    return s_uwb_role;
}

uint16_t app_identity_get_uwb_antenna_delay(void)
{
    if (!s_initialized) {
        (void)app_identity_init();
    }

    return s_uwb_antenna_delay;
}

const char *app_identity_uwb_role_to_string(uint8_t role)
{
    switch (role) {
    case APP_UWB_ROLE_UNSET:
        return "unset";
    case APP_UWB_ROLE_ANCHOR:
        return "anchor";
    case APP_UWB_ROLE_TAG:
        return "tag";
    case APP_UWB_ROLE_DISTANCE_TEST_NODE:
        return "distance_test_node";
    default:
        return "unknown";
    }
}

bool app_identity_module_id_from_nvs(void)
{
    return s_module_id_from_nvs;
}

bool app_identity_module_id_provisioned_this_boot(void)
{
    return s_module_id_provisioned_this_boot;
}

bool app_identity_uwb_role_from_nvs(void)
{
    return s_uwb_role_from_nvs;
}

bool app_identity_uwb_role_provisioned_this_boot(void)
{
    return s_uwb_role_provisioned_this_boot;
}

bool app_identity_uwb_antenna_delay_from_nvs(void)
{
    return s_uwb_antenna_delay_from_nvs;
}

esp_err_t app_identity_set_uwb_antenna_delay(uint16_t delay)
{
    if (!s_initialized) {
        ESP_RETURN_ON_ERROR(app_identity_init(), TAG,
                            "identity init failed before delay write");
    }
    if (!antenna_delay_valid(delay)) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(
        write_u16_to_nvs(APP_IDENTITY_NVS_KEY_UWB_ANTENNA_DELAY, delay), TAG,
        "UWB antenna delay write failed");

    s_uwb_antenna_delay = delay;
    s_uwb_antenna_delay_from_nvs = true;
    ESP_LOGW(TAG, "Configured persistent UWB antenna delay=0x%04x",
             (unsigned)s_uwb_antenna_delay);
    return ESP_OK;
}

esp_err_t app_identity_clear_uwb_antenna_delay(void)
{
    if (!s_initialized) {
        ESP_RETURN_ON_ERROR(app_identity_init(), TAG,
                            "identity init failed before delay clear");
    }

    ESP_RETURN_ON_ERROR(
        erase_key_from_nvs(APP_IDENTITY_NVS_KEY_UWB_ANTENNA_DELAY), TAG,
        "UWB antenna delay clear failed");

    s_uwb_antenna_delay = (uint16_t)APP_UWB_ANTENNA_DELAY_DEFAULT;
    s_uwb_antenna_delay_from_nvs = false;
    ESP_LOGW(TAG, "Cleared persistent UWB antenna delay; fallback=0x%04x",
             (unsigned)s_uwb_antenna_delay);
    return ESP_OK;
}
