#include "app_manager.h"

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"
#include "app_identity.h"
#include "app_led.h"
#include "app_runtime_config.h"
#include "bno085_service.h"
#include "board_config.h"
#include "boot_guard.h"
#include "charger_service.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps_service.h"
#include "max77958_service.h"
#include "ota_service.h"
#include "resource_monitor_service.h"
#include "sdkconfig.h"
#include "uwb_anchor_survey_service.h"
#include "uwb_calibration_service.h"
#include "uwb_config.h"
#include "uwb_distance_test_service.h"
#include "uwb_dw3000.h"
#include "uwb_ranging_service.h"
#include "wifi_service.h"
#include "wireless_log_service.h"
#include "wireless_telemetry_service.h"

static const char *TAG = "app_manager";

enum {
    STATUS_LED_TASK_STACK_WORDS = 2048,
    STATUS_LED_TASK_PRIORITY = 5,
    BOOT_GUARD_TASK_STACK_WORDS = 3072,
    BOOT_GUARD_TASK_PRIORITY = 4,
    BOOT_GUARD_OTA_WAIT_MS = 500,
};

#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
#define STATUS_LED_TASK_CORE 1
#else
#define STATUS_LED_TASK_CORE 0
#endif

static bool s_app_started;

static void boot_guard_stability_task(void *arg)
{
    (void)arg;

    const TickType_t start_tick = xTaskGetTickCount();
    const TickType_t stable_delay_ticks =
        pdMS_TO_TICKS(boot_guard_stable_delay_ms());
    const TickType_t wait_ticks = pdMS_TO_TICKS(BOOT_GUARD_OTA_WAIT_MS);

    while (true) {
        const TickType_t elapsed_ticks = xTaskGetTickCount() - start_tick;
        const bool delay_elapsed = elapsed_ticks >= stable_delay_ticks;
        const bool ota_ready =
            ota_service_get_status() == OTA_SERVICE_STATUS_RUNNING;
        if (delay_elapsed && ota_ready) {
            break;
        }
        vTaskDelay(wait_ticks);
    }

    const esp_err_t err = boot_guard_mark_stable();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Boot guard stable mark failed: %s",
                 esp_err_to_name(err));
    } else if (boot_guard_recovery_mode()) {
        ESP_LOGW(TAG, "Recovery boot is stable; waiting for OTA or clear command");
    } else {
        ESP_LOGI(TAG, "Boot guard marked application stable");
    }

    vTaskDelete(NULL);
}

static void boot_guard_start_stability_task(void)
{
    const BaseType_t created =
        xTaskCreate(boot_guard_stability_task, "boot_guard",
                    BOOT_GUARD_TASK_STACK_WORDS, NULL,
                    BOOT_GUARD_TASK_PRIORITY, NULL);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create boot guard stability task");
    }
}

static int status_led_level(bool led_on)
{
    return led_on ? BOARD_CONFIG_STATUS_LED_ACTIVE_LEVEL
                  : !BOARD_CONFIG_STATUS_LED_ACTIVE_LEVEL;
}

static enum app_led_mode status_led_mode_from_ota_status(
    enum ota_service_status status)
{
    switch (status) {
    case OTA_SERVICE_STATUS_UPDATING:
    case OTA_SERVICE_STATUS_REBOOTING:
        return APP_LED_MODE_OTA_UPDATING;
    case OTA_SERVICE_STATUS_FAILED:
        return APP_LED_MODE_OTA_ERROR;
    default:
        return APP_LED_MODE_RUN;
    }
}

static uint32_t status_led_interval_ms(enum app_led_mode mode)
{
    switch (mode) {
    case APP_LED_MODE_OTA_UPDATING:
        return APP_LED_BLINK_INTERVAL_NORMAL_MS;
    case APP_LED_MODE_OTA_ERROR:
        return APP_LED_BLINK_INTERVAL_FAST_MS;
    case APP_LED_MODE_RUN:
    default:
        return APP_LED_BLINK_INTERVAL_SLOW_MS;
    }
}

static const char *status_led_mode_name(enum app_led_mode mode)
{
    switch (mode) {
    case APP_LED_MODE_OTA_UPDATING:
        return "ota";
    case APP_LED_MODE_OTA_ERROR:
        return "ota_error";
    case APP_LED_MODE_RUN:
    default:
        return "run";
    }
}

static void status_led_task(void *arg)
{
    (void)arg;

    bool led_on = false;
    enum app_led_mode last_mode = (enum app_led_mode)-1;

    while (true) {
        const enum app_led_mode mode =
            status_led_mode_from_ota_status(ota_service_get_status());
        const uint32_t interval_ms = status_led_interval_ms(mode);

        if (mode != last_mode) {
            ESP_LOGI(TAG, "Status LED mode: %s (%u ms)",
                     status_led_mode_name(mode), (unsigned)interval_ms);
            last_mode = mode;
        }

        led_on = !led_on;
        gpio_set_level(BOARD_CONFIG_STATUS_LED_GPIO, status_led_level(led_on));
        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }
}

static void status_led_init(void)
{
    const gpio_config_t led_config = {
        .pin_bit_mask = 1ULL << BOARD_CONFIG_STATUS_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&led_config));
    ESP_ERROR_CHECK(gpio_set_level(BOARD_CONFIG_STATUS_LED_GPIO,
                                   status_led_level(false)));
}

static esp_err_t app_manager_start_selected_runtime(void)
{
    const app_runtime_config_t *runtime_config = app_runtime_config_get();

    if (!runtime_config->uwb_enabled) {
        ESP_LOGW(TAG, "UWB disabled by runtime config; holding DW3000 in reset");
        return uwb_dw3000_hold_in_reset();
    }

    if (!APP_UWB_ENABLED) {
        ESP_LOGW(TAG, "Selected runtime uses UWB, but APP_UWB_ENABLED is 0");
        return uwb_dw3000_hold_in_reset();
    }

    const uint8_t runtime_uwb_role = app_identity_get_uwb_role();
    const uint8_t runtime_mode = runtime_config->runtime_mode;
    ESP_LOGI(TAG, "Application runtime: mode=%s(%d), uwb_role=%s(%u), "
                  "uwb_source_id=%u, hostname=%s, module_id=%u, config=%s",
             app_runtime_config_runtime_mode_to_string(runtime_mode),
             (int)runtime_mode,
             app_identity_uwb_role_to_string(runtime_uwb_role),
             (unsigned)runtime_uwb_role,
             (unsigned)APP_UWB_SOURCE_ID, app_identity_get_hostname(),
             (unsigned)app_identity_get_module_id(),
             runtime_config->from_nvs ? "nvs" : "firmware");

    switch (runtime_mode) {
    case APP_RUNTIME_MODE_UWB_BEACON_SMOKE:
        return uwb_dw3000_start();
    case APP_RUNTIME_MODE_UWB_ANTENNA_DELAY_CALIBRATION:
        return uwb_calibration_service_start();
    case APP_RUNTIME_MODE_UWB_DISTANCE_TEST:
        return uwb_distance_test_service_start();
    case APP_RUNTIME_MODE_UWB_RANGING:
        return uwb_ranging_service_start();
    case APP_RUNTIME_MODE_UWB_ANCHOR_SURVEY:
        return uwb_anchor_survey_service_start();
    case APP_RUNTIME_MODE_UWB_DS_TWR_TDOA:
        return uwb_dw3000_start_ds_twr_tdoa();
    default:
        ESP_LOGE(TAG, "Unsupported application runtime mode: %d",
                 (int)runtime_mode);
        return ESP_ERR_INVALID_ARG;
    }
}

void app_manager_start(void)
{
    if (s_app_started) {
        return;
    }

    status_led_init();

    const BaseType_t created = xTaskCreatePinnedToCore(status_led_task,
                                                       "status_led",
                                                       STATUS_LED_TASK_STACK_WORDS,
                                                       NULL,
                                                       STATUS_LED_TASK_PRIORITY,
                                                       NULL,
                                                       STATUS_LED_TASK_CORE);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create status LED task");
        return;
    }

    s_app_started = true;
    ESP_LOGI(TAG, "Status LED blink started on GPIO%d, core %d",
             BOARD_CONFIG_STATUS_LED_GPIO, STATUS_LED_TASK_CORE);

    const esp_err_t boot_guard_err = boot_guard_init();
    if (boot_guard_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize boot guard: %s",
                 esp_err_to_name(boot_guard_err));
    }

    const esp_err_t identity_err = app_identity_init();
    if (identity_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize app identity: %s",
                 esp_err_to_name(identity_err));
    }

    const esp_err_t runtime_config_err = app_runtime_config_init();
    if (runtime_config_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize runtime config: %s",
                 esp_err_to_name(runtime_config_err));
    }

    const esp_err_t wifi_err = wifi_service_start();
    if (wifi_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Wi-Fi service: %s",
                 esp_err_to_name(wifi_err));
    }

    const esp_err_t wireless_log_err = wireless_log_service_start();
    if (wireless_log_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start wireless log service: %s",
                 esp_err_to_name(wireless_log_err));
    }

    const esp_err_t wireless_telemetry_err = wireless_telemetry_service_start();
    if (wireless_telemetry_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start wireless telemetry service: %s",
                 esp_err_to_name(wireless_telemetry_err));
    }

    const esp_err_t ota_err = ota_service_start();
    if (ota_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start OTA service: %s",
                 esp_err_to_name(ota_err));
    }

    const esp_err_t resource_err = resource_monitor_service_start();
    if (resource_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start resource monitor: %s",
                 esp_err_to_name(resource_err));
    }

    boot_guard_start_stability_task();

    if (boot_guard_recovery_mode()) {
        ESP_LOGW(TAG,
                 "Boot recovery mode active; Wi-Fi, wireless log, telemetry, and OTA are running, risky services are skipped");
        (void)uwb_dw3000_hold_in_reset();
        return;
    }

    const esp_err_t gps_err = gps_service_start();
    if (gps_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start GPS service: %s",
                 esp_err_to_name(gps_err));
    }

    const esp_err_t runtime_err = app_manager_start_selected_runtime();
    if (runtime_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start selected runtime: %s",
                 esp_err_to_name(runtime_err));
    }

    const esp_err_t charger_err = charger_service_start();
    if (charger_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start BQ25792 service: %s",
                 esp_err_to_name(charger_err));
    }

    const esp_err_t max77958_err = max77958_service_start();
    if (max77958_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MAX77958 service: %s",
                 esp_err_to_name(max77958_err));
    }

    const esp_err_t bno085_err = bno085_service_start();
    if (bno085_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start BNO085 service: %s",
                 esp_err_to_name(bno085_err));
    }
}
