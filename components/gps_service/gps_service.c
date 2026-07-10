#include "gps_service.h"

#include <stdbool.h>

#include "app_runtime_config.h"
#include "board_config.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "gps_service";

static int gps_enable_level(bool enabled)
{
    const int active = BOARD_CONFIG_GPS_ENABLE_ACTIVE_LEVEL ? 1 : 0;
    return enabled ? active : !active;
}

esp_err_t gps_service_start(void)
{
    const app_runtime_config_t *runtime_config = app_runtime_config_get();
    const bool enabled = runtime_config->gps_enabled;

    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << BOARD_CONFIG_GPS_ENABLE_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&config), TAG,
                        "GPS enable GPIO init failed");
    ESP_RETURN_ON_ERROR(
        gpio_set_level(BOARD_CONFIG_GPS_ENABLE_GPIO, gps_enable_level(enabled)),
        TAG, "GPS enable GPIO set failed");

    ESP_LOGI(TAG, "GPS %s on GPIO%d active-%s",
             enabled ? "enabled" : "disabled", BOARD_CONFIG_GPS_ENABLE_GPIO,
             BOARD_CONFIG_GPS_ENABLE_ACTIVE_LEVEL ? "high" : "low");
    return ESP_OK;
}
