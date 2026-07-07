#include "app_manager.h"
#include "esp_log.h"

static const char *TAG = "app_main";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting UWB ESP-IDF app");
    app_manager_start();
}
