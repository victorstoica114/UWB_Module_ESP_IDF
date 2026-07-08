#include "uwb_calibration_service.h"

#include "esp_log.h"
#include "uwb_config.h"

static const char *TAG = "uwb_calibration";

esp_err_t uwb_calibration_service_start(void)
{
    ESP_LOGW(TAG,
             "Antenna delay calibration mode selected; runtime skeleton only. "
             "default_delay=0x%04x source_id=%u",
             (unsigned)APP_UWB_ANTENNA_DELAY_DEFAULT,
             (unsigned)APP_UWB_SOURCE_ID);
    return ESP_OK;
}
