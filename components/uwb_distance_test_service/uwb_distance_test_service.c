#include "uwb_distance_test_service.h"

#include "esp_log.h"
#include "uwb_config.h"

static const char *TAG = "uwb_distance_test";

esp_err_t uwb_distance_test_service_start(void)
{
    ESP_LOGW(TAG,
             "Distance test mode selected; runtime skeleton only. "
             "source_id=%u peer_id=%u antenna_delay=0x%04x",
             (unsigned)APP_UWB_SOURCE_ID,
             (unsigned)APP_UWB_DISTANCE_TEST_PEER_ID,
             (unsigned)APP_UWB_ANTENNA_DELAY_DEFAULT);
    return ESP_OK;
}
