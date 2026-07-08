#include "uwb_ranging_service.h"

#include "esp_log.h"
#include "uwb_config.h"

static const char *TAG = "uwb_ranging";

esp_err_t uwb_ranging_service_start(void)
{
    ESP_LOGW(TAG,
             "Ranging mode selected; runtime skeleton only. "
             "role=%u source_id=%u tag_id=%u anchor_count=%u "
             "anchors=[%u,%u,%u,%u] antenna_delay=0x%04x",
             (unsigned)APP_UWB_ROLE,
             (unsigned)APP_UWB_SOURCE_ID,
             (unsigned)APP_UWB_TAG_ID,
             (unsigned)APP_UWB_ANCHOR_COUNT,
             (unsigned)APP_UWB_ANCHOR_0_ID,
             (unsigned)APP_UWB_ANCHOR_1_ID,
             (unsigned)APP_UWB_ANCHOR_2_ID,
             (unsigned)APP_UWB_ANCHOR_3_ID,
             (unsigned)APP_UWB_ANTENNA_DELAY_DEFAULT);
    return ESP_OK;
}
