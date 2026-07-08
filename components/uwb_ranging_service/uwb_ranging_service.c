#include "uwb_ranging_service.h"

#include "app_identity.h"
#include "esp_log.h"
#include "uwb_config.h"
#include "uwb_dw3000.h"

static const char *TAG = "uwb_ranging";

esp_err_t uwb_ranging_service_start(void)
{
    const uint8_t runtime_role = app_identity_get_uwb_role();
    ESP_LOGW(TAG,
             "Ranging mode selected; runtime skeleton only. "
             "role=%s(%u) source_id=%u tag_id=%u anchor_count=%u "
             "anchors=[%u,%u,%u,%u] antenna_delay=0x%04x",
             app_identity_uwb_role_to_string(runtime_role),
             (unsigned)runtime_role,
             (unsigned)APP_UWB_SOURCE_ID,
             (unsigned)APP_UWB_TAG_ID,
             (unsigned)APP_UWB_ANCHOR_COUNT,
             (unsigned)APP_UWB_ANCHOR_0_ID,
             (unsigned)APP_UWB_ANCHOR_1_ID,
             (unsigned)APP_UWB_ANCHOR_2_ID,
             (unsigned)APP_UWB_ANCHOR_3_ID,
             (unsigned)uwb_dw3000_get_antenna_delay());
    return ESP_OK;
}
