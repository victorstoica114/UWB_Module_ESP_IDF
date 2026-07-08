#include "uwb_anchor_survey_service.h"

#include "app_identity.h"
#include "esp_log.h"
#include "uwb_config.h"
#include "uwb_dw3000.h"

static const char *TAG = "uwb_anchor_survey";

esp_err_t uwb_anchor_survey_service_start(void)
{
    const uint8_t runtime_role = app_identity_get_uwb_role();
    ESP_LOGI(TAG,
             "Anchor survey mode selected: role=%s(%u) source_id=%u tag_id=%u "
             "coordinator=%u anchors=[%u,%u,%u,%u] slot=%u ms delay=%u ms "
             "round_gap=%u ms antenna_delay=0x%04x",
             app_identity_uwb_role_to_string(runtime_role),
             (unsigned)runtime_role,
             (unsigned)APP_UWB_SOURCE_ID,
             (unsigned)APP_UWB_TAG_ID,
             (unsigned)APP_UWB_ANCHOR_SURVEY_COORDINATOR_ID,
             (unsigned)APP_UWB_ANCHOR_0_ID,
             (unsigned)APP_UWB_ANCHOR_1_ID,
             (unsigned)APP_UWB_ANCHOR_2_ID,
             (unsigned)APP_UWB_ANCHOR_3_ID,
             (unsigned)APP_UWB_ANCHOR_SURVEY_SLOT_MS,
             (unsigned)APP_UWB_ANCHOR_SURVEY_COMMAND_DELAY_MS,
             (unsigned)APP_UWB_ANCHOR_SURVEY_ROUND_GAP_MS,
             (unsigned)APP_UWB_ANTENNA_DELAY_DEFAULT);
    return uwb_dw3000_start_anchor_survey();
}
