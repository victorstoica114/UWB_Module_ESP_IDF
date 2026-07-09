#include "uwb_ranging_service.h"

#include "app_identity.h"
#include "app_runtime_config.h"
#include "esp_log.h"
#include "uwb_config.h"
#include "uwb_dw3000.h"

static const char *TAG = "uwb_ranging";

esp_err_t uwb_ranging_service_start(void)
{
    const uint8_t runtime_role = app_identity_get_uwb_role();
    const app_runtime_config_t *config = app_runtime_config_get();
    ESP_LOGI(TAG,
             "Ranging mode selected: "
             "role=%s(%u) source_id=%u tag_id=%u anchor_count=%u "
             "anchors=[%u,%u,%u,%u] slot=%u ms round_gap=%u ms antenna_delay=0x%04x",
             app_identity_uwb_role_to_string(runtime_role),
             (unsigned)runtime_role,
             (unsigned)APP_UWB_SOURCE_ID,
             (unsigned)config->tag_id,
             (unsigned)config->anchor_count,
             (unsigned)config->anchor_ids[0],
             (unsigned)config->anchor_ids[1],
             (unsigned)config->anchor_ids[2],
             (unsigned)config->anchor_ids[3],
             (unsigned)config->ranging_slot_ms,
             (unsigned)config->ranging_round_gap_ms,
             (unsigned)uwb_dw3000_get_antenna_delay());
    return uwb_dw3000_start_ranging();
}
