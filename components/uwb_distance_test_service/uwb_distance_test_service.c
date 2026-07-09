#include "uwb_distance_test_service.h"

#include "app_runtime_config.h"
#include "esp_log.h"
#include "uwb_config.h"
#include "uwb_dw3000.h"

static const char *TAG = "uwb_distance_test";

esp_err_t uwb_distance_test_service_start(void)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    ESP_LOGI(TAG,
             "Distance test mode selected: source_id=%u peer_id=%u "
             "initiator_id=%u responder_id=%u antenna_delay=0x%04x",
             (unsigned)APP_UWB_SOURCE_ID,
             (unsigned)config->distance_test_peer_id,
             (unsigned)config->distance_test_initiator_id,
             (unsigned)config->distance_test_responder_id,
             (unsigned)uwb_dw3000_get_antenna_delay());
    return uwb_dw3000_start_distance_test();
}
