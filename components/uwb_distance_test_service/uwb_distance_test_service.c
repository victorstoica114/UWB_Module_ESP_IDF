#include "uwb_distance_test_service.h"

#include "esp_log.h"
#include "uwb_dw3000.h"
#include "uwb_config.h"

static const char *TAG = "uwb_distance_test";

esp_err_t uwb_distance_test_service_start(void)
{
    ESP_LOGI(TAG,
             "Distance test mode selected: source_id=%u peer_id=%u "
             "initiator_id=%u responder_id=%u antenna_delay=0x%04x",
             (unsigned)APP_UWB_SOURCE_ID,
             (unsigned)APP_UWB_DISTANCE_TEST_PEER_ID,
             (unsigned)APP_UWB_DISTANCE_TEST_INITIATOR_ID,
             (unsigned)APP_UWB_DISTANCE_TEST_RESPONDER_ID,
             (unsigned)uwb_dw3000_get_antenna_delay());
    return uwb_dw3000_start_distance_test();
}
