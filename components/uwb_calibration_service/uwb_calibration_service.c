#include "uwb_calibration_service.h"

#include "app_identity.h"
#include "app_runtime_config.h"
#include "esp_log.h"
#include "uwb_dw3000.h"
#include "uwb_config.h"

static const char *TAG = "uwb_calibration";

esp_err_t uwb_calibration_service_start(void)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    ESP_LOGI(TAG,
             "Antenna delay calibration mode selected: method=%u "
             "known_distance=%u mm samples=%u default_delay=0x%04x "
             "source_id=%u role=%s(%u)",
             (unsigned)config->calibration_method,
             (unsigned)config->calibration_known_distance_mm,
             (unsigned)config->calibration_sample_count,
             (unsigned)uwb_dw3000_get_antenna_delay(),
             (unsigned)APP_UWB_SOURCE_ID,
             app_identity_uwb_role_to_string(app_identity_get_uwb_role()),
             (unsigned)app_identity_get_uwb_role());
    return uwb_dw3000_start_calibration();
}
