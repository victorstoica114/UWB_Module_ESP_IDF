idf_component_register(SRCS "ota_service.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "${PROJECT_DIR}"
                       PRIV_REQUIRES app_update esp_app_format esp_http_server esp_partition esp_system freertos log stability_test_service uwb_dw3000 wifi_service wireless_log_service)
