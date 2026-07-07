idf_component_register(SRCS "app_manager.c"
                       INCLUDE_DIRS "include"
                       REQUIRES board_config
                       PRIV_REQUIRES esp_driver_gpio freertos log ota_service uwb_dw3000 wifi_service wireless_log_service)
