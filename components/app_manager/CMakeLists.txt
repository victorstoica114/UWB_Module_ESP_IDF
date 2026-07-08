idf_component_register(SRCS "app_manager.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES config esp_driver_gpio freertos log ota_service stability_test_service
                                     uwb_anchor_survey_service uwb_calibration_service
                                     uwb_distance_test_service uwb_dw3000
                                     uwb_ranging_service wifi_service wireless_log_service)
