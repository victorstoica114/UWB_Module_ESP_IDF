idf_component_register(SRCS "app_manager.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES bno085_service charger_service config
                                     esp_driver_gpio freertos gps_service log ota_service
                                     uwb_anchor_survey_service uwb_calibration_service
                                     uwb_distance_test_service uwb_dw3000
                                     uwb_ranging_service wifi_service wireless_log_service
                                     wireless_telemetry_service)
