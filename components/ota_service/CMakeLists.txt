idf_component_register(SRCS "ota_service.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "${PROJECT_DIR}"
                       PRIV_REQUIRES app_update bno085_service boot_guard
                                     charger_service config
                                     esp_app_format esp_http_server
                                     esp_partition esp_system freertos gps_service
                                     flextdoa_solver_service
                                     i2c_bus_service log
                                     max77958_service
                                     resource_monitor_service
                                     uwb_dw3000 wifi_service wireless_log_service
                                     wireless_telemetry_service)
