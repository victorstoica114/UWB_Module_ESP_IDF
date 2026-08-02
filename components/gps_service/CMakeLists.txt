idf_component_register(SRCS "gps_service.c" "gps_moving_base.c"
                            "gps_ntrip_client.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "${PROJECT_DIR}"
                       PRIV_REQUIRES config esp_driver_gpio esp_driver_uart
                                     esp-tls freertos log lwip mbedtls
                                     wifi_service)
