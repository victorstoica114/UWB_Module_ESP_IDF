idf_component_register(SRCS "gps_service.c" "gps_moving_base.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES config esp_driver_gpio esp_driver_uart
                                     freertos log lwip)
