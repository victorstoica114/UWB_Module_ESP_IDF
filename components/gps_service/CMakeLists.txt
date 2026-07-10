idf_component_register(SRCS "gps_service.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES config esp_driver_gpio esp_driver_uart
                                     freertos log)
