idf_component_register(SRCS "gnss_firmware_updater.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES config esp_driver_gpio esp_driver_uart freertos
                                     gps_service log)
