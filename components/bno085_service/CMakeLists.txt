idf_component_register(SRCS "bno085_service.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES config esp_driver_gpio esp_driver_i2c
                                     freertos log wireless_telemetry_service)
