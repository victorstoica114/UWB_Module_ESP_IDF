idf_component_register(SRCS "i2c_bus_service.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_i2c freertos
                       PRIV_REQUIRES config esp_timer log)
