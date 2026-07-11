idf_component_register(SRCS "i2c_bus_service.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES config esp_driver_i2c freertos log)
