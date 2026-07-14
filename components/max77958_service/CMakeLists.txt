idf_component_register(SRCS "max77958_service.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES config esp_driver_i2c freertos
                                     i2c_bus_service log nvs_flash)
