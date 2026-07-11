idf_component_register(SRCS "charger_service.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES config esp_driver_gpio esp_driver_i2c freertos
                                     i2c_bus_service log nvs_flash)
