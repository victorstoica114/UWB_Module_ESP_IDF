idf_component_register(SRCS "bno085_service.c" "bno085_timing.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES config esp_driver_gpio esp_driver_gptimer
                                     esp_driver_i2c esp_timer freertos
                                     i2c_bus_service log
                                     wireless_telemetry_service)
