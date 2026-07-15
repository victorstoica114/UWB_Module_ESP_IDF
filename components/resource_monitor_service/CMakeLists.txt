idf_component_register(SRCS "resource_monitor_service.c"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos
                       PRIV_REQUIRES esp_driver_tsens esp_partition esp_timer
                                     heap log spi_flash)
