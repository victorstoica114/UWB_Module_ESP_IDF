idf_component_register(SRCS "wireless_telemetry_service.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "${PROJECT_DIR}"
                       PRIV_REQUIRES config freertos log lwip wifi_service)
