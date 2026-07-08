idf_component_register(SRCS "wifi_service.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "${PROJECT_DIR}"
                       PRIV_REQUIRES config esp_event esp_netif esp_wifi freertos log nvs_flash)
