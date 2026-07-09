idf_component_register(SRCS "app_identity.c" "app_runtime_config.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "${PROJECT_DIR}"
                       PRIV_REQUIRES esp_hw_support log nvs_flash)
