idf_component_register(SRCS "boot_guard.c"
                       INCLUDE_DIRS "include"
                       REQUIRES app_update esp_system
                       PRIV_REQUIRES config esp_app_format esp_partition log)
