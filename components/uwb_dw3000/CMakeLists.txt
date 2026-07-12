idf_component_register(SRCS "uwb_dw3000.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "${PROJECT_DIR}"
                       PRIV_REQUIRES config esp_driver_gpio esp_driver_gptimer esp_driver_spi esp_hw_support freertos log)
