idf_component_register(SRCS "uwb_dw3000.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "${PROJECT_DIR}"
                       REQUIRES board_config
                       PRIV_REQUIRES esp_driver_gpio esp_driver_spi esp_hw_support freertos log)
