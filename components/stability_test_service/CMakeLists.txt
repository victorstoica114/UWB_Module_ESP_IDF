idf_component_register(SRCS "stability_test_service.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "${PROJECT_DIR}"
                       PRIV_REQUIRES config freertos log uwb_dw3000 wireless_log_service)
