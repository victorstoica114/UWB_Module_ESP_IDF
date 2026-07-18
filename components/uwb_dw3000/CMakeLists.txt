idf_component_register(SRCS "uwb_dw3000.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "${PROJECT_DIR}"
                       PRIV_REQUIRES config esp_driver_gpio esp_driver_gptimer esp_driver_spi esp_hw_support freertos log wireless_telemetry_service)

# FlexTDOA must empty a DW3000 RX buffer inside a sub-millisecond response
# train. Keep the project-wide debug build, but optimize the radio hot path.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
