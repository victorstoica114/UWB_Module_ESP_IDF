idf_component_register(SRCS "uwb_dw3000.c" "uwb_anchor_range_cache.c"
                            "uwb_flex_tdoa_runtime.c"
                            "uwb_native_ds_twr.c"
                            "uwb_native_ds_protocol.c"
                            "uwb_native_ds_runtime.c"
                            "uwb_native_ds_position_solver.c"
                            "uwb_passive_ds_runtime.c"
                            "uwb_passive_ds_protocol.c"
                            "uwb_passive_ds_observation.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "${PROJECT_DIR}"
                       PRIV_REQUIRES config esp_driver_gpio esp_driver_gptimer esp_driver_spi esp_hw_support flextdoa_protocol flextdoa_solver_service gps_service passive_ds_solver_service uwb_mobile_geometry freertos log wireless_log_service wireless_telemetry_service)

# FlexTDOA must empty a DW3000 RX buffer inside a sub-millisecond response
# train. Keep the project-wide debug build, but optimize the radio hot path.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)

# Keep protocol-private state and solvers behind their runtime facades. This
# target runs on every component build so a future cross-protocol dependency
# fails at compile time instead of surfacing as a field regression.
add_custom_target(uwb_protocol_boundary_check
    COMMAND "${PYTHON}" "${PROJECT_DIR}/tools/check_uwb_protocol_boundaries.py"
    WORKING_DIRECTORY "${PROJECT_DIR}"
    COMMENT "Checking UWB protocol boundaries"
    VERBATIM)
add_dependencies(${COMPONENT_LIB} uwb_protocol_boundary_check)
