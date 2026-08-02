idf_component_register(
    SRCS "passive_ds_position_solver.c" "passive_ds_solver_service.c"
    INCLUDE_DIRS "include"
    REQUIRES config freertos wireless_log_service wireless_telemetry_service)

target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
