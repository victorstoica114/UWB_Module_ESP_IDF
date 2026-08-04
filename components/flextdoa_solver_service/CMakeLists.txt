idf_component_register(SRCS "flextdoa_solver_service.c"
                       INCLUDE_DIRS "include"
                       REQUIRES config flextdoa_algmin flextdoa_protocol
                                freertos wireless_log_service
                                wireless_telemetry_service)

target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
