idf_component_register(SRCS "flextdoa_protocol.c" "flextdoa_collector.c"
                            "flextdoa_cfo_estimator.c"
                       INCLUDE_DIRS "include"
                       REQUIRES uwb_mobile_geometry)

target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
