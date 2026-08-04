idf_component_register(SRCS "flextdoa_protocol.c" "flextdoa_collector.c"
                       INCLUDE_DIRS "include")

target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
