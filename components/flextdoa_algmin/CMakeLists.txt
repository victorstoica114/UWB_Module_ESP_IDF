idf_component_register(SRCS "flextdoa_algmin.c" "flextdoa_frame_aggregator.c"
                       INCLUDE_DIRS "include")

target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
