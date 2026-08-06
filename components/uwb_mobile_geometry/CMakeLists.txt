idf_component_register(SRCS "uwb_mobile_geometry.c"
                       INCLUDE_DIRS "include")

target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
