idf_component_register(SRCS "uwb_localization_solver.c"
                            "uwb_localization_protocol_models.c"
                       INCLUDE_DIRS "include")

target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
