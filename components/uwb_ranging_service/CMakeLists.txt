idf_component_register(SRCS "uwb_ranging_service.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES config log uwb_dw3000)
