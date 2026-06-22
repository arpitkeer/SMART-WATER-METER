idf_component_register(
    SRCS "cyclic_counter.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer log
)
