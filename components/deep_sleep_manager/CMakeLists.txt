idf_component_register(
    SRCS "deep_sleep_manager.c"
    INCLUDE_DIRS "include" "."
    REQUIRES driver log bt
)
