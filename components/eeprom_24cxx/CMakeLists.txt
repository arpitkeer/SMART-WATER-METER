idf_component_register(
    SRCS "eeprom_24cxx.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer
)