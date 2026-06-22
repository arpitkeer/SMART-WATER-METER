idf_component_register(
    SRCS "meter_storage.c"
    INCLUDE_DIRS "include" "."
    REQUIRES eeprom_24cxx
)