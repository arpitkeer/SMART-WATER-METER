# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES cyclic_counter nimble_adv deep_sleep_manager eeprom_24cxx tamper ble_buffer meter_storage meter_helper watchdog lora_manager adc_monitor
)
