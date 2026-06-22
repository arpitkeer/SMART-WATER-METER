idf_component_register(
    SRCS "meter_helper.c"
    INCLUDE_DIRS "include"
    REQUIRES
        nimble_adv
        ble_buffer
        deep_sleep_manager
        meter_storage
        esp_timer
)