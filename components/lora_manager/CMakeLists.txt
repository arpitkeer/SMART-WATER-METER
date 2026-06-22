idf_component_register(
    SRCS "lora_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES adc_monitor esp_timer driver 
)