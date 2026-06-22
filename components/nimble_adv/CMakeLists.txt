idf_component_register(
    SRCS "nimble_adv.c"
    INCLUDE_DIRS "include"
    REQUIRES bt nvs_flash wm_auth ota_manager
)