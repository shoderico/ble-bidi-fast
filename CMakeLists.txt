idf_component_register(
    SRCS "src/ble_bidi_fast.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos esp_common esp_event bt nvs_flash
)