idf_component_register(
    SRCS "ble_simple_echo.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_common esp_event bt freertos nvs_flash driver
        ble_bidi_fast
)