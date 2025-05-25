idf_component_register(SRCS "ble_bidi_fast.c"
                       INCLUDE_DIRS "."
                       REQUIRES freertos esp_common esp_event bt nvs_flash)