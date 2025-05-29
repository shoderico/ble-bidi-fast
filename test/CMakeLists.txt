idf_component_register(
    SRCS "test_ble_bidi_fast.c"
    INCLUDE_DIRS "."
    REQUIRES esp_common unity ble_bidi_fast
    WHOLE_ARCHIVE
)