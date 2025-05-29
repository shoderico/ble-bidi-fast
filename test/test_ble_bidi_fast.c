#include "unity.h"
#include "ble_bidi_fast.h"

// Dummy callback for testing
static void dummy_rx_callback(uint8_t *data, uint8_t len) {
    // Do nothing
}

// Test case: Validate input parameters for ble_bidi_fast_init
TEST_CASE("ble_bidi_fast_init validates input parameters", "[ble_bidi_fast]")
{
    ble_bidi_fast_config_t config = {
        .device_name = "TestDevice",
        .device_name_len = 10,
        .rx_receive_callback = dummy_rx_callback
    };
    uint8_t dummy_uuid[16] = {0};

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ble_bidi_fast_init(NULL, dummy_uuid, dummy_uuid, dummy_uuid));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ble_bidi_fast_init(&config, NULL, dummy_uuid, dummy_uuid));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ble_bidi_fast_init(&config, dummy_uuid, NULL, dummy_uuid));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ble_bidi_fast_init(&config, dummy_uuid, dummy_uuid, NULL));
}

