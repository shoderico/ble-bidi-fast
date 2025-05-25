#ifndef BLE_BIDI_FAST_H
#define BLE_BIDI_FAST_H

#include <stdio.h>
#include "esp_err.h"
#include "esp_bt.h"

// Length of 128-bit UUID
#define UUID128_LEN 16

// Maximum data length for send/receive
#define BLE_BIDI_FAST_MAX_DATA_LEN 10

// Configuration structure for BLE module
typedef struct {
    const char *device_name;          // Device name
    uint8_t device_name_len;          // Length of device name
    void (*on_receive_callback)(const uint8_t *data, uint8_t len); // Callback for received data
} ble_bidi_fast_config_t;

// Initialize the BLE module with UUIDs
esp_err_t ble_bidi_fast_init(
    const ble_bidi_fast_config_t *config,
    const uint8_t *service_uuid,
    const uint8_t *send_char_uuid,
    const uint8_t *receive_char_uuid
);

// Send data to the connected client
esp_err_t ble_bidi_fast_send(const uint8_t *data, uint8_t len);


bool ble_bidi_fast_can_send();

#endif // BLE_BIDI_FAST_H