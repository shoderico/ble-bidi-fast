#ifndef BLE_BIDI_FAST_H
#define BLE_BIDI_FAST_H

#include <stdio.h>
#include "esp_err.h"
#include "esp_bt.h"

// Length of 128-bit UUID
#define UUID128_LEN 16

// Configuration structure for BLE module
typedef struct {
    const char *device_name;          // Device name
    uint8_t device_name_len;          // Length of device name
    void (*on_rx_receive_callback)(/*const*/ uint8_t *data, uint8_t len); // Callback for RX received data
} ble_bidi_fast_config_t;

// Initialize the BLE module with UUIDs
esp_err_t ble_bidi_fast_init(
    const ble_bidi_fast_config_t *config,
    const uint8_t *service_uuid,
    const uint8_t *tx_char_uuid,
    const uint8_t *rx_char_uuid
);

// Send data from TX to the connected client
esp_err_t ble_bidi_fast_tx_send(/*const*/ uint8_t *data, uint8_t len);


// Retreive TX notification is enabled or not
bool ble_bidi_fast_is_tx_notification_enabled();

#endif // BLE_BIDI_FAST_H