# ble_bidi_fast

A lightweight ESP32 Bluetooth Low Energy (BLE) component for fast, bidirectional communication.

## Overview

`ble_bidi_fast` is a reusable ESP32 BLE component designed for efficient, bidirectional data transfer between an ESP32 device and a BLE client (e.g., a mobile app). It provides:

- **Configurable Setup**: Customizable device name, 128-bit Service UUID, Send Characteristic UUID, and Receive Characteristic UUID.
- **Bidirectional Data Transfer**: Send data from the ESP32 to the client (via Send Characteristic, up to 10 bytes) and receive data from the client (via Receive Characteristic, up to 10 bytes) with a user-defined callback.
- **Optimized Performance**: Fast communication with advertising intervals of 32ms–64ms and connection intervals of 7.5ms–15ms.
- **Error Handling**: Robust error checking for initialization, data sending, and GATT/GAP operations.

This component is ideal for low-latency applications requiring compact binary data exchange over BLE.

## Features

- **Bidirectional Communication**: Supports sending data to the client (Notify property) and receiving data from the client (Write and Write No Response properties).
- **Customizable Configuration**: Define device name, UUIDs, and a callback for received data via a configuration structure.
- **Compact Data**: Handles binary data up to 10 bytes for both send and receive operations.
- **High Performance**: Optimized for low latency with short advertising and connection intervals.
- **Modular Design**: Easily integrates into ESP-IDF projects as a component.
- **Connection Status**: Check if sending is enabled with `ble_bidi_fast_can_send()`.

## Requirements

- ESP-IDF v4.4 or later
- ESP32 device with BLE support
- NVS flash initialized by the user application

## Installation

1. Copy the `ble_bidi_fast` component into your ESP-IDF project's `components` directory.
2. Ensure the component is included in your project's `CMakeLists.txt` or automatically detected by ESP-IDF.
3. Initialize NVS flash in your application before using the component.

## Usage

### Example Code

```c
#include "ble_bidi_fast.h"
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "APP";

// Callback for received data
static void on_receive_callback(const uint8_t *data, uint8_t len) {
    ESP_LOGI(TAG, "Received data, len=%d", len);
    for (int i = 0; i < len; i++) {
        ESP_LOGI(TAG, "data[%d]=0x%02x", i, data[i]);
    }
}

void app_main(void) {
    // Initialize NVS flash
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Define 128-bit UUIDs
    uint8_t service_uuid[16] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0xF3, 0x93, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E};
    uint8_t send_uuid[16] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0xF3, 0x93, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E};
    uint8_t receive_uuid[16] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0xF3, 0x93, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E};

    // Configure BLE component
    ble_bidi_fast_config_t config = {
        .device_name = "SwiftDevice",
        .device_name_len = 11,
        .on_receive_callback = on_receive_callback
    };

    // Initialize BLE
    ESP_ERROR_CHECK(ble_bidi_fast_init(&config, service_uuid, send_uuid, receive_uuid));

    // Send example data when ready
    uint8_t data[2] = {0x01, 0x02};
    if (ble_bidi_fast_can_send()) {
        ESP_ERROR_CHECK(ble_bidi_fast_send(data, 2));
    }
}
```

### API

- **`ble_bidi_fast_init`**: Initializes the BLE module with the provided configuration and UUIDs. Returns `ESP_OK` on success or an error code on failure.
- **`ble_bidi_fast_send`**: Sends data (up to 10 bytes) to the connected client using the Notify property. Returns `ESP_OK` on success or `ESP_ERR_INVALID_STATE` if sending is not enabled or data length is invalid.
- **`ble_bidi_fast_can_send`**: Checks if the client has enabled notifications and the send characteristic is ready. Returns `true` if sending is possible, `false` otherwise.
- **`on_receive_callback`**: User-defined callback invoked when data (up to 10 bytes) is received from the client via the Receive Characteristic.

## Notes

- **Device Name Length**: Keep the device name under 20 bytes to fit within the 31-byte advertising data limit.
- **NVS Initialization**: The user application must initialize NVS flash before calling `ble_bidi_fast_init`.
- **UUIDs**: Use 128-bit UUIDs for the service and characteristics to ensure uniqueness.
- **Error Handling**: Check return values of API functions to handle errors appropriately.
- **Connection Parameters**: The component sets preferred connection parameters (7.5ms–15ms interval, 0s latency, 4s timeout) for low-latency communication.

## License

MIT License. See `LICENSE` file for details.