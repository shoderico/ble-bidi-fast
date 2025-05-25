# ble_bidi_fast

A lightweight ESP32 Bluetooth Low Energy (BLE) component for fast, bidirectional communication.

## Overview

`ble_bidi_fast` is a reusable ESP32 BLE component designed for efficient, bidirectional data transfer between an ESP32 device and a BLE client (e.g., a mobile app). It supports:
- Configurable device name, Service UUID, Send Characteristic UUID, and Receive Characteristic UUID.
- Sending data from the ESP32 to the client (via Send Characteristic, up to 10 bytes).
- Receiving data from the client to the ESP32 (via Receive Characteristic, up to 10 bytes) with a user-defined callback.
- Fast communication with optimized advertising (32ms–64ms) and connection intervals (7.5ms–15ms).

This component is ideal for applications requiring low-latency, bidirectional binary data exchange over BLE.

## Features

- **Bidirectional Communication**: Send data from ESP32 to client (Send Characteristic) and receive data from client (Receive Characteristic).
- **Configurable**: Set device name, 128-bit UUIDs, and a receive callback via a configuration structure.
- **Compact Data**: Supports binary data up to 10 bytes for both sending and receiving.
- **High Performance**: Optimized for low-latency communication with short advertising and connection intervals.
- **Modular Design**: Easy to integrate into ESP-IDF projects as a component.

## Requirements

- ESP-IDF v4.4 or later
- ESP32 device with BLE support
- NVS flash initialized by the user application

## Installation

1. Clone or copy the `ble_bidi_fast` component into your ESP-IDF project's `components` directory.
2. Include the component in your project by adding it to your `CMakeLists.txt` or ensuring it is detected by ESP-IDF.
3. Initialize NVS flash in your application before using the component.

## Usage

### Example Code

```c
#include "ble_bidi_fast.h"
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "APP";

static void on_receive_callback(const uint8_t *data, uint8_t len) {
    ESP_LOGI(TAG, "Received data, len=%d", len);
    for (int i = 0; i < len; i++) {
        ESP_LOGI(TAG, "data[%d]=0x%02x", i, data[i]);
    }
}

void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Define UUIDs
    uint8_t service_uuid[16] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0xF3, 0x93, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E};
    uint8_t send_uuid[16] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0xF3, 0x93, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E};
    uint8_t receive_uuid[16] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0xF3, 0x93, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E};

    // Configure BLE
    ble_bidi_fast_config_t config = {
        .device_name = "SwiftDevice",
        .device_name_len = 11,
        .service_uuid = service_uuid,
        .send_char_uuid = send_uuid,
        .receive_char_uuid = receive_uuid,
        .on_receive_callback = on_receive_callback,
    };

    // Initialize BLE
    ESP_ERROR_CHECK(ble_bidi_fast_init(&config));

    // Send example data
    uint8_t data[2] = {0x01, 0x02};
    ble_bidi_fast_send(data, 2);
}
```

### API

- **ble_bidi_fast_init**: Initialize the BLE module with the provided configuration.
- **ble_bidi_fast_send**: Send data (up to 10 bytes) to the connected client.
- **on_receive_callback**: User-defined callback invoked when data (up to 10 bytes) is received from the client.

## Notes

- **Device Name Length**: Keep the device name under 20 bytes to fit within the 31-byte advertising data limit.
- **NVS Initialization**: The user application must initialize NVS flash before calling `ble_bidi_fast_init`.
- **Future Extensions**: The component includes a send timer for potential periodic sending. To implement, modify the `send_timer_callback` function.

## License

MIT License. See `LICENSE` file for details.