# ble_bidi_fast

A lightweight ESP32 Bluetooth Low Energy (BLE) component for fast, bidirectional communication.

## Overview

`ble_bidi_fast` is a reusable ESP32 BLE component designed for efficient, bidirectional data transfer between an ESP32 device and a BLE client (e.g., a mobile app). It provides:

- **Configurable Setup**: Customizable device name, 128-bit Service UUID, TX Characteristic UUID, and RX Characteristic UUID.
- **Bidirectional Data Transfer**: Transmit data from the ESP32 to the client (via TX Characteristic) and receive data from the client (via RX Characteristic) with a user-defined callback.
- **Optimized Performance**: Fast communication with advertising intervals of 32ms–64ms and connection intervals of 7.5ms–15ms.
- **Error Handling**: Robust error checking for initialization, data transmission, and GATT/GAP operations.

This component is ideal for low-latency applications requiring compact binary data exchange over BLE.

## Features

- **Bidirectional Communication**: Supports transmitting data to the client (Notify property on TX Characteristic) and receiving data from the client (Write and Write No Response properties on RX Characteristic).
- **Customizable Configuration**: Define device name, UUIDs, and a callback for received data via a configuration structure.
- **Compact Data**: Handles binary data for both TX and RX operations.
- **High Performance**: Optimized for low latency with short advertising and connection intervals.
- **Modular Design**: Easily integrates into ESP-IDF projects as a component.
- **Connection Status**: Check if TX notifications are enabled with `ble_bidi_fast_is_tx_notification_enabled()`.

## Requirements

- ESP-IDF v4.4 or later
- ESP32 device with BLE support
- NVS flash initialized by the user application

## Installation

1. Copy the `ble_bidi_fast` component into your ESP-IDF project's `components` directory.
2. Ensure the component is included in your project's `CMakeLists.txt` or automatically detected by ESP-IDF.
3. Initialize NVS flash in your application before using the component.

## Usage

### Example 1: Basic Usage

Below is a basic example demonstrating how to initialize the `ble_bidi_fast` component and handle data received on the RX Characteristic.

```c
#include "ble_bidi_fast.h"
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "APP";

// Callback for RX received data
static void on_rx_receive_callback(const uint8_t *data, uint8_t len) {
    ESP_LOGI(TAG, "Received %d bytes on RX Characteristic", len);
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

    // Define 128-bit UUIDs (Little Endian)
    // Service UUID      : 6e400001-b5a3-93f3-e0a9-e50e24dcca9e
    // TX Characteristic : 6e400002-b5a3-93f3-e0a9-e50e24dcca9e
    // RX Characteristic : 6e400003-b5a3-93f3-e0a9-e50e24dcca9e
    uint8_t service_uuid[16] = {0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0xf3, 0x93, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e};
    uint8_t tx_uuid[16]      = {0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0xf3, 0x93, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e};
    uint8_t rx_uuid[16]      = {0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0xf3, 0x93, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e};

    // Configure BLE component
    ble_bidi_fast_config_t config = {
        .device_name = "SwiftDevice",
        .device_name_len = 11,
        .on_rx_receive_callback = on_rx_receive_callback
    };

    // Initialize BLE
    ESP_ERROR_CHECK(ble_bidi_fast_init(&config, service_uuid, tx_uuid, rx_uuid));

    // Send example data via TX Characteristic when ready
    uint8_t data[2] = {0x01, 0x02};
    if (ble_bidi_fast_is_tx_notification_enabled()) {
        ESP_ERROR_CHECK(ble_bidi_fast_tx_send(data, 2));
    }
}
```

### Example 2: Simple Echo

The `ble_simple_echo` example demonstrates bidirectional communication using the `ble_bidi_fast` component. In this example:

- When the **TX Characteristic**'s Notification is enabled (by writing `0x01 0x00` to the Client Characteristic Configuration Descriptor, CCCD), an LED starts blinking every 500ms to indicate the active notification state.
- When data is written to the **RX Characteristic**, the ESP32 immediately echoes the same data back to the client via the **TX Characteristic**.

This example is ideal for testing the component's basic functionality with a BLE client app (e.g., nRF Connect) and visualizing the notification state via an LED.

**Code**:

```c
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <driver/gpio.h>
#include "ble_bidi_fast.h"

#define TAG "BLE_SIMPLE_ECHO"

//  2: Built-in LED on most ESP32 boards
// 15: Xiao ESP32C6
// 21: Xiao ESP32S3
#define LED_GPIO 15

// LED state
static bool s_led_on = false;

// Configure LED
static void configure_led(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

// Toggle LED
static void toggle_led(void) {
    s_led_on = !s_led_on;
    gpio_set_level(LED_GPIO, s_led_on);
}

// Task to blink LED when TX Characteristic's Notification is On (CCCD: On=0x01 0x00, Off=0x00 0x00)
static void led_blink_task(void *pvParameters) {
    while (1) {
        if (ble_bidi_fast_is_tx_notification_enabled()) {
            toggle_led();
        } else {
            gpio_set_level(LED_GPIO, 0); // Stop blinking when TX Notification is Off
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // 500ms
    }
}

// Callback for RX received data
static void on_rx_receive_callback(const uint8_t *data, uint8_t len) {
    ESP_LOGI(TAG, "Received %d bytes on RX Characteristic", len);
    
    // Echo back the received data via TX Characteristic
    if (ble_bidi_fast_is_tx_notification_enabled()) {
        esp_err_t ret = ble_bidi_fast_tx_send(data, len);
        if (ret) {
            ESP_LOGW(TAG, "Failed to send via TX Characteristic, error=%d", ret);
        } else {
            ESP_LOGI(TAG, "Echoed %d bytes via TX Characteristic", len);
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting BLE Simple Echo");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS partition");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    // Define UUIDs (Little Endian)
    // Service UUID      : 6e400001-b5a3-93f3-e0a9-e50e24dcca9e
    // TX Characteristic : 6e400002-b5a3-93f3-e0a9-e50e24dcca9e
    // RX Characteristic : 6e400003-b5a3-93f3-e0a9-e50e24dcca9e
    uint8_t service_uuid[16] = {0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0xf3, 0x93, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e};
    uint8_t tx_uuid[16]      = {0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0xf3, 0x93, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e};
    uint8_t rx_uuid[16]      = {0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0xf3, 0x93, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e};

    // Configure BLE
    ble_bidi_fast_config_t config = {
        .device_name = "SimpleEcho",
        .device_name_len = 10,
        .on_rx_receive_callback = on_rx_receive_callback,
    };

    // Initialize BLE
    ESP_ERROR_CHECK(ble_bidi_fast_init(&config, service_uuid, tx_uuid, rx_uuid));
    ESP_LOGI(TAG, "BLE initialized");

    // Configure LED
    configure_led();
    ESP_LOGI(TAG, "LED configured");

    // Start LED blink task
    xTaskCreate(led_blink_task, "led_blink_task", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "LED blink task started");
}
```

**Testing**:

1. Flash the example to an ESP32 device (e.g., Xiao ESP32C6 with LED on GPIO 15).
2. Use a BLE client app like nRF Connect to connect to the device named `SimpleEcho`.
3. Enable notifications on the TX Characteristic (`6e400002-b5a3-93f3-e0a9-e50e24dcca9e`) by writing `0x01 0x00` to its CCCD.
4. Observe the LED blinking every 500ms.
5. Write data to the RX Characteristic (`6e400003-b5a3-93f3-e0a9-e50e24dcca9e`).
6. Verify that the same data is echoed back via the TX Characteristic.

### API

- **`ble_bidi_fast_init`**: Initializes the BLE module with the provided configuration and UUIDs. Returns `ESP_OK` on success or an error code on failure.
- **`ble_bidi_fast_tx_send`**: Transmits data to the connected client using the Notify property of the TX Characteristic. Returns `ESP_OK` on success or `ESP_ERR_INVALID_STATE` if notifications are not enabled or data length is invalid.
- **`ble_bidi_fast_is_tx_notification_enabled`**: Checks if the client has enabled notifications on the TX Characteristic. Returns `true` if notifications are enabled, `false` otherwise.
- **`on_rx_receive_callback`**: User-defined callback invoked when data is received from the client via the RX Characteristic.

## Notes

- **Device Name Length**: Keep the device name under 20 bytes to fit within the 31-byte advertising data limit.
- **NVS Initialization**: The user application must initialize NVS flash before calling `ble_bidi_fast_init`.
- **UUIDs**: Use 128-bit UUIDs for the service and characteristics to ensure uniqueness.
- **Error Handling**: Check return values of API functions to handle errors appropriately.
- **Connection Parameters**: The component sets preferred connection parameters (7.5ms–15ms interval, 0s latency, 4s timeout) for low-latency communication.

## License

MIT License. See `LICENSE` file for details.