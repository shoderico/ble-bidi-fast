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

// Task to blink LED when TX Characteristic's Notification is On. ( CCCD: On=0x00:00, Off= 0x00:00 )
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
static void on_rx_receive_callback(/*const*/ uint8_t *data, uint8_t len) {
    ESP_LOGI(TAG, "Received %d bytes", len);
    
    // Echo back the received data to TX
    if (ble_bidi_fast_is_tx_notification_enabled()) {
        esp_err_t ret = ble_bidi_fast_tx_send(data, len);
        if (ret) {
            ESP_LOGW(TAG, "Failed to send, error=%d", ret);
        } else {
            ESP_LOGI(TAG, "Echoed %d bytes", len);
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