#include "ble_bidi_fast.h"
#include "esp_log.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#define BLE_BIDI_FAST_TAG "BLE_BIDI_FAST"

// Advertising parameters
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20, // 32ms
    .adv_int_max = 0x40, // 64ms
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// Internal state
static struct {
    esp_gatt_if_t gatt_if;
    uint16_t service_handle;
    uint16_t send_char_handle;
    uint16_t cccd_handle;
    uint16_t receive_char_handle;
    uint16_t conn_id;
    bool is_send_enabled;
    ble_bidi_fast_config_t config; // User configuration
} ble_state = {
    .gatt_if = ESP_GATT_IF_NONE,
    .is_send_enabled = false,
};

// Advertising data buffers
static uint8_t adv_data[31];
static uint8_t scan_rsp_data[31];

// Timer for periodic sending (reserved for future use)
static TimerHandle_t send_timer;

// Configure advertising data
static void configure_adv_data(void) {
    uint8_t *p = adv_data;
    // Flags
    *p++ = 2; // Length
    *p++ = ESP_BLE_AD_TYPE_FLAG;
    *p++ = 0x06; // General discoverable, BLE only

    // Device Name
    *p++ = ble_state.config.device_name_len + 1; // Length
    *p++ = ESP_BLE_AD_TYPE_NAME_CMPL;
    memcpy(p, ble_state.config.device_name, ble_state.config.device_name_len);
    p += ble_state.config.device_name_len;

    // TX Power
    *p++ = 2;
    *p++ = ESP_BLE_AD_TYPE_TX_PWR;
    *p++ = 0xEB; // -21 dBm

    // Scan Response: Service UUID
    p = scan_rsp_data;
    *p++ = UUID128_LEN + 1; // Length
    *p++ = ESP_BLE_AD_TYPE_128SRV_CMPL;
    memcpy(p, ble_state.config.service_uuid, UUID128_LEN);

    // Set advertising data
    esp_ble_gap_config_adv_data_raw(adv_data, p - adv_data);
    esp_ble_gap_config_scan_rsp_data_raw(scan_rsp_data, UUID128_LEN + 2);
}

// GAP event handler
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            ESP_LOGI(BLE_BIDI_FAST_TAG, "Advertising data set complete");
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(BLE_BIDI_FAST_TAG, "Advertising started");
            } else {
                ESP_LOGE(BLE_BIDI_FAST_TAG, "Advertising start failed");
            }
            break;
        default:
            ESP_LOGI(BLE_BIDI_FAST_TAG, "Unhandled GAP event: %d", event);
            break;
    }
}

// GATT event handler
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    esp_err_t ret;
    switch (event) {
        case ESP_GATTS_REG_EVT:
            if (param->reg.status == ESP_GATT_OK) {
                ble_state.gatt_if = gatts_if;

                // Configure advertising data
                configure_adv_data();

                // Create GATT service
                esp_gatt_srvc_id_t service_id = {
                    .id.inst_id = 0,
                    .is_primary = true,
                    .id.uuid.len = ESP_UUID_LEN_128,
                };
                memcpy(service_id.id.uuid.uuid128, ble_state.config.service_uuid, UUID128_LEN);
                esp_ble_gatts_create_service(gatts_if, &service_id, 12);
            }
            break;

        case ESP_GATTS_CREATE_EVT:
            ble_state.service_handle = param->create.service_handle;
            ESP_LOGI(BLE_BIDI_FAST_TAG, "Service created, handle=%d", ble_state.service_handle);

            // Add Send Characteristic
            esp_bt_uuid_t send_uuid = {.len = ESP_UUID_LEN_128};
            memcpy(send_uuid.uuid.uuid128, ble_state.config.send_char_uuid, UUID128_LEN);
            esp_ble_gatts_add_char(ble_state.service_handle, &send_uuid,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_NOTIFY, NULL, NULL);
            break;

        case ESP_GATTS_ADD_CHAR_EVT:
            if (param->add_char.status == ESP_GATT_OK) {
                if (!ble_state.send_char_handle) {
                    ble_state.send_char_handle = param->add_char.attr_handle;
                    ESP_LOGI(BLE_BIDI_FAST_TAG, "Send characteristic added, handle=%d", ble_state.send_char_handle);

                    // Add CCCD
                    esp_bt_uuid_t cccd_uuid = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG};
                    esp_ble_gatts_add_char_descr(ble_state.service_handle, &cccd_uuid,
                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
                } else {
                    ble_state.receive_char_handle = param->add_char.attr_handle;
                    ESP_LOGI(BLE_BIDI_FAST_TAG, "Receive characteristic added, handle=%d", ble_state.receive_char_handle);
                    esp_ble_gatts_start_service(ble_state.service_handle);
                }
            } else {
                ESP_LOGE(BLE_BIDI_FAST_TAG, "Add characteristic failed, status=%d", param->add_char.status);
            }
            break;

        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            if (param->add_char_descr.status == ESP_GATT_OK) {
                ble_state.cccd_handle = param->add_char_descr.attr_handle;
                ESP_LOGI(BLE_BIDI_FAST_TAG, "CCCD added, handle=%d", ble_state.cccd_handle);

                // Add Receive Characteristic
                esp_bt_uuid_t receive_uuid = {.len = ESP_UUID_LEN_128};
                memcpy(receive_uuid.uuid.uuid128, ble_state.config.receive_char_uuid, UUID128_LEN);
                esp_ble_gatts_add_char(ble_state.service_handle, &receive_uuid,
                                       ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                       ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR, NULL, NULL);
            } else {
                ESP_LOGE(BLE_BIDI_FAST_TAG, "Add CCCD failed, status=%d", param->add_char_descr.status);
            }
            break;

        case ESP_GATTS_START_EVT:
            ESP_LOGI(BLE_BIDI_FAST_TAG, "Service started, handle=%d", param->start.service_handle);
            break;

        case ESP_GATTS_CONNECT_EVT:
            ble_state.conn_id = param->connect.conn_id;
            ESP_LOGI(BLE_BIDI_FAST_TAG, "Client connected, conn_id=%d", ble_state.conn_id);
            esp_ble_gap_set_prefer_conn_params(param->connect.remote_bda, 0x06, 0x0C, 0, 400);
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(BLE_BIDI_FAST_TAG, "Client disconnected");
            ble_state.is_send_enabled = false;
            ble_state.conn_id = 0;
            esp_ble_gap_start_advertising(&adv_params);
            break;

        case ESP_GATTS_WRITE_EVT:
            if (param->write.handle == ble_state.cccd_handle) {
                if (param->write.len == 2 && param->write.value[0] == 0x01 && param->write.value[1] == 0x00) {
                    ESP_LOGI(BLE_BIDI_FAST_TAG, "Send enabled");
                    ble_state.is_send_enabled = true;
                } else if (param->write.len == 2 && param->write.value[0] == 0x00 && param->write.value[1] == 0x00) {
                    ESP_LOGI(BLE_BIDI_FAST_TAG, "Send disabled");
                    ble_state.is_send_enabled = false;
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            } else if (param->write.handle == ble_state.receive_char_handle) {
                if (param->write.len <= BLE_BIDI_FAST_MAX_DATA_LEN && ble_state.config.on_receive_callback) {
                    ble_state.config.on_receive_callback(param->write.value, param->write.len);
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            break;

        default:
            ESP_LOGI(BLE_BIDI_FAST_TAG, "Unhandled GATT event: %d", event);
            break;
    }
}

// Callback for send timer (reserved for future use)
static void send_timer_callback(TimerHandle_t xTimer) {
    // User calls ble_bidi_fast_send directly, so this is empty
}

// Initialize the BLE module
esp_err_t ble_bidi_fast_init(const ble_bidi_fast_config_t *config) {
    esp_err_t ret;

    // Copy configuration
    memcpy(&ble_state.config, config, sizeof(ble_bidi_fast_config_t));

    // Initialize Bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    // Initialize Bluedroid
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Register callbacks
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));

    // Register GATT app
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));

    // Create send timer (for future use)
    send_timer = xTimerCreate("SendTimer", pdMS_TO_TICKS(30), pdTRUE, NULL, send_timer_callback);
    if (send_timer == NULL) {
        ESP_LOGE(BLE_BIDI_FAST_TAG, "Failed to create send timer");
        return ESP_FAIL;
    }

    return ESP_OK;
}

// Send data to the connected client
esp_err_t ble_bidi_fast_send(const uint8_t *data, uint8_t len) {
    if (!ble_state.is_send_enabled || len > BLE_BIDI_FAST_MAX_DATA_LEN || ble_state.send_char_handle == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    return esp_ble_gatts_send_indicate(ble_state.gatt_if, ble_state.conn_id,
                                       ble_state.send_char_handle, len, data, false);
}