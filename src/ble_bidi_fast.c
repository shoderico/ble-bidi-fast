#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "ble_bidi_fast.h"

#include "esp_log.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"


#define TAG "BLE_BIDI_FAST"

// Advertising parameters
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20, // advertising interval max (32ms)
    .adv_int_max = 0x40, // advertising interval max (64ms)
    .adv_type = ADV_TYPE_IND, // connectable advertise
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC, // public address
    .channel_map = ADV_CHNL_ALL, // use all channels
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY, // allow scan & connectable
};

// Internal state
static struct {
    esp_gatt_if_t gatt_if;
    uint16_t service_handle;
    uint16_t tx_char_handle;
    uint16_t tx_cccd_handle;
    uint16_t rx_char_handle;
    uint16_t conn_id;
    bool tx_is_notification_enabled;
    struct {
        ble_bidi_fast_config_t base; // Base configuration
        uint8_t service_uuid[UUID128_LEN]; // Service UUID
        uint8_t tx_char_uuid[UUID128_LEN]; // TX Characteristic UUID
        uint8_t rx_char_uuid[UUID128_LEN]; // RX Characteristic UUID
    } config; // Extended configuration with UUIDs
} ble_state = {
    .gatt_if = ESP_GATT_IF_NONE,
    .tx_is_notification_enabled = false,
};

// Advertising data buffers
static uint8_t adv_data[31];
static uint8_t scan_rsp_data[31];

// Configure advertising data
static void configure_adv_data(void)
{
    uint8_t p;

    //--------------------------------------------------------------------------------
    // Advertising data
    p = 0;
    uint8_t adv_data_len = 0;
    
    // Flags
    adv_data[ p++ ] = 2; // Length
    adv_data[ p++ ] = ESP_BLE_AD_TYPE_FLAG; // Type: 0x01: discoverability, connectability
    adv_data[ p++ ] = 0x06; // General discoverable, BLE only

    // Device Name
    adv_data[ p++ ] = ble_state.config.base.device_name_len + 1; // Length
    adv_data[ p++ ] = ESP_BLE_AD_TYPE_NAME_CMPL; // Type: 0x09: Complete Local Name
    memcpy( &adv_data[p], ble_state.config.base.device_name, ble_state.config.base.device_name_len);
    p += ble_state.config.base.device_name_len;

    // TX Power
    adv_data[ p++ ] = 2; // Length
    adv_data[ p++ ] = ESP_BLE_AD_TYPE_TX_PWR; // Type: 
    adv_data[ p++ ] = 0xEB; // -21 dBm

    adv_data_len = p;


    //--------------------------------------------------------------------------------
    // Scan response data
    p = 0;
    uint8_t scan_rsp_data_len = 0;

    // Service UUID
    scan_rsp_data[ p++ ] = UUID128_LEN + 1; // Length
    scan_rsp_data[ p++ ] = ESP_BLE_AD_TYPE_128SRV_CMPL; // Type:
    memcpy( &scan_rsp_data[p], ble_state.config.service_uuid, UUID128_LEN);
    p += UUID128_LEN;

    scan_rsp_data_len = p;

    //--------------------------------------------------------------------------------
    // Set advertising data
    esp_err_t ret;
    ret = esp_ble_gap_config_adv_data_raw(adv_data, adv_data_len);
    if (ret) {
        ESP_LOGE(TAG, "%s Setting raw advertising data failed", __func__);
    }
    ret = esp_ble_gap_config_scan_rsp_data_raw(scan_rsp_data, scan_rsp_data_len);
    if (ret) {
        ESP_LOGE(TAG, "%s Setting raw scan response data failed", __func__);
    }
}


// GAP event handler
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t ret = ESP_OK;
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT: // seq[b-2]
            ESP_LOGI(TAG, "GAP : Advertising data set complete");

            // start advertising
            ret = esp_ble_gap_start_advertising(&adv_params);
            break;
        
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "GAP : Advertising started");
            } else {
                ESP_LOGE(TAG, "GAP : Advertising start failed");
            }
            break;

        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "GAP : ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT");
            break;

        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "GAP : ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT");
            break;
            
        case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
            ESP_LOGI(TAG, "GAP : ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT");
            break;
            
        case ESP_GAP_BLE_CHANNEL_SELECT_ALGORITHM_EVT:
            ESP_LOGI(TAG, "GAP : ESP_GAP_BLE_CHANNEL_SELECT_ALGORITHM_EVT");
            break;;
        
        default:
            ESP_LOGI(TAG, "GAP : Unhandled GAP event: %d", event);
            break;
    }
}

// GATT event handler
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_err_t ret = ESP_OK;
    switch (event) {
        case ESP_GATTS_REG_EVT: // seq[a-2]
            ESP_LOGI(TAG, "GATT: GATT app registered, status=%d", param->reg.status);

            if (param->reg.status == ESP_GATT_OK) {

                // GATT interface
                ble_state.gatt_if = gatts_if;

                // Configure advertising data
                configure_adv_data();

                // prepare 
                esp_gatt_srvc_id_t service_id = {
                    .id.inst_id = 0,
                    .is_primary = true,
                    .id.uuid.len = ESP_UUID_LEN_128,
                };
                memcpy(service_id.id.uuid.uuid.uuid128, ble_state.config.service_uuid, UUID128_LEN);

                // Create GATT service  // triggers ESP_GATTS_CREATE_EVT.
                ret = esp_ble_gatts_create_service(gatts_if
                    , &service_id
                    , 12 // 12: The number of handles requested for this service.
                );
                if (ret) {
                    ESP_LOGE(TAG, "GATT: Create service failed, error code = %s", esp_err_to_name(ret));
                }
            }
            break;

        case ESP_GATTS_CREATE_EVT: // seq[a-3]
            ESP_LOGI(TAG, "GATT: Service created, handle=%d", param->create.service_handle);

            // GATT service handle
            ble_state.service_handle = param->create.service_handle;

            // prepare
            esp_bt_uuid_t tx_uuid = {
                .len = ESP_UUID_LEN_128
            };
            memcpy(tx_uuid.uuid.uuid128, ble_state.config.tx_char_uuid, UUID128_LEN);

            // Add a TX characteristic into a service. // triggers ESP_GATTS_ADD_CHAR_EVT.
            ret = esp_ble_gatts_add_char(ble_state.service_handle
                , &tx_uuid
                , ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE
                , ESP_GATT_CHAR_PROP_BIT_NOTIFY
                , NULL
                , NULL
            );
            if (ret) {
                ESP_LOGE(TAG, "GATT: Add a TX characteristic failed, error code = %s", esp_err_to_name(ret));
            }
            break;

        case ESP_GATTS_ADD_CHAR_EVT: // seq[a-4], seq[a-6]
            if (param->add_char.status == ESP_GATT_OK) {
                if (!ble_state.tx_char_handle) {
                    ESP_LOGI(TAG, "GATT: TX characteristic added, handle=%d", param->add_char.attr_handle);

                    // GATT TX characteristic handle
                    ble_state.tx_char_handle = param->add_char.attr_handle;

                    // prepare
                    esp_bt_uuid_t tx_cccd_uuid = {
                        .len = ESP_UUID_LEN_16, 
                        .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG
                    };

                    // Add TX CCCD
                    ret = esp_ble_gatts_add_char_descr(ble_state.service_handle
                        , &tx_cccd_uuid
                        , ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE
                        , NULL
                        , NULL
                    );
                    if (ret) {
                        ESP_LOGE(TAG, "GATT: Add a TX characteristic descriptor failed, error code = %s", esp_err_to_name(ret));
                    }
                } else {
                    ESP_LOGI(TAG, "GATT: RX characteristic added, handle=%d", param->add_char.attr_handle);

                    // GATT RX characteristic handle
                    ble_state.rx_char_handle = param->add_char.attr_handle;

                    // Start a service. // triggers ESP_GATTS_START_EVT.
                    ret = esp_ble_gatts_start_service(ble_state.service_handle);
                    if (ret) {
                        ESP_LOGE(TAG, "GATT: Start service failed, error code = %s", esp_err_to_name(ret));
                    }
                }
            } else {
                ESP_LOGE(TAG, "GATT: Add TX characteristic failed, status=%d", param->add_char.status);
            }
            break;

        case ESP_GATTS_ADD_CHAR_DESCR_EVT: // seq[a-5]
            if (param->add_char_descr.status == ESP_GATT_OK) {
                ESP_LOGI(TAG, "TX CCCD added, handle=%d", param->add_char_descr.attr_handle);

                // CCCD handle
                ble_state.tx_cccd_handle = param->add_char_descr.attr_handle;

                // prepare
                esp_bt_uuid_t rx_uuid = {
                    .len = ESP_UUID_LEN_128
                };
                memcpy(rx_uuid.uuid.uuid128, ble_state.config.rx_char_uuid, UUID128_LEN);

                // Add Receive Characteristic
                ret = esp_ble_gatts_add_char(ble_state.service_handle
                    , &rx_uuid
                    , ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE
                    , ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR
                    , NULL
                    , NULL
                );
                if (ret) {
                    ESP_LOGE(TAG, "GATT: Add RX characteristic failed, error code = %s", esp_err_to_name(ret));
                }
            } else {
                ESP_LOGE(TAG, "GATT: Add TX CCCD failed, status=%d (0x%x)", param->add_char_descr.status, param->add_char_descr.status);
            }
            break;

        case ESP_GATTS_START_EVT: // seq[a-7]
            ESP_LOGI(TAG, "Service started, handle=%d", param->start.service_handle);
            break;

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "GATT: Client connected, conn_id=%d", param->connect.conn_id);

            // Connection ID
            ble_state.conn_id = param->connect.conn_id;
            
            // Set preferred connection parameter
            ret = esp_ble_gap_set_prefer_conn_params(
                param->connect.remote_bda
                , 0x06 // min interval: 7.5ms
                , 0x0C // max interva: 15ms
                , 0    // latency : 0 sec
                , 400  // timeout : 4 sec
            );
            if (ret) {
                ESP_LOGE(TAG, "GATT: Set preferred connection params failed: %s", esp_err_to_name(ret));
            }
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "GATT: Client disconnected");
            ble_state.tx_is_notification_enabled = false;
            ble_state.conn_id = 0;

            // re-start advertising
            ret = esp_ble_gap_start_advertising(&adv_params);
            if (ret) {
                ESP_LOGE(TAG, "GATT: Re-start advertizing failed: %s", esp_err_to_name(ret));
            }
            break;

        case ESP_GATTS_WRITE_EVT:
            // Write to CCCD
            if (param->write.handle == ble_state.tx_cccd_handle) {
                
                // Notify enabled
                if (param->write.len == 2 && param->write.value[0] == 0x01 && param->write.value[1] == 0x00) {
                    ESP_LOGI(TAG, "GATT: TX Notify enabled");
                    ble_state.tx_is_notification_enabled = true;

                // Notify dsiabled
                } else if (param->write.len == 2 && param->write.value[0] == 0x00 && param->write.value[1] == 0x00) {
                    ESP_LOGI(TAG, "GATT: TX Nofity disabled");
                    ble_state.tx_is_notification_enabled = false;

                }

                // Need response?
                if (param->write.need_rsp) {
                    ESP_LOGI(TAG, "GATT: Send TX response");
                    ret = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                    if (ret) {
                        ESP_LOGE(TAG, "GATT: Send TX response failed: %s", esp_err_to_name(ret));
                    }
                }

            // RX Characteristic
            } else if (param->write.handle == ble_state.rx_char_handle) {

                if (ble_state.config.base.rx_receive_callback) {

                    // Call RX callback
                    ble_state.config.base.rx_receive_callback(param->write.value, param->write.len);
                }

                // Need response?
                if (param->write.need_rsp) {
                    ESP_LOGI(TAG, "GATT: Send RX response");
                    ret = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                    if (ret) {
                        ESP_LOGE(TAG, "GATT: Send RX response failed: %s", esp_err_to_name(ret));
                    }
                }
            }
            break;

        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(TAG, "GATT: ESP_GATTS_MTU_EVT");
            break;

        case ESP_GATTS_CONF_EVT:
            if (param->conf.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "GATT: Notify confirm failed, status=%d (0x%x)", param->conf.status, param->conf.status);
            }
            break;

        case ESP_GATTS_RESPONSE_EVT:
            if (param->rsp.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "GATT: response failed, status=%d (0x%x)", param->rsp.status, param->rsp.status);
            }
            break;

        default:
            ESP_LOGI(TAG, "GATT: Unhandled GATT event: %d", event);
            break;
    }
}

// Initialize the BLE module
esp_err_t ble_bidi_fast_init(const ble_bidi_fast_config_t *config,
                             const uint8_t *service_uuid,
                             const uint8_t *tx_char_uuid,
                             const uint8_t *rx_char_uuid)
{
    esp_err_t ret;

    //-----------------------------------------------------------------------------
    // Validate inputs
    if (!config || !service_uuid || !tx_char_uuid || !rx_char_uuid) {
        ESP_LOGE(TAG, "Invalid input parameters");
        return ESP_ERR_INVALID_ARG;
    }

    //-----------------------------------------------------------------------------
    // Copy configuration
    ble_state.config.base = *config;
    memcpy(ble_state.config.service_uuid, service_uuid, UUID128_LEN);
    memcpy(ble_state.config.tx_char_uuid, tx_char_uuid, UUID128_LEN);
    memcpy(ble_state.config.rx_char_uuid, rx_char_uuid, UUID128_LEN);

    //-----------------------------------------------------------------------------
    // Initialize Bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_LOGI(TAG, "Initializing BT controller");
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Enabling BT controller");
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    //-----------------------------------------------------------------------------
    // Initialize Bluedroid
    ESP_LOGI(TAG, "Initializing Bluedroid");
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Enabling Bluedroid");
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    //-----------------------------------------------------------------------------
    // Register callbacks
    ESP_LOGI(TAG, "Registering GATT callback");
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "%s gatt callback register error, error code = %s", __func__, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Registering GAP callback");
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "%s gap callback register error, error code = %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    //-----------------------------------------------------------------------------
    // Register GATT app
    ESP_LOGI(TAG, "Registering GATT app");
    ret = esp_ble_gatts_app_register(0); // 0: app_id // triggers ESP_GATTS_REG_EVT. seq[a-1]
    if (ret) {
        ESP_LOGE(TAG, "%s gatt app register error, error code = %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

// Send data from TX to the connected client
esp_err_t ble_bidi_fast_tx_send(/*const*/ uint8_t *data, uint8_t len)
{
    if (!ble_state.tx_is_notification_enabled || ble_state.tx_char_handle == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    return esp_ble_gatts_send_indicate(
          ble_state.gatt_if
        , ble_state.conn_id
        , ble_state.tx_char_handle
        , len
        , data
        , false
    );
}

// Retreive TX notification is enabled or not
bool ble_bidi_fast_tx_is_notification_enabled()
{
    return ble_state.tx_is_notification_enabled;
}