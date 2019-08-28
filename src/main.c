/*
 *   Copyright 2019 Byron Ellacott
 *
 *   Portions of this file are derived from examples provided by
 *   Espressif Systems at
 *
 *      https://github.com/espressif/esp-idf
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_main.h"

#define BLEJOY_TAG              "BLEJOY"
#define BLEJOY_DEVICE_NAME      "ESP32 JOYSTICK"
#define BLEJOY_PROFILE_APP_ID   0xb1e
#define BLEJOY_SVC_INST_ID      0x00

#define PIN_BUTTON_0            16
#define PIN_BUTTON_1            17
#define PIN_BUTTON_2            18
#define PIN_BUTTON_3            19
#define PIN_BUTTON_4            21
#define PIN_BUTTON_5            22
#define PIN_BUTTON_START        23
#define PIN_BUTTON_SELECT       32
#define PIN_JOYSTICK_LEFT       33
#define PIN_JOYSTICK_RIGHT      25
#define PIN_JOYSTICK_UP         26
#define PIN_JOYSTICK_DOWN       27

#define PIN_SELECTOR (BIT64(PIN_BUTTON_0) | BIT64(PIN_BUTTON_1) | \
                      BIT64(PIN_BUTTON_2) | BIT64(PIN_BUTTON_3) | \
                      BIT64(PIN_BUTTON_4) | BIT64(PIN_BUTTON_5) | \
                      BIT64(PIN_BUTTON_START) | BIT64(PIN_BUTTON_SELECT) | \
                      BIT64(PIN_JOYSTICK_LEFT) | BIT64(PIN_JOYSTICK_RIGHT) | \
                      BIT64(PIN_JOYSTICK_UP)   | BIT64(PIN_JOYSTICK_DOWN))
#define ESP_INTR_FLAG_DEFAULT 0  

    // Service Description (Battery Service)
    //      

#define BLEJOY_SERVICE_UUID     0xFF

static void gatts_profile_event_handler(esp_gatts_cb_event_t, esp_gatt_if_t, esp_ble_gatts_cb_param_t *);

struct gatts_service_inst {
    esp_bt_uuid_t service_uuid;
    const esp_gatts_attr_db_t *attr_tab;
    uint16_t attr_count;
    uint16_t *handles;
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t service_count;
    struct gatts_service_inst services[];
};

struct hid_report {
    uint16_t button_0 : 1;
    uint16_t button_1 : 1;
    uint16_t button_2 : 1;
    uint16_t button_3 : 1;
    uint16_t button_4 : 1;
    uint16_t button_5 : 1;
    uint16_t button_6 : 1;
    uint16_t button_7 : 1;
    uint16_t button_8 : 1;
    uint16_t button_9 : 1;
    uint16_t button_10 : 1;
    uint16_t button_11 : 1;
    uint16_t button_12 : 1;
    uint16_t button_13 : 1;
    uint16_t button_14 : 1;
    uint16_t button_15 : 1;
    int8_t joystick_x;
    int8_t joystick_y;
} __attribute__((packed));

struct pnp_id {
    uint8_t vendor_source;
    uint16_t vendor_id;
    uint16_t product_id;
    uint16_t version;
} __attribute__((packed));

struct hid_info {
    uint16_t version;   // HID spec version
    uint8_t country;
    uint8_t wakeable : 1;
    uint8_t normally_connectable : 2;
    uint8_t filler : 6;
};

const uint8_t report_map[] = {
  0x05, 0x01,  /* USAGE_PAGE (Generic Desktop)       */
  0x09, 0x05,  /* USAGE (Game Pad)                   */
  0xa1, 0x01,  /* COLLECTION (Application)           */
  0xa1, 0x03,  /*   COLLECTION (Report)              */
  0x85, 0x01,  /*     REPORT_ID (1)                  */
  0x05, 0x09,  /*     USAGE_PAGE (Button)            */
  0x19, 0x01,  /*     USAGE_MINIMUM (Button 1)       */
  0x29, 0x0e,  /*     USAGE_MAXIMUM (Button 14)      */
  0x15, 0x00,  /*     LOGICAL_MINIMUM (0)            */
  0x25, 0x01,  /*     LOGICAL_MAXIMUM (1)            */
  0x95, 0x0e,  /*     REPORT_COUNT (14)              */
  0x75, 0x01,  /*     REPORT_SIZE (1)                */
  0x81, 0x02,  /*     INPUT (Data,Var,Abs)           */
  0x95, 0x01,  /*     REPORT_COUNT (1)               */
  0x75, 0x02,  /*     REPORT_SIZE (2)                */
  0x81, 0x03,  /*     INPUT (Cnst)                   */
  0xa1, 0x00,  /*     COLLECTION (Physical)          */
  0x05, 0x01,  /*       USAGE_PAGE (Generic Desktop) */  
  0x09, 0x30,  /*       USAGE (X)                    */
  0x09, 0x31,  /*       USAGE (Y)                    */
  0x15, 0x00,  /*       LOGICAL_MINIMUM (0)          */
  0x25, 0x64,  /*       LOGICAL_MAXIMUM (100)        */
  0x75, 0x08,  /*       REPORT_SIZE (8)              */
  0x95, 0x02,  /*       REPORT_COUNT (2)             */
  0x81, 0x02,  /*       INPUT (Data,Var,Abs)         */
  0xc0,        /*     END_COLLECTION                 */
  0xc0,        /*   END_COLLECTION                   */
  0xc0         /* END_COLLECTION                     */
};

/** Static definitions for services **/
static uint16_t uuid_primary_service    = ESP_GATT_UUID_PRI_SERVICE;
static uint16_t uuid_char_declaration   = ESP_GATT_UUID_CHAR_DECLARE;
static uint16_t uuid_cccd               = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static uint16_t uuid_report_reference   = ESP_GATT_UUID_RPT_REF_DESCR;

static uint16_t uuid_battery_service    = ESP_GATT_UUID_BATTERY_SERVICE_SVC;
static uint16_t uuid_battery_level      = ESP_GATT_UUID_BATTERY_LEVEL;
static uint16_t uuid_hid_service        = ESP_GATT_UUID_HID_SVC;
static uint16_t uuid_hid_report_map     = ESP_GATT_UUID_HID_REPORT_MAP;
static uint16_t uuid_hid_report         = ESP_GATT_UUID_HID_REPORT;
static uint16_t uuid_dev_info_service   = ESP_GATT_UUID_DEVICE_INFO_SVC;
static uint16_t uuid_pnp_id             = ESP_GATT_UUID_PNP_ID;
static uint16_t uuid_hid_information    = ESP_GATT_UUID_HID_INFORMATION;

static uint8_t char_prop_read           = ESP_GATT_CHAR_PROP_BIT_READ;
static uint8_t char_prop_read_notify    = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static uint8_t battery_level            = 50;
static struct pnp_id pnp_id             = { 0x01, 0x2e5, 0xbeef, 0x100 };
static struct hid_report hid_report     = {};
static struct hid_info hid_info         = { .version = 0x100, .country = 0, .wakeable = 0, .normally_connectable = 1 };
static uint16_t report_ccc              = 0;
static uint16_t report_ref              = 0x0101;       // 01 = id, 01 = INPUT type

// All attributes to be registered for the Battery Service
static const esp_gatts_attr_db_t svc_battery[] = {
    // Service Declaration: battery service
    { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16, (uint8_t *)&uuid_primary_service, ESP_GATT_PERM_READ,
            sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&uuid_battery_service } },
    
    // Characteristic Declaration: battery level
    { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16, (uint8_t *)&uuid_char_declaration, ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), &char_prop_read_notify } },

    // Characteristic Value: battery level
    { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16, (uint8_t *)&uuid_battery_level, ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), &battery_level } },

};
#define SVC_BATTERY_COUNT (sizeof svc_battery / sizeof (esp_gatts_attr_db_t))

// All attributes to be registered for the HID Service
static const esp_gatts_attr_db_t svc_hid[] = {
    // Service Declaration: HID service
    { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16, (uint8_t *)&uuid_primary_service, ESP_GATT_PERM_READ,
            sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&uuid_hid_service } },

    // Protocol Mode: mandatory for keyboard, mouse, and optional for all other HID devices; skip

    // Report Map
    { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16, (uint8_t *)&uuid_char_declaration, ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), &char_prop_read } },
    { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16, (uint8_t *)&uuid_hid_report_map, ESP_GATT_PERM_READ,
            sizeof(report_map), sizeof(report_map), (uint8_t *)report_map } },

    // Input Report for the gamepad data
    { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16, (uint8_t *)&uuid_char_declaration, ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), &char_prop_read_notify } },
    { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16, (uint8_t *)&uuid_hid_report, ESP_GATT_PERM_READ,
            sizeof(hid_report), sizeof(hid_report), (uint8_t *)&hid_report } },
    { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16, (uint8_t *)&uuid_cccd, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&report_ccc } },
    { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16, (uint8_t *)&uuid_report_reference, ESP_GATT_PERM_READ,
            sizeof(report_ref), sizeof(report_ref), (uint8_t *)&report_ref } },
    
    // Boot Protocol input/output reports excluded

    // HID Information
    { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16, (uint8_t *)&uuid_char_declaration, ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), &char_prop_read} },
    { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16, (uint8_t *)&uuid_hid_information, ESP_GATT_PERM_READ,
            sizeof(hid_info), sizeof(hid_info), (uint8_t *)&hid_info } },

    // HID Control Point: skip (used for notifying device of sleep/wake)
};
#define SVC_HID_COUNT (sizeof svc_hid / sizeof (esp_gatts_attr_db_t))

// All attributes to be registered for the Device Information Service
static const esp_gatts_attr_db_t svc_device_info[] = {
    // Service Declaration: device information service
    { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16, (uint8_t *)&uuid_primary_service, ESP_GATT_PERM_READ,
            sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&uuid_dev_info_service } },
    
    // Characteristic Declaration: PnP ID
    { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16, (uint8_t *)&uuid_char_declaration, ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), &char_prop_read } },

    // Characteristic Value: PnP ID
    { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16, (uint8_t *)&uuid_pnp_id, ESP_GATT_PERM_READ,
            sizeof(pnp_id), sizeof(pnp_id), (uint8_t *)&pnp_id } },

};
#define SVC_DEVINFO_COUNT (sizeof svc_device_info / sizeof (esp_gatts_attr_db_t))

static struct gatts_profile_inst blejoy_profile = {
    gatts_profile_event_handler,
    ESP_GATT_IF_NONE,
    3,
    {
        { { ESP_UUID_LEN_16, .uuid.uuid16 = ESP_GATT_UUID_BATTERY_SERVICE_SVC }, svc_battery, SVC_BATTERY_COUNT, NULL },
        { { ESP_UUID_LEN_16, .uuid.uuid16 = ESP_GATT_UUID_DEVICE_INFO_SVC }, svc_device_info, SVC_DEVINFO_COUNT, NULL },
        { { ESP_UUID_LEN_16, .uuid.uuid16 = ESP_GATT_UUID_HID_SVC }, svc_hid, SVC_HID_COUNT, NULL },
    },
};

static esp_ble_adv_params_t blejoy_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// https://www.bluetooth.com/specifications/gatt/services/
static uint8_t service_uuid[] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00, // HID
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x0F, 0x18, 0x00, 0x00, // battery
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x0A, 0x18, 0x00, 0x00, // device info
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = false,
    .min_interval        = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval        = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance          = ESP_BLE_APPEARANCE_HID_GAMEPAD, // or JOYSTICK?
    .manufacturer_len    = 0,
    .p_manufacturer_data = NULL,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = false,
    .min_interval        = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval        = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance          = ESP_BLE_APPEARANCE_HID_GAMEPAD, // or JOYSTICK?
    .manufacturer_len    = 0,
    .p_manufacturer_data = NULL,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static uint8_t start_sem = 0;
static bool connected = false;
static uint16_t conn_id = 0;

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            blejoy_profile.gatts_if = gatts_if;
        } else {
            ESP_LOGE(BLEJOY_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    blejoy_profile.gatts_cb(event, gatts_if, param);
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_err_t ret;
    struct gatts_service_inst *inst;
    esp_ble_conn_update_params_t conn_params = {0};

    switch (event) {
        case ESP_GATTS_REG_EVT:
            ret = esp_ble_gap_set_device_name(BLEJOY_DEVICE_NAME);
            if (ret) {
                ESP_LOGE(BLEJOY_TAG, "set device name failed, error code = %x", ret);
            }

            ret = esp_ble_gap_config_local_icon(ESP_BLE_APPEARANCE_HID_GAMEPAD);
            if (ret) {
                ESP_LOGE(BLEJOY_TAG, "%s configure local icon failed, error code = %x", __func__, ret);
            }

            //config adv data
            ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret) {
                ESP_LOGE(BLEJOY_TAG, "config adv data failed, error code = %x", ret);
            }

            ret = esp_ble_gap_config_adv_data(&scan_data);
            if (ret) {
                ESP_LOGE(BLEJOY_TAG, "config scan rsp data failed, error code = %x", ret);
            }

            start_sem = ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT
                      | ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT;

            for (int i = 0; i < blejoy_profile.service_count; i++) {
                esp_ble_gatts_create_attr_tab(blejoy_profile.services[i].attr_tab, gatts_if,
                        blejoy_profile.services[i].attr_count, BLEJOY_SVC_INST_ID);
            }

            break;

        case ESP_GATTS_CREAT_ATTR_TAB_EVT:

            inst = NULL;
            for (int i = 0; i < blejoy_profile.service_count; i++) {
                // Note: this only supports 16-bit UUIDs
                if (param->add_attr_tab.svc_uuid.uuid.uuid16 == blejoy_profile.services[i].service_uuid.uuid.uuid16) {
                    inst = blejoy_profile.services + i;
                    break;
                }
            }

            if (inst != NULL) {
                ESP_LOGI(BLEJOY_TAG, "create attr table for service %x", param->add_attr_tab.svc_uuid.uuid.uuid16);
                if (param->add_attr_tab.status != ESP_GATT_OK){
                    ESP_LOGE(BLEJOY_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
                } else if (param->add_attr_tab.num_handle != inst->attr_count){
                    ESP_LOGE(BLEJOY_TAG, "create attribute table incorrect handle count, num_handle (%d) \
                        != attr_count (%d)", param->add_attr_tab.num_handle, inst->attr_count);
                } else {
                    ESP_LOGI(BLEJOY_TAG, "create attribute table successfully, num_handle = %d",param->add_attr_tab.num_handle);
                    for (int i = 0; i < param->add_attr_tab.num_handle; i++) {
                        ESP_LOGI(BLEJOY_TAG, "  handle %d = %d", i, param->add_attr_tab.handles[i]);
                    }
                    inst->handles = malloc(inst->attr_count * sizeof(uint16_t));
                    if (inst->handles != NULL) {
                        memcpy(inst->handles, param->add_attr_tab.handles, inst->attr_count * sizeof(uint16_t));
                        esp_ble_gatts_start_service(inst->handles[0]);
                    } else {
                        ESP_LOGE(BLEJOY_TAG, "failed to allocate handles array");
                    }
                }
            }

            break;

        case ESP_GATTS_START_EVT:
            ESP_LOGI(BLEJOY_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;

        case ESP_GATTS_READ_EVT:
            ESP_LOGI(BLEJOY_TAG, "READ_EVT, connection %d, attribute %d, offset %d, is_long %s, need_rsp %s",
                param->read.conn_id, param->read.handle, param->read.offset,
                param->read.is_long ? "true" : "false", param->read.need_rsp ? "true" : "false");
            break;

        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(BLEJOY_TAG, "WRITE_EVT, connection %d, attribute %d, offset %d, is_prep %s, need_rsp %s",
                param->write.conn_id, param->write.handle, param->write.offset,
                param->write.is_prep ? "true" : "false", param->write.need_rsp ? "true" : "false");
            esp_log_buffer_hex(BLEJOY_TAG, param->write.value, param->write.len);
            ESP_LOGI(BLEJOY_TAG, "report_ccc = %x", report_ccc);
            hid_report.joystick_x = 100;
            esp_ble_gatts_send_indicate(blejoy_profile.gatts_if, param->write.conn_id,
                blejoy_profile.services[2].handles[4], sizeof(hid_report), (uint8_t *)&hid_report,
                false);
            break;

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(BLEJOY_TAG, "CONNECT_EVT");
            esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT);
            conn_id = param->connect.conn_id;
            connected = true;
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            break;

        case ESP_GATTS_CONF_EVT:
            // ESP_LOGI(BLEJOY_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(BLEJOY_TAG, "DISCONNECT_EVT");
            connected = false;
            break;

        default:
            ESP_LOGW(BLEJOY_TAG, "gatts profile event %d", event);
            break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            start_sem &= ~(ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT);
            if (!start_sem) esp_ble_gap_start_advertising(&blejoy_adv_params);
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            start_sem &= ~(ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT);
            if (!start_sem) esp_ble_gap_start_advertising(&blejoy_adv_params);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            //advertising start complete event to indicate advertising start successfully or failed
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(BLEJOY_TAG, "advertising start failed");
            }
            break;
        default:
            ESP_LOGW(BLEJOY_TAG, "gap event %d", event);
    }
}

static TaskHandle_t input_task_handle;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    BaseType_t woken = pdFALSE;
    xTaskNotifyFromISR(input_task_handle, gpio_num, eSetBits, &woken);  
    if (woken == pdTRUE) portYIELD_FROM_ISR();
}

/* Process the input queue and send notifications to the client if connected */
void input_task(void *args) {
    while (1) {

        // Wait for a notification, or 500ms, whichever comes first
        uint32_t changed = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));

        if (connected) {

            uint32_t gpio1 = REG_READ(GPIO_IN_REG);
            uint32_t gpio2 = REG_READ(GPIO_IN1_REG);
            uint64_t gpio = gpio1 | ((uint64_t)gpio2 << 32);

            hid_report.joystick_x = (gpio & BIT64(PIN_JOYSTICK_LEFT))
                                  ? ((gpio & BIT64(PIN_JOYSTICK_RIGHT)) ? 50 : 100)
                                  : 0;
            hid_report.joystick_y = (gpio & BIT64(PIN_JOYSTICK_UP))
                                  ? ((gpio & BIT64(PIN_JOYSTICK_DOWN)) ? 50 : 0)
                                  : 100;
            hid_report.button_0 = !(gpio & BIT64(PIN_BUTTON_0));
            hid_report.button_1 = !(gpio & BIT64(PIN_BUTTON_1));
            hid_report.button_2 = !(gpio & BIT64(PIN_BUTTON_2));
            hid_report.button_3 = !(gpio & BIT64(PIN_BUTTON_3));
            hid_report.button_4 = !(gpio & BIT64(PIN_BUTTON_4));
            hid_report.button_5 = !(gpio & BIT64(PIN_BUTTON_5));
            hid_report.button_9 = !(gpio & BIT64(PIN_BUTTON_START));
            hid_report.button_8 = !(gpio & BIT64(PIN_BUTTON_SELECT));
            if (changed) {
                ESP_LOGI(BLEJOY_TAG, "joystick x: %d, joystick y: %d", hid_report.joystick_x, hid_report.joystick_y);
                ESP_LOGI(BLEJOY_TAG, "reg: %08llx (%08llx)", gpio, gpio & PIN_SELECTOR);
            }

            esp_ble_gatts_send_indicate(blejoy_profile.gatts_if, conn_id,
                blejoy_profile.services[2].handles[4], sizeof(hid_report), (uint8_t *)&hid_report,
                false);

        }
    }
}

static void show_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();

    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    ESP_LOGI(BLEJOY_TAG, "Bonded devices number : %d", dev_num);

    ESP_LOGI(BLEJOY_TAG, "Bonded devices list : %d", dev_num);
    for (int i = 0; i < dev_num; i++) {
        esp_log_buffer_hex(BLEJOY_TAG, (void *)dev_list[i].bd_addr, sizeof(esp_bd_addr_t));
    }

    free(dev_list);
}

void app_main() {
    esp_err_t ret;

    // Initialise NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(BLEJOY_TAG, "%s resetting NVS flash", __func__);
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialise BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(BLEJOY_TAG, "%s enable controller failed", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(BLEJOY_TAG, "%s enable controller failed", __func__);
        return;
    }

    ESP_LOGI(BLEJOY_TAG, "%s init bluetooth", __func__);
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(BLEJOY_TAG, "%s init bluetooth failed", __func__);
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(BLEJOY_TAG, "%s enable bluetooth failed", __func__);
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(BLEJOY_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(BLEJOY_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(BLEJOY_PROFILE_APP_ID);
    if (ret) {
        ESP_LOGE(BLEJOY_TAG, "%s register app failed, error code = %x", __func__, ret);
        return;
    }

    // Configure security parameters
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_BOND;
    esp_ble_io_cap_t io_cap = ESP_IO_CAP_NONE;  // no input, no output
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &io_cap, sizeof(io_cap));

    ESP_LOGI(BLEJOY_TAG, "%s bluetooth enabled", __func__);
    show_bonded_devices();
    
    gpio_config_t config = {
        .pin_bit_mask = 0x04,       // GPIO2
        .mode         = GPIO_MODE_OUTPUT,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&config);

    xTaskCreate(&input_task, "INPUT", configMINIMAL_STACK_SIZE * 5,
            NULL, tskIDLE_PRIORITY + 5, &input_task_handle);

    // Configure buttons and joystick as input, pull-up, intr on rise/fall
    config.intr_type = GPIO_PIN_INTR_ANYEDGE;
    config.mode = GPIO_MODE_INPUT;
    config.pull_up_en = 1;
    config.pull_down_en = 0;
    config.pin_bit_mask = PIN_SELECTOR;
    gpio_config(&config);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_BUTTON_0, gpio_isr_handler, (void *)PIN_BUTTON_0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_BUTTON_1, gpio_isr_handler, (void *)PIN_BUTTON_1));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_BUTTON_2, gpio_isr_handler, (void *)PIN_BUTTON_2));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_BUTTON_3, gpio_isr_handler, (void *)PIN_BUTTON_3));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_BUTTON_4, gpio_isr_handler, (void *)PIN_BUTTON_4));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_BUTTON_5, gpio_isr_handler, (void *)PIN_BUTTON_5));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_BUTTON_START, gpio_isr_handler, (void *)PIN_BUTTON_START));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_BUTTON_SELECT, gpio_isr_handler, (void *)PIN_BUTTON_SELECT));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_JOYSTICK_LEFT, gpio_isr_handler, (void *)PIN_JOYSTICK_LEFT));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_JOYSTICK_RIGHT, gpio_isr_handler, (void *)PIN_JOYSTICK_RIGHT));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_JOYSTICK_UP, gpio_isr_handler, (void *)PIN_JOYSTICK_UP));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_JOYSTICK_DOWN, gpio_isr_handler, (void *)PIN_JOYSTICK_DOWN));
}

// TODO: adjust bonding techniques
//       when not bonded previously, advertise for ~180 seconds at 30-50ms
//       intervals and be in "bondable mode" whatever that may mean
//       when bonded, save the HID Host address and use in a whitelist
//       if previously bonded, use undirected or directed connectable mode;
//       directed first for 1.28 seconds, then 30 seconds of undirected
//       advertisement with 20-30ms intervals.
// TODO: may need to require device to re-establish connection when it has
