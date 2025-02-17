/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
 *
 * This demo showcases BLE GATT server. It can send adv data, be connected by client.
 * Run the gatt_client demo, the client demo will automatically connect to the gatt_server demo.
 * Client demo will enable gatt_server's notify after connection. The two devices will then exchange
 * data.
 *
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/projdefs.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"

#include "sdkconfig.h"

#include "gattserver.h"
#define ONLY_USE_A

#define GATTS_TAG "GATTSERVER"

typedef struct
{
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;

#define PROFILE_A_APP_ID 0
#define CHAR_A_NAME "Duty Cycle"
static esp_gatt_char_prop_t a_property = 0;
static prepare_type_env_t a_prepare_write_env;
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#ifndef ONLY_USE_A
#define PROFILE_B_APP_ID 1
#define GATTS_SERVICE_UUID_TEST_B 0x00EE
#define GATTS_CHAR_UUID_TEST_B 0xEE01
#define GATTS_DESCR_UUID_TEST_B 0x2222
#define GATTS_NUM_HANDLE_TEST_B 4
static esp_gatt_char_prop_t b_property = 0;
static prepare_type_env_t b_prepare_write_env;
static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
#endif

#ifdef ONLY_USE_A
#define PROFILE_NUM 1
#else
#define PROFILE_NUM 2
#endif

#define TEST_MANUFACTURER_DATA_LEN 17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

#define WRITE_EVENT_ID 1234566
#define POST_WAIT_MS 5
typedef enum
{
    PARAM_TYPE_INT,
    PARAM_TYPE_FLOAT,
    PARAM_TYPE_STRING
} param_type_t;

typedef struct gatt_param_t
{
    uint16_t uuid;     // Characteristic UUID (16-bit)
    char name[20];     // Characteristic Name
    param_type_t type; // Data type (int, float, string)
    union
    {
        uint8_t uint8_val;
        int int_val;
        float float_val;
        char str_val[32]; // String storage
    } value;
    esp_gatt_perm_t perm;      // Read/Write Permissions
    esp_gatt_char_prop_t prop; // Properties (Read, Write, Notify)
    uint16_t handle;           // BLE characteristic handle
    uint16_t cccd_enabled;     // 0x0000 = off, 0x0001 = notifications, 0x0002 = indications. Controlled by the client
    gatt_write_cb_t write_cb;  // Callback function for writing
    esp_event_loop_handle_t loop_handle;
    esp_event_base_t loop_base;
} gatt_param_t;

#define MAX_PARAMS 10
static gatt_param_t gatt_params[MAX_PARAMS];
static int param_count = 0;

static size_t gatts_chars_idx = 0;
static uint8_t adv_config_done = 0;
#define adv_config_flag (1 << 0)
#define scan_rsp_config_flag (1 << 1)

// #define CONFIG_EXAMPLE_SET_RAW_ADV_DATA
#ifdef CONFIG_EXAMPLE_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
    /* Flags */
    0x02, ESP_BLE_AD_TYPE_FLAG, 0x06, // Length 2, Data Type ESP_BLE_AD_TYPE_FLAG, Data 1 (LE General Discoverable Mode, BR/EDR Not Supported)
    /* TX Power Level */
    0x02, ESP_BLE_AD_TYPE_TX_PWR, 0xEB, // Length 2, Data Type ESP_BLE_AD_TYPE_TX_PWR, Data 2 (-21)
    /* Complete 16-bit Service UUIDs */
    0x03, ESP_BLE_AD_TYPE_16SRV_CMPL, 0xAB, 0xCD // Length 3, Data Type ESP_BLE_AD_TYPE_16SRV_CMPL, Data 3 (UUID)
};

static uint8_t raw_scan_rsp_data[] = {
    /* Complete Local Name */
    0x0F, ESP_BLE_AD_TYPE_NAME_CMPL, 'E', 'S', 'P', '_', 'G', 'A', 'T', 'T', 'S', '_', 'D', 'E', 'M', 'O' // Length 15, Data Type ESP_BLE_AD_TYPE_NAME_CMPL, Data (ESP_GATTS_DEMO)
};
#else

// The length of adv data must be less than 31 bytes
// static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
// adv data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    //.min_interval = 0x0006,
    //.max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst
{
    uint8_t adv_service_uuid128[16];
    const char *device_name;
    esp_gatts_cb_t gatts_cb;
    esp_gatt_if_t gatts_if; // Store GATT interface
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    int install_index;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
        .conn_id = ESP_GATT_IF_NONE,
        .install_index = 0}
};

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

esp_bt_uuid_t convert_uuid(const char *uuid_str) {
    esp_bt_uuid_t uuid;
    uuid.len = ESP_UUID_LEN_128;
    sscanf(uuid_str,
           "%02hhx%02hhx%02hhx%02hhx-%02hhx%02hhx-%02hhx%02hhx-%02hhx%02hhx-%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx",
           &uuid.uuid.uuid128[15], &uuid.uuid.uuid128[14], &uuid.uuid.uuid128[13], &uuid.uuid.uuid128[12],
           &uuid.uuid.uuid128[11], &uuid.uuid.uuid128[10], &uuid.uuid.uuid128[9], &uuid.uuid.uuid128[8],
           &uuid.uuid.uuid128[7], &uuid.uuid.uuid128[6], &uuid.uuid.uuid128[5], &uuid.uuid.uuid128[4],
           &uuid.uuid.uuid128[3], &uuid.uuid.uuid128[2], &uuid.uuid.uuid128[1], &uuid.uuid.uuid128[0]);
    return uuid;
}

uint16_t gattserver_calculate_handles()
{
    uint16_t num_notify_chars = 0;

    for (int i = 0; i < param_count; i++)
    {
        if (gatt_params[i].prop & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
        {
            num_notify_chars++;
        }
    }

    return 1 + (param_count * 2) + num_notify_chars; // Service handle + characteristic handles + CCCD handles
}

void gattserver_list_subscriptions()
{
    ESP_LOGI(GATTS_TAG, "=== Active Subscriptions ===");

    if (gl_profile_tab[PROFILE_A_APP_ID].conn_id == ESP_GATT_IF_NONE)
    {
        ESP_LOGW(GATTS_TAG, "No client connected.");
        return;
    }

    for (int i = 0; i < param_count; i++)
    {
        const char *sub_status;
        if (gatt_params[i].cccd_enabled == 0x0001)
        {
            sub_status = "Notifications Enabled";
        }
        else if (gatt_params[i].cccd_enabled == 0x0002)
        {
            sub_status = "Indications Enabled";
        }
        else
        {
            sub_status = "Not Subscribed";
        }

        ESP_LOGI(GATTS_TAG, "Characteristic: %s (UUID: 0x%X) - %s",
                 gatt_params[i].name, gatt_params[i].uuid, sub_status);
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
#ifdef CONFIG_SET_RAW_ADV_DATA
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#endif
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        // advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
            break;
        }
        // ESP_LOGI(GATTS_TAG, "Advertising start successfully");
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed, status %d", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TAG, "Advertising stop successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTS_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                 param->update_conn_params.status,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "Packet length update, status %d, rx %d, tx %d",
                 param->pkt_data_length_cmpl.status,
                 param->pkt_data_length_cmpl.params.rx_len,
                 param->pkt_data_length_cmpl.params.tx_len);
        break;
    default:
        break;
    }
}

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp)
    {
        if (param->write.is_prep)
        {
            if (param->write.offset > PREPARE_BUF_MAX_SIZE)
            {
                status = ESP_GATT_INVALID_OFFSET;
            }
            else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE)
            {
                status = ESP_GATT_INVALID_ATTR_LEN;
            }
            if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL)
            {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL)
                {
                    ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem");
                    status = ESP_GATT_NO_RESOURCES;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            if (gatt_rsp)
            {
                gatt_rsp->attr_value.len = param->write.len;
                gatt_rsp->attr_value.handle = param->write.handle;
                gatt_rsp->attr_value.offset = param->write.offset;
                gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
                memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
                esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
                if (response_err != ESP_OK)
                {
                    ESP_LOGE(GATTS_TAG, "Send response error\n");
                }
                free(gatt_rsp);
            }
            else
            {
                ESP_LOGE(GATTS_TAG, "malloc failed, no resource to send response error\n");
                status = ESP_GATT_NO_RESOURCES;
            }
            if (status != ESP_GATT_OK)
            {
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;
        }
        else
        {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC)
    {
        ESP_LOG_BUFFER_HEX(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }
    else
    {
        ESP_LOGI(GATTS_TAG, "Prepare write cancel");
    }
    if (prepare_write_env->prepare_buf)
    {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static const char *gatts_profile_event_to_str(esp_gatts_cb_event_t event)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        return "ESP_GATTS_REG_EVT";
    case ESP_GATTS_READ_EVT:
        return "ESP_GATTS_READ_EVT";
    case ESP_GATTS_WRITE_EVT:
        return "ESP_GATTS_WRITE_EVT";
    case ESP_GATTS_EXEC_WRITE_EVT:
        return "ESP_GATTS_EXEC_WRITE_EVT";
    case ESP_GATTS_MTU_EVT:
        return "ESP_GATTS_MTU_EVT";
    case ESP_GATTS_CONF_EVT:
        return "ESP_GATTS_CONF_EVT";
    case ESP_GATTS_UNREG_EVT:
        return "ESP_GATTS_UNREG_EVT";
    case ESP_GATTS_CREATE_EVT:
        return "ESP_GATTS_CREATE_EVT";
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        return "ESP_GATTS_ADD_INCL_SRVC_EVT";
    case ESP_GATTS_ADD_CHAR_EVT:
        return "ESP_GATTS_ADD_CHAR_EVT";
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        return "ESP_GATTS_ADD_CHAR_DESCR_EVT";
    case ESP_GATTS_DELETE_EVT:
        return "ESP_GATTS_DELETE_EVT";
    case ESP_GATTS_START_EVT:
        return "ESP_GATTS_START_EVT";
    case ESP_GATTS_STOP_EVT:
        return "ESP_GATTS_STOP_EVT";
    case ESP_GATTS_CONNECT_EVT:
        return "ESP_GATTS_CONNECT_EVT";
    case ESP_GATTS_DISCONNECT_EVT:
        return "ESP_GATTS_DISCONNECT_EVT";
    case ESP_GATTS_OPEN_EVT:
        return "ESP_GATTS_OPEN_EVT";
    case ESP_GATTS_CANCEL_OPEN_EVT:
        return "ESP_GATTS_CANCEL_OPEN_EVT";
    case ESP_GATTS_RESPONSE_EVT:
        return "ESP_GATTS_RESPONSE_EVT";
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        return "ESP_GATTS_CREAT_ATTR_TAB_EVT";
    case ESP_GATTS_SET_ATTR_VAL_EVT:
        return "ESP_GATTS_SET_ATTR_VAL_EVT";
    case ESP_GATTS_SEND_SERVICE_CHANGE_EVT:
        return "ESP_GATTS_SEND_SERVICE_CHANGE_EVT";
    default:
        return "UNKNOWN";
    }
}

void add_parameter(gatts_profile_inst *profile_inst)
{
    if (profile_inst->install_index >= param_count)
    {
        ESP_LOGI(GATTS_TAG, "Installation finished at %d", profile_inst->install_index);
        return;
    }

    gatt_param_t *par = &gatt_params[profile_inst->install_index];

    esp_bt_uuid_t char_uuid;
    char_uuid.len = ESP_UUID_LEN_16;
    char_uuid.uuid.uuid16 = par->uuid;

    esp_attr_value_t attr_value;
    memset(&attr_value, 0, sizeof(esp_attr_value_t));

    switch (par->type)
    {
    case PARAM_TYPE_INT:
        attr_value.attr_max_len = sizeof(int);
        attr_value.attr_len = sizeof(int);
        attr_value.attr_value = (uint8_t *)&par->value.int_val;
        break;

    case PARAM_TYPE_FLOAT:
        attr_value.attr_max_len = sizeof(float);
        attr_value.attr_len = sizeof(float);
        attr_value.attr_value = (uint8_t *)&par->value.float_val;
        break;

    case PARAM_TYPE_STRING:
        attr_value.attr_max_len = sizeof(par->value.str_val);
        attr_value.attr_len = strlen(par->value.str_val);
        attr_value.attr_value = (uint8_t *)par->value.str_val;
        break;
    }

    esp_err_t add_char_ret = esp_ble_gatts_add_char(
        gl_profile_tab[PROFILE_A_APP_ID].service_handle,
        &char_uuid,
        par->perm,
        par->prop,
        &attr_value, NULL);

    if (add_char_ret)
    {
        ESP_LOGE(GATTS_TAG, "    Failed to add characteristic %s (UUID: 0x%X), error: %x",
                 par->name, par->uuid, add_char_ret);
    }
}

void add_cccd_descriptor(gatt_param_t *par, esp_ble_gatts_cb_param_t *param)
{
    esp_bt_uuid_t descr_uuid;
    descr_uuid.len = ESP_UUID_LEN_16;
    descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG; // 0x2902

    par->cccd_enabled = 0;
    esp_attr_value_t descr_value = {
        .attr_max_len = sizeof(uint16_t),
        .attr_len = sizeof(uint16_t),
        .attr_value = (uint8_t *)&par->cccd_enabled};

    ESP_LOGI(GATTS_TAG, "Adding CCCD descriptor for %s (UUID: 0x%X)", par->name, par->uuid);
    if (ESP_OK != esp_ble_gatts_add_char_descr(
                      param->add_char.service_handle,
                      &descr_uuid,
                      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                      &descr_value,
                      NULL))
    {
        ESP_LOGE(GATTS_TAG, "Failed to add CCCD descriptor for %s", par->name);
    }
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    // ESP_LOGI(GATTS_TAG, "(%2d) %s", event, gatts_profile_event_to_str(event));
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
    {
        ESP_LOGI(GATTS_TAG, "GATT server register, status %d, app_id %d, gatts_if %d", param->reg.status, param->reg.app_id, gatts_if);

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(gl_profile_tab[PROFILE_A_APP_ID].device_name);
        if (set_dev_name_ret)
        {
            ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
        // config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret)
        {
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= adv_config_flag;
        // config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret)
        {
            ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;

        uint16_t GATTS_NUM_HANDLE = gattserver_calculate_handles();
        ESP_LOGI(GATTS_TAG, "Calculated required handles: %d", GATTS_NUM_HANDLE);
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE);
        break;
    }
    case ESP_GATTS_CREATE_EVT:
    {
        // ESP_LOGI(GATTS_TAG, "Service created, status %d, handle %d",
        //          param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);

        // Add the first parameter
        add_parameter(&gl_profile_tab[PROFILE_A_APP_ID]);
        break;
    }
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "Service started, status %d, service_handle %d, available characteristic:",
                 param->start.status, param->start.service_handle);

        for (int i = 0; i < param_count; i++)
        {
            ESP_LOGI(GATTS_TAG, "    %s (UUID: 0x%X)", gatt_params[i].name, gatt_params[i].uuid);
        }
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
    {
        // ESP_LOGI(GATTS_TAG, "----\nCharacteristic added, status %d, attr_handle %d, service_handle %d, UUID: 0x%X",
        //           param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle, param->add_char.char_uuid.uuid.uuid16);

        // Store handle and ensure correct attribute size
        gatt_param_t *par = &gatt_params[gl_profile_tab[PROFILE_A_APP_ID].install_index];
        if (par->uuid == param->add_char.char_uuid.uuid.uuid16)
        {
            par->handle = param->add_char.attr_handle;

            // Determine correct attribute size based on data type
            size_t attr_size = 0;
            switch (par->type)
            {
            case PARAM_TYPE_INT:
                attr_size = sizeof(int);
                break;
            case PARAM_TYPE_FLOAT:
                attr_size = sizeof(float);
                break;
            case PARAM_TYPE_STRING:
                attr_size = sizeof(par->value.str_val); // Full string buffer
                break;
            default:
                ESP_LOGW(GATTS_TAG, "Unknown data type for characteristic %s", par->name);
                break;
            }

            // Update attribute max size dynamically
            esp_attr_value_t attr_value;
            memset(&attr_value, 0, sizeof(esp_attr_value_t));
            attr_value.attr_max_len = attr_size;
            attr_value.attr_len = attr_size;
            attr_value.attr_value = (uint8_t *)&par->value.uint8_val;

            esp_ble_gatts_set_attr_value(par->handle, attr_value.attr_len, attr_value.attr_value);

            // Add CCCD descriptor only for notify characteristics
            // if (par->prop & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
                add_cccd_descriptor(par, param);
            // else
            // {
            //     gl_profile_tab[PROFILE_A_APP_ID].install_index++;
            //     add_parameter(&gl_profile_tab[PROFILE_A_APP_ID]);
            // }
            ESP_LOGI(GATTS_TAG, "Characteristic %s (UUID: 0x%X) - Properties: 0x%X, Max Length: %d, handle: %d",
                par->name, par->uuid, par->prop, attr_size, par->handle);
            ESP_LOGI(GATTS_TAG, "    Notify  : %s", (par->prop & ESP_GATT_CHAR_PROP_BIT_NOTIFY) ? "Yes" : "No");
            ESP_LOGI(GATTS_TAG, "    Indicate: %s", (par->prop & ESP_GATT_CHAR_PROP_BIT_INDICATE) ? "Yes" : "No");
            ESP_LOGI(GATTS_TAG, "    Read    : %s", (par->perm & ESP_GATT_PERM_READ) ? "Yes" : "No");
            ESP_LOGI(GATTS_TAG, "    Write   : %s", (par->perm & ESP_GATT_PERM_WRITE) ? "Yes" : "No");
        }
    }
    break;

    case ESP_GATTS_ADD_CHAR_DESCR_EVT: // A descriptor (e.g., CCCD) was successfully added to a characteristic.
    {
        gatt_param_t *par = &gatt_params[gl_profile_tab[PROFILE_A_APP_ID].install_index];
        if (par->handle + 1 == param->add_char_descr.attr_handle) // CCCD should follow characteristic
        {
            // gatt_params[i].cccd_handle = param->add_char_descr.attr_handle; // Store CCCD handle
            ESP_LOGI(GATTS_TAG, "    CCCD descriptor handle for %s (UUID: 0x%X), attr_handle %d, service_handle %d",
                par->name, par->uuid, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        }
        gl_profile_tab[PROFILE_A_APP_ID].install_index++;
        add_parameter(&gl_profile_tab[PROFILE_A_APP_ID]);
    }
    break;

    case ESP_GATTS_READ_EVT:
    {
        ESP_LOGI(GATTS_TAG, "Characteristic read, conn_id %d, trans_id %ld, handle %d",
                 param->read.conn_id, param->read.trans_id, param->read.handle);

        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;

        for (int i = 0; i < param_count; i++)
        {
            if (gatt_params[i].handle == param->read.handle)
            {
                switch (gatt_params[i].type)
                {
                case PARAM_TYPE_INT:
                    rsp.attr_value.len = sizeof(int);
                    memcpy(rsp.attr_value.value, &gatt_params[i].value.int_val, sizeof(int));
                    break;
                case PARAM_TYPE_FLOAT:
                    rsp.attr_value.len = sizeof(float);
                    memcpy(rsp.attr_value.value, &gatt_params[i].value.float_val, sizeof(float));
                    break;
                case PARAM_TYPE_STRING:
                    rsp.attr_value.len = strlen(gatt_params[i].value.str_val);
                    memcpy(rsp.attr_value.value, gatt_params[i].value.str_val, rsp.attr_value.len);
                    break;
                }
                break;
            }
        }

        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;
    }

    case ESP_GATTS_WRITE_EVT:
    {
        ESP_LOGI(GATTS_TAG, "From client - caracteristic write, conn_id %d, trans_id %ld, handle %d",
                 param->write.conn_id, param->write.trans_id, param->write.handle);

        for (int i = 0; i < param_count; i++)
        {
            if (gatt_params[i].handle + 1 == param->write.handle) // CCCD is typically the next handle
            {
                uint16_t cccd_value = param->write.value[0] | (param->write.value[1] << 8);
                gatt_params[i].cccd_enabled = cccd_value;
                if (cccd_value == 0x0001)
                {
                    ESP_LOGI(GATTS_TAG, "    Enabled notifications for: %s (UUID: 0x%X)",
                             gatt_params[i].name, gatt_params[i].uuid);
                }
                else if (cccd_value == 0x0002)
                {
                    ESP_LOGI(GATTS_TAG, "    Enabled indications for: %s (UUID: 0x%X)",
                             gatt_params[i].name, gatt_params[i].uuid);
                }
                else
                {
                    ESP_LOGI(GATTS_TAG, "    Disabled notifications/indications for: %s (UUID: 0x%X)",
                             gatt_params[i].name, gatt_params[i].uuid);
                }

                break;
                gattserver_list_subscriptions();
            }
            if (gatt_params[i].handle == param->write.handle)
            {
                switch (gatt_params[i].type)
                {
                case PARAM_TYPE_INT:
                    memcpy(&gatt_params[i].value.int_val, param->write.value, sizeof(int));
                    ESP_LOGI(GATTS_TAG, "    Updated %s to %d", gatt_params[i].name, gatt_params[i].value.int_val);
                    break;
                case PARAM_TYPE_FLOAT:
                    memcpy(&gatt_params[i].value.float_val, param->write.value, sizeof(float));
                    ESP_LOGI(GATTS_TAG, "    Updated %s to %.2f", gatt_params[i].name, gatt_params[i].value.float_val);
                    break;
                case PARAM_TYPE_STRING:
                    strncpy(gatt_params[i].value.str_val, (char *)param->write.value, param->write.len);
                    gatt_params[i].value.str_val[param->write.len] = '\0';
                    ESP_LOGI(GATTS_TAG, "    Updated %s to %s", gatt_params[i].name, gatt_params[i].value.str_val);
                    break;
                }

                if (gatt_params[i].write_cb != NULL)
                {
                    esp_err_t err = esp_event_post_to(gatt_params[i].loop_handle, gatt_params[i].loop_base,
                                                      WRITE_EVENT_ID, &gatt_params[i], sizeof(gatt_param_t *), pdMS_TO_TICKS(POST_WAIT_MS));
                    if (err != ESP_OK)
                    {
                        ESP_LOGE(GATTS_TAG, "Failed to post event to loop: %s, %s", gatt_params[i].name, esp_err_to_name(err));
                    }
                }
                break;
            }
        }

        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        break;
    }

    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(GATTS_TAG, "Execute write");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&a_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "MTU exchange, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;

    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
    {
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20; // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;  // timeout = 400*10ms = 4000ms
        ESP_LOGI(GATTS_TAG, "Connected, conn_id %u, remote " ESP_BD_ADDR_STR "",
                 param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
        gl_profile_tab[PROFILE_A_APP_ID].gatts_if = gatts_if;
        // start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Disconnected, remote " ESP_BD_ADDR_STR ", reason 0x%02x",
                 ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CONF_EVT:
        // ESP_LOGI(GATTS_TAG, "Confirm receive, status %d, attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK)
        {
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == gl_profile_tab[idx].gatts_if)
            {
                if (gl_profile_tab[idx].gatts_cb)
                {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

gatt_param_handle_t gattserver_register_int(const char *name, uint16_t uuid, esp_gatt_perm_t perm,
                                            esp_gatt_char_prop_t prop, int init_value)
{
    if (param_count >= MAX_PARAMS)
    {
        ESP_LOGE(GATTS_TAG, "Max parameters reached, cannot register more.");
        return NULL; // Invalid handle
    }

    gatt_param_t *param = &gatt_params[param_count++];
    param->uuid = uuid;
    param->type = PARAM_TYPE_INT;
    strncpy(param->name, name, sizeof(param->name) - 1);
    param->perm = perm;
    param->prop = prop;
    param->value.int_val = init_value;
    param->handle = 0; // Will be assigned later

    ESP_LOGI(GATTS_TAG, "Registered Integer Parameter: %s (UUID: 0x%X, Value: %d)",
             param->name, param->uuid, param->value.int_val);
    return param;
}

gatt_param_handle_t gattserver_register_float(const char *name, uint16_t uuid, esp_gatt_perm_t perm,
                                              esp_gatt_char_prop_t prop, float init_value)
{
    if (param_count >= MAX_PARAMS)
    {
        ESP_LOGE(GATTS_TAG, "Max parameters reached, cannot register more.");
        return NULL;
    }

    gatt_param_t *param = &gatt_params[param_count++];
    param->uuid = uuid;
    param->type = PARAM_TYPE_FLOAT;
    strncpy(param->name, name, sizeof(param->name) - 1);
    param->perm = perm;
    param->prop = prop;
    param->value.float_val = init_value;
    param->handle = 0;

    ESP_LOGI(GATTS_TAG, "Registered Float: %s (UUID: 0x%X, Value: %.2f)",
             param->name, param->uuid, param->value.float_val);
    return param;
}

gatt_param_handle_t gattserver_register_string(const char *name, uint16_t uuid, esp_gatt_perm_t perm,
                                               esp_gatt_char_prop_t prop, const char *init_value)
{
    if (param_count >= MAX_PARAMS)
    {
        ESP_LOGE(GATTS_TAG, "Max parameters reached, cannot register more.");
        return NULL;
    }

    gatt_param_t *param = &gatt_params[param_count++];
    param->uuid = uuid;
    param->type = PARAM_TYPE_STRING;
    strncpy(param->name, name, sizeof(param->name) - 1);
    param->perm = perm;
    param->prop = prop;
    strncpy(param->value.str_val, init_value, sizeof(param->value.str_val) - 1);
    param->handle = 0;

    ESP_LOGI(GATTS_TAG, "Registered String Parameter: %s (UUID: 0x%X, Value: %s)",
             param->name, param->uuid, param->value.str_val);
    return param;
}

esp_err_t gattserver_notify_int(gatt_param_handle_t handle, int new_value)
{
    if (handle == NULL)
        return ESP_FAIL;

    handle->value.int_val = new_value;
    gatts_profile_inst *profile_inst = &gl_profile_tab[PROFILE_A_APP_ID];

    if (profile_inst->conn_id == ESP_GATT_IF_NONE)
    {
        ESP_LOGW(GATTS_TAG, "No client connected, skipping notification.");
        return ESP_FAIL;
    }

    if (handle->cccd_enabled != 0x0001 && handle->cccd_enabled != 0x0002)
        return ESP_FAIL;

    printf("%s got gatt_if %d (notify int)\n", handle->name, profile_inst->gatts_if);

    return esp_ble_gatts_send_indicate(profile_inst->gatts_if, profile_inst->conn_id,
                                       handle->handle, sizeof(int),
                                       (uint8_t *)&handle->value.int_val, false);
}

esp_err_t gattserver_notify_float(gatt_param_handle_t handle, float new_value)
{
    if (handle == NULL)
        return ESP_FAIL;

    handle->value.float_val = new_value;
    gatts_profile_inst *profile_inst = &gl_profile_tab[PROFILE_A_APP_ID];

    if (profile_inst->conn_id == ESP_GATT_IF_NONE)
    {
        return ESP_FAIL;
    }

    if (handle->cccd_enabled != 0x0001 && handle->cccd_enabled != 0x0002)
    {
        return ESP_FAIL;
    }

    esp_err_t err = esp_ble_gatts_send_indicate(profile_inst->gatts_if, profile_inst->conn_id,
                                                handle->handle, sizeof(float),
                                                (uint8_t *)&handle->value.float_val, false);

    if (err != ESP_OK)
    {
        ESP_LOGE(GATTS_TAG, "Failed to send notification for %s, error: %s",
                 handle->name, esp_err_to_name(err));
    }

    return err;
}

esp_err_t gattserver_notify_string(gatt_param_handle_t handle, const char *new_value)
{
    if (handle == NULL || new_value == NULL)
        return ESP_FAIL;

    strncpy(handle->value.str_val, new_value, sizeof(handle->value.str_val) - 1);
    handle->value.str_val[sizeof(handle->value.str_val) - 1] = '\0';

    gatts_profile_inst *profile_inst = &gl_profile_tab[PROFILE_A_APP_ID];

    if (profile_inst->conn_id == ESP_GATT_IF_NONE)
    {
        ESP_LOGW(GATTS_TAG, "No client connected, skipping notification.");
        return ESP_FAIL;
    }

    if (handle->cccd_enabled != 0x0001 && handle->cccd_enabled != 0x0002)
        return ESP_FAIL;

    printf("%s got gatt_if %d (notify string)\n", handle->name, profile_inst->gatts_if);

    return esp_ble_gatts_send_indicate(profile_inst->gatts_if, profile_inst->conn_id,
                                       handle->handle, strlen(handle->value.str_val),
                                       (uint8_t *)handle->value.str_val, false);
}

static void evloop_write_cb(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    gatt_param_t *param = (gatt_param_t *)arg;
    if (param && param->write_cb)
    {
        ESP_LOGI(GATTS_TAG, "Executing write callback for %s (UUID: 0x%X)", param->name, param->uuid);
        param->write_cb(param, &param->value, sizeof(param->value));
    }
}

esp_err_t gattserver_register_write_cb(gatt_param_handle_t handle, esp_event_loop_handle_t loop_handle, esp_event_base_t base, gatt_write_cb_t cb)
{
    if (handle == NULL || cb == NULL || loop_handle == NULL)
    {
        ESP_LOGE(GATTS_TAG, "Invalid parameters for write callback registration.");
        return ESP_ERR_INVALID_ARG;
    }
    gatt_param_t *param = (gatt_param_t *)handle;
    param->write_cb = cb;
    param->loop_handle = loop_handle;
    param->loop_base = base;

    esp_err_t err = esp_event_handler_instance_register_with(loop_handle, base, WRITE_EVENT_ID,
                                                             &evloop_write_cb, param, NULL);
    if (err != ESP_OK)
    {
        ESP_LOGE(GATTS_TAG, "Failed to register write callback for %s (UUID: 0x%X)", param->name, param->uuid);
    }
    return err;
}

void gattserver_init(const char* name, const char *service_uuid)
{
    esp_err_t ret;
    esp_bt_uuid_t service_uuid_struct = convert_uuid(service_uuid);
    memcpy(gl_profile_tab[PROFILE_A_APP_ID].adv_service_uuid128, service_uuid_struct.uuid.uuid128, ESP_UUID_LEN_128);
    gl_profile_tab[PROFILE_A_APP_ID].device_name = name;
    gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
    gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
    gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid = service_uuid_struct;

    // Update advertising data
    adv_data.service_uuid_len = ESP_UUID_LEN_128;
    adv_data.p_service_uuid = gl_profile_tab[PROFILE_A_APP_ID].adv_service_uuid128;

    scan_rsp_data.service_uuid_len = ESP_UUID_LEN_128;
    scan_rsp_data.p_service_uuid = gl_profile_tab[PROFILE_A_APP_ID].adv_service_uuid128;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
    {
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    return;
}