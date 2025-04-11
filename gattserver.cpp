/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <memory.h>
#include <inttypes.h>
#include <map>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#ifdef CONFIG_BT_ENABLED
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

#include "gattserver.h"

#define GATTS_TAG "GATTSERVER"

#define PROFILE_A_APP_ID 0

#define PROFILE_NUM                               1
#define PROFILE_APP_IDX                           0
#define APP_ID                                    0x55
#define EXT_ADV_HANDLE                            0
#define NUM_EXT_ADV_SET                           1
#define EXT_ADV_DURATION                          0
#define EXT_ADV_MAX_EVENTS                        0
#define WRITE_EVENT_ID 1234566 // TODO: Find the correct number
#define POST_WAIT_MS 50


#define GATTS_DEMO_CHAR_VAL_LEN_MAX               0x40

static uint8_t ext_adv_raw_data[64] = {};
size_t ext_adv_raw_data_len = 0;

static esp_ble_gap_ext_adv_t ext_adv[1] = {
    [0] = {EXT_ADV_HANDLE, EXT_ADV_DURATION, EXT_ADV_MAX_EVENTS},
};

esp_ble_gap_ext_adv_params_t ext_adv_params_2M = {
    .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_LEGACY_IND,
    .interval_min = 0x20,
    .interval_max = 0x40,
    .channel_map = ADV_CHNL_ALL,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr = {0},
    .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .tx_power = EXT_ADV_TX_PWR_NO_PREFERENCE,
    .primary_phy = ESP_BLE_GAP_PHY_1M,
    .max_skip = 0,
    .secondary_phy = ESP_BLE_GAP_PHY_2M,
    .sid = 0,
    .scan_req_notif = false,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
    esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
        .app_id = APP_ID,
        .conn_id = 0,
        .service_handle = 0,
        .service_id = {},
        .char_handle = 0,
        .char_uuid = {},
        .perm = {},
        .property = 0,
        .descr_handle = 0,
        .descr_uuid = {},
    },
};

#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))
// #define SVC_INST_ID                 0
static const uint16_t CHARACTER_DECLARATION_UUID = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t CHARACTER_CLIENT_CONFIG_UUID = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t PRIMARY_SERVICE_UUID = ESP_GATT_UUID_PRI_SERVICE;

// static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
// static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t CHAR_PROP_READ_WRITE_NOTIFY = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

/* Full Database Description - Used to add attributes into the database */
typedef struct gatt_param_t
{
    const char* name;
    esp_gatts_attr_db_t* propAttr;
    esp_gatts_attr_db_t* valueAttr;
    esp_gatts_attr_db_t* descrAttr;

    /// @brief Callback parameters for writing
    gatt_write_cb_t write_cb;
    uint16_t handle;           // BLE characteristic handle
    uint16_t cccd_enabled;     // 0x0000 = off, 0x0001 = notifications, 0x0002 = indications. Controlled by the client
    esp_event_base_t loop_base;
    esp_event_loop_handle_t loop_handle;
    uint32_t event_id;
    uint8_t char_property;
    gatt_param_type_t type;
    esp_bt_uuid_t uuid;

    union {
        float f;
        int i;
        uint32_t u32;
        uint16_t u16;
        char* s;
        void* v;
        uint8_t u8;
    } value;
} gatt_param_t;

/// @brief  GATT parameter map. Key is the UUID of the characteristic
/// @note   UUIDs are 16-bit numbers, so the map is limited to 65536 entries
/// @note   The map is not thread-safe.
static std::map<uint16_t, gatt_param_t> gatt_map;
static std::map<uint16_t, uint16_t> handle2uuid_map;
//static gatt_param_t gatt_param[GATT_MAX_PARAMS] = {};
static esp_gatts_attr_db_t gatt_db[GATT_MAX_PARAMS] = {};
static size_t gatt_db_idx = 1; // O is taken for service id
static bool debug = false;
static char gatt_name[32] = "Not defined";
static char gatt_short_name[32] = "Not defined";
static uint8_t scan_rsp_data[64] = {};
static size_t scan_rsp_data_len = 0;

static const char* esp_key_type_to_str(esp_ble_key_type_t key_type)
{
    const char* key_str = NULL;
    switch (key_type) {
    case ESP_LE_KEY_NONE:
        key_str = "ESP_LE_KEY_NONE";
        break;
    case ESP_LE_KEY_PENC:
        key_str = "ESP_LE_KEY_PENC";
        break;
    case ESP_LE_KEY_PID:
        key_str = "ESP_LE_KEY_PID";
        break;
    case ESP_LE_KEY_PCSRK:
        key_str = "ESP_LE_KEY_PCSRK";
        break;
    case ESP_LE_KEY_PLK:
        key_str = "ESP_LE_KEY_PLK";
        break;
    case ESP_LE_KEY_LLK:
        key_str = "ESP_LE_KEY_LLK";
        break;
    case ESP_LE_KEY_LENC:
        key_str = "ESP_LE_KEY_LENC";
        break;
    case ESP_LE_KEY_LID:
        key_str = "ESP_LE_KEY_LID";
        break;
    case ESP_LE_KEY_LCSRK:
        key_str = "ESP_LE_KEY_LCSRK";
        break;
    default:
        key_str = "INVALID BLE KEY TYPE";
        break;

    }

    return key_str;
}

static const char* esp_auth_req_to_str(esp_ble_auth_req_t auth_req)
{
    const char* auth_str = NULL;
    switch (auth_req) {
    case ESP_LE_AUTH_NO_BOND:
        auth_str = "ESP_LE_AUTH_NO_BOND";
        break;
    case ESP_LE_AUTH_BOND:
        auth_str = "ESP_LE_AUTH_BOND";
        break;
    case ESP_LE_AUTH_REQ_MITM:
        auth_str = "ESP_LE_AUTH_REQ_MITM";
        break;
    case ESP_LE_AUTH_REQ_BOND_MITM:
        auth_str = "ESP_LE_AUTH_REQ_BOND_MITM";
        break;
    case ESP_LE_AUTH_REQ_SC_ONLY:
        auth_str = "ESP_LE_AUTH_REQ_SC_ONLY";
        break;
    case ESP_LE_AUTH_REQ_SC_BOND:
        auth_str = "ESP_LE_AUTH_REQ_SC_BOND";
        break;
    case ESP_LE_AUTH_REQ_SC_MITM:
        auth_str = "ESP_LE_AUTH_REQ_SC_MITM";
        break;
    case ESP_LE_AUTH_REQ_SC_MITM_BOND:
        auth_str = "ESP_LE_AUTH_REQ_SC_MITM_BOND";
        break;
    default:
        auth_str = "INVALID BLE AUTH REQ";
        break;
    }

    return auth_str;
}

static const char* property_to_string(uint8_t property)
{
    const char* prop_str = NULL;
    switch (property) {
    case ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY:
        prop_str = "READ|WRITE|NOTIFY";
        break;
    case ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE:
        prop_str = "READ|WRITE";
        break;
    case ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY:
        prop_str = "READ|NOTIFY";
        break;
    case ESP_GATT_CHAR_PROP_BIT_BROADCAST:
        prop_str = "BROADCAST";
        break;
    case ESP_GATT_CHAR_PROP_BIT_READ:
        prop_str = "READ";
        break;
    case ESP_GATT_CHAR_PROP_BIT_WRITE_NR:
        prop_str = "WRITE_NR";
        break;
    case ESP_GATT_CHAR_PROP_BIT_WRITE:
        prop_str = "WRITE";
        break;
    case ESP_GATT_CHAR_PROP_BIT_NOTIFY:
        prop_str = "NOTIFY";
        break;
    case ESP_GATT_CHAR_PROP_BIT_INDICATE:
        prop_str = "INDICATE";
        break;
    case ESP_GATT_CHAR_PROP_BIT_AUTH:
        prop_str = "AUTH";
        break;
    case ESP_GATT_CHAR_PROP_BIT_EXT_PROP:
        prop_str = "EXT_PROP";
        break;
    default:
        prop_str = "INVALID";
        break;
    }

    return prop_str;
}

static const char* permission_to_string(esp_gatts_attr_db_t* attr)
{
    if (attr == NULL)
        return "NULL ATTRIBUTE";

    const char* perm_str = NULL;
    switch (attr->att_desc.perm) {
    case ESP_GATT_PERM_READ:
        perm_str = "PERM_READ";
        break;
    case ESP_GATT_PERM_READ_ENCRYPTED:
        perm_str = "PERM_READ_ENCRYPTED";
        break;
    case ESP_GATT_PERM_READ_ENC_MITM:
        perm_str = "PERM_READ_ENC_MITM";
        break;
    case ESP_GATT_PERM_WRITE:
        perm_str = "PERM_WRITE";
        break;
    case ESP_GATT_PERM_WRITE_ENCRYPTED:
        perm_str = "PERM_WRITE_ENCRYPTED";
        break;
    case ESP_GATT_PERM_WRITE_ENC_MITM:
        perm_str = "PERM_WRITE_ENC_MITM";
        break;
    case ESP_GATT_PERM_WRITE_SIGNED:
        perm_str = "PERM_WRITE_SIGNED";
        break;
    case ESP_GATT_PERM_WRITE_SIGNED_MITM:
        perm_str = "PERM_WRITE_SIGNED_MITM";
        break;
    default:
        perm_str = "INVALID";
        break;
    }

    return perm_str;
}

static const char* type_to_string(gatt_param_type_t type)
{
    switch (type)
    {
    case GATT_PARAM_TYPE_FLOAT:
        return "FLOAT";
    case GATT_PARAM_TYPE_INT:
        return "INT";
    case GATT_PARAM_TYPE_UINT32:
        return "UINT32";
    case GATT_PARAM_TYPE_STRING:
        return "STRING";
    case GATT_PARAM_TYPE_GENERIC:
        return "GENERIC";
    default:
        return "UNKNOWN";
    }
}

static void show_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();
    if (dev_num == 0) {
        ESP_LOGI(GATTS_TAG, "Bonded devices number zero\n");
        return;
    }

    esp_ble_bond_dev_t* dev_list = (esp_ble_bond_dev_t*)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    if (!dev_list) {
        ESP_LOGE(GATTS_TAG, "malloc failed\n");
        return;
    }
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    ESP_LOGI(GATTS_TAG, "Bonded devices number : %d", dev_num);
    for (int i = 0; i < dev_num; i++) {
        ESP_LOG_BUFFER_HEX(GATTS_TAG, (void*)dev_list[i].bd_addr, sizeof(esp_bd_addr_t));
    }

    free(dev_list);
}

static void __attribute__((unused)) remove_all_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();
    if (dev_num == 0) {
        ESP_LOGI(GATTS_TAG, "Bonded devices number zero\n");
        return;
    }

    esp_ble_bond_dev_t* dev_list = (esp_ble_bond_dev_t*)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    if (!dev_list) {
        ESP_LOGE(GATTS_TAG, "malloc failed\n");
        return;
    }
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    for (int i = 0; i < dev_num; i++) {
        esp_ble_remove_bond_device(dev_list[i].bd_addr);
    }

    free(dev_list);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    esp_err_t err;

    switch (event) {
    case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT status %d", param->ext_adv_set_params.status);
        esp_ble_gap_config_ext_adv_data_raw(EXT_ADV_HANDLE, ext_adv_raw_data_len, &ext_adv_raw_data[0]);
        esp_ble_gap_config_ext_scan_rsp_data_raw(EXT_ADV_HANDLE, scan_rsp_data_len, scan_rsp_data);
        break;
    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT status %d", param->ext_adv_data_set.status);
        esp_ble_gap_ext_adv_start(NUM_EXT_ADV_SET, &ext_adv[0]);
        break;
    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT, status = %d", param->ext_adv_data_set.status);
        break;
    case ESP_GAP_BLE_ADV_TERMINATED_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_ADV_TERMINATED_EVT, status = %d", param->adv_terminate.status);
        if (param->adv_terminate.status == 0x00) {
            ESP_LOGI(GATTS_TAG, "ADV successfully ended with a connection being created");
        }
        break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:                           /* passkey request event */
        ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
        /* Call the following function to input the passkey which is displayed on the remote device */
            //esp_ble_passkey_reply(gl_profile_tab[PROFILE_APP_IDX].remote_bda, true, 0x00);
        break;
    case ESP_GAP_BLE_OOB_REQ_EVT: {
        ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_OOB_REQ_EVT");
        uint8_t tk[16] = { 1 }; //If you paired with OOB, both devices need to use the same tk
        esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, tk, sizeof(tk));
        break;
    }
    case ESP_GAP_BLE_LOCAL_IR_EVT:                               /* BLE local IR event */
        ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_LOCAL_IR_EVT");
        break;
    case ESP_GAP_BLE_LOCAL_ER_EVT:                               /* BLE local ER event */
        ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_LOCAL_ER_EVT");
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        /* The app will receive this evt when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
        show the passkey number to the user to confirm it with the number displayed by peer device. */
        esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
        ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%" PRIu32, param->ble_security.key_notif.passkey);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_SEC_REQ_EVT");
        /* send the positive(true) security response to the peer device to accept the security request.
            If not accept the security request, should send the security response with negative(false) accept value*/
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:  ///the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
        ///show the passkey number to the user to input it in the peer device.
        ESP_LOGI(GATTS_TAG, "The passkey Notify number:%06" PRIu32, param->ble_security.key_notif.passkey);
        break;
    case ESP_GAP_BLE_KEY_EVT:
        //shows the ble key info share with peer device to the user.
        ESP_LOGI(GATTS_TAG, "key type = %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTS_TAG, "remote BD_ADDR: %08x%04x", \
            (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
            (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(GATTS_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(GATTS_TAG, "pair status = %s", param->ble_security.auth_cmpl.success ? "success" : "fail");
        if (!param->ble_security.auth_cmpl.success) {
            ESP_LOGI(GATTS_TAG, "fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);
        }
        else {
            ESP_LOGI(GATTS_TAG, "auth mode = %s", esp_auth_req_to_str(param->ble_security.auth_cmpl.auth_mode));
        }
        show_bonded_devices();
        break;
    }
    case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT: {
        ESP_LOGD(GATTS_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT status = %d", param->remove_bond_dev_cmpl.status);
        ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV");
        ESP_LOGI(GATTS_TAG, "-----ESP_GAP_BLE_REMOVE_BOND_DEV----");
        ESP_LOG_BUFFER_HEX(GATTS_TAG, (void*)param->remove_bond_dev_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTS_TAG, "------------------------------------");
        break;
    }
    case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT, tatus = %x", param->local_privacy_cmpl.status);
        err = esp_ble_gap_ext_adv_set_params(EXT_ADV_HANDLE, &ext_adv_params_2M);
        if (err != ESP_OK) {
            ESP_LOGE(GATTS_TAG, "esp_ble_gap_ext_adv_set_params failed: %s", esp_err_to_name(err));
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
            param->update_conn_params.status,
            param->update_conn_params.min_int,
            param->update_conn_params.max_int,
            param->update_conn_params.conn_int,
            param->update_conn_params.latency,
            param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
    esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        //generate a resolvable random address
        esp_ble_gap_config_local_privacy(true);
        for (size_t i = 0; i < gatt_db_idx; i++) {
            if (gatt_db[i].att_desc.uuid_p == NULL) {
                ESP_LOGE(GATTS_TAG, "gatt_db[%d] has NULL UUID pointer!", i);
            }
            else {
                uint16_t uuid = *((uint16_t*)gatt_db[i].att_desc.uuid_p);
                ESP_LOGI(GATTS_TAG, "gatt_db[%d] UUID: 0x%04X", i, uuid);
            }
        }
        esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, gatt_db_idx, 0);
        break;
    case ESP_GATTS_READ_EVT:
        break;
    case ESP_GATTS_WRITE_EVT:
    {
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_WRITE_EVT, handle: %d", param->write.handle);

        // Get the UUID from the handle
        auto it = handle2uuid_map.find(param->write.handle);
        if (it == handle2uuid_map.end())
        {
            ESP_LOGE(GATTS_TAG, "Handle %d not found in handle2uuid_map", param->write.handle);
            break;
        }
        uint16_t uuid = it->second;

        // Check if the write is for the CCCD
        if (uuid == ESP_GATT_UUID_CHAR_CLIENT_CONFIG)
        {
            uint16_t value_handle = param->write.handle - 1;
            auto it1 = handle2uuid_map.find(value_handle);
            if (it1 == handle2uuid_map.end())
            {
                ESP_LOGE(GATTS_TAG, "Handle %d not found in handle2uuid_map", value_handle);
                break;
            }
            uint16_t uuid = it1->second;
            auto it3 = gatt_map.find(uuid);
            if (it3 == gatt_map.end())
            {
                ESP_LOGE(GATTS_TAG, "UUID %04X not found in gatt_map", uuid);
                break;
            }
            gatt_param_t* gp = &it3->second;
            ESP_LOGI(GATTS_TAG, "CCCD handle: %d, value handle: %d", param->write.handle, value_handle);
            gp->cccd_enabled = *(uint16_t*)param->write.value;
            break;
        }

        ESP_LOGI(GATTS_TAG, "ESP_GATTS_WRITE_EVT, characteristic handle: %d", param->write.handle);

        // Find the gatt_param_t from the map using the UUID
        auto it2 = gatt_map.find(uuid);
        if (it2 == gatt_map.end())
        {
            ESP_LOGE(GATTS_TAG, "UUID %04X not found in gatt_map", uuid);
            break;
        }
        gatt_param_t* gp = &it2->second;
        if (param->write.len <= gp->valueAttr->att_desc.max_length)
        {
            memcpy(gp->valueAttr->att_desc.value, param->write.value, param->write.len);
            if (debug)
            {
                ESP_LOGI(GATTS_TAG, "Parameter written: \"%s\" (UUID: 0x%X), handle: %d,  %p", gp->name, gp->uuid.uuid.uuid16, gp->handle, gp->write_cb);
                ESP_LOG_BUFFER_HEXDUMP(GATTS_TAG, param->write.value, param->write.len, ESP_LOG_INFO);
            }
        }
        else
        {
            ESP_LOGE(GATTS_TAG, "Write value too long for \"%s\", %d > %d (max)", gp->name, param->write.len, gp->valueAttr->att_desc.max_length);
            break;
        }

        if (gp->write_cb)
        {
            if (gp->loop_handle == NULL)
                ESP_ERROR_CHECK(esp_event_post(gp->loop_base, gp->event_id, param->write.value, param->write.len, pdMS_TO_TICKS(POST_WAIT_MS)));
            ESP_ERROR_CHECK(esp_event_post_to(gp->loop_handle, gp->loop_base, gp->event_id, param->write.value, param->write.len, pdMS_TO_TICKS(POST_WAIT_MS)));
        }
        else if (debug)
        {
            ESP_LOGI(GATTS_TAG, "No write callback for \"%s\"", gp->name);
        }
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        break;
    case ESP_GATTS_MTU_EVT:
        break;
    case ESP_GATTS_CONF_EVT:
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
        /* start security connect with peer device when receive the connect event sent by the master */
        esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
        /* start advertising again when missing the connect */
        esp_ble_gap_ext_adv_start(NUM_EXT_ADV_SET, &ext_adv[0]);
        break;
    case ESP_GATTS_OPEN_EVT:
        break;
    case ESP_GATTS_CANCEL_OPEN_EVT:
        break;
    case ESP_GATTS_CLOSE_EVT:
        break;
    case ESP_GATTS_LISTEN_EVT:
        break;
    case ESP_GATTS_CONGEST_EVT:
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
        if (param->create.status == ESP_GATT_OK) {
            if (param->add_attr_tab.num_handle == gatt_db_idx) {
                if (debug) ESP_LOGI(GATTS_TAG, "Create attribute table, handles = %d, Service uuid type %04X, Status %X", param->add_attr_tab.num_handle, param->add_attr_tab.svc_uuid.uuid.uuid16, param->add_attr_tab.status);
                for (int i = 0; i < param->add_attr_tab.num_handle; i++) {
                    uint16_t uuid = *((uint16_t*)gatt_db[i].att_desc.uuid_p);

                    handle2uuid_map[param->add_attr_tab.handles[i]] = uuid;

                    if (uuid == ESP_GATT_UUID_PRI_SERVICE)
                    {
                        esp_ble_gatts_start_service(param->add_attr_tab.handles[i]);
                        continue;
                    }
                    else if (uuid == ESP_GATT_UUID_CHAR_DECLARE || uuid == ESP_GATT_UUID_CHAR_CLIENT_CONFIG) {
                        continue;
                    }

                    auto it = gatt_map.find(uuid);
                    if (it == gatt_map.end()) {
                        ESP_LOGE(GATTS_TAG, "UUID %04X not found in gatt_map", uuid);
                        continue;
                    }

                    gatt_param_t* gp = &it->second;
                    gp->handle = param->add_attr_tab.handles[i];

                    if (debug) {
                        ESP_LOGI(GATTS_TAG, "    %3d:  \"%20s\"  %04X  %8s  %22s  %17s  %p",
                            gp->handle, gp->name, gp->uuid.uuid.uuid16,
                            type_to_string(gp->type), permission_to_string(gp->valueAttr),
                            property_to_string(gp->char_property), gp->write_cb);
                    }
                }
            }
            else {
                ESP_LOGE(GATTS_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to gatt_db_idx(%d)",
                    param->add_attr_tab.num_handle, gatt_db_idx);
            }
        }
        else {
            ESP_LOGE(GATTS_TAG, " Create attribute table failed, error code = %x", param->create.status);
        }
        break;
    }

    default:
        break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
    esp_ble_gatts_cb_param_t* param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        }
        else {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d",
                param->reg.app_id,
                param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

static void setup_advertising(esp_bt_uuid_t uuid)
{
    // Setup advertising data
    // This is the first packet sent by a BLE peripheral when it's advertising. 
    // It's broadcast periodically and contains limited information about the device.
    // 31 bytes max (legacy mode).

    int i = 0;

    ext_adv_raw_data[i++] = 0x02;
    ext_adv_raw_data[i++] = ESP_BLE_AD_TYPE_FLAG;
    ext_adv_raw_data[i++] = 0x06;

    // Setup service name
    size_t short_name_len = strlen(gatt_short_name);
    ext_adv_raw_data[i++] = short_name_len + 1;
    ext_adv_raw_data[i++] = ESP_BLE_AD_TYPE_NAME_SHORT;
    for (size_t j = 0; j < short_name_len; j++)
        ext_adv_raw_data[i++] = gatt_short_name[j];

    // 16 bit service UUID
    ext_adv_raw_data[i++] = 0x03;
    ext_adv_raw_data[i++] = ESP_BLE_AD_TYPE_16SRV_CMPL;
    ext_adv_raw_data[i++] = uuid.uuid.uuid16 & 0xFF;
    ext_adv_raw_data[i++] = uuid.uuid.uuid16 >> 8;

    ext_adv_raw_data_len = i;
}

static void setup_scan_response(esp_bt_uuid_t uuid)
{
    // Setup scan response data
    // This is a follow-up packet, only sent if the central device actively 
    // requests it by sending a SCAN_REQ.
    // 31 bytes max (legacy mode).

    int i = 0;

    // Add full local name
    size_t name_len = strlen(gatt_name);
    scan_rsp_data[i++] = name_len + 1;
    scan_rsp_data[i++] = ESP_BLE_AD_TYPE_NAME_CMPL; 
    for (size_t j = 0; j < name_len; j++)
        scan_rsp_data[i++] = gatt_name[j];

    // Manufacturer specific type
    scan_rsp_data[i++] = 5;
    scan_rsp_data[i++] = ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE;
    scan_rsp_data[i++] = 0x34; // Company ID LSB (example)
    scan_rsp_data[i++] = 0x12; // Company ID MSB
    scan_rsp_data[i++] = 0x01; // Custom data
    scan_rsp_data[i++] = 0x02;

    // Transmit power
    scan_rsp_data[i++] = 2;
    scan_rsp_data[i++] = ESP_BLE_AD_TYPE_TX_PWR;
    scan_rsp_data[i++] = 0x00; // 0 dBm

    scan_rsp_data_len = i;
}

gatt_param_handle_t gattserver_register_generic(const char* name, const char* uuid_str,
    gatt_param_type_t type, esp_gatt_perm_t perm,
    esp_gatt_char_prop_t prop, const void* initial_value_ptr, size_t size)
{
    if (!name || !uuid_str || !initial_value_ptr || size == 0)
    {
        ESP_LOGE(GATTS_TAG, "Invalid parameters for registering characteristic: %s", name);
        return NULL;
    }

    if (gatt_db_idx >= GATT_MAX_PARAMS)
    {
        ESP_LOGE(GATTS_TAG, "Maximum number of parameters reached");
        return NULL;
    }

    char* endptr;
    long parsed = strtol(uuid_str, &endptr, 16);
    if (*endptr != '\0' || parsed < 0 || parsed > 0xFFFF) {
        ESP_LOGE(GATTS_TAG, "Invalid UUID string: %s", uuid_str);
        return NULL;
    }
    uint16_t uuid = static_cast<uint16_t>(parsed);

    if (gatt_map.find(uuid) != gatt_map.end())
    {
        ESP_LOGE(GATTS_TAG, "UUID %04X already registered", uuid);
        return NULL;
    }
    gatt_map[uuid] = {};
    gatt_param_t* gp = &gatt_map[uuid];

    uint16_t value_max_size;
    switch (type)
    {
    case GATT_PARAM_TYPE_INT:
        value_max_size = sizeof(int);
        gp->value.i = *(int*)initial_value_ptr;
        break;
    case GATT_PARAM_TYPE_UINT32:
        value_max_size = sizeof(uint32_t);
        gp->value.u32 = *(uint32_t*)initial_value_ptr;
        break;
    case GATT_PARAM_TYPE_FLOAT:
        value_max_size = sizeof(float);
        gp->value.f = *(float*)initial_value_ptr;
        break;
    case GATT_PARAM_TYPE_STRING:
        value_max_size = size;
        gp->value.s = (char*)malloc(size);
        if (gp->value.s == NULL)
        {
            ESP_LOGE(GATTS_TAG, "Failed to allocate memory for string value");
            return NULL;
        }
        strncpy(gp->value.s, (char*)initial_value_ptr, size);
        break;
    case GATT_PARAM_TYPE_GENERIC:
        value_max_size = size;
        gp->value.v = malloc(size);
        if (gp->value.v == NULL)
        {
            ESP_LOGE(GATTS_TAG, "Failed to allocate memory for generic value");
            return NULL;
        }
        memcpy(gp->value.v, initial_value_ptr, size);
        break;
    default:
        value_max_size = size;
        break;
    }

    gp->name = name;
    gp->type = type;
    gp->uuid.uuid.uuid16 = uuid;
    gp->char_property = prop;

    // ---- Characteristic Property ----
    gp->propAttr = &gatt_db[gatt_db_idx];
    *gp->propAttr = {
        .attr_control = {ESP_GATT_AUTO_RSP},
        .att_desc = {
            ESP_UUID_LEN_16,                        // Length of the UUID in bytes
            (uint8_t*)&CHARACTER_DECLARATION_UUID,  // UUID Character declaration
            ESP_GATT_PERM_READ,                     // Attribute permissions
            CHAR_DECLARATION_SIZE,                  // Maximum length of the attribute
            CHAR_DECLARATION_SIZE,                  // Current length of the attribute
            (uint8_t*)&CHAR_PROP_READ_WRITE_NOTIFY  // Pointer to the value of the attribute
        }
    };
    gatt_db_idx++;

    // ---- Characteristic Value ----
    if (debug)
        ESP_LOGI(GATTS_TAG, "Registering characteristic: \"%s\". UUID: %s, Perm: %X, Prop: %X, Size: %d, gatt_db_idx: %d",
            name, uuid_str, perm, prop, size, gatt_db_idx);

    gp->valueAttr = &gatt_db[gatt_db_idx];
    *gp->valueAttr = {
        .attr_control = {ESP_GATT_AUTO_RSP},
        .att_desc = {
            ESP_UUID_LEN_16,                        // Length of the UUID in bytes
            (uint8_t*)&gp->uuid.uuid.uuid16,        // UUID Character value
            perm,                                   // Attribute permissions
            value_max_size,                         // Maximum length of the attribute
            (uint16_t)size,                         // Current length of the attribute
            (uint8_t*)&gp->value.u8                 // Pointer to the value of the attribute
        }
    };
    gatt_db_idx++;

    // ---- Client Characteristic Configuration Descriptor ----
    if (prop & (ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_INDICATE))
    {
        gp->descrAttr = &gatt_db[gatt_db_idx];
        *gp->descrAttr = {
            .attr_control = {ESP_GATT_AUTO_RSP},
            .att_desc = {
                ESP_UUID_LEN_16,                        // Length of the UUID in bytes
                (uint8_t*)&CHARACTER_CLIENT_CONFIG_UUID,// UUID Client Characteristic Configuration
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,// Attribute permissions
                sizeof(uint16_t),                       // Maximum length of the attribute
                sizeof(gp->cccd_enabled),               // Current length of the attribute
                (uint8_t*)&gp->cccd_enabled             // Pointer to the value of the attribute
            }
        };
        gatt_db_idx++;
    }
    return (gatt_param_handle_t)gp;
}

gatt_param_handle_t gattserver_register_float(const char* name, const char* uuid_str,
    esp_gatt_perm_t perm, esp_gatt_char_prop_t prop, float init_value) {
    return gattserver_register_generic(name, uuid_str, GATT_PARAM_TYPE_FLOAT, perm, prop, &init_value, sizeof(float));
}

gatt_param_handle_t gattserver_register_int(const char* name, const char* uuid_str,
    esp_gatt_perm_t perm, esp_gatt_char_prop_t prop, int init_value) {
    return gattserver_register_generic(name, uuid_str, GATT_PARAM_TYPE_INT, perm, prop, &init_value, sizeof(int));
}

gatt_param_handle_t gattserver_register_uint32(const char* name, const char* uuid_str,
    esp_gatt_perm_t perm, esp_gatt_char_prop_t prop, uint32_t init_value) {
    return gattserver_register_generic(name, uuid_str, GATT_PARAM_TYPE_UINT32, perm, prop, &init_value, sizeof(uint32_t));
}

gatt_param_handle_t gattserver_register_string(const char* name, const char* uuid_str,
    esp_gatt_perm_t perm, esp_gatt_char_prop_t prop, const char* init_value) {
    return gattserver_register_generic(name, uuid_str, GATT_PARAM_TYPE_STRING, perm, prop, init_value, strlen(init_value));
}

esp_err_t gattserver_notify(gatt_param_handle_t handle, const void* new_value, size_t value_size)
{
    if (handle == NULL || new_value == NULL || value_size == 0)
    {
        ESP_LOGE(GATTS_TAG, "Invalid parameters for notification");
        return ESP_ERR_INVALID_ARG;
    }

    gatts_profile_inst* profile_inst = &gl_profile_tab[PROFILE_A_APP_ID];

    if (profile_inst->conn_id == ESP_GATT_IF_NONE)
    {
        ESP_LOGW(GATTS_TAG, "No client connected, skipping notification.");
        return ESP_FAIL;
    }

    if (handle->cccd_enabled != 0x0001 && handle->cccd_enabled != 0x0002)
    {
        // ESP_LOGW(GATTS_TAG, "Client has not enabled notifications/indications for %s (UUID: 0x%X)",
        //     handle->name, handle->uuid.uuid.uuid16);
        return ESP_FAIL;
    }

    if (debug)
        ESP_LOGI(GATTS_TAG, "Sending notification for %s (UUID: 0x%X), value size %d",
            handle->name, handle->uuid.uuid.uuid16, value_size);

    esp_err_t err = esp_ble_gatts_send_indicate(profile_inst->gatts_if, profile_inst->conn_id,
        handle->handle, value_size, (uint8_t*)new_value, false);

    if (err != ESP_OK)
    {
        ESP_LOGE(GATTS_TAG, "Failed to send notification for %s, error: %s",
            handle->name, esp_err_to_name(err));
    }

    return err;
}

esp_err_t gattserver_notify_int(gatt_param_handle_t handle, int new_value)
{
    return gattserver_notify(handle, &new_value, sizeof(int));
}

esp_err_t gattserver_notify_uint32(gatt_param_handle_t handle, uint32_t new_value)
{
    return gattserver_notify(handle, &new_value, sizeof(uint32_t));
}

esp_err_t gattserver_notify_float(gatt_param_handle_t handle, float new_value)
{
    return gattserver_notify(handle, &new_value, sizeof(float));
}

esp_err_t gattserver_notify_string(gatt_param_handle_t handle, const char* new_value)
{
    return gattserver_notify(handle, new_value, strlen(new_value));
}

static void evloop_write_cb(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    gatt_param_t* gp = (gatt_param_t*)arg;
    gatt_param_handle_t handle = (gatt_param_handle_t)gp;
    if (gp)
    {
        ESP_LOGI(GATTS_TAG, "Executing write callback for %s (UUID: 0x%X)", gp->name, gp->uuid.uuid.uuid16);
        gp->write_cb(handle, gp->valueAttr->att_desc.value, gp->valueAttr->att_desc.length);
    }
}

esp_err_t gattserver_register_write_cb(gatt_param_handle_t handle, esp_event_loop_handle_t loop_handle, esp_event_base_t base, uint32_t event_id, gatt_write_cb_t cb)
{
    if (handle == NULL || cb == NULL || loop_handle == NULL)
    {
        ESP_LOGE(GATTS_TAG, "Invalid parameters for write callback registration of parameter %s: handle: %s, cb: %s, loop_handle %s",
            handle->name, handle ? "OK" : "NULL", cb ? "OK" : "NULL", loop_handle ? "OK" : "NULL");
        return ESP_ERR_INVALID_ARG;
    }
    gatt_param_t* gp = (gatt_param_t*)handle;
    if (gp->write_cb)
    {
        ESP_LOGW(GATTS_TAG, "Write callback already registered for %s (UUID: 0x%X), overwriting", gp->name, gp->uuid.uuid.uuid16);
    }
    gp->write_cb = cb;
    gp->loop_handle = loop_handle;
    gp->loop_base = base;
    gp->event_id = event_id;

    esp_err_t err = esp_event_handler_instance_register_with(loop_handle, base, event_id,
        &evloop_write_cb, gp, NULL);
    if (err != ESP_OK)
    {
        ESP_LOGE(GATTS_TAG, "Failed to register write callback for %s (UUID: 0x%X)", gp->name, gp->uuid.uuid.uuid16);
    }
    return err;
}

esp_bt_uuid_t convert_uuid(const char* uuid_str) {
    esp_bt_uuid_t uuid;
    // The uuid_str is a 4 char long string with the UUID in hex format
    uint16_t uuid16 = strtol(uuid_str, NULL, 16);
    uuid.len = ESP_UUID_LEN_16;
    uuid.uuid.uuid16 = uuid16;
    return uuid;
}

void gattserver_set_debug(bool d)
{
    debug = d;
}

void gattserver_init(const char* name, const char* short_name, const char* service_uuid)
{
    esp_err_t ret;

    static esp_bt_uuid_t uuid = convert_uuid(service_uuid);

    strncpy(gatt_name, name, sizeof(gatt_name) - 1);
    gatt_name[sizeof(gatt_name) - 1] = '\0';
    strncpy(gatt_short_name, short_name, sizeof(gatt_short_name) - 1);
    gatt_short_name[sizeof(gatt_short_name) - 1] = '\0';

    esp_ble_gap_set_device_name(gatt_name);

    setup_advertising(uuid);
    setup_scan_response(uuid);

    gatt_db[0] = {
        .attr_control = {ESP_GATT_AUTO_RSP},
        .att_desc = {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p = (uint8_t*)&PRIMARY_SERVICE_UUID,
            .perm = ESP_GATT_PERM_READ,
            .max_length = sizeof(uint16_t),
            .length = sizeof(uint16_t),
            .value = (uint8_t*)&uuid.uuid.uuid16
        }
    };

    printf("uuid: %04X\n", *gatt_db[0].att_desc.value);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(GATTS_TAG, "%s init bluetooth", __func__);

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(APP_ID);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    //set static passkey
    uint32_t passkey = 123456;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
    /* If your BLE device acts as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the master;
    If your BLE device acts as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    /* Just show how to clear all the bonded devices
     * Delay 30s, clear all the bonded devices
     *
     * vTaskDelay(30000 / portTICK_PERIOD_MS);
     * remove_all_bonded_devices();
     */
    ESP_LOGI(GATTS_TAG, "Gattserver initialized, gatt_db_idx: %d", gatt_db_idx);
}
#endif
