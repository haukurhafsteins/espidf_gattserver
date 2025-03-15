/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <memory.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

#include "gattserver.h"

#define GATTS_TAG "GATTSERVER"

#define PROFILE_A_APP_ID 0

#define PROFILE_NUM                               1
#define HEART_PROFILE_APP_IDX                     0
#define ESP_HEART_RATE_APP_ID                     0x55
#define HEART_RATE_SVC_INST_ID                    0
#define EXT_ADV_HANDLE                            0
#define NUM_EXT_ADV_SET                           1
#define EXT_ADV_DURATION                          0
#define EXT_ADV_MAX_EVENTS                        0
#define WRITE_EVENT_ID 1234566 // TODO: Find the correct number


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
    [HEART_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },

};

/* Service */
// static const uint16_t GATTS_SERVICE_UUID_TEST = 0x00FF;
// static const uint16_t GATTS_CHAR_UUID_TEST_A = 0xFF01;
// static const uint16_t GATTS_CHAR_UUID_TEST_B = 0xFF02;
// static const uint16_t GATTS_CHAR_UUID_TEST_C = 0xFF03;

// static const uint8_t heart_measurement_ccc[2] = { 0x00, 0x00 };
// static const uint8_t char_value[4] = { 0x11, 0x22, 0x33, 0x44 };
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))
// #define SVC_INST_ID                 0

// static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
// static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;
// static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

/* Full Database Description - Used to add attributes into the database */
typedef struct gatt_param_t
{
    const char* name;
    esp_gatts_attr_db_t* attr;

    /// @brief Callback parameters for writing
    gatt_write_cb_t write_cb;
    uint16_t handle;           // BLE characteristic handle
    uint16_t cccd_enabled;     // 0x0000 = off, 0x0001 = notifications, 0x0002 = indications. Controlled by the client
    esp_event_base_t loop_base;
    esp_event_loop_handle_t loop_handle;
    uint8_t char_property;

    esp_bt_uuid_t uuid;

    union {
        float f;
        int i;
        char* s;
        void* v;
    } value;
} gatt_param_t;

static gatt_param_t gatt_param[GATT_MAX_PARAMS] = {};
static esp_gatts_attr_db_t gatt_db[GATT_MAX_PARAMS] = {};
static size_t gatt_db_idx = 1; // O is taken for service id

static char* esp_key_type_to_str(esp_ble_key_type_t key_type)
{
    char* key_str = NULL;
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

static char* esp_auth_req_to_str(esp_ble_auth_req_t auth_req)
{
    char* auth_str = NULL;
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
        esp_log_buffer_hex(GATTS_TAG, (void*)dev_list[i].bd_addr, sizeof(esp_bd_addr_t));
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
        /* Call the following function to input the passkey which is displayed on the remote device */
        //esp_ble_passkey_reply(gl_profile_tab[HEART_PROFILE_APP_IDX].remote_bda, true, 0x00);
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
        esp_log_buffer_hex(GATTS_TAG, (void*)param->remove_bond_dev_cmpl.bd_addr, sizeof(esp_bd_addr_t));
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
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_REG_EVT");
        //generate a resolvable random address
        esp_ble_gap_config_local_privacy(true);
        esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, gatt_db_idx, 0);
        break;
    case ESP_GATTS_READ_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_READ_EVT");
        break;
    case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_WRITE_EVT, write value:");
        esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
        for (int i = 0; i < gatt_db_idx; i++) 
        {
            if (gatt_param[i].handle + 1 == param->write.handle) // CCCD is the next handle, see gattserver_register_generic
            {
                uint16_t cccd_value = param->write.value[0] | (param->write.value[1] << 8);
                gatt_param[i].cccd_enabled = cccd_value;
                ESP_LOGI(GATTS_TAG, "Descriptor Parameter written: %s (UUID: 0x%X)", gatt_param[i].name, gatt_param[i].uuid.uuid.uuid16);
                break;
            }
            if (gatt_param[i].handle == param->write.handle) 
            {
                ESP_LOGI(GATTS_TAG, "Parameter written: %s (UUID: 0x%X)", gatt_param[i].name, gatt_param[i].uuid.uuid.uuid16);
                if (gatt_param[i].write_cb) {
                    gatt_param[i].write_cb((gatt_param_handle_t)&gatt_param[i], param->write.value, param->write.len);
                }
                break;
            }
        }
        break;
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
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT");
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
        ESP_LOGI(GATTS_TAG, "The number handle = %x", param->add_attr_tab.num_handle);
        if (param->create.status == ESP_GATT_OK) {
            if (param->add_attr_tab.num_handle == gatt_db_idx) {
                for (int i=0;i<gatt_db_idx;i++) {
                    gatt_param[i].handle = param->add_attr_tab.handles[i];
                }
                esp_ble_gatts_start_service(gatt_param[0].handle); //start the service, 0 is the index of the first service
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
            gl_profile_tab[HEART_PROFILE_APP_IDX].gatts_if = gatts_if;
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

gatt_param_handle_t gattserver_register_generic(const char* name, const char* uuid_str,
    gatt_param_type_t type, esp_gatt_perm_t perm,
    esp_gatt_char_prop_t prop, const void* initial_value_ptr, size_t size)
{
    static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
    static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

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

    gatt_param_t* param;
    
    param = &gatt_param[gatt_db_idx];
    param->name = name;
    param->write_cb = NULL;
    param->uuid.uuid.uuid16 = strtol(uuid_str, NULL, 16);
    param->char_property = prop;

    uint16_t value_max_size;
    uint8_t* value_ptr = NULL;
    switch (type)
    {
    case GATT_PARAM_TYPE_INT:
        value_max_size = sizeof(int);
        value_ptr = (uint8_t*)&param->value.i;
        break;
    case GATT_PARAM_TYPE_FLOAT:
        value_max_size = sizeof(float);
        value_ptr = (uint8_t*)&param->value.f;
        break;
    case GATT_PARAM_TYPE_STRING:
        value_max_size = GATT_MAX_VALUE_SIZE;
        param->value.s = (char*)malloc(GATT_MAX_VALUE_SIZE);
        if (param->value.s == NULL)
        {
            ESP_LOGE(GATTS_TAG, "Failed to allocate memory for string value");
            return NULL;
        }
        strncpy(param->value.s, (char*)initial_value_ptr, GATT_MAX_VALUE_SIZE);
        value_ptr = (uint8_t*)param->value.s;
        break;
    case GATT_PARAM_TYPE_GENERIC:
        value_max_size = size;
        param->value.v = malloc(size);
        if (param->value.v == NULL)
        {
            ESP_LOGE(GATTS_TAG, "Failed to allocate memory for generic value");
            return NULL;
        }
        memcpy(param->value.v, initial_value_ptr, size);
        value_ptr = (uint8_t*)param->value.v;
        break;
    default:
        value_max_size = size;
    }

    param = &gatt_param[gatt_db_idx];
    param->name = name;
    param->write_cb = NULL;
    param->uuid.uuid.uuid16 = character_declaration_uuid;
    param->char_property = prop;

    // ---- Characteristic Property ----
    gatt_db[gatt_db_idx] = {
        .attr_control = {ESP_GATT_AUTO_RSP},
        .att_desc = {
            ESP_UUID_LEN_16,                        // Length of the UUID in bytes
            (uint8_t*)&character_declaration_uuid,  // UUID Character declaration
            ESP_GATT_PERM_READ,                     // Attribute permissions
            CHAR_DECLARATION_SIZE,                  // Maximum length of the attribute
            CHAR_DECLARATION_SIZE,                  // Current length of the attribute
            (uint8_t*)&param->char_property         // Pointer to the value of the attribute
        }
    };
    gatt_db_idx++;

    param = &gatt_param[gatt_db_idx];
    gatt_param_handle_t ret_handle = (gatt_param_handle_t)param;
    param->name = name;
    param->write_cb = NULL;
    param->uuid.uuid.uuid16 = strtol(uuid_str, NULL, 16);
    param->char_property = prop;

    // ---- Characteristic Value ----
    gatt_db[gatt_db_idx] = {
        .attr_control = {ESP_GATT_AUTO_RSP},
        .att_desc = {
            ESP_UUID_LEN_16,                        // Length of the UUID in bytes
            (uint8_t*)&param->uuid.uuid.uuid16,     // UUID Character value
            perm,                                   // Attribute permissions
            value_max_size,                         // Maximum length of the attribute
            (uint16_t)size,                         // Current length of the attribute
            (uint8_t*)value_ptr                     // Pointer to the value of the attribute
        }
    };
    gatt_db_idx++;

    param = &gatt_param[gatt_db_idx];
    param->name = name;
    param->write_cb = NULL;
    param->uuid.uuid.uuid16 = character_client_config_uuid;
    param->char_property = prop;

    // ---- Client Characteristic Configuration Descriptor ----
    if (prop | ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_INDICATE)
    {
        param->cccd_enabled = 0x0000;
        gatt_db[gatt_db_idx] = {
            .attr_control = {ESP_GATT_AUTO_RSP},
            .att_desc = {
                ESP_UUID_LEN_16,                        // Length of the UUID in bytes
                (uint8_t*)&character_client_config_uuid,// UUID Client Characteristic Configuration
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,// Attribute permissions
                sizeof(uint16_t),                       // Maximum length of the attribute
                sizeof(uint16_t),                       // Current length of the attribute
                (uint8_t*)&param->cccd_enabled          // Pointer to the value of the attribute
            }
        };
        gatt_db_idx++;
    }
    return (gatt_param_handle_t)ret_handle;
}

gatt_param_handle_t gattserver_register_float(const char* name, const char* uuid_str,
    esp_gatt_perm_t perm, esp_gatt_char_prop_t prop, float init_value) {
    return gattserver_register_generic(name, uuid_str, GATT_PARAM_TYPE_FLOAT, perm, prop, &init_value, sizeof(float));
}

gatt_param_handle_t gattserver_register_int(const char* name, const char* uuid_str,
    esp_gatt_perm_t perm, esp_gatt_char_prop_t prop, int init_value) {
    return gattserver_register_generic(name, uuid_str, GATT_PARAM_TYPE_INT, perm, prop, &init_value, sizeof(int));
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
        ESP_LOGW(GATTS_TAG, "Client has not enabled notifications/indications for %s (UUID: 0x%X)",
            handle->name, handle->uuid.uuid.uuid16);
        return ESP_FAIL;
    }

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
    gatt_param_t* param = (gatt_param_t*)arg;
    gatt_param_handle_t handle = (gatt_param_handle_t)param;
    if (param)
    {
        ESP_LOGI(GATTS_TAG, "Executing write callback for %s (UUID: 0x%X)", param->name, param->uuid.uuid.uuid16);
        param->write_cb(handle, &param->attr->att_desc.value, param->attr->att_desc.length);
    }
}

esp_err_t gattserver_register_write_cb(gatt_param_handle_t handle, esp_event_loop_handle_t loop_handle, esp_event_base_t base, gatt_write_cb_t cb)
{
    if (handle == NULL || cb == NULL || loop_handle == NULL)
    {
        ESP_LOGE(GATTS_TAG, "Invalid parameters for write callback registration of parameter %s: handle: %s, cb: %s, loop_handle %s",
            handle->name, handle ? "OK" : "NULL", cb ? "OK" : "NULL", loop_handle ? "OK" : "NULL");
        return ESP_ERR_INVALID_ARG;
    }
    gatt_param_t* param = (gatt_param_t*)handle;
    param->write_cb = cb;
    param->loop_handle = loop_handle;
    param->loop_base = base;

    esp_err_t err = esp_event_handler_instance_register_with(loop_handle, base, WRITE_EVENT_ID,
        &evloop_write_cb, param, NULL);
    if (err != ESP_OK)
    {
        ESP_LOGE(GATTS_TAG, "Failed to register write callback for %s (UUID: 0x%X)", param->name, param->uuid.uuid.uuid16);
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

void gattserver_init(const char* name, const char* service_uuid)
{
    esp_err_t ret;

    static esp_bt_uuid_t uuid = convert_uuid(service_uuid);

    // Setup general discoverable mode
    int i = 0;
    ext_adv_raw_data[i++] = 0x02;
    ext_adv_raw_data[i++] = 0x01;
    ext_adv_raw_data[i++] = 0x06;

    // Setup service name
    size_t name_len = strlen(name);
    ext_adv_raw_data[i++] = name_len + 1;
    ext_adv_raw_data[i++] = 0x09;
    for (size_t j = 0; j < name_len; j++)
        ext_adv_raw_data[i++] = name[j];

    // 16 bit service UUID
    ext_adv_raw_data[i++] = 0x03;
    ext_adv_raw_data[i++] = 0x03;
    ext_adv_raw_data[i++] = uuid.uuid.uuid16 & 0xFF;
    ext_adv_raw_data[i++] = uuid.uuid.uuid16 >> 8;
    ext_adv_raw_data_len = i;

    static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;

    esp_gatts_attr_db_t service = {
        .attr_control = {ESP_GATT_AUTO_RSP},
        .att_desc = {ESP_UUID_LEN_16, (uint8_t*)&primary_service_uuid, ESP_GATT_PERM_READ,
            sizeof(uint16_t), sizeof(uint16_t), (uint8_t*)&uuid.uuid.uuid16}
    };
    gatt_db[0] = service;

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
    ret = esp_ble_gatts_app_register(ESP_HEART_RATE_APP_ID);
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
    ESP_LOGI(GATTS_TAG, "Gattserver initialized, characters: %d", gatt_db_idx);
}
