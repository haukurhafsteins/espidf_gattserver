#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/ans/ble_svc_ans.h"
#include <string.h>

#include "gattserver_priv.h"
#include "gattserver.h"

#define TAG "GATTServer"
#define GATT_MAX_PARAMS 60

typedef gatt_service_t* gatt_service_handle_t;

typedef void (*gatt_write_cb_t)(gatt_param_handle_t handle, void* value, size_t len);

static uint8_t own_addr_type;

void bleprph_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

static void
bleprph_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    /**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     *     o 16-bit service UUIDs (alert notifications).
     */

    memset(&fields, 0, sizeof fields);

    /* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /* Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    // fields.uuids16 = (ble_uuid16_t[]) {
    //     BLE_UUID16_INIT(GATT_SVR_SVC_ALERT_UUID)
    // };
    // fields.num_uuids16 = 1;
    // fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, bleprph_gap_event_cb, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

static void bleprph_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

static void bleprph_on_sync(void)
{
    int rc;

#if CONFIG_EXAMPLE_RANDOM_ADDR
    /* Generate a non-resolvable private address. */
    ble_app_set_addr();
#endif

    /* Make sure we have proper identity address set (public preferred) */
#if CONFIG_EXAMPLE_RANDOM_ADDR
    rc = ble_hs_util_ensure_addr(1);
#else
    rc = ble_hs_util_ensure_addr(0);
#endif
    assert(rc == 0);

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Printing ADDR */
    uint8_t addr_val[6] = {};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

    MODLOG_DFLT(INFO, "Device Address: ");
    //print_addr(addr_val);
    MODLOG_DFLT(INFO, "\n");
    /* Begin advertising. */
#if CONFIG_EXAMPLE_EXTENDED_ADV
    ext_bleprph_advertise();
#else
    bleprph_advertise();
#endif
}

gatt_service_handle_t gattserver_register_service(const ble_uuid_any_t* uuid) 
{
    return gatt_register_service(uuid);
}

gatt_param_handle_t gattserver_register_generic_to_service(
    gatt_service_handle_t service, const char* name, const ble_uuid_any_t* uuid,
    gatt_param_type_t type, uint8_t flags, const void* init_value, size_t value_size) 
{
    return gatt_register_generic_to_service(service, name, uuid, type, flags, init_value, value_size);
}

gatt_param_handle_t gattserver_register_float_to_service(
    gatt_service_handle_t service, const char* name,
    const ble_uuid_any_t* uuid, uint8_t flags, float init_value) {
    return gattserver_register_generic_to_service(service, name, uuid, GATT_PARAM_TYPE_FLOAT, flags, &init_value, sizeof(float));
}

gatt_param_handle_t gattserver_register_uint32_to_service(
    gatt_service_handle_t service, const char* name,
    const ble_uuid_any_t* uuid, uint8_t flags, uint32_t init_value) {
    return gattserver_register_generic_to_service(service, name, uuid, GATT_PARAM_TYPE_UINT32, flags, &init_value, sizeof(uint32_t));
}

gatt_param_handle_t gattserver_register_string_to_service(
    gatt_service_handle_t service, const char* name,
    const ble_uuid_any_t* uuid, uint8_t flags, const char* init_value) {
    return gattserver_register_generic_to_service(service, name, uuid, GATT_PARAM_TYPE_STRING, flags, init_value, strlen(init_value));
}

esp_err_t gattserver_register_write_cb(gatt_param_handle_t handle, gatt_write_cb_t cb) 
{
    return gatt_register_write_cb(handle, cb);
}

esp_err_t gattserver_notify(gatt_param_handle_t handle, const void* new_value, size_t len) 
{
    return gatt_notify(handle, new_value, len);
}

void gattserver_start(const char* name) {
    /* Initialize the NimBLE host configuration. */
    ble_hs_cfg.reset_cb = bleprph_on_reset;
    ble_hs_cfg.sync_cb = bleprph_on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
#ifdef CONFIG_EXAMPLE_BONDING
    ble_hs_cfg.sm_bonding = 1;
    /* Enable the appropriate bit masks to make sure the keys
     * that are needed are exchanged
     */
    ble_hs_cfg.sm_our_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC;
    ble_hs_cfg.sm_their_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC;
#endif
#ifdef CONFIG_EXAMPLE_MITM
    ble_hs_cfg.sm_mitm = 1;
#endif
#ifdef CONFIG_EXAMPLE_USE_SC
    ble_hs_cfg.sm_sc = 1;
#else
    ble_hs_cfg.sm_sc = 0;
#endif
#ifdef CONFIG_EXAMPLE_RESOLVE_PEER_ADDR
    /* Stores the IRK */
    ble_hs_cfg.sm_our_key_dist |= BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist |= BLE_SM_PAIR_KEY_DIST_ID;
#endif

    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init nimble %d ", ret);
        return;
    }
    
    gatt_svr_init();

    int rc = ble_svc_gap_device_name_set(name);
    assert(rc == 0);

    /* XXX Need to have template for store */
    //ble_store_config_init();

    nimble_port_freertos_init(bleprph_host_task);
}
