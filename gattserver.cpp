#include "esp_log.h"
#if defined(CONFIG_BT_NIMBLE_ENABLED)

#include "host/ble_gatt.h"
#include "host/ble_hs.h"
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


void bleprph_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

static void bleprph_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

gatt_service_handle_t gattserver_register_service(const ble_uuid_any_t uuid) 
{
    return gatt_register_service(uuid);
}

gatt_param_handle_t gattserver_register_characteristics_to_service(
    gatt_service_handle_t service, const ble_uuid_any_t uuid,
    gatt_param_type_t type, uint8_t flags, const void* init_value, size_t value_size) 
{
    return gatt_register_characteristics_to_service(service, uuid, type, flags, init_value, value_size);
}

gatt_param_handle_t gattserver_register_float_to_service(
    gatt_service_handle_t service,
    const ble_uuid_any_t uuid, uint8_t flags, float init_value) {
    return gatt_register_characteristics_to_service(service, uuid, GATT_PARAM_TYPE_FLOAT, flags, &init_value, sizeof(init_value));
}

gatt_param_handle_t gattserver_register_int8_to_service(
    gatt_service_handle_t service,
    const ble_uuid_any_t uuid, uint8_t flags, int8_t init_value) {
    return gatt_register_characteristics_to_service(service, uuid, GATT_PARAM_TYPE_UINT8, flags, &init_value, sizeof(init_value));
}

gatt_param_handle_t gattserver_register_uint8_to_service(
    gatt_service_handle_t service,
    const ble_uuid_any_t uuid, uint8_t flags, uint8_t init_value) {
    return gatt_register_characteristics_to_service(service, uuid, GATT_PARAM_TYPE_UINT8, flags, &init_value, sizeof(init_value));
}

gatt_param_handle_t gattserver_register_uint32_to_service(
    gatt_service_handle_t service,
    const ble_uuid_any_t uuid, uint8_t flags, uint32_t init_value) {
    return gatt_register_characteristics_to_service(service, uuid, GATT_PARAM_TYPE_UINT32, flags, &init_value, sizeof(init_value));
}

gatt_param_handle_t gattserver_register_bool_to_service(
    gatt_service_handle_t service,
    const ble_uuid_any_t uuid, uint8_t flags, bool init_value) {
    return gatt_register_characteristics_to_service(service, uuid, GATT_PARAM_TYPE_UINT32, flags, &init_value, sizeof(init_value));
}

gatt_param_handle_t gattserver_register_int32_to_service(
    gatt_service_handle_t service,
    const ble_uuid_any_t uuid, uint8_t flags, int32_t init_value) {
    return gatt_register_characteristics_to_service(service, uuid, GATT_PARAM_TYPE_INT32, flags, &init_value, sizeof(init_value));
}

gatt_param_handle_t gattserver_register_string_to_service(
    gatt_service_handle_t service,
    const ble_uuid_any_t uuid, uint8_t flags, const char* init_value) {
        if (init_value == NULL) { ESP_LOGE(TAG, "String value is NULL"); return NULL; }
    return gatt_register_characteristics_to_service(service, uuid, GATT_PARAM_TYPE_STRING, flags, init_value, strlen(init_value)+1);
}

esp_err_t gattserver_register_write_cb(gatt_param_handle_t handle, gatt_write_cb_t cb) 
{
    return gatt_register_write_cb(handle, cb);
}

esp_err_t gattserver_register_read_cb(gatt_param_handle_t handle, gatt_read_cb_t cb)
{
    return gatt_register_read_cb(handle, cb);
}

esp_err_t gattserver_notify(gatt_param_handle_t handle, const void* new_value, size_t len) 
{
    return gatt_notify(handle, new_value, len);
}

esp_err_t gattserver_notify_int32(gatt_param_handle_t handle, int32_t value) { return gattserver_notify(handle, &value, sizeof(int32_t));}
esp_err_t gattserver_notify_uint32(gatt_param_handle_t handle, uint32_t value) { return gattserver_notify(handle, &value, sizeof(uint32_t));}
esp_err_t gattserver_notify_int8(gatt_param_handle_t handle, int8_t value) { return gattserver_notify(handle, &value, sizeof(int8_t));}
esp_err_t gattserver_notify_uint8(gatt_param_handle_t handle, uint8_t value) { return gattserver_notify(handle, &value, sizeof(uint8_t));}
esp_err_t gattserver_notify_bool(gatt_param_handle_t handle, bool value) { return gattserver_notify(handle, &value, sizeof(bool));}
esp_err_t gattserver_notify_float(gatt_param_handle_t handle, float value) { return gattserver_notify(handle, &value, sizeof(float));}

void gattserver_start(const char* name) {
    /* Initialize the NimBLE host configuration. */
    ble_hs_cfg.reset_cb = bleprph_on_reset;
    ble_hs_cfg.sync_cb = gap_bleprph_on_sync;
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
#endif // CONFIG_BT_ENABLED