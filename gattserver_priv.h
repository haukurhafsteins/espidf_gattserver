#pragma once
#include "gattserver.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"

int gatt_svr_init(void);
void gatt_svr_deinit(void);
gatt_service_handle_t gatt_register_service(const ble_uuid_any_t uuid);
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
gatt_param_handle_t gatt_register_characteristics_to_service(
    gatt_service_handle_t service, const ble_uuid_any_t uuid,
    gatt_param_type_t type, ble_gatt_chr_flags flags, const void* init_value, size_t value_size);
esp_err_t gatt_notify(gatt_param_handle_t handle, const void* new_value, size_t len);
esp_err_t gatt_notify_custom(
    gatt_param_handle_t handle, const void *value, size_t len);
esp_err_t gatt_set_value(
    gatt_param_handle_t handle, const void *new_value, size_t len);
bool gatt_is_notify_subscribed(gatt_param_handle_t handle);
esp_err_t gatt_schedule_service_changed(
    uint16_t start_handle, uint16_t end_handle);
bool gatt_service_changed_applied(void);
void gatt_apply_scheduled_service_change(void);
esp_err_t gatt_register_write_cb(gatt_param_handle_t handle, gatt_write_cb_t cb);
esp_err_t gatt_register_write_status_cb(
    gatt_param_handle_t handle, gatt_write_status_cb_t cb);
esp_err_t gatt_register_read_cb(gatt_param_handle_t handle, gatt_read_cb_t cb);
void gatt_update_subscription_state(uint16_t conn_handle, uint16_t attr_handle,
    bool notify_enabled, bool indicate_enabled);
void gatt_clear_subscription_state(uint16_t conn_handle);

void gap_advertise(void);
int gap_bleprph_event_cb(struct ble_gap_event *event, void *arg);
void gap_bleprph_on_sync(void);

const ble_uuid_any_t *gatt_get_primary_service_uuid();

void bleprph_print_conn_desc(struct ble_gap_conn_desc *desc);
void print_addr(const void *addr);

extern uint16_t g_conn_handle;
