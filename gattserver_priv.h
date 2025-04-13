#pragma once
#include "gattserver.h"
#include "host/ble_gap.h"

int gatt_svr_init(void);
gatt_service_handle_t gatt_register_service(const ble_uuid_any_t* uuid);
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
gatt_param_handle_t gatt_register_generic_to_service(
    gatt_service_handle_t service, const char* name, const ble_uuid_any_t* uuid,
    gatt_param_type_t type, uint8_t flags, const void* init_value, size_t value_size);
esp_err_t gatt_notify(gatt_param_handle_t handle, const void* new_value, size_t len);
esp_err_t gatt_register_write_cb(gatt_param_handle_t handle, gatt_write_cb_t cb);

int bleprph_gap_event_cb(struct ble_gap_event *event, void *arg);
