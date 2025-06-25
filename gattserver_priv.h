#pragma once
#include "gattserver.h"
#include "host/ble_gap.h"

int gatt_svr_init(void);
void gatt_svr_deinit(void);
gatt_service_handle_t gatt_register_service(const ble_uuid_any_t uuid);
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
gatt_param_handle_t gatt_register_characteristics_to_service(
    gatt_service_handle_t service, const ble_uuid_any_t uuid,
    gatt_param_type_t type, uint8_t flags, const void* init_value, size_t value_size);
esp_err_t gatt_notify(gatt_param_handle_t handle, const void* new_value, size_t len);
esp_err_t gatt_register_write_cb(gatt_param_handle_t handle, gatt_write_cb_t cb);
esp_err_t gatt_register_read_cb(gatt_param_handle_t handle, gatt_read_cb_t cb);

void gap_advertise(void);
int gap_bleprph_event_cb(struct ble_gap_event *event, void *arg);
void gap_bleprph_on_sync(void);


void bleprph_print_conn_desc(struct ble_gap_conn_desc *desc);
void print_addr(const void *addr);

extern uint16_t g_conn_handle;

