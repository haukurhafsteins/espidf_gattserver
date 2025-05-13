#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#include "host/ble_uuid.h"
#include "host/ble_gatt.h"

#define GATT_MAX_PARAMS 60
#define GATT_MAX_SERVICES 8

#define GATT_UUID16(uuid) {.u16 = {.u = {.type = BLE_UUID_TYPE_16}, .value = uuid}}

typedef enum
{
    GATT_PARAM_TYPE_INT32,
    GATT_PARAM_TYPE_UINT32,
    GATT_PARAM_TYPE_UINT8,
    GATT_PARAM_TYPE_BOOL,
    GATT_PARAM_TYPE_FLOAT,
    GATT_PARAM_TYPE_STRING,
    GATT_PARAM_TYPE_GENERIC
} gatt_param_type_t;

typedef struct gatt_param_t *gatt_param_handle_t;
typedef struct gatt_service_t *gatt_service_handle_t;
typedef void (*gatt_write_cb_t)(gatt_param_handle_t handle, void *value, size_t len);

gatt_service_handle_t gattserver_register_service(const ble_uuid_any_t uuid);

gatt_param_handle_t gattserver_register_characteristics_to_service(
    gatt_service_handle_t service, const ble_uuid_any_t uuid,
    gatt_param_type_t type, uint8_t flags, const void *init_value, size_t value_size);

gatt_param_handle_t gattserver_register_float_to_service(
    gatt_service_handle_t service,
    const ble_uuid_any_t uuid, uint8_t flags, float init_value);

gatt_param_handle_t gattserver_register_uint8_to_service(
    gatt_service_handle_t service,
    const ble_uuid_any_t uuid, uint8_t flags, uint8_t init_value);

gatt_param_handle_t gattserver_register_uint32_to_service(
    gatt_service_handle_t service,
    const ble_uuid_any_t uuid, uint8_t flags, uint32_t init_value);

gatt_param_handle_t gattserver_register_int32_to_service(
    gatt_service_handle_t service,
    const ble_uuid_any_t uuid, uint8_t flags, int32_t init_value);

gatt_param_handle_t gattserver_register_bool_to_service(
    gatt_service_handle_t service,
    const ble_uuid_any_t uuid, uint8_t flags, bool init_value);

gatt_param_handle_t gattserver_register_string_to_service(
    gatt_service_handle_t service,
    const ble_uuid_any_t uuid, uint8_t flags, const char *init_value);

esp_err_t gattserver_register_write_cb(gatt_param_handle_t handle, gatt_write_cb_t cb);
esp_err_t gattserver_notify(gatt_param_handle_t handle, const void *value, size_t len);
esp_err_t gattserver_notify_int32(gatt_param_handle_t handle, int32_t value);
esp_err_t gattserver_notify_uint8(gatt_param_handle_t handle, uint8_t value);
esp_err_t gattserver_notify_uint32(gatt_param_handle_t handle, uint32_t value);
esp_err_t gattserver_notify_bool(gatt_param_handle_t handle, bool value);
esp_err_t gattserver_notify_float(gatt_param_handle_t handle, float value);

void gattserver_start(const char *name);
