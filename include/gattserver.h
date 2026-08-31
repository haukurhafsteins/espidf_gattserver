#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#define GATT_MAX_PARAMS 60
#define GATT_MAX_SERVICES 8

// Platform-neutral public API: no Bluetooth-stack types leak out of this
// header. The backend converts gatt_uuid_t at the boundary, so callers stay
// portable across stacks (NimBLE today, Zephyr BLE host later).
typedef enum
{
    GATT_UUID_TYPE_16 = 0,
    GATT_UUID_TYPE_128 = 1,
} gatt_uuid_type_t;

typedef struct
{
    uint8_t type; // gatt_uuid_type_t
    union
    {
        uint16_t u16;
        uint8_t u128[16]; // little-endian, as transmitted on air
    } value;
} gatt_uuid_t;

#define GATT_UUID16(uuid) {.type = GATT_UUID_TYPE_16, .value = {.u16 = (uuid)}}
#define GATT_UUID128(...) {.type = GATT_UUID_TYPE_128, .value = {.u128 = __VA_ARGS__}}

// Characteristic property flags - Bluetooth spec bit values. The backend
// static_asserts these match its stack's constants, so passing them straight
// through is safe on any conforming stack.
#define GATT_CHR_PROP_BROADCAST 0x01
#define GATT_CHR_PROP_READ 0x02
#define GATT_CHR_PROP_WRITE_NO_RSP 0x04
#define GATT_CHR_PROP_WRITE 0x08
#define GATT_CHR_PROP_NOTIFY 0x10
#define GATT_CHR_PROP_INDICATE 0x20

// Full-width characteristic flags. Security requirements occupy bits above
// the one-byte Bluetooth property field, so callers and backends must retain
// all 32 bits.
typedef uint32_t gatt_chr_flags_t;
#define GATT_CHR_F_READ_ENC 0x00000200u
#define GATT_CHR_F_WRITE_ENC 0x00001000u

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
typedef uint8_t gatt_write_status_t;
#define GATT_WRITE_OK 0x00u
#define GATT_WRITE_ERR_UNLIKELY 0x0eu
#define GATT_WRITE_ERR_INSUFFICIENT_RESOURCES 0x11u
typedef gatt_write_status_t (*gatt_write_status_cb_t)(
    gatt_param_handle_t handle, void *value, size_t len);
typedef void (*gatt_read_cb_t)(gatt_param_handle_t handle, void *value, size_t len);
typedef void (*gatt_disconnect_cb_t)(void);
void gattserver_register_disconnect_cb(gatt_disconnect_cb_t cb);
// Last BLE disconnect reason code (link-layer), for reconnect-time triage.
uint8_t gattserver_get_last_disconnect_reason(void);

gatt_service_handle_t gattserver_register_service(const gatt_uuid_t uuid);

gatt_param_handle_t gattserver_register_characteristics_to_service(
    gatt_service_handle_t service, const gatt_uuid_t uuid,
    gatt_param_type_t type, gatt_chr_flags_t flags, const void *init_value, size_t value_size);

gatt_param_handle_t gattserver_register_float_to_service(
    gatt_service_handle_t service,
    const gatt_uuid_t uuid, gatt_chr_flags_t flags, float init_value);

gatt_param_handle_t gattserver_register_int8_to_service(
    gatt_service_handle_t service,
    const gatt_uuid_t uuid, gatt_chr_flags_t flags, int8_t init_value);

gatt_param_handle_t gattserver_register_uint8_to_service(
    gatt_service_handle_t service,
    const gatt_uuid_t uuid, gatt_chr_flags_t flags, uint8_t init_value);

gatt_param_handle_t gattserver_register_uint32_to_service(
    gatt_service_handle_t service,
    const gatt_uuid_t uuid, gatt_chr_flags_t flags, uint32_t init_value);

gatt_param_handle_t gattserver_register_int32_to_service(
    gatt_service_handle_t service,
    const gatt_uuid_t uuid, gatt_chr_flags_t flags, int32_t init_value);

gatt_param_handle_t gattserver_register_bool_to_service(
    gatt_service_handle_t service,
    const gatt_uuid_t uuid, gatt_chr_flags_t flags, bool init_value);

gatt_param_handle_t gattserver_register_string_to_service(
    gatt_service_handle_t service,
    const gatt_uuid_t uuid, gatt_chr_flags_t flags, const char *init_value);

esp_err_t gattserver_register_write_cb(gatt_param_handle_t handle, gatt_write_cb_t cb);
esp_err_t gattserver_register_write_status_cb(
    gatt_param_handle_t handle, gatt_write_status_cb_t cb);
esp_err_t gattserver_register_read_cb(gatt_param_handle_t handle, gatt_read_cb_t cb);
// Replace the readable value without notifying. The new length may be smaller
// than the capacity supplied when the characteristic was registered.
esp_err_t gattserver_set_value(
    gatt_param_handle_t handle, const void *value, size_t len);
esp_err_t gattserver_notify(gatt_param_handle_t handle, const void *value, size_t len);
// Notify with an explicit payload without changing the readable value.
esp_err_t gattserver_notify_custom(
    gatt_param_handle_t handle, const void *value, size_t len);
esp_err_t gattserver_notify_int32(gatt_param_handle_t handle, int32_t value);
esp_err_t gattserver_notify_int8(gatt_param_handle_t handle, int8_t value);
esp_err_t gattserver_notify_uint8(gatt_param_handle_t handle, uint8_t value);
esp_err_t gattserver_notify_uint32(gatt_param_handle_t handle, uint32_t value);
esp_err_t gattserver_notify_bool(gatt_param_handle_t handle, bool value);
esp_err_t gattserver_notify_float(gatt_param_handle_t handle, float value);

// True when a peer has notifications enabled on this characteristic and is
// connected (i.e. a notify would actually be sent). Use to skip generating a
// payload nobody is subscribed to receive.
bool gattserver_is_notify_subscribed(gatt_param_handle_t handle);

// Number of stack buffers currently available for outgoing notifications.
// Backends without an equivalent pool metric return 0 so callers fail closed.
int gattserver_get_available_notify_buffers(void);

// Current link state for low-priority notification producers. Returns ATT's
// default MTU (23) and false whenever no peer is connected.
uint16_t gattserver_get_att_mtu(void);
bool gattserver_is_link_encrypted(void);

// Schedule one standard GATT Service Changed indication before the host starts.
// The range is applied from the NimBLE sync callback after bond-store setup and
// before advertising. Only one range can be scheduled per server lifetime.
esp_err_t gattserver_schedule_service_changed(
    uint16_t start_handle, uint16_t end_handle);
bool gattserver_service_changed_applied(void);

void gattserver_start(const char *name);
void gattserver_set_name(const char *name);
void gattserver_stop();

// True once host-controller synchronization has completed after
// gattserver_start(): the stack is up and host procedures may begin. This
// is a raw passthrough of the backend's sync state (NimBLE: ble_hs_synced),
// so it says nothing about identity setup or advertising, which the sync
// callback performs afterwards. Query only while the server is started;
// behavior after gattserver_stop() is backend-defined.
bool gattserver_synced(void);

// Request connection parameters on the active link. fast=true asks for a short
// (~15-30 ms) interval for high-throughput transfers like firmware OTA; fast=false
// restores the power-saving interval. No-op if nothing is connected. The central
// (e.g. iOS) may accept or reject the request per its own rules.
void gattserver_set_fast_conn(bool fast);
