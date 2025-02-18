#pragma once
#include <stddef.h>
#include "esp_err.h"
#include "esp_gatt_defs.h"

// Change gatt_param_handle_t to be a pointer to gatt_params[]
typedef struct gatt_param_t gatt_param_t;
typedef gatt_param_t *gatt_param_handle_t;
typedef void (*gatt_write_cb_t)(gatt_param_handle_t handle, void *value, size_t len);

#ifdef __cplusplus
extern "C" {
#endif

/// @brief Initialize the GATT server
/// @param service_uuid The UUID of the service
void gattserver_init(const char* name, const char *service_uuid);

/// @brief Register a new parameter to be exposed via GATT
/// @param name Parameter name
/// @param uuid A 128 bit UUID string
/// @param perm Permissions like read, write, notify
/// @param prop Properties like read, write, notify
/// @param init_value Initial value for the parameter
/// @return A handle to the registered parameter
gatt_param_handle_t gattserver_register_float(const char* name, const char* uuid_str,
    esp_gatt_perm_t perm, esp_gatt_char_prop_t prop, float init_value);
gatt_param_handle_t gattserver_register_int(const char* name, const char* uuid_str,
    esp_gatt_perm_t perm, esp_gatt_char_prop_t prop, int init_value);
gatt_param_handle_t gattserver_register_string(const char* name, const char* uuid_str,
    esp_gatt_perm_t perm, esp_gatt_char_prop_t prop, const char* init_value);

/// @brief Trigger a notification for a parameter
/// @param handle A handle to the parameter
/// @param new_value The new value to notify
/// @return ESP_OK if successful, otherwise an error code
esp_err_t gattserver_notify_int(gatt_param_handle_t handle, int new_value);
esp_err_t gattserver_notify_float(gatt_param_handle_t handle, float new_value);
esp_err_t gattserver_notify_string(gatt_param_handle_t handle, const char *new_value);

/// @brief Register a callback for client writes
/// @param handle Parameter handle
/// @param loop_handle Evloop handle
/// @param base Evloop base
/// @param cb The callback
/// @return ESP_OK if successful, otherwise an error code
esp_err_t gattserver_register_write_cb(gatt_param_handle_t handle, esp_event_loop_handle_t loop_handle, esp_event_base_t base, gatt_write_cb_t cb);

#ifdef __cplusplus
}
#endif