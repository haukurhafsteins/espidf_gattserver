#pragma once
#include <stddef.h>
#include "esp_event.h"
#include "esp_err.h"
#include "esp_gatt_defs.h"

#define GATT_MAX_PARAMS 60

typedef enum
{
    GATT_PARAM_TYPE_INT,
    GATT_PARAM_TYPE_UINT32,
    GATT_PARAM_TYPE_FLOAT,
    GATT_PARAM_TYPE_STRING,
    GATT_PARAM_TYPE_GENERIC
} gatt_param_type_t;

// Change gatt_param_handle_t to be a pointer to gatt_params[]
typedef struct gatt_param_t gatt_param_t;
typedef gatt_param_t* gatt_param_handle_t;

/// @brief Callback for client writes
/// @param handle Parameter handle
/// @param value The new value
/// @param len Size of the value
typedef void (*gatt_write_cb_t)(gatt_param_handle_t handle, void* value, size_t len);

#ifdef __cplusplus
extern "C"
{
#endif

    /// @brief Initialize the GATT server
    /// @param service_uuid The UUID of the service
    void gattserver_init(const char* name, const char* short_name, const char* service_uuid);

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
    gatt_param_handle_t gattserver_register_uint32(const char* name, const char* uuid_str,
        esp_gatt_perm_t perm, esp_gatt_char_prop_t prop, uint32_t init_value);
    gatt_param_handle_t gattserver_register_string(const char* name, const char* uuid_str,
        esp_gatt_perm_t perm, esp_gatt_char_prop_t prop, const char* init_value);

    /// @brief Register a new characteristics to be exposed via GATT. It can be used for any type of parameter like int, float, string, struct etc.
    /// @param name Parameter name
    /// @param uuid A 128 bit UUID string
    /// @param type Parameter type
    /// @param perm Permissions like read, write
    /// @param prop Properties like read, write, notify
    /// @param init_value Initial value for the parameter
    /// @param value_size Size of the value
    gatt_param_handle_t gattserver_register_generic(const char* name, const char* uuid_str,
        gatt_param_type_t type, esp_gatt_perm_t perm,
        esp_gatt_char_prop_t prop, const void* init_value, size_t value_size);

    /// @brief Trigger a notification for a parameter
    /// @param handle A handle to the parameter
    /// @param new_value The new value to notify
    /// @param value_size Size of the value
    /// @return ESP_OK if successful, otherwise an error code
    esp_err_t gattserver_notify(gatt_param_handle_t handle, const void* new_value, size_t value_size);

    /// @brief Trigger a notification for a parameter
    /// @param handle A handle to the parameter
    /// @param new_value The new value to notify
    /// @return ESP_OK if successful, otherwise an error code
    esp_err_t gattserver_notify_int(gatt_param_handle_t handle, int new_value);
    esp_err_t gattserver_notify_uint32(gatt_param_handle_t handle, uint32_t new_value);
    esp_err_t gattserver_notify_float(gatt_param_handle_t handle, float new_value);
    esp_err_t gattserver_notify_string(gatt_param_handle_t handle, const char* new_value);

    /// @brief Register a callback for client writes
    /// @param handle Parameter handle
    /// @param loop_handle Evloop handle
    /// @param base Evloop base
    /// @param cb The callback
    /// @return ESP_OK if successful, otherwise an error code
    esp_err_t gattserver_register_write_cb(gatt_param_handle_t handle, esp_event_loop_handle_t loop_handle, esp_event_base_t base, uint32_t event_id, gatt_write_cb_t cb);

    void gattserver_set_debug(bool debug);
#ifdef __cplusplus
}
#endif