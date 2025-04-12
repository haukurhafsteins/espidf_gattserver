#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <string.h>

#define TAG "GATTServer"
#define GATT_MAX_PARAMS 60
#define GATT_MAX_SERVICES 8

typedef enum {
    GATT_PARAM_TYPE_UINT32,
    GATT_PARAM_TYPE_FLOAT,
    GATT_PARAM_TYPE_STRING,
    GATT_PARAM_TYPE_GENERIC
} gatt_param_type_t;

typedef struct gatt_param_t gatt_param_t;
typedef gatt_param_t* gatt_param_handle_t;

typedef struct gatt_service_t {
    ble_uuid_any_t uuid;
    struct ble_gatt_svc_def def;
    struct ble_gatt_chr_def* characteristics;
    int char_count;
} gatt_service_t;

typedef gatt_service_t* gatt_service_handle_t;

typedef void (*gatt_write_cb_t)(gatt_param_handle_t handle, void* value, size_t len);

typedef struct gatt_param_t {
    const char* name;
    ble_uuid_any_t uuid;
    uint8_t flags;
    gatt_param_type_t type;
    uint8_t value_buf[64];
    uint16_t value_len;
    uint16_t handle;
    gatt_write_cb_t write_cb;
    gatt_service_handle_t service;
} gatt_param_t;

static gatt_param_t gatt_params[GATT_MAX_PARAMS];
static int gatt_param_count = 0;

static gatt_service_t gatt_services[GATT_MAX_SERVICES];
static int gatt_service_count = 0;

static int gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt* ctxt, void* arg) {
    gatt_param_t* param = (gatt_param_t*)arg;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return os_mbuf_append(ctxt->om, param->value_buf, param->value_len);
    } else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        int len = OS_MBUF_PKTLEN(ctxt->om);
        if (len <= sizeof(param->value_buf)) {
            os_mbuf_copydata(ctxt->om, 0, len, param->value_buf);
            param->value_len = len;
            if (param->write_cb) {
                param->write_cb(param, param->value_buf, param->value_len);
            }
        }
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

gatt_service_handle_t gattserver_register_service(const ble_uuid_any_t* uuid) {
    if (gatt_service_count >= GATT_MAX_SERVICES) return NULL;
    gatt_service_t* svc = &gatt_services[gatt_service_count++];
    svc->uuid = *uuid;
    svc->char_count = 0;
    svc->characteristics = NULL;
    return svc;
}

gatt_param_handle_t gattserver_register_generic_to_service(
    gatt_service_handle_t service, const char* name, const ble_uuid_any_t* uuid,
    gatt_param_type_t type, uint8_t flags, const void* init_value, size_t value_size) {

    if (!service || gatt_param_count >= GATT_MAX_PARAMS || value_size > sizeof(gatt_params[0].value_buf))
        return NULL;

    gatt_param_t* p = &gatt_params[gatt_param_count++];
    p->name = name;
    p->uuid = *uuid;
    p->flags = flags;
    p->type = type;
    memcpy(p->value_buf, init_value, value_size);
    p->value_len = value_size;
    p->write_cb = NULL;
    p->service = service;

    service->char_count++;
    return p;
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

esp_err_t gattserver_register_write_cb(gatt_param_handle_t handle, gatt_write_cb_t cb) {
    if (!handle) return ESP_ERR_INVALID_ARG;
    handle->write_cb = cb;
    return ESP_OK;
}

esp_err_t gattserver_notify(gatt_param_handle_t handle, const void* new_value, size_t len) {
    if (!handle || len > sizeof(handle->value_buf)) return ESP_ERR_INVALID_ARG;
    memcpy(handle->value_buf, new_value, len);
    handle->value_len = len;
    return ble_gatts_notify(0, handle->handle);
}

static void ble_app_advertise(void) {
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };

    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t*)ble_svc_gap_device_name();
    fields.name_len = strlen(ble_svc_gap_device_name());
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);
    ble_gap_adv_start(0, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL);
}

void gattserver_start(void) {
    static struct ble_gatt_svc_def services[GATT_MAX_SERVICES + 1];
    static struct ble_gatt_chr_def characteristics[GATT_MAX_PARAMS + 1];

    int svc_index = 0;
    int chr_index = 0;

    for (int s = 0; s < gatt_service_count; ++s) {
        gatt_service_t* svc = &gatt_services[s];

        svc->characteristics = &characteristics[chr_index];
        int local_char_count = 0;

        for (int i = 0; i < gatt_param_count; ++i) {
            gatt_param_t* p = &gatt_params[i];
            if (p->service == svc) {
                characteristics[chr_index++] = (struct ble_gatt_chr_def){
                    .uuid = &p->uuid.u,
                    .access_cb = gatt_access_cb,
                    .arg = p,
                    .flags = p->flags
                };
                local_char_count++;
            }
        }

        characteristics[chr_index++] = (struct ble_gatt_chr_def){0};

        services[svc_index++] = (struct ble_gatt_svc_def){
            .type = BLE_GATT_SVC_TYPE_PRIMARY,
            .uuid = &svc->uuid.u,
            .characteristics = svc->characteristics
        };
    }

    services[svc_index] = (struct ble_gatt_svc_def){0};

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ESP_ERROR_CHECK(ble_gatts_add_svcs(services));
    ble_app_advertise();
}
