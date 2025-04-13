/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
// #include "bleprph.h"
#include "services/ans/ble_svc_ans.h"
#include "gattserver.h"

/*** Maximum number of characteristics with the notify flag ***/
#define MAX_NOTIFY 5

static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0x2d, 0x71, 0xa2, 0x59, 0xb4, 0x58, 0xc8, 0x12,
                     0x99, 0x99, 0x43, 0x95, 0x12, 0x2f, 0x46, 0x59);

/* A characteristic that can be subscribed to */
static uint8_t gatt_svr_chr_val;
static uint16_t gatt_svr_chr_val_handle;
static const ble_uuid128_t gatt_svr_chr_uuid =
    BLE_UUID128_INIT(0x00, 0x00, 0x00, 0x00, 0x11, 0x11, 0x11, 0x11,
                     0x22, 0x22, 0x22, 0x22, 0x33, 0x33, 0x33, 0x33);

/* A custom descriptor */
static uint8_t gatt_svr_dsc_val;
static const ble_uuid128_t gatt_svr_dsc_uuid =
    BLE_UUID128_INIT(0x01, 0x01, 0x01, 0x01, 0x12, 0x12, 0x12, 0x12,
                     0x23, 0x23, 0x23, 0x23, 0x34, 0x34, 0x34, 0x34);

static int gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt,
                           void *arg);

static struct ble_gatt_svc_def gatt_svr_svcs[GATT_MAX_SERVICES + 1];
static struct ble_gatt_chr_def characteristics[GATT_MAX_PARAMS + 1];

typedef struct gatt_service_t {
    ble_uuid_any_t uuid;
    struct ble_gatt_svc_def def;
    struct ble_gatt_chr_def* characteristics;
    int char_count;
} gatt_service_t;

typedef struct gatt_param_t gatt_param_t;
typedef gatt_param_t* gatt_param_handle_t;

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

static gatt_service_t gatt_services[GATT_MAX_SERVICES];
static int gatt_service_count = 0;
static gatt_param_t gatt_params[GATT_MAX_PARAMS];
static int gatt_param_count = 0;

//  static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
//      {
//          /*** Service ***/
//          .type = BLE_GATT_SVC_TYPE_PRIMARY,
//          .uuid = &gatt_svr_svc_uuid.u,
//          .characteristics = (struct ble_gatt_chr_def[])
//          { {
//                  /*** This characteristic can be subscribed to by writing 0x00 and 0x01 to the CCCD ***/
//                  .uuid = &gatt_svr_chr_uuid.u,
//                  .access_cb = gatt_svc_access,
//  #if CONFIG_EXAMPLE_ENCRYPTION
//                  .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
//                  BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
//                  BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
//  #else
//                  .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
//  #endif
//                  .val_handle = &gatt_svr_chr_val_handle,
//                  .descriptors = (struct ble_gatt_dsc_def[])
//                  { {
//                        .uuid = &gatt_svr_dsc_uuid.u,
//  #if CONFIG_EXAMPLE_ENCRYPTION
//                        .att_flags = BLE_ATT_F_READ | BLE_ATT_F_READ_ENC,
//  #else
//                        .att_flags = BLE_ATT_F_READ,
//  #endif
//                        .access_cb = gatt_svc_access,
//                      }, {
//                        0, /* No more descriptors in this characteristic */
//                      }
//                  },
//              }, {
//                  0, /* No more characteristics in this service. */
//              }
//          },
//      },

//      {
//          0, /* No more services. */
//      },
//  };

static int gatt_svr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
                          void *dst, uint16_t *len)
{
    uint16_t om_len;
    int rc;

    om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len)
    {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    if (rc != 0)
    {
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}

/**
 * Access callback whenever a characteristic/descriptor is read or written to.
 * Here reads and writes need to be handled.
 * ctxt->op tells weather the operation is read or write and
 * weather it is on a characteristic or descriptor,
 * ctxt->dsc->uuid tells which characteristic/descriptor is accessed.
 * attr_handle give the value handle of the attribute being accessed.
 * Accordingly do:
 *     Append the value to ctxt->om if the operation is READ
 *     Write ctxt->om to the value if the operation is WRITE
 **/
static int gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    const ble_uuid_t *uuid;
    int rc;

    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE)
        {
            MODLOG_DFLT(INFO, "Characteristic read; conn_handle=%d attr_handle=%d\n",
                        conn_handle, attr_handle);
        }
        else
        {
            MODLOG_DFLT(INFO, "Characteristic read by NimBLE stack; attr_handle=%d\n",
                        attr_handle);
        }
        uuid = ctxt->chr->uuid;
        if (attr_handle == gatt_svr_chr_val_handle)
        {
            rc = os_mbuf_append(ctxt->om,
                                &gatt_svr_chr_val,
                                sizeof(gatt_svr_chr_val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        goto unknown;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE)
        {
            MODLOG_DFLT(INFO, "Characteristic write; conn_handle=%d attr_handle=%d",
                        conn_handle, attr_handle);
        }
        else
        {
            MODLOG_DFLT(INFO, "Characteristic write by NimBLE stack; attr_handle=%d",
                        attr_handle);
        }
        uuid = ctxt->chr->uuid;
        if (attr_handle == gatt_svr_chr_val_handle)
        {
            rc = gatt_svr_write(ctxt->om,
                                sizeof(gatt_svr_chr_val),
                                sizeof(gatt_svr_chr_val),
                                &gatt_svr_chr_val, NULL);
            ble_gatts_chr_updated(attr_handle);
            MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                              "all subscribed peers.\n");
            return rc;
        }
        goto unknown;

    case BLE_GATT_ACCESS_OP_READ_DSC:
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE)
        {
            MODLOG_DFLT(INFO, "Descriptor read; conn_handle=%d attr_handle=%d\n",
                        conn_handle, attr_handle);
        }
        else
        {
            MODLOG_DFLT(INFO, "Descriptor read by NimBLE stack; attr_handle=%d\n",
                        attr_handle);
        }
        uuid = ctxt->dsc->uuid;
        if (ble_uuid_cmp(uuid, &gatt_svr_dsc_uuid.u) == 0)
        {
            rc = os_mbuf_append(ctxt->om,
                                &gatt_svr_dsc_val,
                                sizeof(gatt_svr_chr_val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        goto unknown;

    case BLE_GATT_ACCESS_OP_WRITE_DSC:
        goto unknown;

    default:
        goto unknown;
    }

unknown:
    /* Unknown characteristic/descriptor;
     * The NimBLE host should not have called this function;
     */
    assert(0);
    return BLE_ATT_ERR_UNLIKELY;
}

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op)
    {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "registering characteristic %s with "
                           "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

static int gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    gatt_param_t *param = (gatt_param_t *)arg;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        return os_mbuf_append(ctxt->om, param->value_buf, param->value_len);
    }
    else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
    {
        int len = OS_MBUF_PKTLEN(ctxt->om);
        if (len <= sizeof(param->value_buf))
        {
            os_mbuf_copydata(ctxt->om, 0, len, param->value_buf);
            param->value_len = len;
            if (param->write_cb)
            {
                param->write_cb(param, param->value_buf, param->value_len);
            }
        }
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

gatt_param_handle_t gatt_register_generic_to_service(
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

gatt_service_handle_t gatt_register_service(const ble_uuid_any_t *uuid)
{
    if (gatt_service_count >= GATT_MAX_SERVICES)
        return NULL;
    gatt_service_t *svc = &gatt_services[gatt_service_count++];
    svc->uuid = *uuid;
    svc->char_count = 0;
    svc->characteristics = NULL;
    return svc;
}

esp_err_t gatt_notify(gatt_param_handle_t handle, const void* new_value, size_t len) 
{
    if (!handle || len > sizeof(handle->value_buf)) return ESP_ERR_INVALID_ARG;
    memcpy(handle->value_buf, new_value, len);
    handle->value_len = len;
    return ble_gatts_notify(0, handle->handle);
}

esp_err_t gatt_register_write_cb(gatt_param_handle_t handle, gatt_write_cb_t cb) {
    if (!handle) return ESP_ERR_INVALID_ARG;
    gatt_param_t* param = (gatt_param_t*)handle;
    param->write_cb = cb;
    return ESP_OK;
}

int gatt_svr_init(void)
{
    int svc_index = 0;
    int chr_index = 0;

    for (int s = 0; s < gatt_service_count; ++s)
    {
        gatt_service_t *svc = &gatt_services[s];

        svc->characteristics = &characteristics[chr_index];
        int local_char_count = 0;

        for (int i = 0; i < gatt_param_count; ++i)
        {
            gatt_param_t *p = &gatt_params[i];
            if (p->service == svc)
            {
                characteristics[chr_index++] = (struct ble_gatt_chr_def){
                    .uuid = &p->uuid.u,
                    .access_cb = gatt_access_cb,
                    .arg = p,
                    .flags = p->flags};
                local_char_count++;
            }
        }

        characteristics[chr_index++] = (struct ble_gatt_chr_def){};

        gatt_svr_svcs[svc_index++] = (struct ble_gatt_svc_def){
            .type = BLE_GATT_SVC_TYPE_PRIMARY,
            .uuid = &svc->uuid.u,
            .characteristics = svc->characteristics};
    }

    gatt_svr_svcs[svc_index] = (struct ble_gatt_svc_def){};
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    //ble_svc_ans_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0)
    {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0)
    {
        return rc;
    }

    /* Setting a value for the read-only descriptor */
    gatt_svr_dsc_val = 0x99;

    return 0;
}