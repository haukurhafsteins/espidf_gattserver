#include "gattserver.h"

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "gatt_table_builder.hpp"
#include "gatt_notify_retry.hpp"
#include "gatt_value_arena.hpp"
#include "gatt_write_assembler.hpp"
#include "gattserver_value_storage.hpp"

LOG_MODULE_REGISTER(gattserver, LOG_LEVEL_INF);

static_assert(GATT_CHR_PROP_BROADCAST == BT_GATT_CHRC_BROADCAST);
static_assert(GATT_CHR_PROP_READ == BT_GATT_CHRC_READ);
static_assert(GATT_CHR_PROP_WRITE_NO_RSP == BT_GATT_CHRC_WRITE_WITHOUT_RESP);
static_assert(GATT_CHR_PROP_WRITE == BT_GATT_CHRC_WRITE);
static_assert(GATT_CHR_PROP_NOTIFY == BT_GATT_CHRC_NOTIFY);
static_assert(GATT_CHR_PROP_INDICATE == BT_GATT_CHRC_INDICATE);
static_assert(gattserver::zephyr::kPermissionNone == BT_GATT_PERM_NONE);
static_assert(gattserver::zephyr::kPermissionRead == BT_GATT_PERM_READ);
static_assert(gattserver::zephyr::kPermissionWrite == BT_GATT_PERM_WRITE);
static_assert(gattserver::zephyr::kPermissionPrepareWrite ==
              BT_GATT_PERM_PREPARE_WRITE);
static_assert(CONFIG_BT_ATT_PREPARE_COUNT >= 4,
              "gattserver long writes require four prepare buffers");
static_assert(GATT_WRITE_ERR_UNLIKELY == BT_ATT_ERR_UNLIKELY);
static_assert(
    GATT_WRITE_ERR_INSUFFICIENT_RESOURCES == BT_ATT_ERR_INSUFFICIENT_RESOURCES);

namespace
{

constexpr size_t kMaxAttributes = GATT_MAX_SERVICES + 3 * GATT_MAX_PARAMS;
constexpr size_t kMaxNameLength = 19;

union ZephyrUuid
{
    bt_uuid uuid;
    bt_uuid_16 uuid16;
    bt_uuid_128 uuid128;
};

const bt_uuid *copy_uuid(ZephyrUuid &destination, const gatt_uuid_t &source)
{
    if (source.type == GATT_UUID_TYPE_16)
    {
        destination.uuid16.uuid.type = BT_UUID_TYPE_16;
        destination.uuid16.val = source.value.u16;
        return &destination.uuid16.uuid;
    }

    destination.uuid128.uuid.type = BT_UUID_TYPE_128;
    const auto bytes = gattserver::zephyr::uuid128_value_bytes(source);
    memcpy(destination.uuid128.val, bytes.data(), bytes.size());
    return &destination.uuid128.uuid;
}

esp_err_t result_to_esp(int result)
{
    return result == 0 ? ESP_OK : ESP_FAIL;
}

} // namespace

struct gatt_service_t
{
    ZephyrUuid uuid_storage{};
    const bt_uuid *uuid = nullptr;
    bt_gatt_service service{};
    bool registered = false;
};

struct gatt_param_t
{
    ZephyrUuid uuid_storage{};
    const bt_uuid *uuid = nullptr;
    gatt_param_type_t type = GATT_PARAM_TYPE_GENERIC;
    gatt_chr_flags_t flags = 0;
    uint8_t *value_buf = nullptr;
    uint16_t value_len = 0;
    uint16_t value_capacity = 0;
    gatt_write_cb_t write_cb = nullptr;
    gatt_write_status_cb_t write_status_cb = nullptr;
    gatt_read_cb_t read_cb = nullptr;
    gatt_service_handle_t service = nullptr;
    bt_gatt_chrc characteristic{};
    _bt_gatt_ccc ccc{};
    bt_gatt_attr *value_attr = nullptr;
    k_work write_callback_work{};
    bool write_callback_initialized = false;
};

namespace
{

gatt_service_t g_services[GATT_MAX_SERVICES]{};
gatt_param_t g_params[GATT_MAX_PARAMS]{};
bt_gatt_attr g_attributes[kMaxAttributes]{};
size_t g_service_count = 0;
size_t g_param_count = 0;
size_t g_attribute_count = 0;
char g_name[kMaxNameLength + 1] = "URUwear";
bool g_started = false;
bool g_callbacks_registered = false;
bool g_service_change_scheduled = false;
bool g_service_change_applied = false;
bool g_advertising = false;
uint8_t g_last_disconnect_reason = 0;
gatt_disconnect_cb_t g_disconnect_cb = nullptr;
gatt_notify_attempt_cb_t g_notify_attempt_cb = nullptr;
bt_conn *g_connection = nullptr;
alignas(std::max_align_t)
uint8_t g_value_storage[CONFIG_GATTSERVER_VALUE_ARENA_BYTES]{};
gattserver::zephyr::ValueArena g_value_arena(
    g_value_storage, sizeof(g_value_storage));

K_MUTEX_DEFINE(g_connection_mutex);

ssize_t read_value(bt_conn *conn,
                   const bt_gatt_attr *attr,
                   void *buffer,
                   uint16_t length,
                   uint16_t offset)
{
    auto *param = static_cast<gatt_param_t *>(attr->user_data);
    if (param == nullptr)
    {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    if (param->read_cb != nullptr)
    {
        param->read_cb(param, param->value_buf, param->value_len);
    }
    return bt_gatt_attr_read(conn,
                             attr,
                             buffer,
                             length,
                             offset,
                             param->value_buf,
                             param->value_len);
}

void deferred_write_callback(k_work *work)
{
    auto *param = CONTAINER_OF(work, gatt_param_t, write_callback_work);
    if (param->write_status_cb != nullptr)
    {
        const gatt_write_status_t status =
            param->write_status_cb(param, param->value_buf, param->value_len);
        if (status != GATT_WRITE_OK)
        {
            // Execute Write has already succeeded on air, so a deferred
            // application rejection cannot be returned as an ATT error.
            LOG_WRN("Deferred long-write callback rejected status 0x%02x",
                    status);
        }
    }
    else if (param->write_cb != nullptr)
    {
        param->write_cb(param, param->value_buf, param->value_len);
    }
}

ssize_t write_value(bt_conn *,
                    const bt_gatt_attr *attr,
                    const void *buffer,
                    uint16_t length,
                    uint16_t offset,
                    uint8_t flags)
{
    auto *param = static_cast<gatt_param_t *>(attr->user_data);
    if (param == nullptr)
    {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    const auto mode = (flags & BT_GATT_WRITE_FLAG_PREPARE) != 0
                          ? gattserver::zephyr::WriteMode::prepare
                      : (flags & BT_GATT_WRITE_FLAG_EXECUTE) != 0
                          ? gattserver::zephyr::WriteMode::execute
                          : gattserver::zephyr::WriteMode::plain;
    const auto result = gattserver::zephyr::assemble_write(
        param->value_buf,
        param->value_capacity,
        param->value_len,
        buffer,
        length,
        offset,
        mode);
    if (!result.accepted())
    {
        return result.error == gattserver::zephyr::WriteError::invalid_offset
                   ? BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET)
                   : BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    // Zephyr requires PREPARE validation callbacks to return zero. NCS 2.9.1
    // then reassembles all queued fragments and calls us once with EXECUTE.
    if (mode == gattserver::zephyr::WriteMode::prepare)
    {
        return 0;
    }
    if (result.callback_deferred)
    {
        // Re-submitting a queued work item is a no-op. This also coalesces
        // execute chunks on Zephyr versions that do not reassemble first.
        (void)k_work_submit(&param->write_callback_work);
        return length;
    }

    if (param->write_status_cb != nullptr)
    {
        const gatt_write_status_t status =
            param->write_status_cb(param, param->value_buf, param->value_len);
        return status == GATT_WRITE_OK ? length : BT_GATT_ERR(status);
    }
    if (param->write_cb != nullptr)
    {
        param->write_cb(param, param->value_buf, param->value_len);
    }
    return length;
}

bt_conn *active_connection_ref()
{
    k_mutex_lock(&g_connection_mutex, K_FOREVER);
    bt_conn *connection =
        g_connection == nullptr ? nullptr : bt_conn_ref(g_connection);
    k_mutex_unlock(&g_connection_mutex);
    return connection;
}

void log_link_info(bt_conn *connection, const char *phase)
{
    bt_conn_info info{};
    const int result = bt_conn_get_info(connection, &info);
    if (result != 0 || info.type != BT_CONN_TYPE_LE)
    {
        LOG_WRN("BLE link %s read failed: %d", phase, result);
        return;
    }

    unsigned txPhy = 0;
    unsigned rxPhy = 0;
    unsigned txMaxLen = 0;
    unsigned txMaxTime = 0;
    unsigned rxMaxLen = 0;
    unsigned rxMaxTime = 0;
#if defined(CONFIG_BT_USER_PHY_UPDATE)
    if (info.le.phy != nullptr)
    {
        txPhy = info.le.phy->tx_phy;
        rxPhy = info.le.phy->rx_phy;
    }
#endif
#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
    if (info.le.data_len != nullptr)
    {
        txMaxLen = info.le.data_len->tx_max_len;
        txMaxTime = info.le.data_len->tx_max_time;
        rxMaxLen = info.le.data_len->rx_max_len;
        rxMaxTime = info.le.data_len->rx_max_time;
    }
#endif
    LOG_INF(
        "BLE link %s: interval=%u interval_us=%u latency=%u timeout=%u phy=%u/%u dle=%u/%u/%u/%u",
        phase,
        static_cast<unsigned>(info.le.interval),
        static_cast<unsigned>(BT_CONN_INTERVAL_TO_US(info.le.interval)),
        static_cast<unsigned>(info.le.latency),
        static_cast<unsigned>(info.le.timeout),
        txPhy,
        rxPhy,
        txMaxLen,
        txMaxTime,
        rxMaxLen,
        rxMaxTime);
}

void request_link_parity(bt_conn *connection)
{
#if defined(CONFIG_BT_USER_PHY_UPDATE)
    static const bt_conn_le_phy_param preferredPhy =
        BT_CONN_LE_PHY_PARAM_INIT(BT_GAP_LE_PHY_2M, BT_GAP_LE_PHY_2M);
    const int phyResult = bt_conn_le_phy_update(
        connection, &preferredPhy);
    if (phyResult != 0 && phyResult != -EALREADY)
        LOG_WRN("BLE 2M PHY request failed: %d", phyResult);
#endif
#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
    static const bt_conn_le_data_len_param preferredDataLength =
        BT_CONN_LE_DATA_LEN_PARAM_INIT(
            BT_GAP_DATA_LEN_MAX, BT_GAP_DATA_TIME_MAX);
    const int dataLengthResult = bt_conn_le_data_len_update(
        connection, &preferredDataLength);
    if (dataLengthResult != 0 && dataLengthResult != -EALREADY)
        LOG_WRN("BLE data length request failed: %d", dataLengthResult);
#endif
}

int start_advertising()
{
    if (!g_started || g_advertising)
    {
        return 0;
    }

    const uint8_t flags = BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR;
    const uint8_t name_length = static_cast<uint8_t>(strlen(g_name));
    const bt_data name_data =
        BT_DATA(BT_DATA_NAME_COMPLETE, g_name, name_length);
    bt_data advertising_data[3]{};
    size_t advertising_count = 0;
    advertising_data[advertising_count++] =
        BT_DATA(BT_DATA_FLAGS, &flags, sizeof(flags));

    if (g_service_count > 0)
    {
        const bt_uuid *uuid = g_services[0].uuid;
        if (uuid->type == BT_UUID_TYPE_16)
        {
            const auto *uuid16 = BT_UUID_16(uuid);
            advertising_data[advertising_count++] = BT_DATA(
                BT_DATA_UUID16_ALL, &uuid16->val, sizeof(uuid16->val));
        }
        else if (uuid->type == BT_UUID_TYPE_128 && name_length <= 8)
        {
            const auto *uuid128 = BT_UUID_128(uuid);
            advertising_data[advertising_count++] = BT_DATA(
                BT_DATA_UUID128_ALL, uuid128->val, sizeof(uuid128->val));
        }
    }
    advertising_data[advertising_count++] = name_data;

    const bt_data scan_response[] = {name_data};
    const int result = bt_le_adv_start(BT_LE_ADV_CONN,
                                       advertising_data,
                                       advertising_count,
                                       scan_response,
                                       ARRAY_SIZE(scan_response));
    if (result == 0)
    {
        g_advertising = true;
        LOG_INF("Advertising as %s", g_name);
    }
    else
    {
        LOG_ERR("Advertising start failed: %d", result);
    }
    return result;
}

void advertising_work_handler(k_work *)
{
    (void)start_advertising();
}

K_WORK_DEFINE(g_advertising_work, advertising_work_handler);

void connected(bt_conn *connection, uint8_t error)
{
    g_advertising = false;
    if (error != 0)
    {
        LOG_WRN("Connection failed: 0x%02x", error);
        (void)k_work_submit(&g_advertising_work);
        return;
    }

    k_mutex_lock(&g_connection_mutex, K_FOREVER);
    if (g_connection != nullptr)
    {
        bt_conn_unref(g_connection);
    }
    g_connection = bt_conn_ref(connection);
    k_mutex_unlock(&g_connection_mutex);
    LOG_INF("Connected");
    log_link_info(connection, "connected");
    request_link_parity(connection);
}

void disconnected(bt_conn *connection, uint8_t reason)
{
    k_mutex_lock(&g_connection_mutex, K_FOREVER);
    if (g_connection == connection)
    {
        bt_conn_unref(g_connection);
        g_connection = nullptr;
    }
    g_last_disconnect_reason = reason;
    k_mutex_unlock(&g_connection_mutex);

    LOG_INF("Disconnected: 0x%02x", reason);
    if (g_disconnect_cb != nullptr)
    {
        g_disconnect_cb();
    }
    if (g_started)
    {
        (void)k_work_submit(&g_advertising_work);
    }
}

void le_param_updated(
    bt_conn *connection, uint16_t, uint16_t, uint16_t)
{
    log_link_info(connection, "params-updated");
}

#if defined(CONFIG_BT_USER_PHY_UPDATE)
void le_phy_updated(bt_conn *connection, bt_conn_le_phy_info *)
{
    log_link_info(connection, "phy-updated");
}
#endif

#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
void le_data_len_updated(bt_conn *connection, bt_conn_le_data_len_info *)
{
    log_link_info(connection, "dle-updated");
}
#endif

bt_conn_cb g_connection_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .le_param_updated = le_param_updated,
#if defined(CONFIG_BT_USER_PHY_UPDATE)
    .le_phy_updated = le_phy_updated,
#endif
#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
    .le_data_len_updated = le_data_len_updated,
#endif
};

void reset_registration_state()
{
    for (size_t i = 0; i < g_param_count; ++i)
    {
        if (g_params[i].write_callback_initialized)
            (void)k_work_cancel(&g_params[i].write_callback_work);
        g_params[i] = {};
    }
    for (auto &service : g_services)
        service = {};
    for (auto &attribute : g_attributes)
        attribute = {};
    g_value_arena.reset();
    g_service_count = 0;
    g_param_count = 0;
    g_attribute_count = 0;
    g_service_change_scheduled = false;
    g_service_change_applied = false;
}

int register_services()
{
    g_attribute_count = 0;
    for (size_t service_index = 0;
         service_index < g_service_count;
         ++service_index)
    {
        auto &service = g_services[service_index];
        const size_t first_attribute = g_attribute_count;
        g_attributes[g_attribute_count++] = {
            .uuid = BT_UUID_GATT_PRIMARY,
            .read = bt_gatt_attr_read_service,
            .write = nullptr,
            .user_data = const_cast<bt_uuid *>(service.uuid),
            .handle = 0,
            .perm = BT_GATT_PERM_READ,
        };

        for (size_t param_index = 0; param_index < g_param_count; ++param_index)
        {
            auto &param = g_params[param_index];
            if (param.service != &service)
            {
                continue;
            }

            const auto layout =
                gattserver::zephyr::build_characteristic_layout(param.flags);
            if (layout.encryption_deferred)
            {
                // TODO(stage 4): enforce the ENC flags after Zephyr settings
                // and rtos NVS receive non-overlapping flash ownership.
                LOG_WRN("Encrypted GATT permissions deferred for Stage 2");
            }

            param.characteristic = {
                .uuid = param.uuid,
                .value_handle = 0,
                .properties = layout.properties,
            };
            g_attributes[g_attribute_count++] = {
                .uuid = BT_UUID_GATT_CHRC,
                .read = bt_gatt_attr_read_chrc,
                .write = nullptr,
                .user_data = &param.characteristic,
                .handle = 0,
                .perm = BT_GATT_PERM_READ,
            };
            param.value_attr = &g_attributes[g_attribute_count];
            g_attributes[g_attribute_count++] = {
                .uuid = param.uuid,
                .read = (layout.permissions & BT_GATT_PERM_READ) != 0
                            ? read_value
                            : nullptr,
                .write = (layout.permissions & BT_GATT_PERM_WRITE) != 0
                             ? write_value
                             : nullptr,
                .user_data = &param,
                .handle = 0,
                .perm = layout.permissions,
            };
            if (layout.has_ccc)
            {
                param.ccc = {};
                g_attributes[g_attribute_count++] = {
                    .uuid = BT_UUID_GATT_CCC,
                    .read = bt_gatt_attr_read_ccc,
                    .write = bt_gatt_attr_write_ccc,
                    .user_data = &param.ccc,
                    .handle = 0,
                    .perm = BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                };
            }
        }

        service.service.attrs = &g_attributes[first_attribute];
        service.service.attr_count = g_attribute_count - first_attribute;
        const int result = bt_gatt_service_register(&service.service);
        if (result != 0)
        {
            LOG_ERR("Service registration failed: %d", result);
            return result;
        }
        service.registered = true;
    }
    return 0;
}

void unregister_services()
{
    for (size_t i = g_service_count; i > 0; --i)
    {
        auto &service = g_services[i - 1];
        if (!service.registered)
        {
            continue;
        }
        const int result = bt_gatt_service_unregister(&service.service);
        if (result != 0)
        {
            LOG_WRN("Service unregister failed: %d", result);
        }
        service.registered = false;
    }
}

esp_err_t notify_payload(
    gatt_param_handle_t handle,
    const void *value,
    size_t length,
    bool retry_transient = false)
{
    if (handle == nullptr || (value == nullptr && length != 0) ||
        length > UINT16_MAX || handle->value_attr == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    bt_conn *connection = active_connection_ref();
    if (connection == nullptr)
    {
        return ESP_OK;
    }
    if (!bt_gatt_is_subscribed(
            connection, handle->value_attr, BT_GATT_CCC_NOTIFY))
    {
        bt_conn_unref(connection);
        return ESP_OK;
    }

    const auto attempt = [&] {
        const int result = bt_gatt_notify(
            connection,
            handle->value_attr,
            value,
            static_cast<uint16_t>(length));
        if (g_notify_attempt_cb != nullptr)
            g_notify_attempt_cb(handle, result);
        return result;
    };
    const int result = retry_transient
        ? gattserver::zephyr::notify_retry::transmit(
              attempt,
              [](unsigned milliseconds) { k_sleep(K_MSEC(milliseconds)); })
        : attempt();
    bt_conn_unref(connection);
    return result_to_esp(result);
}

} // namespace

gatt_service_handle_t gattserver_register_service(const gatt_uuid_t uuid)
{
    if (g_started || g_service_count >= GATT_MAX_SERVICES ||
        !gattserver::zephyr::is_supported_uuid(uuid))
    {
        return nullptr;
    }

    auto &service = g_services[g_service_count++];
    service.uuid = copy_uuid(service.uuid_storage, uuid);
    return &service;
}

gatt_param_handle_t gattserver_register_characteristics_to_service(
    gatt_service_handle_t service,
    const gatt_uuid_t uuid,
    gatt_param_type_t type,
    gatt_chr_flags_t flags,
    const void *initial_value,
    size_t value_size)
{
    if (g_started || service == nullptr || g_param_count >= GATT_MAX_PARAMS ||
        !gattserver::zephyr::is_supported_uuid(uuid) ||
        value_size > UINT16_MAX)
    {
        return nullptr;
    }

    uint8_t *storage = nullptr;
    if (value_size != 0)
    {
        storage = static_cast<uint8_t *>(g_value_arena.allocate(value_size));
        if (storage == nullptr)
        {
            LOG_ERR("GATT value arena exhausted: request %u, used %u/%u",
                    static_cast<unsigned>(value_size),
                    static_cast<unsigned>(g_value_arena.used()),
                    static_cast<unsigned>(g_value_arena.capacity()));
            return nullptr;
        }
        if (initial_value != nullptr)
            memcpy(storage, initial_value, value_size);
    }

    auto &param = g_params[g_param_count++];
    param.uuid = copy_uuid(param.uuid_storage, uuid);
    param.type = type;
    param.flags = flags;
    param.value_buf = storage;
    param.value_len = static_cast<uint16_t>(value_size);
    param.value_capacity = static_cast<uint16_t>(value_size);
    param.service = service;
    k_work_init(&param.write_callback_work, deferred_write_callback);
    param.write_callback_initialized = true;
    return &param;
}

gatt_param_handle_t gattserver_register_float_to_service(
    gatt_service_handle_t service,
    const gatt_uuid_t uuid,
    gatt_chr_flags_t flags,
    float initial_value)
{
    return gattserver_register_characteristics_to_service(
        service, uuid, GATT_PARAM_TYPE_FLOAT, flags,
        &initial_value, sizeof(initial_value));
}

gatt_param_handle_t gattserver_register_int8_to_service(
    gatt_service_handle_t service,
    const gatt_uuid_t uuid,
    gatt_chr_flags_t flags,
    int8_t initial_value)
{
    return gattserver_register_characteristics_to_service(
        service, uuid, GATT_PARAM_TYPE_UINT8, flags,
        &initial_value, sizeof(initial_value));
}

gatt_param_handle_t gattserver_register_uint8_to_service(
    gatt_service_handle_t service,
    const gatt_uuid_t uuid,
    gatt_chr_flags_t flags,
    uint8_t initial_value)
{
    return gattserver_register_characteristics_to_service(
        service, uuid, GATT_PARAM_TYPE_UINT8, flags,
        &initial_value, sizeof(initial_value));
}

gatt_param_handle_t gattserver_register_uint32_to_service(
    gatt_service_handle_t service,
    const gatt_uuid_t uuid,
    gatt_chr_flags_t flags,
    uint32_t initial_value)
{
    return gattserver_register_characteristics_to_service(
        service, uuid, GATT_PARAM_TYPE_UINT32, flags,
        &initial_value, sizeof(initial_value));
}

gatt_param_handle_t gattserver_register_int32_to_service(
    gatt_service_handle_t service,
    const gatt_uuid_t uuid,
    gatt_chr_flags_t flags,
    int32_t initial_value)
{
    return gattserver_register_characteristics_to_service(
        service, uuid, GATT_PARAM_TYPE_INT32, flags,
        &initial_value, sizeof(initial_value));
}

gatt_param_handle_t gattserver_register_bool_to_service(
    gatt_service_handle_t service,
    const gatt_uuid_t uuid,
    gatt_chr_flags_t flags,
    bool initial_value)
{
    return gattserver_register_characteristics_to_service(
        service, uuid, GATT_PARAM_TYPE_BOOL, flags,
        &initial_value, sizeof(initial_value));
}

gatt_param_handle_t gattserver_register_string_to_service(
    gatt_service_handle_t service,
    const gatt_uuid_t uuid,
    gatt_chr_flags_t flags,
    const char *initial_value)
{
    if (initial_value == nullptr)
    {
        return nullptr;
    }
    return gattserver_register_characteristics_to_service(
        service, uuid, GATT_PARAM_TYPE_STRING, flags,
        initial_value, strlen(initial_value) + 1);
}

esp_err_t gattserver_register_write_cb(
    gatt_param_handle_t handle, gatt_write_cb_t callback)
{
    if (handle == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }
    handle->write_cb = callback;
    return ESP_OK;
}

esp_err_t gattserver_register_write_status_cb(
    gatt_param_handle_t handle, gatt_write_status_cb_t callback)
{
    if (handle == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }
    handle->write_status_cb = callback;
    return ESP_OK;
}

esp_err_t gattserver_register_read_cb(
    gatt_param_handle_t handle, gatt_read_cb_t callback)
{
    if (handle == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }
    handle->read_cb = callback;
    return ESP_OK;
}

void gattserver_register_disconnect_cb(gatt_disconnect_cb_t callback)
{
    g_disconnect_cb = callback;
}

void gattserver_register_notify_attempt_cb(gatt_notify_attempt_cb_t callback)
{
    g_notify_attempt_cb = callback;
}

esp_err_t gattserver_set_value(
    gatt_param_handle_t handle, const void *value, size_t length)
{
    if (handle == nullptr || length > UINT16_MAX)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return gatt_store_value(handle->value_buf,
                            handle->value_capacity,
                            handle->value_len,
                            value,
                            length)
               ? ESP_OK
               : ESP_ERR_INVALID_ARG;
}

esp_err_t gattserver_notify(
    gatt_param_handle_t handle, const void *value, size_t length)
{
    const esp_err_t result = gattserver_set_value(handle, value, length);
    return result == ESP_OK ? notify_payload(handle, value, length) : result;
}

esp_err_t gattserver_notify_reliable(
    gatt_param_handle_t handle, const void *value, size_t length)
{
    const esp_err_t result = gattserver_set_value(handle, value, length);
    return result == ESP_OK
        ? notify_payload(handle, value, length, true)
        : result;
}

esp_err_t gattserver_notify_custom(
    gatt_param_handle_t handle, const void *value, size_t length)
{
    return notify_payload(handle, value, length);
}

esp_err_t gattserver_notify_int32(gatt_param_handle_t handle, int32_t value)
{
    return gattserver_notify(handle, &value, sizeof(value));
}

esp_err_t gattserver_notify_int8(gatt_param_handle_t handle, int8_t value)
{
    return gattserver_notify(handle, &value, sizeof(value));
}

esp_err_t gattserver_notify_uint8(gatt_param_handle_t handle, uint8_t value)
{
    return gattserver_notify(handle, &value, sizeof(value));
}

esp_err_t gattserver_notify_uint32(gatt_param_handle_t handle, uint32_t value)
{
    return gattserver_notify(handle, &value, sizeof(value));
}

esp_err_t gattserver_notify_bool(gatt_param_handle_t handle, bool value)
{
    return gattserver_notify(handle, &value, sizeof(value));
}

esp_err_t gattserver_notify_float(gatt_param_handle_t handle, float value)
{
    return gattserver_notify(handle, &value, sizeof(value));
}

bool gattserver_is_notify_subscribed(gatt_param_handle_t handle)
{
    if (handle == nullptr || handle->value_attr == nullptr)
    {
        return false;
    }
    bt_conn *connection = active_connection_ref();
    if (connection == nullptr)
    {
        return false;
    }
    const bool subscribed = bt_gatt_is_subscribed(
        connection, handle->value_attr, BT_GATT_CCC_NOTIFY);
    bt_conn_unref(connection);
    return subscribed;
}

int gattserver_get_available_notify_buffers(void)
{
    return 0;
}

uint16_t gattserver_get_att_mtu(void)
{
    bt_conn *connection = active_connection_ref();
    if (connection == nullptr)
    {
        return 23;
    }
    const uint16_t mtu = bt_gatt_get_mtu(connection);
    bt_conn_unref(connection);
    return mtu;
}

bool gattserver_is_link_encrypted(void)
{
    bt_conn *connection = active_connection_ref();
    if (connection == nullptr)
    {
        return false;
    }
    const bool encrypted = bt_conn_get_security(connection) >= BT_SECURITY_L2;
    bt_conn_unref(connection);
    return encrypted;
}

esp_err_t gattserver_schedule_service_changed(
    uint16_t start_handle, uint16_t end_handle)
{
    if (g_started || start_handle == 0 || end_handle < start_handle ||
        g_service_change_scheduled)
    {
        return ESP_ERR_INVALID_ARG;
    }
    g_service_change_scheduled = true;
    return ESP_OK;
}

bool gattserver_service_changed_applied(void)
{
    return g_service_change_applied;
}

void gattserver_set_name(const char *name)
{
    if (name == nullptr || name[0] == '\0')
    {
        return;
    }
    const size_t length = MIN(strlen(name), kMaxNameLength);
    memcpy(g_name, name, length);
    g_name[length] = '\0';
    if (bt_is_ready())
    {
        const int result = bt_set_name(g_name);
        if (result != 0)
        {
            LOG_WRN("GAP name update failed: %d", result);
        }
    }
}

void gattserver_start(const char *name)
{
    if (g_started)
    {
        return;
    }
    gattserver_set_name(name);

    int result = 0;
    if (!bt_is_ready())
    {
        result = bt_enable(nullptr);
        if (result != 0)
        {
            LOG_ERR("Bluetooth enable failed: %d", result);
            return;
        }
    }
    result = bt_set_name(g_name);
    if (result != 0)
    {
        LOG_WRN("GAP name update failed: %d", result);
    }

    if (!g_callbacks_registered)
    {
        result = bt_conn_cb_register(&g_connection_callbacks);
        if (result != 0)
        {
            LOG_ERR("Connection callback registration failed: %d", result);
            return;
        }
        g_callbacks_registered = true;
    }

    result = register_services();
    if (result != 0)
    {
        unregister_services();
        return;
    }

    // Dynamic registration triggers Zephyr's Service Changed handling. Stage 2
    // has no bond store, so the requested handle range needs no persistence.
    g_service_change_applied = g_service_change_scheduled;
    g_started = true;
    if (start_advertising() != 0)
    {
        g_started = false;
        unregister_services();
    }
}

void gattserver_stop()
{
    if (!g_started && !bt_is_ready())
    {
        reset_registration_state();
        return;
    }
    g_started = false;
    (void)k_work_cancel(&g_advertising_work);
    if (g_advertising)
    {
        const int result = bt_le_adv_stop();
        if (result != 0 && result != -EALREADY)
        {
            LOG_WRN("Advertising stop failed: %d", result);
        }
        g_advertising = false;
    }

    bt_conn *connection = active_connection_ref();
    if (connection != nullptr)
    {
        (void)bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        bt_conn_unref(connection);
    }

    unregister_services();
    if (bt_is_ready())
    {
        const int result = bt_disable();
        if (result != 0)
        {
            LOG_WRN("Bluetooth disable failed: %d", result);
        }
    }
    reset_registration_state();
}

bool gattserver_synced(void)
{
    return g_started && bt_is_ready();
}

uint8_t gattserver_get_last_disconnect_reason(void)
{
    return g_last_disconnect_reason;
}

void gattserver_set_fast_conn(bool fast)
{
    bt_conn *connection = active_connection_ref();
    if (connection == nullptr)
    {
        return;
    }
    const bt_le_conn_param parameters = fast
        ? bt_le_conn_param{0x000c, 0x0018, 0, 400}
        : bt_le_conn_param{0x0048, 0x0068, 0, 400};
    const int result = bt_conn_le_param_update(connection, &parameters);
    if (result != 0)
    {
        LOG_WRN("Connection parameter update failed: %d", result);
    }
    bt_conn_unref(connection);
}
