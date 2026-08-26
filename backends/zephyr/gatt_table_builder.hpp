#pragma once

#include <cstddef>
#include <cstdint>

#include "gattserver.h"

namespace gattserver::zephyr
{

// These portable permission bits deliberately match Zephyr's basic read and
// write permissions. The backend proves that relationship with static_asserts.
constexpr uint16_t kPermissionNone = 0;
constexpr uint16_t kPermissionRead = 1u << 0;
constexpr uint16_t kPermissionWrite = 1u << 1;

struct CharacteristicLayout
{
    uint8_t properties;
    uint16_t permissions;
    bool has_ccc;
    bool encryption_deferred;
    size_t attribute_count;
};

constexpr CharacteristicLayout build_characteristic_layout(
    gatt_chr_flags_t flags)
{
    constexpr gatt_chr_flags_t property_mask =
        GATT_CHR_PROP_BROADCAST |
        GATT_CHR_PROP_READ |
        GATT_CHR_PROP_WRITE_NO_RSP |
        GATT_CHR_PROP_WRITE |
        GATT_CHR_PROP_NOTIFY |
        GATT_CHR_PROP_INDICATE;

    const uint8_t properties = static_cast<uint8_t>(flags & property_mask);
    uint16_t permissions = kPermissionNone;
    if ((properties & GATT_CHR_PROP_READ) != 0)
    {
        permissions |= kPermissionRead;
    }
    if ((properties &
         (GATT_CHR_PROP_WRITE | GATT_CHR_PROP_WRITE_NO_RSP)) != 0)
    {
        permissions |= kPermissionWrite;
    }

    const bool has_ccc =
        (properties & (GATT_CHR_PROP_NOTIFY | GATT_CHR_PROP_INDICATE)) != 0;
    const bool encryption_deferred =
        (flags & (GATT_CHR_F_READ_ENC | GATT_CHR_F_WRITE_ENC)) != 0;

    return {
        properties,
        permissions,
        has_ccc,
        encryption_deferred,
        static_cast<size_t>(has_ccc ? 3 : 2),
    };
}

constexpr size_t service_attribute_count(
    const gatt_chr_flags_t *flags, size_t count)
{
    size_t attributes = 1; // Primary service declaration.
    for (size_t i = 0; i < count; ++i)
    {
        attributes += build_characteristic_layout(flags[i]).attribute_count;
    }
    return attributes;
}

constexpr bool is_supported_uuid(const gatt_uuid_t &uuid)
{
    return uuid.type == GATT_UUID_TYPE_16 ||
           uuid.type == GATT_UUID_TYPE_128;
}

} // namespace gattserver::zephyr
