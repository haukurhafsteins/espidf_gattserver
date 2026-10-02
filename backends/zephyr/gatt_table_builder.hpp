#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "gattserver.h"

namespace gattserver::zephyr
{

// These portable permission bits deliberately match Zephyr's read, write and
// encryption permissions. The backend proves that relationship with
// static_asserts.
constexpr uint16_t kPermissionNone = 0;
constexpr uint16_t kPermissionRead = 1u << 0;
constexpr uint16_t kPermissionWrite = 1u << 1;
constexpr uint16_t kPermissionReadEncrypt = 1u << 2;
constexpr uint16_t kPermissionWriteEncrypt = 1u << 3;
constexpr uint16_t kPermissionPrepareWrite = 1u << 6;

struct CharacteristicLayout
{
    uint8_t properties;
    uint16_t permissions;
    bool has_ccc;
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
    // Each ENC flag protects only the operation it names, and only when the
    // properties already allow that operation (NimBLE semantics on ESP).
    if ((properties & GATT_CHR_PROP_READ) != 0)
    {
        permissions |= kPermissionRead;
        if ((flags & GATT_CHR_F_READ_ENC) != 0)
            permissions |= kPermissionReadEncrypt;
    }
    if ((properties &
         (GATT_CHR_PROP_WRITE | GATT_CHR_PROP_WRITE_NO_RSP)) != 0)
    {
        permissions |= kPermissionWrite | kPermissionPrepareWrite;
        if ((flags & GATT_CHR_F_WRITE_ENC) != 0)
            permissions |= kPermissionWriteEncrypt;
    }

    const bool has_ccc =
        (properties & (GATT_CHR_PROP_NOTIFY | GATT_CHR_PROP_INDICATE)) != 0;

    return {
        properties,
        permissions,
        has_ccc,
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

constexpr std::array<uint8_t, 16> uuid128_value_bytes(
    const gatt_uuid_t &uuid)
{
    std::array<uint8_t, 16> bytes{};
    for (std::size_t i = 0; i < bytes.size(); ++i)
        bytes[i] = uuid.value.u128[i];
    return bytes;
}

} // namespace gattserver::zephyr
