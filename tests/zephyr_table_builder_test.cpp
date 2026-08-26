#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

#include "../backends/zephyr/gatt_table_builder.hpp"

using gattserver::zephyr::CharacteristicLayout;
using gattserver::zephyr::build_characteristic_layout;
using gattserver::zephyr::is_supported_uuid;
using gattserver::zephyr::service_attribute_count;

int main()
{
    const CharacteristicLayout readable =
        build_characteristic_layout(GATT_CHR_PROP_READ);
    assert(readable.properties == GATT_CHR_PROP_READ);
    assert(readable.permissions == gattserver::zephyr::kPermissionRead);
    assert(!readable.has_ccc);
    assert(!readable.encryption_deferred);
    assert(readable.attribute_count == 2);

    const CharacteristicLayout writable = build_characteristic_layout(
        GATT_CHR_PROP_WRITE | GATT_CHR_PROP_WRITE_NO_RSP);
    assert(writable.properties ==
           (GATT_CHR_PROP_WRITE | GATT_CHR_PROP_WRITE_NO_RSP));
    assert(writable.permissions == gattserver::zephyr::kPermissionWrite);
    assert(!writable.has_ccc);
    assert(writable.attribute_count == 2);

    const CharacteristicLayout notify_only =
        build_characteristic_layout(GATT_CHR_PROP_NOTIFY);
    assert(notify_only.permissions == gattserver::zephyr::kPermissionNone);
    assert(notify_only.has_ccc);
    assert(notify_only.attribute_count == 3);

    const CharacteristicLayout encrypted = build_characteristic_layout(
        GATT_CHR_PROP_READ | GATT_CHR_PROP_WRITE |
        GATT_CHR_F_READ_ENC | GATT_CHR_F_WRITE_ENC);
    assert(encrypted.permissions ==
           (gattserver::zephyr::kPermissionRead |
            gattserver::zephyr::kPermissionWrite));
    assert(encrypted.encryption_deferred);

    constexpr std::array<gatt_chr_flags_t, 3> flags = {
        GATT_CHR_PROP_READ,
        GATT_CHR_PROP_NOTIFY,
        GATT_CHR_PROP_WRITE | GATT_CHR_PROP_NOTIFY,
    };
    assert(service_attribute_count(flags.data(), flags.size()) == 9);
    assert(service_attribute_count(nullptr, 0) == 1);

    const gatt_uuid_t uuid16 = GATT_UUID16(0x180f);
    const gatt_uuid_t uuid128 = GATT_UUID128(
        {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
         0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f});
    gatt_uuid_t invalid = uuid16;
    invalid.type = 42;
    assert(is_supported_uuid(uuid16));
    assert(is_supported_uuid(uuid128));
    assert(!is_supported_uuid(invalid));

    return 0;
}
