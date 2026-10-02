#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

#include "../backends/zephyr/gatt_table_builder.hpp"

using gattserver::zephyr::CharacteristicLayout;
using gattserver::zephyr::build_characteristic_layout;
using gattserver::zephyr::is_supported_uuid;
using gattserver::zephyr::service_attribute_count;
using gattserver::zephyr::uuid128_value_bytes;

int main()
{
    const CharacteristicLayout readable =
        build_characteristic_layout(GATT_CHR_PROP_READ);
    assert(readable.properties == GATT_CHR_PROP_READ);
    assert(readable.permissions == gattserver::zephyr::kPermissionRead);
    assert(!readable.has_ccc);
    assert(readable.attribute_count == 2);

    const CharacteristicLayout writable = build_characteristic_layout(
        GATT_CHR_PROP_WRITE | GATT_CHR_PROP_WRITE_NO_RSP);
    assert(writable.properties ==
           (GATT_CHR_PROP_WRITE | GATT_CHR_PROP_WRITE_NO_RSP));
    assert(writable.permissions ==
           (gattserver::zephyr::kPermissionWrite |
            gattserver::zephyr::kPermissionPrepareWrite));
    assert(!writable.has_ccc);
    assert(writable.attribute_count == 2);

    const CharacteristicLayout notify_only =
        build_characteristic_layout(GATT_CHR_PROP_NOTIFY);
    assert(notify_only.permissions == gattserver::zephyr::kPermissionNone);
    assert(notify_only.has_ccc);
    assert(notify_only.attribute_count == 3);

    namespace zp = gattserver::zephyr;

    // Declared ENC flags are enforced per operation, as NimBLE does on ESP.
    const CharacteristicLayout encrypted = build_characteristic_layout(
        GATT_CHR_PROP_READ | GATT_CHR_PROP_WRITE |
        GATT_CHR_F_READ_ENC | GATT_CHR_F_WRITE_ENC);
    assert(encrypted.permissions ==
           (zp::kPermissionRead | zp::kPermissionWrite |
            zp::kPermissionPrepareWrite | zp::kPermissionReadEncrypt |
            zp::kPermissionWriteEncrypt));
    assert(encrypted.attribute_count == 2);

    const CharacteristicLayout read_enc_only = build_characteristic_layout(
        GATT_CHR_PROP_READ | GATT_CHR_PROP_WRITE | GATT_CHR_F_READ_ENC);
    assert(read_enc_only.permissions ==
           (zp::kPermissionRead | zp::kPermissionWrite |
            zp::kPermissionPrepareWrite | zp::kPermissionReadEncrypt));

    const CharacteristicLayout write_enc_only = build_characteristic_layout(
        GATT_CHR_PROP_READ | GATT_CHR_PROP_WRITE | GATT_CHR_F_WRITE_ENC);
    assert(write_enc_only.permissions ==
           (zp::kPermissionRead | zp::kPermissionWrite |
            zp::kPermissionPrepareWrite | zp::kPermissionWriteEncrypt));

    // Write-without-response alone is still a write and must be protected.
    const CharacteristicLayout write_no_rsp_enc = build_characteristic_layout(
        GATT_CHR_PROP_WRITE_NO_RSP | GATT_CHR_F_WRITE_ENC);
    assert(write_no_rsp_enc.permissions ==
           (zp::kPermissionWrite | zp::kPermissionPrepareWrite |
            zp::kPermissionWriteEncrypt));

    // An ENC flag never invents an operation the properties do not allow.
    const CharacteristicLayout read_with_write_enc = build_characteristic_layout(
        GATT_CHR_PROP_READ | GATT_CHR_F_WRITE_ENC);
    assert(read_with_write_enc.permissions == zp::kPermissionRead);
    const CharacteristicLayout write_with_read_enc = build_characteristic_layout(
        GATT_CHR_PROP_WRITE | GATT_CHR_F_READ_ENC);
    assert(write_with_read_enc.permissions ==
           (zp::kPermissionWrite | zp::kPermissionPrepareWrite));
    const CharacteristicLayout notify_enc = build_characteristic_layout(
        GATT_CHR_PROP_NOTIFY | GATT_CHR_F_READ_ENC);
    assert(notify_enc.permissions == zp::kPermissionNone);
    assert(notify_enc.has_ccc);
    assert(notify_enc.attribute_count == 3);

    // Encrypted read + notify keeps its CCC and three attributes.
    const CharacteristicLayout read_notify_enc = build_characteristic_layout(
        GATT_CHR_PROP_READ | GATT_CHR_PROP_NOTIFY | GATT_CHR_F_READ_ENC);
    assert(read_notify_enc.permissions ==
           (zp::kPermissionRead | zp::kPermissionReadEncrypt));
    assert(read_notify_enc.has_ccc);
    assert(read_notify_enc.attribute_count == 3);

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

    // gatt_uuid_t stores 128-bit UUIDs least-significant byte first, just
    // like NimBLE and Zephyr. The device-log UUID must cross unchanged.
    const gatt_uuid_t device_log_uuid = GATT_UUID128(
        {0x21, 0xb7, 0x10, 0x3d, 0x30, 0xca, 0xef, 0x87,
         0x4a, 0x40, 0xb7, 0xcb, 0xbe, 0x71, 0x23, 0x9f});
    constexpr std::array<uint8_t, 16> expected_device_log_bytes = {
        0x21, 0xb7, 0x10, 0x3d, 0x30, 0xca, 0xef, 0x87,
        0x4a, 0x40, 0xb7, 0xcb, 0xbe, 0x71, 0x23, 0x9f,
    };
    assert(uuid128_value_bytes(device_log_uuid) == expected_device_log_bytes);

    return 0;
}
