#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

inline bool gatt_store_value(
    uint8_t *storage,
    std::size_t capacity,
    uint16_t &storedLength,
    const void *value,
    std::size_t length) noexcept
{
    if (storage == nullptr || length > capacity ||
        length > std::numeric_limits<uint16_t>::max() ||
        (value == nullptr && length != 0))
    {
        return false;
    }
    if (length != 0)
        std::memcpy(storage, value, length);
    storedLength = static_cast<uint16_t>(length);
    return true;
}
