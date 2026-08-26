#include "gatt_write_assembler.hpp"

#include <algorithm>
#include <cstring>
#include <limits>

namespace gattserver::zephyr
{

WriteResult assemble_write(
    uint8_t *storage,
    std::size_t capacity,
    uint16_t &stored_length,
    const void *value,
    std::size_t length,
    std::size_t offset,
    WriteMode mode) noexcept
{
    if (mode == WriteMode::plain && offset != 0)
        return {WriteError::invalid_offset, false};
    if (offset > capacity)
        return {WriteError::invalid_offset, false};
    if (length > capacity - offset ||
        offset + length > std::numeric_limits<uint16_t>::max())
    {
        return {WriteError::invalid_length, false};
    }
    if (storage == nullptr || (value == nullptr && length != 0))
        return {WriteError::invalid_value, false};

    if (mode == WriteMode::prepare)
        return {};

    if (length != 0)
    {
        std::memcpy(storage + offset, value, length);
    }

    const std::size_t end = offset + length;
    if (mode == WriteMode::plain || offset == 0)
    {
        stored_length = static_cast<uint16_t>(end);
    }
    else
    {
        stored_length = static_cast<uint16_t>(
            std::max<std::size_t>(stored_length, end));
    }

    return {WriteError::none, mode == WriteMode::execute};
}

} // namespace gattserver::zephyr
