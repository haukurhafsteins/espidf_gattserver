#pragma once

#include <cstddef>
#include <cstdint>

namespace gattserver::zephyr
{

enum class WriteMode
{
    plain,
    prepare,
    execute,
};

enum class WriteError
{
    none,
    invalid_offset,
    invalid_length,
    invalid_value,
};

struct WriteResult
{
    WriteError error = WriteError::none;
    bool callback_deferred = false;

    constexpr bool accepted() const noexcept
    {
        return error == WriteError::none;
    }
};

WriteResult assemble_write(
    uint8_t *storage,
    std::size_t capacity,
    uint16_t &stored_length,
    const void *value,
    std::size_t length,
    std::size_t offset,
    WriteMode mode) noexcept;

} // namespace gattserver::zephyr
