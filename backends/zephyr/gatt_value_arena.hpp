#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace gattserver::zephyr
{

class ValueArena
{
public:
    ValueArena(uint8_t *storage, std::size_t capacity) noexcept
        : storage_(storage), capacity_(capacity)
    {
    }

    void *allocate(std::size_t size) noexcept
    {
        if (storage_ == nullptr || size == 0)
            return nullptr;

        constexpr std::size_t alignment = alignof(std::max_align_t);
        const std::uintptr_t current =
            reinterpret_cast<std::uintptr_t>(storage_ + used_);
        const std::uintptr_t aligned =
            (current + alignment - 1) & ~(alignment - 1);
        const std::size_t offset =
            static_cast<std::size_t>(aligned -
                                     reinterpret_cast<std::uintptr_t>(storage_));
        if (offset > capacity_ || size > capacity_ - offset)
            return nullptr;

        used_ = offset + size;
        void *allocation = storage_ + offset;
        std::memset(allocation, 0, size);
        return allocation;
    }

    void reset() noexcept
    {
        used_ = 0;
    }

    std::size_t used() const noexcept
    {
        return used_;
    }

    std::size_t capacity() const noexcept
    {
        return capacity_;
    }

private:
    uint8_t *storage_ = nullptr;
    std::size_t capacity_ = 0;
    std::size_t used_ = 0;
};

} // namespace gattserver::zephyr
