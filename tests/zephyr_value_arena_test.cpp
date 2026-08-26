#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

#include "../backends/zephyr/gatt_value_arena.hpp"

int main()
{
    alignas(std::max_align_t) std::array<uint8_t, 64> bytes{};
    gattserver::zephyr::ValueArena arena(bytes.data(), bytes.size());

    void *first = arena.allocate(3);
    void *second = arena.allocate(sizeof(uint32_t));
    assert(first != nullptr);
    assert(second != nullptr);
    assert(reinterpret_cast<std::uintptr_t>(first) % alignof(std::max_align_t) == 0);
    assert(reinterpret_cast<std::uintptr_t>(second) % alignof(std::max_align_t) == 0);
    assert(arena.used() >= 3 + sizeof(uint32_t));

    assert(arena.allocate(128) == nullptr);
    const std::size_t used_before_failure = arena.used();
    assert(arena.allocate(128) == nullptr);
    assert(arena.used() == used_before_failure);

    arena.reset();
    assert(arena.used() == 0);
    assert(arena.allocate(bytes.size()) == bytes.data());
    assert(arena.allocate(1) == nullptr);
    return 0;
}
