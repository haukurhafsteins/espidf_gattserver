#include <array>
#include <cassert>
#include <cstdint>

#include "../gattserver_value_storage.hpp"

int main()
{
    std::array<uint8_t, 8> storage{};
    uint16_t length = 0;
    const std::array<uint8_t, 3> value{1, 2, 3};

    assert(gatt_store_value(
        storage.data(), storage.size(), length, value.data(), value.size()));
    assert(length == 3);
    assert(storage[0] == 1);
    assert(storage[1] == 2);
    assert(storage[2] == 3);

    assert(!gatt_store_value(
        storage.data(), storage.size(), length, value.data(), 9));
    assert(length == 3);

    assert(!gatt_store_value(
        storage.data(), storage.size(), length, nullptr, 1));
    assert(length == 3);

    assert(gatt_store_value(
        storage.data(), storage.size(), length, nullptr, 0));
    assert(length == 0);
    return 0;
}
