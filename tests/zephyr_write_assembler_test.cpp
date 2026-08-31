#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

#include "../backends/zephyr/gatt_write_assembler.hpp"

using gattserver::zephyr::WriteMode;
using gattserver::zephyr::WriteResult;
using gattserver::zephyr::assemble_write;

int main()
{
    std::array<uint8_t, 470> storage{};
    std::array<uint8_t, 470> payload{};
    for (std::size_t i = 0; i < payload.size(); ++i)
        payload[i] = static_cast<uint8_t>(i & 0xffu);

    uint16_t stored_length = 1;
    storage[0] = 0xa5;

    WriteResult result = assemble_write(
        storage.data(), storage.size(), stored_length,
        payload.data(), 244, 0, WriteMode::prepare);
    assert(result.accepted());
    assert(!result.callback_deferred);
    assert(stored_length == 1);
    assert(storage[0] == 0xa5);

    result = assemble_write(
        storage.data(), storage.size(), stored_length,
        payload.data() + 244, 226, 244, WriteMode::prepare);
    assert(result.accepted());
    assert(stored_length == 1);

    result = assemble_write(
        storage.data(), storage.size(), stored_length,
        payload.data(), 244, 0, WriteMode::execute);
    assert(result.accepted());
    assert(result.callback_deferred);
    assert(stored_length == 244);

    result = assemble_write(
        storage.data(), storage.size(), stored_length,
        payload.data() + 244, 226, 244, WriteMode::execute);
    assert(result.accepted());
    assert(result.callback_deferred);
    assert(stored_length == payload.size());
    assert(storage == payload);

    const auto before_rejected_prepare = storage;
    result = assemble_write(
        storage.data(), storage.size(), stored_length,
        payload.data(), 227, 244, WriteMode::prepare);
    assert(result.error == gattserver::zephyr::WriteError::invalid_length);
    assert(storage == before_rejected_prepare);
    assert(stored_length == payload.size());

    const std::array<uint8_t, 3> plain{7, 8, 9};
    result = assemble_write(
        storage.data(), storage.size(), stored_length,
        plain.data(), plain.size(), 0, WriteMode::plain);
    assert(result.accepted());
    assert(!result.callback_deferred);
    assert(stored_length == plain.size());
    assert(storage[0] == 7 && storage[1] == 8 && storage[2] == 9);

    result = assemble_write(
        storage.data(), storage.size(), stored_length,
        plain.data(), plain.size(), 1, WriteMode::plain);
    assert(result.error == gattserver::zephyr::WriteError::invalid_offset);

    return 0;
}
