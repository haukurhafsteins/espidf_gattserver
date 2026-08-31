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

    // NCS 2.9.1 validates each prepare fragment, then invokes the attribute
    // once with the reassembled payload and EXECUTE. Pin two complete 504-byte
    // transactions around an unrelated one-byte characteristic write so no
    // value length or bytes can leak between handles or transactions.
    std::array<uint8_t, 504> long_storage{};
    std::array<uint8_t, 504> long_payload{};
    for (std::size_t i = 0; i < long_payload.size(); ++i)
        long_payload[i] = static_cast<uint8_t>((i * 37u + 11u) & 0xffu);
    std::array<uint8_t, 1> short_storage{};
    const std::array<uint8_t, 1> short_payload{0x5a};
    uint16_t long_length = 0;
    uint16_t short_length = 0;

    const auto run_long_write = [&] {
        constexpr std::array<std::size_t, 3> offsets{0, 242, 484};
        constexpr std::array<std::size_t, 3> lengths{242, 242, 20};
        for (std::size_t i = 0; i < offsets.size(); ++i)
        {
            const auto prepared = assemble_write(
                long_storage.data(), long_storage.size(), long_length,
                long_payload.data() + offsets[i], lengths[i], offsets[i],
                WriteMode::prepare);
            assert(prepared.accepted());
            assert(!prepared.callback_deferred);
        }

        // This is the single reassembled callback made by Zephyr's
        // att_exec_write_rsp(), not three application-level fragments.
        const auto executed = assemble_write(
            long_storage.data(), long_storage.size(), long_length,
            long_payload.data(), long_payload.size(), 0, WriteMode::execute);
        assert(executed.accepted());
        assert(executed.callback_deferred);
        assert(long_length == long_payload.size());
        assert(long_storage == long_payload);
    };

    run_long_write();
    const auto short_result = assemble_write(
        short_storage.data(), short_storage.size(), short_length,
        short_payload.data(), short_payload.size(), 0, WriteMode::plain);
    assert(short_result.accepted());
    assert(short_length == short_payload.size());
    assert(short_storage == short_payload);
    long_storage.fill(0xcc);
    run_long_write();

    return 0;
}
