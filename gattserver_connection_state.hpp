#pragma once

#include <atomic>
#include <cstdint>

class GattConnectionState
{
public:
    static constexpr uint16_t NO_CONNECTION = 0xffff;
    static constexpr uint16_t DEFAULT_ATT_MTU = 23;

    void connected(uint16_t handle, bool encrypted) noexcept
    {
        mtu_.store(DEFAULT_ATT_MTU, std::memory_order_relaxed);
        encrypted_.store(encrypted, std::memory_order_relaxed);
        connectionHandle_.store(handle, std::memory_order_release);
    }

    void mtuChanged(uint16_t handle, uint16_t mtu) noexcept
    {
        if (connectionHandle_.load(std::memory_order_acquire) == handle)
            mtu_.store(mtu, std::memory_order_release);
    }

    void encryptionChanged(uint16_t handle, bool encrypted) noexcept
    {
        if (connectionHandle_.load(std::memory_order_acquire) == handle)
            encrypted_.store(encrypted, std::memory_order_release);
    }

    void disconnected(uint16_t handle) noexcept
    {
        if (connectionHandle_.load(std::memory_order_acquire) == handle)
            reset();
    }

    void reset() noexcept
    {
        connectionHandle_.store(NO_CONNECTION, std::memory_order_release);
        mtu_.store(DEFAULT_ATT_MTU, std::memory_order_release);
        encrypted_.store(false, std::memory_order_release);
    }

    uint16_t connectionHandle() const noexcept
    {
        return connectionHandle_.load(std::memory_order_acquire);
    }

    uint16_t mtu() const noexcept
    {
        return mtu_.load(std::memory_order_acquire);
    }

    bool encrypted() const noexcept
    {
        return encrypted_.load(std::memory_order_acquire);
    }

private:
    std::atomic<uint16_t> connectionHandle_{NO_CONNECTION};
    std::atomic<uint16_t> mtu_{DEFAULT_ATT_MTU};
    std::atomic<bool> encrypted_{false};
};
