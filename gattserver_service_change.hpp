#pragma once

#include <atomic>
#include <cstdint>

class GattServiceChangeState
{
public:
    bool schedule(uint16_t startHandle, uint16_t endHandle) noexcept
    {
        if (startHandle == 0 || endHandle == 0 || startHandle > endHandle)
            return false;
        if (state_.load(std::memory_order_acquire) != State::idle)
            return false;

        startHandle_.store(startHandle, std::memory_order_relaxed);
        endHandle_.store(endHandle, std::memory_order_relaxed);
        state_.store(State::scheduled, std::memory_order_release);
        return true;
    }

    bool pending(uint16_t &startHandle, uint16_t &endHandle) const noexcept
    {
        if (state_.load(std::memory_order_acquire) != State::scheduled)
            return false;
        startHandle = startHandle_.load(std::memory_order_relaxed);
        endHandle = endHandle_.load(std::memory_order_relaxed);
        return true;
    }

    void markApplied() noexcept
    {
        if (state_.load(std::memory_order_acquire) == State::scheduled)
            state_.store(State::applied, std::memory_order_release);
    }

    bool applied() const noexcept
    {
        return state_.load(std::memory_order_acquire) == State::applied;
    }

    void reset() noexcept
    {
        state_.store(State::idle, std::memory_order_release);
        startHandle_.store(0, std::memory_order_relaxed);
        endHandle_.store(0, std::memory_order_relaxed);
    }

private:
    enum class State : uint8_t
    {
        idle,
        scheduled,
        applied,
    };

    std::atomic<State> state_{State::idle};
    std::atomic<uint16_t> startHandle_{0};
    std::atomic<uint16_t> endHandle_{0};
};
