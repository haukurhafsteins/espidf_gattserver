#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>

#if defined(ESP_PLATFORM)
#include "freertos/FreeRTOS.h"
#endif

class GattNotifyLogLock
{
public:
    void lock() noexcept
    {
#if defined(ESP_PLATFORM)
        portENTER_CRITICAL(&lock_);
#else
        lock_.lock();
#endif
    }

    void unlock() noexcept
    {
#if defined(ESP_PLATFORM)
        portEXIT_CRITICAL(&lock_);
#else
        lock_.unlock();
#endif
    }

private:
#if defined(ESP_PLATFORM)
    portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
#else
    std::mutex lock_;
#endif
};

enum class GattNotifyLogDecision
{
    individual,
    suppressionSummary,
    suppressed,
};

template <std::size_t MaxConnections, typename Lock = GattNotifyLogLock>
class GattNotifyLogLimiterFor
{
public:
    using Decision = GattNotifyLogDecision;

    Decision recordFailure(uint16_t connectionHandle) noexcept
    {
        const std::lock_guard<Lock> guard(lock_);
        ConnectionState* state = findOrCreateConnection(connectionHandle);
        if (!state)
            return Decision::suppressed;

        if (state->individualFailures < kIndividualLimit)
        {
            ++state->individualFailures;
            return Decision::individual;
        }

        if (!state->suppressionSummaryLogged)
        {
            state->suppressionSummaryLogged = true;
            return Decision::suppressionSummary;
        }

        return Decision::suppressed;
    }

    void resetConnection(uint16_t connectionHandle) noexcept
    {
        const std::lock_guard<Lock> guard(lock_);
        for (ConnectionState& state : connections_)
        {
            if (state.active && state.connectionHandle == connectionHandle)
            {
                state = {};
                return;
            }
        }
    }

private:
    struct ConnectionState
    {
        uint16_t connectionHandle = 0;
        uint8_t individualFailures = 0;
        bool suppressionSummaryLogged = false;
        bool active = false;
    };

    static constexpr uint8_t kIndividualLimit = 8;

    Lock lock_;

    ConnectionState* findOrCreateConnection(uint16_t connectionHandle) noexcept
    {
        ConnectionState* available = nullptr;
        for (ConnectionState& state : connections_)
        {
            if (state.active && state.connectionHandle == connectionHandle)
                return &state;
            if (!state.active && !available)
                available = &state;
        }

        if (available)
        {
            available->connectionHandle = connectionHandle;
            available->active = true;
        }
        return available;
    }

    std::array<ConnectionState, MaxConnections> connections_ = {};
};

using GattNotifyLogLimiter = GattNotifyLogLimiterFor<2>;
