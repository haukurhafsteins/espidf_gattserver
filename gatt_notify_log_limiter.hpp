#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

enum class GattNotifyLogDecision
{
    individual,
    suppressionSummary,
    suppressed,
};

template <std::size_t MaxConnections>
class GattNotifyLogLimiterFor
{
public:
    using Decision = GattNotifyLogDecision;

    Decision recordFailure(uint16_t connectionHandle) noexcept
    {
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
