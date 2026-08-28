#pragma once

#include <cstdint>

class GattNotifyLogLimiter
{
public:
    enum class Decision
    {
        individual,
        suppressionSummary,
        suppressed,
    };

    Decision recordFailure(uint16_t connectionHandle) noexcept
    {
        if (connectionHandle_ != connectionHandle)
        {
            connectionHandle_ = connectionHandle;
            individualFailures_ = 0;
            suppressionSummaryLogged_ = false;
        }

        if (individualFailures_ < kIndividualLimit)
        {
            ++individualFailures_;
            return Decision::individual;
        }

        if (!suppressionSummaryLogged_)
        {
            suppressionSummaryLogged_ = true;
            return Decision::suppressionSummary;
        }

        return Decision::suppressed;
    }

    void resetConnection(uint16_t connectionHandle) noexcept
    {
        if (connectionHandle_ == connectionHandle)
        {
            connectionHandle_ = kNoConnection;
            individualFailures_ = 0;
            suppressionSummaryLogged_ = false;
        }
    }

private:
    static constexpr uint8_t kIndividualLimit = 8;
    static constexpr uint16_t kNoConnection = UINT16_MAX;

    uint16_t connectionHandle_ = kNoConnection;
    uint8_t individualFailures_ = 0;
    bool suppressionSummaryLogged_ = false;
};
