#include <cassert>
#include <cstdint>

#include "../gatt_notify_log_limiter.hpp"

int main()
{
    GattNotifyLogLimiter limiter;

    for (int failure = 0; failure < 8; ++failure)
    {
        assert(limiter.recordFailure(7) ==
               GattNotifyLogLimiter::Decision::individual);
    }
    assert(limiter.recordFailure(7) ==
           GattNotifyLogLimiter::Decision::suppressionSummary);
    assert(limiter.recordFailure(7) ==
           GattNotifyLogLimiter::Decision::suppressed);

    limiter.resetConnection(7);
    assert(limiter.recordFailure(7) ==
           GattNotifyLogLimiter::Decision::individual);

    assert(limiter.recordFailure(8) ==
           GattNotifyLogLimiter::Decision::individual);
    return 0;
}
