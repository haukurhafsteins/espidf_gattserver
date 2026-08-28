#include <cassert>
#include <cstdint>
#include <type_traits>

#include "../gatt_notify_log_limiter.hpp"

static_assert(std::is_same_v<
              GattNotifyLogLimiterFor<2>::Decision,
              GattNotifyLogLimiterFor<60>::Decision>);

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

    GattNotifyLogLimiter interleaved;
    for (int failure = 0; failure < 8; ++failure)
    {
        assert(interleaved.recordFailure(7) ==
               GattNotifyLogLimiter::Decision::individual);
        assert(interleaved.recordFailure(8) ==
               GattNotifyLogLimiter::Decision::individual);
    }
    assert(interleaved.recordFailure(7) ==
           GattNotifyLogLimiter::Decision::suppressionSummary);
    assert(interleaved.recordFailure(8) ==
           GattNotifyLogLimiter::Decision::suppressionSummary);
    assert(interleaved.recordFailure(7) ==
           GattNotifyLogLimiter::Decision::suppressed);
    assert(interleaved.recordFailure(8) ==
           GattNotifyLogLimiter::Decision::suppressed);

    interleaved.resetConnection(7);
    assert(interleaved.recordFailure(7) ==
           GattNotifyLogLimiter::Decision::individual);
    assert(interleaved.recordFailure(8) ==
           GattNotifyLogLimiter::Decision::suppressed);
    return 0;
}
