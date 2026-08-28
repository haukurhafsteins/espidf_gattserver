#include <cassert>
#include <atomic>
#include <cstdint>
#include <thread>
#include <type_traits>
#include <vector>

#include "../gatt_notify_log_limiter.hpp"

class RecordingLock
{
public:
    void lock() noexcept
    {
        assert(!held);
        held = true;
        locks.fetch_add(1, std::memory_order_relaxed);
    }

    void unlock() noexcept
    {
        assert(held);
        held = false;
        unlocks.fetch_add(1, std::memory_order_relaxed);
    }

    static void reset() noexcept
    {
        locks.store(0, std::memory_order_relaxed);
        unlocks.store(0, std::memory_order_relaxed);
    }

    inline static std::atomic<unsigned> locks{0};
    inline static std::atomic<unsigned> unlocks{0};

private:
    bool held = false;
};

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

    RecordingLock::reset();
    GattNotifyLogLimiterFor<2, RecordingLock> serialized;
    assert(serialized.recordFailure(11) ==
           GattNotifyLogLimiter::Decision::individual);
    serialized.resetConnection(11);
    assert(RecordingLock::locks.load(std::memory_order_relaxed) == 2);
    assert(RecordingLock::unlocks.load(std::memory_order_relaxed) == 2);

    GattNotifyLogLimiter concurrent;
    constexpr int threadCount = 8;
    constexpr int failuresPerThread = 64;
    std::atomic<bool> start{false};
    std::atomic<int> ready{0};
    std::atomic<int> individual{0};
    std::atomic<int> summaries{0};
    std::atomic<int> suppressed{0};
    std::vector<std::thread> threads;
    for (int thread = 0; thread < threadCount; ++thread)
    {
        threads.emplace_back([&] {
            ready.fetch_add(1, std::memory_order_release);
            while (!start.load(std::memory_order_acquire))
                std::this_thread::yield();
            for (int failure = 0; failure < failuresPerThread; ++failure)
            {
                switch (concurrent.recordFailure(17))
                {
                case GattNotifyLogLimiter::Decision::individual:
                    individual.fetch_add(1, std::memory_order_relaxed);
                    break;
                case GattNotifyLogLimiter::Decision::suppressionSummary:
                    summaries.fetch_add(1, std::memory_order_relaxed);
                    break;
                case GattNotifyLogLimiter::Decision::suppressed:
                    suppressed.fetch_add(1, std::memory_order_relaxed);
                    break;
                }
            }
        });
    }
    while (ready.load(std::memory_order_acquire) != threadCount)
        std::this_thread::yield();
    start.store(true, std::memory_order_release);
    for (auto &thread : threads)
        thread.join();
    assert(individual.load() == 8);
    assert(summaries.load() == 1);
    assert(suppressed.load() == threadCount * failuresPerThread - 9);
    return 0;
}
