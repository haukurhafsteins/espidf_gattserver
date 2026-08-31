#include <cassert>
#include <cerrno>
#include <vector>

#include "gatt_notify_retry.hpp"

int main()
{
    {
        const std::vector<int> results{-ENOMEM, -EAGAIN, 0};
        std::vector<unsigned> sleeps;
        std::size_t attempt = 0;

        const int result = gattserver::zephyr::notify_retry::transmit(
            [&] { return results.at(attempt++); },
            [&](unsigned ms) { sleeps.push_back(ms); });

        assert(result == 0);
        assert(attempt == 3u);
        assert(sleeps == (std::vector<unsigned>{5, 10}));
    }

    {
        unsigned attempts = 0;
        unsigned sleeps = 0;

        const int result = gattserver::zephyr::notify_retry::transmit(
            [&] {
                ++attempts;
                return -EINVAL;
            },
            [&](unsigned) { ++sleeps; });

        assert(result == -EINVAL);
        assert(attempts == 1u);
        assert(sleeps == 0u);
    }

    {
        unsigned attempts = 0;
        std::vector<unsigned> sleeps;

        const int result = gattserver::zephyr::notify_retry::transmit(
            [&] {
                ++attempts;
                return -ENOMEM;
            },
            [&](unsigned ms) { sleeps.push_back(ms); });

        assert(result == -ENOMEM);
        assert(attempts == 3u);
        assert(sleeps == (std::vector<unsigned>{5, 10}));
    }
    return 0;
}
