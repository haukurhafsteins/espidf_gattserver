#pragma once

#include <cerrno>

namespace gattserver::zephyr::notify_retry
{

constexpr unsigned MAX_ATTEMPTS = 3;
constexpr unsigned FIRST_BACKOFF_MS = 5;

constexpr bool transient(int result) noexcept
{
    return result == -ENOMEM || result == -EAGAIN;
}

template <typename Sender, typename Sleeper>
int transmit(Sender &&send, Sleeper &&sleep)
{
    int result = 0;
    for (unsigned attempt = 0; attempt < MAX_ATTEMPTS; ++attempt)
    {
        result = send();
        if (result == 0 || !transient(result))
            return result;
        if (attempt + 1 < MAX_ATTEMPTS)
            sleep(FIRST_BACKOFF_MS << attempt);
    }
    return result;
}

} // namespace gattserver::zephyr::notify_retry
