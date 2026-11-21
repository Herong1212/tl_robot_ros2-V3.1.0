#ifndef TL_TELEOP__STARTUP_WAIT_H_
#define TL_TELEOP__STARTUP_WAIT_H_

#include <algorithm>
#include <chrono>
#include <future>

namespace tl_teleop
{

enum class StartupWaitResult { ready, stopped, timeout };

// Startup runs outside the executor thread. Poll briefly so SIGINT can stop
// startup without waiting for a missing service response's entire timeout.
template<typename Future, typename Continue>
StartupWaitResult wait_startup_response(Future &future, std::chrono::steady_clock::duration timeout,
                                        Continue keep_running)
{
    using Clock = std::chrono::steady_clock;
    const auto deadline = Clock::now() + timeout;
    const Clock::duration interval = std::chrono::milliseconds(50);
    while (keep_running())
    {
        const auto now = Clock::now();
        if (now >= deadline)
            return StartupWaitResult::timeout;
        if (future.wait_for(std::min(interval, deadline - now)) == std::future_status::ready)
            return keep_running() ? StartupWaitResult::ready : StartupWaitResult::stopped;
    }
    return StartupWaitResult::stopped;
}

} // namespace tl_teleop

#endif // TL_TELEOP__STARTUP_WAIT_H_
