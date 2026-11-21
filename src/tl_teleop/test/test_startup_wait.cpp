#include <atomic>
#include <chrono>
#include <future>
#include <thread>

#include <gtest/gtest.h>

#include "tl_teleop/startup_wait.h"

TEST(StartupWait, MissingServiceResponseIsInterruptedPromptly)
{
    using namespace std::chrono_literals;
    std::promise<int> promise;
    auto future = promise.get_future();
    std::atomic<bool> running{true};
    const auto started = std::chrono::steady_clock::now();
    std::thread stop([&running]() { std::this_thread::sleep_for(20ms); running = false; });
    const auto result = tl_teleop::wait_startup_response(future, 5s, [&running]() { return running.load(); });
    stop.join();
    EXPECT_EQ(result, tl_teleop::StartupWaitResult::stopped);
    EXPECT_LT(std::chrono::steady_clock::now() - started, 500ms);
}

TEST(StartupWait, CompletedResponseIsReturnedOnlyWhileRunning)
{
    using namespace std::chrono_literals;
    std::promise<int> promise;
    auto future = promise.get_future();
    promise.set_value(7);
    EXPECT_EQ(tl_teleop::wait_startup_response(future, 5s, []() { return true; }),
              tl_teleop::StartupWaitResult::ready);
    EXPECT_EQ(tl_teleop::wait_startup_response(future, 5s, []() { return false; }),
              tl_teleop::StartupWaitResult::stopped);
    EXPECT_EQ(future.get(), 7);
}

TEST(StartupWait, MissingResponseReportsItsDeadline)
{
    using namespace std::chrono_literals;
    std::promise<int> promise;
    auto future = promise.get_future();
    const auto started = std::chrono::steady_clock::now();
    EXPECT_EQ(tl_teleop::wait_startup_response(future, 80ms, []() { return true; }),
              tl_teleop::StartupWaitResult::timeout);
    EXPECT_GE(std::chrono::steady_clock::now() - started, 80ms);
    EXPECT_LT(std::chrono::steady_clock::now() - started, 500ms);
}
