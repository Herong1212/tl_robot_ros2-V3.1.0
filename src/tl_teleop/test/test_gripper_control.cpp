#include <chrono>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "tl_teleop/gripper_control.h"

TEST(GripperControl, MissingOrInvalidTriggerDoesNotMeanRelease)
{
    for (const auto &controller : {
             nlohmann::json::object(), nlohmann::json{{"trigger", nullptr}},
             nlohmann::json{{"trigger", false}}, nlohmann::json{{"trigger", "0"}},
             nlohmann::json{{"trigger", -0.01}}, nlohmann::json{{"trigger", 1.01}},
             nlohmann::json{{"trigger", std::numeric_limits<double>::quiet_NaN()}},
             nlohmann::json{{"trigger", std::numeric_limits<double>::infinity()}}})
        EXPECT_FALSE(std::isfinite(tl_teleop::read_trigger(controller)));
    EXPECT_DOUBLE_EQ(tl_teleop::read_trigger({{"trigger", 0}}), 0.0);
    EXPECT_DOUBLE_EQ(tl_teleop::read_trigger({{"trigger", 1.0}}), 1.0);
    EXPECT_DOUBLE_EQ(tl_teleop::read_trigger({{"trigger", 0.63}}), 0.63);
}

TEST(GripperControl, StartupAndRecoveryRequireFreshReadyAndReleasedControls)
{
    using namespace std::chrono_literals;
    tl_teleop::GripperInterlock gate;
    const auto start = tl_teleop::GripperInterlock::Clock::time_point{} + 10s;
    EXPECT_FALSE(gate.allows_motion(start, true));
    gate.update(true, start);
    EXPECT_FALSE(gate.allows_motion(start, false));
    EXPECT_TRUE(gate.allows_motion(start + 10ms, true));
    EXPECT_TRUE(gate.allows_motion(start + 20ms, false));
    gate.update(false, start + 30ms);
    EXPECT_FALSE(gate.allows_motion(start + 30ms, true));
    gate.update(true, start + 40ms);
    EXPECT_FALSE(gate.allows_motion(start + 40ms, false));
    EXPECT_TRUE(gate.allows_motion(start + 50ms, true));
}

TEST(GripperControl, LostReadyHeartbeatCannotResumeWhileGripIsHeld)
{
    using namespace std::chrono_literals;
    tl_teleop::GripperInterlock gate;
    const auto start = tl_teleop::GripperInterlock::Clock::time_point{} + 10s;
    gate.update(true, start);
    ASSERT_TRUE(gate.allows_motion(start, true));
    EXPECT_TRUE(gate.allows_motion(start + 999ms, false));
    EXPECT_FALSE(gate.allows_motion(start + 1s, false));
    gate.update(true, start + 1100ms);
    EXPECT_FALSE(gate.allows_motion(start + 1100ms, false));
    EXPECT_TRUE(gate.allows_motion(start + 1200ms, true));
}
