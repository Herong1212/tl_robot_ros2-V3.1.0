#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "tl_teleop/joint_trajectory.h"

namespace
{
constexpr double kRad = 3.14159265358979323846 / 180.0;
constexpr double kVelocity = 80.0 * kRad;
constexpr double kAcceleration = 3000.0 * kRad;
constexpr double kJerk = 50000.0 * kRad;

tl_teleop::JointTrajectory make_trajectory(size_t joints, double dt)
{
    return tl_teleop::JointTrajectory(joints, dt, std::vector<double>(7, kVelocity),
                                     std::vector<double>(7, kAcceleration), std::vector<double>(7, kJerk));
}

void check_derivatives(const tl_teleop::JointTrajectory &trajectory,
                       const std::vector<double> &previous_acceleration, double dt)
{
    for (size_t axis = 0; axis < trajectory.position().size(); ++axis)
    {
        EXPECT_LE(std::abs(trajectory.velocity()[axis]), kVelocity + 1e-8);
        EXPECT_LE(std::abs(trajectory.acceleration()[axis]), kAcceleration + 1e-8);
        EXPECT_LE(std::abs(trajectory.acceleration()[axis] - previous_acceleration[axis]) / dt, kJerk + 1e-7);
    }
}

class JointTrajectory : public ::testing::TestWithParam<int>
{
protected:
    double dt() const { return 1.0 / GetParam(); }
    int steps_for(double seconds) const { return static_cast<int>(std::lround(seconds * GetParam())); }
};
} // namespace

TEST_P(JointTrajectory, SmallStaticTargetSettlesWithoutAlternatingAcrossTarget)
{
    for (size_t joints : {6, 7})
    {
        auto trajectory = make_trajectory(joints, dt());
        const std::vector<double> target(joints, 0.01 * kRad);
        double previous = 0.0;
        for (int step = 0; step < steps_for(1.0); ++step)
        {
            auto acceleration = trajectory.acceleration();
            ASSERT_TRUE(trajectory.step(target));
            check_derivatives(trajectory, acceleration, dt());
            EXPECT_GE(trajectory.position()[0], previous - 1e-12);
            EXPECT_LE(trajectory.position()[0], target[0] + 1e-12);
            previous = trajectory.position()[0];
        }
        EXPECT_NEAR(trajectory.position()[0], target[0], 1e-12);
        EXPECT_NEAR(trajectory.velocity()[0], 0.0, 1e-12);
    }
}

TEST_P(JointTrajectory, ReversalsAndStreamingTargetsRespectAllDerivativeLimits)
{
    auto trajectory = make_trajectory(6, dt());
    std::vector<double> target(6, 0.0);
    for (int step = 0; step < steps_for(10.0); ++step)
    {
        const double time = step * dt();
        target[0] = (time < 0.15 ? 10.0 : -10.0) * kRad;
        // Preserve the same 30 Hz target motion at both control frequencies.
        target[1] = 0.02 * std::sin(time * 60.0 * 3.14159265358979323846) * kRad;
        target[2] = 5.0 * std::sin(time * 2.0) * kRad;
        auto acceleration = trajectory.acceleration();
        ASSERT_TRUE(trajectory.step(target));
        check_derivatives(trajectory, acceleration, dt());
    }
    EXPECT_NEAR(trajectory.position()[0], target[0], 1e-10);
}

TEST_P(JointTrajectory, InvalidTargetDoesNotChangeStateAndBrakePreservesContinuity)
{
    auto trajectory = make_trajectory(7, dt());
    std::vector<double> target(7, 1.0);
    for (int step = 0; step < steps_for(0.1); ++step)
        ASSERT_TRUE(trajectory.step(target));
    const auto position = trajectory.position();
    EXPECT_FALSE(trajectory.step(std::vector<double>(6, 0.0)));
    target[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(trajectory.step(target));
    EXPECT_EQ(trajectory.position(), position);
    for (int step = 0; step < steps_for(1.0); ++step)
    {
        auto acceleration = trajectory.acceleration();
        ASSERT_TRUE(trajectory.brake());
        check_derivatives(trajectory, acceleration, dt());
    }
    for (double velocity : trajectory.velocity())
        EXPECT_NEAR(velocity, 0.0, 1e-12);
    auto acceleration = trajectory.acceleration();
    ASSERT_TRUE(trajectory.step(trajectory.position()));
    check_derivatives(trajectory, acceleration, dt());
}

TEST_P(JointTrajectory, ReengagementStartsAtFeedback)
{
    auto trajectory = make_trajectory(6, dt());
    for (int step = 0; step < steps_for(0.2); ++step)
        ASSERT_TRUE(trajectory.step(std::vector<double>(6, 0.8)));
    const std::vector<double> measured(6, 0.15);
    ASSERT_TRUE(trajectory.reset(measured));
    EXPECT_EQ(trajectory.position(), measured);
    ASSERT_TRUE(trajectory.step(measured));
    EXPECT_EQ(trajectory.position(), measured);
    for (double velocity : trajectory.velocity())
        EXPECT_DOUBLE_EQ(velocity, 0.0);
}

TEST_P(JointTrajectory, ConstantTargetsAtConfiguredJointEndpointsStayInsideLimits)
{
    const std::vector<double> lower_deg{-179.0, -135.0, -58.0, -179.0, -220.0, -179.0};
    const std::vector<double> upper_deg{179.0, 135.0, 237.0, 179.0, 38.0, 179.0};
    for (double direction : {-1.0, 1.0})
    {
        auto trajectory = make_trajectory(6, dt());
        std::vector<double> initial(6), target(6);
        for (size_t axis = 0; axis < target.size(); ++axis)
        {
            target[axis] = (direction > 0.0 ? upper_deg[axis] : lower_deg[axis]) * kRad;
            initial[axis] = target[axis] - direction * 0.5 * kRad;
        }
        ASSERT_TRUE(trajectory.reset(initial));
        for (int step = 0; step < steps_for(1.0); ++step)
        {
            auto acceleration = trajectory.acceleration();
            ASSERT_TRUE(trajectory.step(target));
            check_derivatives(trajectory, acceleration, dt());
            for (size_t axis = 0; axis < target.size(); ++axis)
            {
                EXPECT_GE(trajectory.position()[axis], lower_deg[axis] * kRad - 1e-12);
                EXPECT_LE(trajectory.position()[axis], upper_deg[axis] * kRad + 1e-12);
            }
        }
        for (size_t axis = 0; axis < target.size(); ++axis)
            EXPECT_NEAR(trajectory.position()[axis], target[axis], 1e-12);
    }
}

INSTANTIATE_TEST_SUITE_P(ControlFrequency, JointTrajectory, ::testing::Values(100, 250),
                        [](const ::testing::TestParamInfo<int> &info)
                        { return "Hz" + std::to_string(info.param); });
