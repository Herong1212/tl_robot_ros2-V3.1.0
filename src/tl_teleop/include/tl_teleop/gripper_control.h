#ifndef TL_TELEOP__GRIPPER_CONTROL_H_
#define TL_TELEOP__GRIPPER_CONTROL_H_

#include <chrono>
#include <cmath>
#include <limits>

#include <nlohmann/json.hpp>

namespace tl_teleop
{

// An absent/invalid trigger is not a released trigger: it must never open a claw.
inline double read_trigger(const nlohmann::json &controller)
{
    const auto field = controller.find("trigger");
    if (field == controller.end() || !field->is_number())
        return std::numeric_limits<double>::quiet_NaN();
    const double value = field->get<double>();
    return std::isfinite(value) && value >= 0.0 && value <= 1.0 ?
        value : std::numeric_limits<double>::quiet_NaN();
}

// Call under the owner's mutex; the ROS callback and control loop share this state.
class GripperInterlock
{
public:
    using Clock = std::chrono::steady_clock;

    void update(bool ready, Clock::time_point received)
    {
        ready_ = ready;
        received_ = received;
        if (!ready)
            released_since_ready_ = false;
    }

    bool feedback_ready(Clock::time_point now) const
    {
        const auto age = now - received_;
        return ready_ && age >= Clock::duration::zero() && age < std::chrono::seconds(1);
    }

    bool allows_motion(Clock::time_point now, bool controls_released)
    {
        if (!feedback_ready(now))
        {
            released_since_ready_ = false;
            return false;
        }
        if (controls_released)
            released_since_ready_ = true;
        return released_since_ready_;
    }

private:
    bool ready_{false};
    bool released_since_ready_{false};
    Clock::time_point received_{};
};

} // namespace tl_teleop

#endif // TL_TELEOP__GRIPPER_CONTROL_H_
