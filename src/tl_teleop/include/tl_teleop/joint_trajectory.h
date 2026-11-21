#ifndef TL_TELEOP__JOINT_TRAJECTORY_H_
#define TL_TELEOP__JOINT_TRAJECTORY_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include <ruckig/ruckig.hpp>

namespace tl_teleop
{

// Each update advances the same position/velocity/acceleration state by one
// control period. Replacing a target therefore preserves velocity and acceleration.
class JointTrajectory
{
public:
    JointTrajectory(size_t joints, double period, const std::vector<double> &vmax,
                    const std::vector<double> &amax, const std::vector<double> &jmax)
        : joints_(joints), generator_(joints, period), input_(joints), output_(joints)
    {
        if (joints == 0 || !std::isfinite(period) || period <= 0.0)
            throw std::invalid_argument("Invalid trajectory size or control period");
        set_limit(vmax, input_.max_velocity);
        set_limit(amax, input_.max_acceleration);
        set_limit(jmax, input_.max_jerk);
        input_.synchronization = ruckig::Synchronization::None;
        input_.target_velocity.assign(joints, 0.0);
        input_.target_acceleration.assign(joints, 0.0);
        reset(std::vector<double>(joints, 0.0));
    }

    bool reset(const std::vector<double> &position)
    {
        if (!valid_position(position))
            return false;
        input_.current_position = position;
        input_.current_velocity.assign(joints_, 0.0);
        input_.current_acceleration.assign(joints_, 0.0);
        input_.target_position = position;
        generator_.reset();
        return true;
    }

    bool step(const std::vector<double> &target)
    {
        if (!valid_position(target))
            return false;
        input_.control_interface = ruckig::ControlInterface::Position;
        input_.target_position = target;
        return advance();
    }

    // A lost/rejected IK result calls this until a valid target arrives.
    // Preserve the current velocity and acceleration while braking to rest.
    bool brake()
    {
        input_.control_interface = ruckig::ControlInterface::Velocity;
        return advance();
    }

    const std::vector<double> &position() const { return input_.current_position; }
    const std::vector<double> &velocity() const { return input_.current_velocity; }
    const std::vector<double> &acceleration() const { return input_.current_acceleration; }

private:
    const size_t joints_;
    ruckig::Ruckig<0> generator_;
    ruckig::InputParameter<0> input_;
    ruckig::OutputParameter<0> output_;

    bool valid_position(const std::vector<double> &position) const
    {
        return position.size() == joints_ &&
               std::all_of(position.begin(), position.end(), [](double value) { return std::isfinite(value); });
    }

    void set_limit(const std::vector<double> &source, std::vector<double> &destination)
    {
        // ServoJ carries seven SDK axes even for a six-axis manipulator.
        if (source.size() < joints_ ||
            !std::all_of(source.begin(), source.begin() + joints_,
                         [](double value) { return std::isfinite(value) && value > 0.0; }))
            throw std::invalid_argument("Trajectory limits must be finite and positive for every joint");
        destination.assign(source.begin(), source.begin() + joints_);
    }

    bool advance()
    {
        const auto result = generator_.update(input_, output_);
        if (result != ruckig::Result::Working && result != ruckig::Result::Finished)
            return false;
        output_.pass_to_input(input_);
        return true;
    }
};

} // namespace tl_teleop

#endif // TL_TELEOP__JOINT_TRAJECTORY_H_
