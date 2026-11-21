#ifndef TL_TELEOP__TL_TELEOP_H_
#define TL_TELEOP__TL_TELEOP_H_

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "PXREARobotSDK.h"
#include "tl_teleop/gripper_control.h"
#include "tl_teleop/joint_trajectory.h"

#include <tl_ros2_interface/msg/cartesian_pose.hpp>
#include <tl_ros2_interface/srv/coord_transform.hpp>
#include <tl_ros2_interface/srv/open_servo_j.hpp>
#include <tl_ros2_interface/srv/set_current_mode.hpp>
#include <tl_ros2_interface/srv/set_speed.hpp>

struct VRState
{
    std::array<double, 7> pose_{0.0};
    double grip_{0.0};
    double trigger_{std::numeric_limits<double>::quiet_NaN()};
    bool a_button_{false};
    std::mutex mutex_;
    std::chrono::steady_clock::time_point received_{};

    void update(const std::array<double, 7> &pose, double grip, bool a_button,
                double trigger = std::numeric_limits<double>::quiet_NaN())
    {
        std::lock_guard<std::mutex> lock(mutex_);
        received_ = std::chrono::steady_clock::now();
        pose_ = pose;
        grip_ = grip;
        trigger_ = trigger;
        a_button_ = a_button;
    }

    void get(std::array<double, 7> &pose, double &grip, bool &a_button, double &trigger,
             std::chrono::steady_clock::time_point &received)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pose = pose_;
        grip = grip_;
        trigger = trigger_;
        received = received_;
        a_button = a_button_;
    }
};

class TL_Teleop : public rclcpp::Node
{
public:
    TL_Teleop();
    ~TL_Teleop();

    // C-linkage callback; must be static for PXREA SDK
    static void on_pxrea_client_cb(void *context, PXREAClientCallbackType type, int status, void *userData);

    void run_control_loop();
    void shutdown_teleop(rclcpp::Executor &exec);
    void stop()
    {
        running_ = false;
    }
    bool is_running() const { return running_; }
    bool init_ok() const
    {
        return !init_failed_;
    }

private:
    VRState vr_state_;
    // PXREA 状态帧率统计（仅在 PXREA 回调线程访问，无需加锁）
    int pxrea_frame_count_{0};
    std::atomic<uint64_t> valid_vr_frames_{0};
    std::chrono::steady_clock::time_point pxrea_rate_window_start_{std::chrono::steady_clock::now()};

    std::vector<double> last_joints_;
    std::unique_ptr<tl_teleop::JointTrajectory> trajectory_;

    int arm_joints_{7};
    // * 手柄位移与末端位移的比例，无单位
    // 计算方式：末端目标位移 = 映射到机械臂坐标系的手柄位移 × pos_scale
    // 例如，设置为 0.5，表示手柄移动 10 cm，机械臂末端目标移动 5 cm
    double pos_scale_{0.5};
    // * 手柄位置死区，单位：米
    // 程序计算手柄相对“本次握紧时的位置”的偏移，分别检查 X、Y、Z 三个方向。某一方向的偏移绝对值小于 5 mm 时为零，超出时减去死区宽度。
    // 例如，手柄相对基准偏移 3 mm → 按 0 处理；手柄相对基准偏移 6 mm → 按 1 mm 处理，再乘位置比例
    double pos_deadzone_{0.005};
    // * 相对基准的各轴目标偏移上限，单位：米
    // 手柄位移经过坐标映射和缩放后，机械臂三个方向的目标偏移分别被限制在：[-0.3, 0.3] m
    // 例如，握紧时末端 X 坐标为 0.4 m，那么本次握持期间，计算出的 X 目标被限制在 0.1～0.7 m
    double max_pos_delta_m_{0.3};
    // * 触发额外位置缩放的关节角阈值
    double singular_angle_{2.792526803190927}; // 160°
    // * 超过上述角度阈值后的额外位置比例
    double singular_scale_{0.2};

    // * 相邻有效逆解的跳变阈值，内部单位 rad
    // 检查相邻逆解的连续性，实际发出的目标再经过速度、加速度和 jerk 约束。
    // 例如，上次 J2 的目标为 10°，新目标为 45°，差值为 35°，这组目标就不会发布。
    double joint_jump_threshold_{0.5235987755982988};
    std::vector<std::pair<double, double>> joint_limits_;
    // * ServoJ 最大关节速度约束，内部单位 rad/s
    // 节点开启 ServoJ 时，用这个值为各关节设置速度上限。调大后，允许更快的关节运动；调小后，允许的速度降低，快速移动手柄时可能产生更明显的跟随滞后
    std::vector<double> servo_vmax_;
    // * ServoJ 最大关节加速度约束，内部单位 rad/s²
    // 加速度描述“关节速度变化得有多快”。这个参数约束加速和减速的快慢。调大后，允许速度更快地变化；调小后，加减速过程通常更缓慢，也可能增加跟随滞
    std::vector<double> servo_amax_;
    // * ServoJ 最大关节加加速度约束，内部单位 rad/s³
    // 加加速度也叫 jerk，描述“加速度变化得有多快”
    std::vector<double> servo_jmax_;
    // * 控制器全局运行速度百分比，25.0 表示向控制器设置 25% 的全局运行速度
    double servo_speed_{25.0};

    using IKClient = rclcpp::Client<tl_ros2_interface::srv::CoordTransform>;
    std::optional<IKClient::FutureAndRequestId> pending_ik_;
    std::chrono::steady_clock::time_point ik_started_;
    std::vector<double> last_ik_result_;
    std::vector<double> ik_reference_;
    std::vector<double> accepted_ik_pose_, requested_ik_pose_;
    double ik_step_scale_{1.0};
    double max_linear_speed_{0.25}, max_angular_speed_{1.5};
    std::vector<double> next_ik_pose(const std::vector<double> &goal) const;
    void cancel_ik();
    void update_ik(const std::vector<double> &pose);

    // ROS2 service clients
    rclcpp::Client<tl_ros2_interface::srv::SetSpeed>::SharedPtr set_speed_client_;
    rclcpp::Client<tl_ros2_interface::srv::SetCurrentMode>::SharedPtr set_current_mode_client_;
    rclcpp::Client<tl_ros2_interface::srv::OpenServoJ>::SharedPtr open_servoj_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr close_servoj_client_;
    rclcpp::Client<tl_ros2_interface::srv::CoordTransform>::SharedPtr coord_transform_client_;

    // Right controller snapshot: x, y, z, qx, qy, qz, qw, grip, A.
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vr_input_pub_;

    // The serial worker is a separate process, so its I/O cannot block ServoJ/IK.
    bool gripper_enabled_{false};
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr gripper_input_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_ready_sub_;
    std::mutex gripper_mutex_;
    tl_teleop::GripperInterlock gripper_interlock_;
    void publish_gripper_input(double trigger, bool enabled,
                               std::chrono::steady_clock::time_point received);

    // Topic publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr servoj_pos_pub_;
    std_msgs::msg::Float64MultiArray msg_;

    // Topic subscriber + cache
    rclcpp::Subscription<tl_ros2_interface::msg::CartesianPose>::SharedPtr tcp_pose_sub_;
    tl_ros2_interface::msg::CartesianPose latest_tcp_pose_;
    std::mutex tcp_pose_mutex_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    std::vector<double> measured_joints_;
    std::chrono::steady_clock::time_point tcp_received_{}, joints_received_{};

    std::atomic<bool> running_{true};
    std::atomic<bool> init_failed_{false};
    bool servo_close_needed_{false}; // Only read by main after the control thread joins.

    bool init_servo();
    static std::array<double, 4> rpy2quat(double rx, double ry, double rz);
    std::vector<double> clamp_joints(const std::vector<double> &joints);
    bool joints_safe(const std::vector<double> &new_joints);
    bool servoJ_send(const std::vector<double> &joint_target);

    static std::array<double, 4> quat_multiply(const std::array<double, 4> &q1, const std::array<double, 4> &q2);
    static std::array<double, 4> quat_inverse(const std::array<double, 4> &q);
    static std::array<double, 3> quat2rpy(const std::array<double, 4> &q);

    void control_loop();

    static std::array<double, 7> parse_pose_str(const std::string &s);
};

#endif // TL_TELEOP__TL_TELEOP_H_
