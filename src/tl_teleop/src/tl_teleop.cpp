#include "tl_teleop/tl_teleop.h"
#include "tl_teleop/single_instance.h"
#include "tl_teleop/startup_wait.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>

using json = nlohmann::json;

static constexpr double GRIP_THRESHOLD = 0.9;
static constexpr double GRIP_RELEASE = 0.8;
static constexpr auto STATE_TIMEOUT = std::chrono::milliseconds(200);
static constexpr int CONTROL_RATE_HZ = 250;
// Share the 4 ms period between ServoJ pacing, Cartesian slew and Ruckig.
static constexpr double CONTROL_PERIOD_S = 1.0 / CONTROL_RATE_HZ;
static constexpr double kDegToRad = M_PI / 180.0;
struct TeleopTiming
{
    using Clock = std::chrono::steady_clock;

    struct Slot
    {
        Clock::duration min{Clock::duration::max()};
        Clock::duration max{Clock::duration::zero()};
        Clock::duration total{0};
        int count{0};

        void record(Clock::duration d)
        {
            min = std::min(min, d);
            max = std::max(max, d);
            total += d;
            ++count;
        }
        void reset()
        {
            *this = Slot{};
        }

        long long avg_us() const
        {
            return count > 0 ? std::chrono::duration_cast<std::chrono::microseconds>(total).count() / count : 0;
        }
        long long min_us() const
        {
            return min == Clock::duration::max() ? 0 : std::chrono::duration_cast<std::chrono::microseconds>(min).count();
        }
        long long max_us() const
        {
            return max == Clock::duration::zero() ? 0 : std::chrono::duration_cast<std::chrono::microseconds>(max).count();
        }
    };

    enum Idx : int
    {
        VR_GET = 0,
        TCP_POSE,
        RPY2QUAT,
        ORIENT,
        IK,
        INTERP,
        SERVO,
        TOTAL,
        SECTION_COUNT
    };

    static constexpr int REPORT_INTERVAL = CONTROL_RATE_HZ;
    static constexpr const char *NAMES[] = {"vr_get", "tcp_pose", "rpy2quat", "orient", "ik", "interp", "servo", "total"};

    Slot slots[SECTION_COUNT]{};
    int counter{0};
    Clock::time_point window_start_{Clock::now()};

    void reset_all()
    {
        for (auto &s : slots)
        {
            s.reset();
        }
        counter = 0;
        window_start_ = Clock::now();
    }

    void tick(rclcpp::Logger logger)
    {
        ++counter;
        if (counter >= REPORT_INTERVAL)
        {
            report(logger);
            reset_all();
        }
    }

    void report(rclcpp::Logger logger)
    {
        auto elapsed = Clock::now() - window_start_;
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
        double freq = elapsed_ms > 0 ? (counter * 1000.0 / elapsed_ms) : 0.0;

        std::stringstream ss;
        ss << "\n=== Control Loop Timing (us) ===";
        ss << "  window=" << elapsed_ms << "ms  freq=" << std::fixed << std::setprecision(1) << freq << "Hz\n";
        for (int i = 0; i < SECTION_COUNT; ++i)
        {
            Slot &s = slots[i];
            if (s.count > 0)
            {
                ss << "  " << NAMES[i] << ": min=" << s.min_us() << " avg=" << s.avg_us() << " max=" << s.max_us()
                   << " cnt=" << s.count << "\n";
            }
        }
        RCLCPP_INFO(logger, "%s", ss.str().c_str());
    }
};

TL_Teleop::TL_Teleop() : Node("tl_teleop_node")
{
    declare_parameter("arm_axis_mode", 0);
    declare_parameter("pos_scale", 0.5);
    declare_parameter("max_linear_speed", 0.25);
    declare_parameter("max_angular_speed", 1.5);
    declare_parameter("pos_deadzone", 0.005);
    declare_parameter("max_pos_delta_m", 0.3);
    declare_parameter("singular_angle", 160.0);
    declare_parameter("singular_scale", 0.2);
    declare_parameter("joint_jump_threshold", 30.0);
    declare_parameter("servo_speed", 25.0);
    declare_parameter("joint_limits", std::vector<double>{0.0});
    declare_parameter("servo_vmax", 80.0);
    declare_parameter("servo_amax", 3000.0);
    declare_parameter("servo_jmax", 50000.0);
    declare_parameter("ik_interval", 3);
    declare_parameter("interp_mode", 1);
    declare_parameter("gripper_enabled", false);

    arm_joints_ = get_parameter("arm_axis_mode").as_int();
    if (arm_joints_ != 6 && arm_joints_ != 7)
    {
        RCLCPP_FATAL(get_logger(),
                     "arm_axis_mode must be 6 or 7, got: %d. "
                     "Please provide a valid arm_axis_mode in the YAML config file.",
                     arm_joints_);
        init_failed_ = true;
        return;
    }
    pos_scale_ = get_parameter("pos_scale").as_double();
    max_linear_speed_ = get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = get_parameter("max_angular_speed").as_double();
    if (!std::isfinite(max_linear_speed_) || max_linear_speed_ <= 0.0 ||
        !std::isfinite(max_angular_speed_) || max_angular_speed_ <= 0.0)
    {
        RCLCPP_FATAL(get_logger(), "Cartesian speed limits must be finite and positive");
        init_failed_ = true;
        return;
    }
    pos_deadzone_ = get_parameter("pos_deadzone").as_double();
    max_pos_delta_m_ = get_parameter("max_pos_delta_m").as_double();
    // 配置沿用度制，内部统一为 rad
    singular_angle_ = get_parameter("singular_angle").as_double() * kDegToRad;
    singular_scale_ = get_parameter("singular_scale").as_double();
    joint_jump_threshold_ = get_parameter("joint_jump_threshold").as_double() * kDegToRad;
    servo_speed_ = get_parameter("servo_speed").as_double();
    auto joint_limits_flat = get_parameter("joint_limits").as_double_array();
    if (joint_limits_flat.size() != static_cast<size_t>(arm_joints_ * 2))
    {
        RCLCPP_FATAL(get_logger(),
                     "joint_limits flat array length mismatch: arm_axis_mode=%d, expected "
                     "exactly %d values, "
                     "got %zu. Each joint needs [min, max] — %d joints × 2 = %d values. "
                     "Please fix the joint_limits parameter in the YAML config file.",
                     arm_joints_, arm_joints_ * 2, joint_limits_flat.size(), arm_joints_, arm_joints_ * 2);
        init_failed_ = true;
        return;
    }

    joint_limits_.clear();
    for (size_t i = 0; i < joint_limits_flat.size(); i += 2)
    {
        if (!std::isfinite(joint_limits_flat[i]) || !std::isfinite(joint_limits_flat[i + 1]) ||
            joint_limits_flat[i] >= joint_limits_flat[i + 1])
        {
            RCLCPP_ERROR(get_logger(), "Invalid joint limits at J%zu", i / 2 + 1);
            init_failed_ = true;
            return;
        }
        // 配置为度，内部统一为 rad
        joint_limits_.emplace_back(joint_limits_flat[i] * kDegToRad, joint_limits_flat[i + 1] * kDegToRad);
    }

    double servo_vmax_val = get_parameter("servo_vmax").as_double();
    double servo_amax_val = get_parameter("servo_amax").as_double();
    double servo_jmax_val = get_parameter("servo_jmax").as_double();
    if (!(std::isfinite(servo_vmax_val) && servo_vmax_val > 0 &&
          std::isfinite(servo_amax_val) && servo_amax_val > 0 &&
          std::isfinite(servo_jmax_val) && servo_jmax_val > 0))
    {
        RCLCPP_ERROR(get_logger(), "Servo velocity, acceleration and jerk limits must be finite and positive");
        init_failed_ = true;
        return;
    }
    // OpenServoJ 接口为 rad/s、rad/s²、rad/s³；配置沿用度制
    servo_vmax_.assign(7, servo_vmax_val * kDegToRad);
    servo_amax_.assign(7, servo_amax_val * kDegToRad);
    servo_jmax_.assign(7, servo_jmax_val * kDegToRad);
    trajectory_ = std::make_unique<tl_teleop::JointTrajectory>(
        arm_joints_, CONTROL_PERIOD_S, servo_vmax_, servo_amax_, servo_jmax_);
    RCLCPP_INFO(get_logger(), "ServoJ command rate: %d Hz (%.1f ms period)",
                CONTROL_RATE_HZ, CONTROL_PERIOD_S * 1000.0);

    set_speed_client_ = create_client<tl_ros2_interface::srv::SetSpeed>("/tl_driver/set_speed");
    set_current_mode_client_ = create_client<tl_ros2_interface::srv::SetCurrentMode>("/tl_driver/set_current_mode");
    open_servoj_client_ = create_client<tl_ros2_interface::srv::OpenServoJ>("/tl_driver/open_servoj");
    close_servoj_client_ = create_client<std_srvs::srv::Trigger>("/tl_driver/close_servoj");
    coord_transform_client_ = create_client<tl_ros2_interface::srv::CoordTransform>("/tl_driver/coord_transform");

    vr_input_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/vr_input", rclcpp::SensorDataQoS());
    servoj_pos_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/tl_driver/set_servoj_pos", 10);

    gripper_enabled_ = get_parameter("gripper_enabled").as_bool();
    RCLCPP_INFO(get_logger(), "Gripper integration: %s", gripper_enabled_ ? "enabled" : "disabled (arm only)");
    if (gripper_enabled_)
    {
        auto input_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
        input_qos.lifespan(std::chrono::milliseconds(200));
        gripper_input_pub_ = create_publisher<sensor_msgs::msg::Joy>("/tl_teleop/gripper_input", input_qos);
        gripper_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/tl_teleop/gripper_ready", rclcpp::QoS(1).reliable().durability_volatile(),
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(gripper_mutex_);
                gripper_interlock_.update(msg->data, std::chrono::steady_clock::now());
            });
    }

    tcp_pose_sub_ = create_subscription<tl_ros2_interface::msg::CartesianPose>(
        "/tcp_pose", 10,
        [this](const tl_ros2_interface::msg::CartesianPose::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(tcp_pose_mutex_);
            latest_tcp_pose_ = *msg;
            tcp_received_ = std::chrono::steady_clock::now();
        });

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 1, [this](const sensor_msgs::msg::JointState &msg)
        {
            if (msg.position.size() != static_cast<size_t>(arm_joints_))
                return;
            for (size_t i = 0; i < msg.position.size(); ++i)
            {
                const double q = msg.position[i];
                const auto [lo, hi] = joint_limits_[i];
                if (!std::isfinite(q) || q < lo || q > hi)
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Invalid joint feedback at J%zu", i + 1);
                    std::lock_guard<std::mutex> lock(tcp_pose_mutex_);
                    joints_received_ = {};
                    return;
                }
            }
            std::lock_guard<std::mutex> lock(tcp_pose_mutex_);
            measured_joints_ = msg.position;  // tl_driver publishes in controller axis order
            joints_received_ = std::chrono::steady_clock::now();
        });

    RCLCPP_INFO(get_logger(), "TL_Teleop node initialized");
}

TL_Teleop::~TL_Teleop()
{
    RCLCPP_INFO(get_logger(), "TL_Teleop destroyed");
}

void TL_Teleop::on_pxrea_client_cb(void *context, PXREAClientCallbackType type, int status, void *userData)
{
    auto *self = static_cast<TL_Teleop *>(context);
    (void)status;

    switch (type)
    {
    case PXREAServerConnect:
        RCLCPP_INFO(self->get_logger(), "PXREA: server connected");
        break;
    case PXREAServerDisconnect:
        self->vr_state_.update({}, 0.0, false);
        RCLCPP_WARN(self->get_logger(), "PXREA: server disconnected");
        break;
    case PXREADeviceFind:
        RCLCPP_INFO(self->get_logger(), "PXREA: device found %s",
                    userData ? reinterpret_cast<const char *>(userData) : "");
        break;
    case PXREADeviceMissing:
        self->vr_state_.update({}, 0.0, false);
        RCLCPP_WARN(self->get_logger(), "PXREA: device missing %s",
                    userData ? reinterpret_cast<const char *>(userData) : "");
        break;
    case PXREADeviceConnect:
        RCLCPP_INFO(self->get_logger(), "PXREA: device connected %s",
                    userData ? reinterpret_cast<const char *>(userData) : "");
        break;
    case PXREADeviceStateJson:
    {
        // 帧率统计：每 100 帧输出一次窗口平均频率，替代逐帧打印避免刷屏
        // pxrea_frame_count_ / pxrea_rate_window_start_ 仅在 PXREA 回调线程访问，无需加锁
        ++self->pxrea_frame_count_;
        auto now = std::chrono::steady_clock::now();
        if (self->pxrea_frame_count_ >= 100)
        {
            double hz = self->pxrea_frame_count_ * 1000.0 /
                        std::chrono::duration<double, std::milli>(now - self->pxrea_rate_window_start_).count();
            RCLCPP_INFO(self->get_logger(), "PXREA state frames: %.1f Hz", hz);
            self->pxrea_frame_count_ = 0;
            self->pxrea_rate_window_start_ = now;
        }
        auto &dsj = *reinterpret_cast<PXREADevStateJson *>(userData);
        try
        {
            // PXREA SDK 的 JSON 是双层嵌套结构，外层 value 字段包含内层 JSON 字符串
            // 格式：{"value": "{\"Controller\": {\"right\": {\"pose\": \"...\",
            // \"grip\": 0.0, ...}}}"}
            json data = json::parse(dsj.stateJson);
            if (!data.contains("value"))
            {
                break;
            }
            json value = json::parse(data["value"].get<std::string>());
            if (!value.contains("Controller") || !value["Controller"].contains("right"))
            {
                break;
            }
            auto &right = value["Controller"]["right"];

            // pose 格式: "x,y,z,qx,qy,qz,qw"（逗号分隔的七个浮点数，m 和 无量纲）
            auto pose = parse_pose_str(right["pose"].get<std::string>());
            // grip 值域 [0, 1]，0=完全松开，1=完全握紧
            double grip = right.value("grip", 0.0);
            if (!std::isfinite(grip) || grip < 0.0 || grip > 1.0)
            {
                self->vr_state_.update({}, 0.0, false);
                break;
            }
            const double trigger = tl_teleop::read_trigger(right);
            // primaryButton 是 VR 手柄 A 键，用于复位到零点
            bool a_button = right.value("primaryButton", false);

            double norm = std::hypot(std::hypot(pose[3], pose[4]), std::hypot(pose[5], pose[6]));
            if (!std::all_of(pose.begin(), pose.end(), [](double x) { return std::isfinite(x); }) ||
                !std::isfinite(norm) || norm < 1e-6)
                break;
            for (size_t i = 3; i < pose.size(); ++i)
                pose[i] /= norm;
            self->vr_state_.update(pose, grip, a_button, trigger);
            ++self->valid_vr_frames_;
            if (self->vr_input_pub_->get_subscription_count() > 0)
            {
                std_msgs::msg::Float64MultiArray input;
                input.data.assign(pose.begin(), pose.end());
                input.data.push_back(grip);
                input.data.push_back(a_button ? 1.0 : 0.0);
                self->vr_input_pub_->publish(input);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(self->get_logger(), "PXREA JSON parse error: %s", e.what());
        }
        break;
    }
    default:
        break;
    }
}

// 将 VR 姿态字符串解析为数组，格式: "x,y,z,qx,qy,qz,qw"（分别表示位置 m
// 和四元数）
std::array<double, 7> TL_Teleop::parse_pose_str(const std::string &s)
{
    std::array<double, 7> result{};
    std::string token;
    std::istringstream iss(s);
    int i = 0;
    while (std::getline(iss, token, ',') && i < 7)
    {
        result[i++] = std::stod(token);
    }
    return result;
}

bool TL_Teleop::init_servo()
{
    const auto keep_running = [this]() { return running_ && rclcpp::ok(); };
    if (!keep_running())
        return false;
    // 1. Set current mode to 2 (run mode)
    auto mode_req = std::make_shared<tl_ros2_interface::srv::SetCurrentMode::Request>();
    mode_req->mode = 2;
    auto mode_future = set_current_mode_client_->async_send_request(mode_req);
    const auto mode_wait = tl_teleop::wait_startup_response(mode_future, std::chrono::seconds(5), keep_running);
    if (mode_wait != tl_teleop::StartupWaitResult::ready)
    {
        set_current_mode_client_->remove_pending_request(mode_future);
        if (mode_wait == tl_teleop::StartupWaitResult::timeout)
            RCLCPP_ERROR(get_logger(), "set_current_mode timeout");
        return false;
    }
    const auto mode_response = mode_future.get();
    if (!mode_response->success)
    {
        RCLCPP_ERROR(get_logger(), "set_current_mode failed: %s", mode_response->message.c_str());
        return false;
    }

    // 2. Set speed
    if (!keep_running())
        return false;
    auto speed_req = std::make_shared<tl_ros2_interface::srv::SetSpeed::Request>();
    speed_req->speed = servo_speed_;
    auto speed_future = set_speed_client_->async_send_request(speed_req);
    const auto speed_wait = tl_teleop::wait_startup_response(speed_future, std::chrono::seconds(5), keep_running);
    if (speed_wait != tl_teleop::StartupWaitResult::ready)
    {
        set_speed_client_->remove_pending_request(speed_future);
        if (speed_wait == tl_teleop::StartupWaitResult::timeout)
            RCLCPP_ERROR(get_logger(), "set_speed timeout");
        return false;
    }
    if (!speed_future.get()->success)
    {
        RCLCPP_ERROR(get_logger(), "set_speed failed");
        return false;
    }

    // 3. Open ServoJ
    if (!keep_running())
        return false;
    auto servoj_req = std::make_shared<tl_ros2_interface::srv::OpenServoJ::Request>();
    servoj_req->vmax = servo_vmax_;
    servoj_req->amax = servo_amax_;
    servoj_req->jmax = servo_jmax_;
    // A cancelled/timed-out reply does not prove that the controller ignored
    // the open request. Main must close this possibly opened session on exit.
    servo_close_needed_ = true;
    auto servoj_future = open_servoj_client_->async_send_request(servoj_req);
    const auto servo_wait = tl_teleop::wait_startup_response(servoj_future, std::chrono::seconds(5), keep_running);
    if (servo_wait != tl_teleop::StartupWaitResult::ready)
    {
        open_servoj_client_->remove_pending_request(servoj_future);
        if (servo_wait == tl_teleop::StartupWaitResult::timeout)
            RCLCPP_ERROR(get_logger(), "open_servoj timeout");
        return false;
    }
    if (!servoj_future.get()->success)
    {
        // A failure may be a driver-side reply timeout after opening; retain
        // best-effort cleanup once an open request has been sent.
        RCLCPP_ERROR(get_logger(), "open_servoj failed");
        return false;
    }

    RCLCPP_INFO(get_logger(), "ServoJ initialized via tl_driver services");
    for (int i = 0; i < 4 && keep_running(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return keep_running();
}

void TL_Teleop::shutdown_teleop(rclcpp::Executor &exec)
{
    if (!servo_close_needed_)
        return;
    if (!rclcpp::ok())
    {
        RCLCPP_ERROR(get_logger(), "rclcpp not ok, cannot close ServoJ");
        return;
    }
    auto close_req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = close_servoj_client_->async_send_request(close_req);

    auto ret = exec.spin_until_future_complete(future, std::chrono::seconds(2));
    if (ret != rclcpp::FutureReturnCode::SUCCESS)
    {
        close_servoj_client_->remove_pending_request(future);
        RCLCPP_ERROR(get_logger(), "close_servoj failed (code %d)", static_cast<int>(ret));
        return;
    }
    if (!future.get()->success)
    {
        RCLCPP_ERROR(get_logger(), "close_servoj service reported failure");
        return;
    }
    RCLCPP_INFO(get_logger(), "ServoJ closed via tl_driver services");
    servo_close_needed_ = false;
}

void TL_Teleop::cancel_ik()
{
    if (pending_ik_)
    {
        coord_transform_client_->remove_pending_request(*pending_ik_);
        pending_ik_.reset();
    }
    last_ik_result_.clear();
}

std::vector<double> TL_Teleop::next_ik_pose(const std::vector<double> &goal) const
{
    auto result = accepted_ik_pose_;
    double distance_squared = 0.0;
    for (size_t i = 0; i < 3; ++i)
        distance_squared += std::pow(goal[i] - result[i], 2);
    const double distance = std::sqrt(distance_squared);
    const double position_fraction = distance > 1e-12 ?
        std::min(1.0, max_linear_speed_ * CONTROL_PERIOD_S * ik_step_scale_ / distance) : 1.0;
    for (size_t i = 0; i < 3; ++i)
        result[i] += position_fraction * (goal[i] - result[i]);

    auto start = rpy2quat(result[3], result[4], result[5]);
    auto finish = rpy2quat(goal[3], goal[4], goal[5]);
    double dot = 0.0;
    for (size_t i = 0; i < 4; ++i)
        dot += start[i] * finish[i];
    if (dot < 0.0)
    {
        for (double &value : finish)
            value = -value;
        dot = -dot;
    }
    const double half_angle = std::acos(std::clamp(dot, 0.0, 1.0));
    const double fraction = half_angle > 1e-12 ?
        std::min(1.0, max_angular_speed_ * CONTROL_PERIOD_S * ik_step_scale_ / (2 * half_angle)) : 1.0;
    std::array<double, 4> rotation = finish;
    if (half_angle > 1e-8)
    {
        for (size_t i = 0; i < 4; ++i)
            rotation[i] = (std::sin((1-fraction)*half_angle)*start[i] +
                           std::sin(fraction*half_angle)*finish[i]) / std::sin(half_angle);
    }
    const auto rpy = quat2rpy(rotation);
    std::copy(rpy.begin(), rpy.end(), result.begin() + 3);
    result[6] = goal[6];
    return result;
}

void TL_Teleop::update_ik(const std::vector<double> &pose)
{
    const auto now = std::chrono::steady_clock::now();
    if (pending_ik_)
    {
        if (now - ik_started_ >= STATE_TIMEOUT)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "IK response timed out");
            cancel_ik();
        }
        else
        {
            if (pending_ik_->wait_for(std::chrono::seconds(0)) != std::future_status::ready)
                return;
            auto response = pending_ik_->get();
            pending_ik_.reset();
            auto &q = response->target_pos;
            bool accepted = false;
            if (response->success && (q.size() == static_cast<size_t>(arm_joints_) || q.size() == 7))
            {
                q.resize(arm_joints_);
                // Compare successive IK solutions. The emitted command can lag a
                // continuous target while respecting the trajectory speed limits.
                if (joints_safe(q))
                {
                    ik_reference_ = q;
                    last_ik_result_ = q;
                    accepted_ik_pose_ = requested_ik_pose_;
                    accepted = true;
                }
            }
            // Retry toward the newest hand goal from the last accepted pose.
            // Smaller steps let the solver approach singularities continuously.
            ik_step_scale_ = accepted ? std::min(1.0, ik_step_scale_ * 2) : std::max(1.0/128, ik_step_scale_ / 2);
            if (!accepted)
                last_ik_result_.clear();
        }
    }
    auto req = std::make_shared<tl_ros2_interface::srv::CoordTransform::Request>();
    req->origin_coord = 1;
    req->target_coord = 0;
    req->form = 0;
    requested_ik_pose_ = next_ik_pose(pose);
    req->origin_pos = requested_ik_pose_;
    req->reference_pos = ik_reference_;
    req->reference_pos.resize(7, 0.0);
    pending_ik_.emplace(coord_transform_client_->async_send_request(req));
    ik_started_ = now;
}

// RPY 欧拉角 → 四元数，返回顺序：[w, x, y, z]
// 旋转序与控制器位姿一致：R = Rx(rx)·Ry(ry)·Rz(rz)（见 tl_interface.h 的 rpy2quat）
std::array<double, 4> TL_Teleop::rpy2quat(double rx, double ry, double rz)
{
    const double cr = std::cos(rx * 0.5);
    const double sr = std::sin(rx * 0.5);
    const double cp = std::cos(ry * 0.5);
    const double sp = std::sin(ry * 0.5);
    const double cy = std::cos(rz * 0.5);
    const double sy = std::sin(rz * 0.5);
    return {
        cr * cp * cy - sr * sp * sy,
        sr * cp * cy + cr * sp * sy,
        cr * sp * cy - sr * cp * sy,
        cr * cp * sy + sr * sp * cy,
    };
}

// 将关节角度裁剪到限位范围内，防止运动超出机械臂物理限制
// 输入/输出单位：rad
std::vector<double> TL_Teleop::clamp_joints(const std::vector<double> &joints)
{
    std::vector<double> clamped;
    for (size_t i = 0; i < joints.size() && i < joint_limits_.size(); ++i)
    {
        double lo = joint_limits_[i].first;
        double hi = joint_limits_[i].second;
        clamped.push_back(std::max(lo, std::min(hi, joints[i])));
    }
    return clamped;
}

// Validate continuous IK solutions before the trajectory generator tracks them.
bool TL_Teleop::joints_safe(const std::vector<double> &new_joints)
{
    if (new_joints.size() != ik_reference_.size())
        return false;
    for (size_t i = 0; i < new_joints.size(); ++i)
    {
        const auto [lo, hi] = joint_limits_[i];
        if (!std::isfinite(new_joints[i]) || new_joints[i] < lo || new_joints[i] > hi)
            return false;
        if (std::abs(new_joints[i] - ik_reference_[i]) > joint_jump_threshold_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "J%zu IK branch jump: %.3f rad", i + 1,
                                std::abs(new_joints[i] - ik_reference_[i]));
            return false;
        }
    }
    return true;
}

bool TL_Teleop::servoJ_send(const std::vector<double> &joint_target)
{
    const bool valid = joint_target.empty() ? trajectory_->brake() : trajectory_->step(joint_target);
    if (!valid)
    {
        RCLCPP_ERROR(get_logger(), "Joint trajectory calculation failed");
        return false;
    }
    const auto &next = trajectory_->position();
    for (size_t i = 0; i < next.size(); ++i)
    {
        const auto [lo, hi] = joint_limits_[i];
        if (!std::isfinite(next[i]) || next[i] < lo - 1e-10 || next[i] > hi + 1e-10)
        {
            RCLCPP_ERROR(get_logger(), "Trajectory reached J%zu position limit", i + 1);
            return false;
        }
    }
    last_joints_ = clamp_joints(next);  // Remove sub-nanoradian endpoint roundoff.
    msg_.data = last_joints_;
    servoj_pos_pub_->publish(msg_);
    return true;
}

// 四元数乘法：q1 * q2，表示先旋转 q2 再旋转 q1
// 约定：[w, x, y, z] 顺序
std::array<double, 4> TL_Teleop::quat_multiply(const std::array<double, 4> &q1, const std::array<double, 4> &q2)
{
    double w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
    double w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];
    return {w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2};
}

// 四元数共轭（逆），用于计算旋转差值
std::array<double, 4> TL_Teleop::quat_inverse(const std::array<double, 4> &q)
{
    return {q[0], -q[1], -q[2], -q[3]};
}

// 四元数 → RPY 欧拉角（XYZ 内旋格式），返回值单位：rad
std::array<double, 3> TL_Teleop::quat2rpy(const std::array<double, 4> &q)
{
    double w = q[0], x = q[1], y = q[2], z = q[3];
    const double sin_pitch = std::clamp(2.0 * (w*y + x*z), -1.0, 1.0);
    if (std::abs(sin_pitch) >= 1.0 - 1e-12)
        return {2.0 * std::atan2(x, w), std::copysign(M_PI / 2, sin_pitch), 0.0};
    return {std::atan2(2.0 * (w*x - y*z), 1.0 - 2.0 * (x*x + y*y)),
            std::asin(sin_pitch),
            std::atan2(2.0 * (w*z - x*y), 1.0 - 2.0 * (y*y + z*z))};
}

void TL_Teleop::publish_gripper_input(double trigger, bool enabled,
                                    std::chrono::steady_clock::time_point received)
{
    if (!gripper_input_pub_)
        return;
    sensor_msgs::msg::Joy input;
    const auto ros_now = now();
    input.header.stamp = ros_now;
    if (received != std::chrono::steady_clock::time_point{})
    {
        const auto age_ns = std::max<int64_t>(0, std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - received).count());
        // Preserve source freshness instead of restamping an old VR sample as new.
        input.header.stamp = age_ns <= ros_now.nanoseconds() ?
            static_cast<builtin_interfaces::msg::Time>(
                ros_now - rclcpp::Duration(std::chrono::nanoseconds(age_ns))) : builtin_interfaces::msg::Time{};
    }
    input.axes = {static_cast<float>(trigger)};
    input.buttons = {enabled && std::isfinite(trigger) && trigger >= 0.0 && trigger <= 1.0 ? 1 : 0};
    gripper_input_pub_->publish(input);
}

void TL_Teleop::control_loop()
{
    // 基准点：握下 Grip 瞬间的机械臂笛卡尔位置（m）
    double base_x = 0.0, base_y = 0.0, base_z = 0.0, arm_angle = 0.0;
    TeleopTiming prof;
    enum class Mode { idle, tracking, zeroing };
    Mode mode = Mode::idle;
    bool armed = false;
    RCLCPP_INFO(get_logger(), "Release grip and A to arm teleoperation");
    std::array<double, 7> home_pose{};
    std::array<double, 4> arm_q{}, vr_hq{};
    double scale = 1.0;
    const auto zero = clamp_joints(std::vector<double>(arm_joints_, 0.0));

    while (running_)
    {
        auto t0 = std::chrono::steady_clock::now();

        std::array<double, 7> pose;
        double grip;
        double trigger;
        bool a_button;
        std::chrono::steady_clock::time_point vr_received;
        vr_state_.get(pose, grip, a_button, trigger, vr_received);
        const auto vr_age = t0 - vr_received;
        const bool vr_fresh = vr_age < STATE_TIMEOUT;
        if (valid_vr_frames_ == 0)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "No valid VR controller frames received yet; check PC Service and headset connection");
        else if (!vr_fresh)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "VR input stale (%.0f ms); waiting for fresh controller frames",
                std::chrono::duration<double, std::milli>(vr_age).count());
        bool gripper_ok = true;
        bool gripper_ready = true;
        if (gripper_enabled_)
        {
            std::lock_guard<std::mutex> lock(gripper_mutex_);
            gripper_ready = gripper_interlock_.feedback_ready(std::chrono::steady_clock::now());
            gripper_ok = gripper_interlock_.allows_motion(
                std::chrono::steady_clock::now(), vr_fresh && grip <= GRIP_RELEASE && !a_button);
        }
        if (!gripper_ok)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "%s", !gripper_ready ? "Waiting for fresh calibrated gripper feedback" :
                !vr_fresh ? "Gripper ready; waiting for fresh VR input" :
                "Gripper ready; release Grip and A to arm teleoperation");
        if (vr_age >= std::chrono::milliseconds(500) || !gripper_ok)
            armed = false;
        if (!armed && gripper_ok && vr_fresh && grip <= GRIP_RELEASE && !a_button)
        {
            armed = true;
            RCLCPP_INFO(get_logger(), "Teleoperation armed");
        }
        prof.slots[TeleopTiming::VR_GET].record(std::chrono::steady_clock::now() - t0);

        double x = pose[0], y = pose[1], z = pose[2];
        double qx = pose[3], qy = pose[4], qz = pose[5], qw = pose[6];

        Mode requested = armed ? mode : Mode::idle;
        if (armed && vr_fresh)
        {
            requested = Mode::idle;
            if (a_button)
                requested = Mode::zeroing;
            else if (grip > (mode == Mode::tracking ? GRIP_RELEASE : GRIP_THRESHOLD))
                requested = Mode::tracking;
        }

        {
            std::lock_guard<std::mutex> lock(tcp_pose_mutex_);
            if (t0 - joints_received_ >= STATE_TIMEOUT || t0 - tcp_received_ >= STATE_TIMEOUT)
            {
                requested = Mode::idle;
                armed = false;
            }

            if (requested != mode && requested != Mode::idle)
            {
                last_joints_ = measured_joints_;
                trajectory_->reset(last_joints_);
                ik_reference_ = last_joints_;
                // Keep the scale fixed for this grip to avoid moving the absolute
                // target when a joint crosses the configured threshold.
                scale = (std::abs(last_joints_[arm_joints_ - 2]) > singular_angle_ ||
                         std::abs(last_joints_.back()) > singular_angle_) ? singular_scale_ : 1.0;
                home_pose = pose;
                vr_hq = {qx, qy, qz, qw};
                const auto &tcp = latest_tcp_pose_;
                base_x = tcp.position.x;
                base_y = tcp.position.y;
                base_z = tcp.position.z;
                arm_angle = tcp.arm_angle;
                arm_q = rpy2quat(tcp.rpy.x, tcp.rpy.y, tcp.rpy.z);
                accepted_ik_pose_ = {base_x, base_y, base_z, tcp.rpy.x, tcp.rpy.y, tcp.rpy.z, arm_angle};
                ik_step_scale_ = 1.0;
            }
        }
        const bool entering = requested != mode;
        if (entering)
        {
            cancel_ik();
            mode = requested;
            RCLCPP_INFO(get_logger(), "Teleop mode %d", static_cast<int>(mode));
            if (mode != Mode::idle)
            {
                msg_.data = last_joints_;
                servoj_pos_pub_->publish(msg_);
            }
        }
        else if (mode != Mode::idle && !vr_fresh)
        {
            cancel_ik();
            if (!servoJ_send({}))
                break;
        }
        else if (mode == Mode::zeroing)
        {
            if (!servoJ_send(zero))
                break;
        }
        publish_gripper_input(trigger, armed && mode == Mode::tracking && vr_fresh, vr_received);
        if (mode == Mode::tracking && !entering && vr_fresh)
        {
            // 手柄当前位置相对于 home 的偏移（m）
            double dx = x - home_pose[0];
            double dy = y - home_pose[1];
            double dz = z - home_pose[2];
            dx = std::copysign(std::max(0.0, std::abs(dx) - pos_deadzone_), dx);
            dy = std::copysign(std::max(0.0, std::abs(dy) - pos_deadzone_), dy);
            dz = std::copysign(std::max(0.0, std::abs(dz) - pos_deadzone_), dz);

            // VR 手柄坐标系 → 机械臂基座坐标系映射
            // VR: X右 Y上 Z前  |  机械臂: X前 Y左 Z上
            // 映射关系：X_arm = -Z_vr, Y_arm = -X_vr, Z_arm = Y_vr
            // 手柄与机械臂均为 m，直接乘位置缩放系数
            double arm_delta_X = -dz * pos_scale_ * scale;
            double arm_delta_Y = -dx * pos_scale_ * scale;
            double arm_delta_Z = dy * pos_scale_ * scale;
            arm_delta_X = std::max(-max_pos_delta_m_, std::min(max_pos_delta_m_, arm_delta_X));
            arm_delta_Y = std::max(-max_pos_delta_m_, std::min(max_pos_delta_m_, arm_delta_Y));
            arm_delta_Z = std::max(-max_pos_delta_m_, std::min(max_pos_delta_m_, arm_delta_Z));

            double target_x = base_x + arm_delta_X;
            double target_y = base_y + arm_delta_Y;
            double target_z = base_z + arm_delta_Z;

            // 取最短旋转路径：若当前 quat 与 home quat 点积为负，翻转其中一个
            // 避免 360°→0° 走长路
            auto t_orient = std::chrono::steady_clock::now();
            std::array<double, 4> vr_quat_wxyz = {qw, qx, qy, qz};
            std::array<double, 4> vr_home_wxyz = {vr_hq[3], vr_hq[0], vr_hq[1], vr_hq[2]};

            double dot = 0.0;
            for (int i = 0; i < 4; ++i)
            {
                dot += vr_quat_wxyz[i] * vr_home_wxyz[i];
            }
            if (dot < 0.0)
            {
                for (int i = 0; i < 4; ++i)
                {
                    vr_quat_wxyz[i] = -vr_quat_wxyz[i];
                }
            }

            auto vr_inv = quat_inverse(vr_home_wxyz);
            auto vr_delta = quat_multiply(vr_quat_wxyz, vr_inv);

            // VR 四元数 → 机械臂四元数坐标系映射：[w, -z, -x, +y]
            std::array<double, 4> vr_delta_mapped = {vr_delta[0], -vr_delta[3], -vr_delta[1], vr_delta[2]};
            auto target_quat = quat_multiply(vr_delta_mapped, arm_q);

            // 目标四元数 → RPY → 逆运动学求解
            auto rpy = quat2rpy(target_quat);
            double target_rx = rpy[0];
            double target_ry = rpy[1];
            double target_rz = rpy[2];
            prof.slots[TeleopTiming::ORIENT].record(std::chrono::steady_clock::now() - t_orient);

            auto t_ik = std::chrono::steady_clock::now();
            update_ik({target_x, target_y, target_z, target_rx, target_ry, target_rz, arm_angle});
            prof.slots[TeleopTiming::IK].record(std::chrono::steady_clock::now() - t_ik);
            if (!servoJ_send(last_ik_result_))
                break;
        }

        // 固定频率控制：250Hz，混合休眠 + 自旋保持精度
        auto t1 = std::chrono::steady_clock::now();
        prof.slots[TeleopTiming::TOTAL].record(t1 - t0);
        auto elapsed = std::chrono::duration<double>(t1 - t0).count();
        double sleep_time = CONTROL_PERIOD_S - elapsed;
        if (sleep_time > 0)
        {
            // 前段用 sleep_for（省电），后段 500us 用自旋（维持精度）
            constexpr double SPIN_SAFE_US = 500.0;
            double sleep_us = sleep_time * 1e6;
            if (sleep_us > SPIN_SAFE_US)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(sleep_us - SPIN_SAFE_US)));
            }
            // 自旋等待剩余时间，避免 sleep_for 精度不足
            while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < CONTROL_PERIOD_S)
            {
                std::this_thread::yield();
            }
        }
        prof.tick(get_logger());
    }
    publish_gripper_input(std::numeric_limits<double>::quiet_NaN(), false,
                          std::chrono::steady_clock::now());
}

void TL_Teleop::run_control_loop()
{
    bool pxrea_initialized = false;
    try
    {
        do
        {
            if (init_failed_)
            {
                RCLCPP_FATAL(get_logger(), "Initialization failed, refusing to start control loop");
                break;
            }

            // Discover the driver before acquiring an SDK connection. A stopped
            // launch waiting for the arm must not keep an unnecessary VR client.
            RCLCPP_INFO(get_logger(), "Waiting for tl_driver services (up to 30s; Ctrl-C cancels)...");
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
            bool services_ready = false;
            while (running_ && rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
            {
                if (set_speed_client_->wait_for_service(std::chrono::milliseconds(100)))
                {
                    services_ready = true;
                    break;
                }
            }
            if (!running_ || !rclcpp::ok())
            {
                RCLCPP_INFO(get_logger(), "Teleop startup cancelled while waiting for tl_driver");
                break;
            }
            if (!services_ready)
            {
                RCLCPP_ERROR(get_logger(), "tl_driver services not available after 30s; exiting");
                break;
            }
            RCLCPP_INFO(get_logger(), "tl_driver services available");

            RCLCPP_INFO(get_logger(), "Initializing PXREA SDK...");
            const int pxrea_ret = PXREAInit(this, on_pxrea_client_cb, PXREAFullMask);
            if (pxrea_ret != 0)
            {
                RCLCPP_ERROR(get_logger(), "PXREAInit failed: %d", pxrea_ret);
                break;
            }
            pxrea_initialized = true;
            RCLCPP_INFO(get_logger(), "PXREA SDK initialized");

            if (!init_servo())
            {
                if (running_ && rclcpp::ok())
                    RCLCPP_ERROR(get_logger(), "Servo initialization failed; exiting");
                else
                    RCLCPP_INFO(get_logger(), "Teleop startup cancelled during ServoJ initialization");
                break;
            }
            RCLCPP_INFO(get_logger(), "ServoJ ready; motion requires fresh VR input and released Grip/A before arming");
            control_loop();
        } while (false);
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(get_logger(), "Teleop control thread failed: %s", error.what());
    }
    running_ = false;
    // Main remains responsible for closing an opened ServoJ session and ROS.
    if (pxrea_initialized)
        PXREADeinit();
}

namespace
{

    std::atomic<bool> g_sigint_received{false};

    extern "C" void sigint_handler(int)
    {
        g_sigint_received = true;
    }
} // namespace

int main(int argc, char **argv)
{
    tl_teleop::SingleInstance instance;
    std::string instance_error;
    if (!instance.acquire(instance_error))
    {
        std::cerr << instance_error << std::endl;
        return 1;
    }
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TL_Teleop>();

    if (!node->init_ok())
    {
        rclcpp::shutdown();
        return 1;
    }

    // 替换 rclcpp 的 SIGINT/SIGTERM 处理：仅设标志，不立即 shutdown
    // 确保 close_servo() 在 rclcpp 存活时执行
    // Install before starting the worker, including cancellable startup waits.
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    // 控制循环在单独线程运行，避免 250Hz 循环阻塞 ROS2 spin
    // 主线程负责 ROS2 回调（服务调用、话题订阅）
    std::thread control_thread(
        [&node]()
        {
            node->run_control_loop();
        });

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    // 自定义 spin 循环：检测 SIGINT 标志而非依赖 rclcpp::shutdown()
    while (rclcpp::ok() && !g_sigint_received && node->is_running())
    {
        exec.spin_once(std::chrono::milliseconds(5));
    }

    if (rclcpp::ok())
    {
        // Ctrl+C：rclcpp 仍存活，有序关闭
        node->stop();
        control_thread.join();       // control_loop 退出 → PXREADeinit()
        node->shutdown_teleop(exec); // exec.spin_until_future_complete 等 response
        rclcpp::shutdown();
    }
    else
    {
        // ROS context was shut down externally. Join the worker before releasing
        // its node; no service call can be made after context shutdown.
        node->stop();
        if (control_thread.joinable())
        {
            control_thread.join();
        }
        rclcpp::shutdown();
    }

    return 0;
}
