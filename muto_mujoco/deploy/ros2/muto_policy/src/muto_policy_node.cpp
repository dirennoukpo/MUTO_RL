// MUTO RS walking policy node (interface v1: 29 inputs, see muto_mujoco/muto_env/env.py).
//
// In:  /muto/imu (sensor_msgs/Imu, orientation + gyro), /cmd_vel (geometry_msgs/Twist: linear.x, linear.y,
//      angular.z), /joint_states (only once, for the start pose)
// Out: /leg_controller/commands (Float64MultiArray, URDF joint angles [rad] in the controller's joint order)
//      /muto_policy/obs (Float32MultiArray, the 29 inputs; published in every state, for checking the IMU axes)
//
// Every 20 ms: heading hold -> 29 inputs -> ONNX -> joint targets. The hardware plugin then converts the
// URDF angles to servo degrees (inversion + offsets in the xacro) and rounds them, exactly like the sim.
//
// States: WAIT (no IMU or joint states yet) -> RAMP (move slowly to the home pose) -> RUN (policy)
//         -> FAULT (tilt too large or IMU silent: hold the home pose, policy stopped until restart).

#include <chrono>
#include <map>
#include <mutex>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "muto_policy/onnx_policy.hpp"
#include "muto_policy/policy_io.hpp"

using namespace std::chrono_literals;
using muto_policy::kAct;

class MutoPolicyNode : public rclcpp::Node {
public:
  MutoPolicyNode() : Node("muto_policy_inference_node") {
    auto path = declare_parameter<std::string>("policy_path", "muto_walk_policy.onnx");
    if (!path.empty() && path.front() != '/')
      path = ament_index_cpp::get_package_share_directory("muto_description") + "/config/" + path;
    // order of /leg_controller/commands = joints list in controllers.yaml
    controller_joints_ = declare_parameter<std::vector<std::string>>("controller_joints", {
      "yq1_Joint", "yq2_Joint", "yq3_Joint", "yz1_Joint", "yz2_Joint", "yz3_Joint",
      "yh1_Joint", "yh2_Joint", "yh3_Joint", "zq1_Joint", "zq2_Joint", "zq3_Joint",
      "zz1_Joint", "zz2_Joint", "zz3_Joint", "zh1_Joint", "zh2_Joint", "zh3_Joint"});
    heading_hold_ = declare_parameter<bool>("heading_hold", true);
    ramp_s_ = declare_parameter<double>("ramp_s", 2.0);
    max_tilt_deg_ = declare_parameter<double>("max_tilt_deg", 50.0);
    imu_timeout_s_ = declare_parameter<double>("imu_timeout_s", 0.2);
    cmd_timeout_s_ = declare_parameter<double>("cmd_timeout_s", 0.5);

    policy_ = std::make_unique<muto_policy::OnnxPolicy>(path);
    io_ = std::make_unique<muto_policy::PolicyIO>(policy_->interface_json);

    // policy output index (servo-ID order) -> controller index
    for (const auto & name : controller_joints_) {
      auto it = std::find(io_->action_names.begin(), io_->action_names.end(), name);
      if (it == io_->action_names.end()) throw std::runtime_error("controller joint not in policy: " + name);
      to_policy_.push_back(static_cast<int>(it - io_->action_names.begin()));
    }
    if (to_policy_.size() != kAct) throw std::runtime_error("controller_joints must list the 18 joints");

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/muto/imu", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        quat_ = {m->orientation.w, m->orientation.x, m->orientation.y, m->orientation.z};
        gyro_ = {m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z};
        imu_time_ = now();
        has_imu_ = true;
      });
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        command_ = {std::clamp(m->linear.x, -io_->cmd_max[0], io_->cmd_max[0]),
                    std::clamp(m->linear.y, -io_->cmd_max[1], io_->cmd_max[1]),
                    std::clamp(m->angular.z, -io_->cmd_max[2], io_->cmd_max[2])};
        cmd_time_ = now();
      });
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,
      [this](sensor_msgs::msg::JointState::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_start_) return;
        std::map<std::string, double> pos;
        for (size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) pos[m->name[i]] = m->position[i];
        for (int i = 0; i < kAct; ++i) {
          auto it = pos.find(io_->action_names[i]);
          if (it == pos.end()) return;
          start_[i] = it->second;
        }
        has_start_ = true;
      });
    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/leg_controller/commands", 1);
    obs_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/muto_policy/obs", 1);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / io_->control_hz), [this] { step(); });

    RCLCPP_INFO(get_logger(), "policy %s loaded: 29 inputs -> 18 joints at %.0f Hz, heading hold %s",
      path.c_str(), io_->control_hz, heading_hold_ ? "on" : "off");
  }

private:
  enum class State { WAIT, RAMP, RUN, FAULT };

  void publish(const std::array<double, kAct> & q_policy_order) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(kAct);
    for (int c = 0; c < kAct; ++c) msg.data[c] = q_policy_order[to_policy_[c]];
    pub_->publish(msg);
  }

  void publish_obs(const std::array<float, muto_policy::kObs> & obs) {
    std_msgs::msg::Float32MultiArray msg;
    msg.data.assign(obs.begin(), obs.end());
    obs_pub_->publish(msg);
  }

  std::array<double, kAct> home() const {
    std::array<double, kAct> q{};
    std::copy(io_->home.begin(), io_->home.end(), q.begin());
    return q;
  }

  void fault(const std::string & why) {
    RCLCPP_ERROR(get_logger(), "FAULT: %s. Holding the home pose; restart the node to walk again.", why.c_str());
    state_ = State::FAULT;
  }

  void step() {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto t = now();
    if (has_imu_ && state_ != State::RUN) publish_obs(io_->observation(quat_, gyro_, {0.0, 0.0, 0.0}));
    switch (state_) {
      case State::WAIT:
        if (has_imu_ && has_start_) {
          const auto g = muto_policy::projected_gravity(quat_);
          RCLCPP_INFO(get_logger(), "start: gravity in body frame (%.2f, %.2f, %.2f), should be ~(0, 0, -1) "
            "on flat ground. Ramping to the home pose in %.1f s.", g[0], g[1], g[2], ramp_s_);
          ramp_start_ = t;
          state_ = State::RAMP;
        } else {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for %s%s",
            has_imu_ ? "" : "/muto/imu ", has_start_ ? "" : "/joint_states");
        }
        return;

      case State::RAMP: {
        const double s = std::min(1.0, (t - ramp_start_).seconds() / std::max(ramp_s_, 1e-3));
        std::array<double, kAct> q{};
        for (int i = 0; i < kAct; ++i) q[i] = start_[i] + s * (io_->home[i] - start_[i]);
        publish(q);
        if (s >= 1.0) {
          io_->reset();
          state_ = State::RUN;
          RCLCPP_INFO(get_logger(), "home pose reached, policy running");
        }
        return;
      }

      case State::FAULT:
        publish(home());
        return;

      case State::RUN:
        break;
    }

    // ── safety ──
    if ((t - imu_time_).seconds() > imu_timeout_s_) return fault("no IMU message for more than imu_timeout_s");
    const auto g = muto_policy::projected_gravity(quat_);
    if (-g[2] < std::cos(max_tilt_deg_ * M_PI / 180.0)) return fault("body tilted more than max_tilt_deg");

    // ── one policy step, exactly like MutoEnv.step ──
    muto_policy::Vec3 command{0.0, 0.0, 0.0};
    if (cmd_time_.nanoseconds() > 0 && (t - cmd_time_).seconds() < cmd_timeout_s_) command = command_;
    if (heading_hold_) command = io_->hold_heading(command, quat_);
    const auto obs = io_->observation(quat_, gyro_, command);
    const auto action = (*policy_)(obs);
    publish(io_->joint_targets(action.data()));
    publish_obs(obs);
    io_->advance(action.data());
  }

  std::unique_ptr<muto_policy::OnnxPolicy> policy_;
  std::unique_ptr<muto_policy::PolicyIO> io_;
  std::vector<std::string> controller_joints_;
  std::vector<int> to_policy_;
  bool heading_hold_{true};
  double ramp_s_{2.0}, max_tilt_deg_{50.0}, imu_timeout_s_{0.2}, cmd_timeout_s_{0.5};

  std::mutex mutex_;
  State state_{State::WAIT};
  muto_policy::Quat quat_{1, 0, 0, 0};
  muto_policy::Vec3 gyro_{0, 0, 0}, command_{0, 0, 0};
  std::array<double, kAct> start_{};
  bool has_imu_{false}, has_start_{false};
  rclcpp::Time imu_time_{0, 0, RCL_ROS_TIME}, cmd_time_{0, 0, RCL_ROS_TIME}, ramp_start_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obs_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MutoPolicyNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("muto_policy"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
