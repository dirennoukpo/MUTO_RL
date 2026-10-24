// Robot side of the policy interface: a line-by-line port of muto_mujoco/deploy/policy_io.py.
// Every constant is read from the ONNX metadata ("muto_interface"), so model and robot code can't disagree.
//
// One control cycle (50 Hz):
//   obs    = io.observation(imu_quat_wxyz, gyro_rad_s, command)   // latest IMU sample (IMU runs at 25 Hz)
//   action = onnx(obs)                                            // 18 numbers in [-1, 1]
//   q      = io.joint_targets(action)                             // URDF joint angles [rad], servo-ID order
//   io.advance(action)                                            // remember the action, move the clock
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace muto_policy {

constexpr int kObs = 29;
constexpr int kAct = 18;
using Quat = std::array<double, 4>;  // w, x, y, z
using Vec3 = std::array<double, 3>;

inline double yaw_of(const Quat & q) {
  return std::atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
}

// Gravity direction in the body frame: R(q)^T * (0, 0, -1) = minus the third row of R.
inline Vec3 projected_gravity(Quat q) {
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  for (auto & c : q) c /= n;
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  return {-(2 * (x * z - w * y)), -(2 * (y * z + w * x)), -(1 - 2 * (x * x + y * y))};
}

class PolicyIO {
public:
  explicit PolicyIO(const std::string & interface_json) {
    const auto j = nlohmann::json::parse(interface_json);
    action_names = j.at("action").get<std::vector<std::string>>();
    cmd_max = j.at("cmd_max").get<std::vector<double>>();
    home = j.at("home_rad").get<std::vector<double>>();
    lo = j.at("joint_range_rad").at(0).get<std::vector<double>>();
    hi = j.at("joint_range_rad").at(1).get<std::vector<double>>();
    servo_sign = j.at("servo_sign").get<std::vector<double>>();
    servo_offset_deg = j.at("servo_offset_deg").get<std::vector<double>>();
    action_scale = j.at("action_scale_rad").get<double>();
    gyro_scale = j.at("gyro_scale").get<double>();
    phase_step = j.at("gait_hz").get<double>() / j.at("control_hz").get<double>();  // 2 Hz / 50 Hz = 0.04
    control_hz = j.at("control_hz").get<double>();
    heading_gain = j.value("heading_gain", 2.0);
    if (action_names.size() != kAct || home.size() != kAct || cmd_max.size() != 3)
      throw std::runtime_error("muto_interface: unexpected sizes");
    reset();
  }

  void reset(double start_phase = 0.0) {
    last_action.fill(0.0);
    phase = start_phase;  // any start value is fine (training starts at random)
    has_target_yaw = false;
  }

  // 29 inputs. imu quat: body orientation (w, x, y, z); gyro: body rates [rad/s]; command: vx, vy, yaw rate.
  std::array<float, kObs> observation(const Quat & imu_quat, const Vec3 & gyro, const Vec3 & command) const {
    std::array<float, kObs> obs{};
    const Vec3 g = projected_gravity(imu_quat);
    int k = 0;
    for (double v : g) obs[k++] = static_cast<float>(v);
    for (double v : gyro) obs[k++] = static_cast<float>(gyro_scale * v);
    for (double v : last_action) obs[k++] = static_cast<float>(v);
    for (int i = 0; i < 3; ++i) obs[k++] = static_cast<float>(command[i] / cmd_max[i]);
    const bool moving = command[0] != 0 || command[1] != 0 || command[2] != 0;
    const double angle = 2 * M_PI * phase;
    obs[k++] = moving ? static_cast<float>(std::sin(angle)) : 0.0f;  // stopped clock = stand still
    obs[k++] = moving ? static_cast<float>(std::cos(angle)) : 0.0f;
    return obs;
  }

  // URDF joint targets [rad], servo-ID order (action_names), before rounding to servo degrees.
  std::array<double, kAct> joint_targets(const float * action) const {
    std::array<double, kAct> q{};
    for (int i = 0; i < kAct; ++i) {
      const double a = std::clamp(static_cast<double>(action[i]), -1.0, 1.0);
      q[i] = std::clamp(home[i] + action_scale * a, lo[i], hi[i]);
    }
    return q;
  }

  // Whole servo degrees for servo IDs 1..18, as the hardware plugin computes them from joint_targets.
  std::array<int, kAct> servo_degrees(const float * action) const {
    const auto q = joint_targets(action);
    std::array<int, kAct> s{};
    for (int i = 0; i < kAct; ++i) {
      const double deg = servo_sign[i % 3] * q[i] * 180.0 / M_PI + servo_offset_deg[i % 3];
      s[i] = static_cast<int>(std::clamp(std::round(deg), -90.0, 90.0));
    }
    return s;
  }

  // End of the cycle: the action becomes "previous action", the clock moves one step.
  void advance(const float * action) {
    for (int i = 0; i < kAct; ++i) last_action[i] = std::clamp(static_cast<double>(action[i]), -1.0, 1.0);
    phase = std::fmod(phase + phase_step, 1.0);
  }

  // Heading hold (outside the policy): when moving with zero turn command, steer back to the held heading.
  Vec3 hold_heading(Vec3 command, const Quat & imu_quat) {
    const double yaw = yaw_of(imu_quat);
    const bool moving = command[0] != 0 || command[1] != 0 || command[2] != 0;
    if (command[2] != 0 || !moving) {
      target_yaw = yaw;  // turning or standing: the current heading becomes the one to hold
      has_target_yaw = true;
      return command;
    }
    if (!has_target_yaw) { target_yaw = yaw; has_target_yaw = true; }
    double err = std::fmod(target_yaw - yaw + M_PI, 2 * M_PI);
    if (err < 0) err += 2 * M_PI;
    err -= M_PI;
    command[2] = std::clamp(heading_gain * err, -cmd_max[2], cmd_max[2]);
    return command;
  }

  std::vector<std::string> action_names;
  std::vector<double> cmd_max, home, lo, hi, servo_sign, servo_offset_deg;
  double action_scale{}, gyro_scale{}, phase_step{}, control_hz{}, heading_gain{};
  std::array<double, kAct> last_action{};
  double phase{};
  double target_yaw{};
  bool has_target_yaw{false};
};

}  // namespace muto_policy
