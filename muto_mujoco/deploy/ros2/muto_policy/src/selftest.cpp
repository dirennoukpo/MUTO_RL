// Replays deploy/test_vectors.json (made in the simulator) through the C++ code and checks every step:
// the 29 inputs, the ONNX output and the servo degrees. Run it on the robot before the first walk:
//
//   ros2 run muto_policy muto_policy_selftest <policy.onnx> <test_vectors.json>
//
// Exit code 0 = the robot computes exactly what the simulator computed.
#include <cstdio>
#include <fstream>

#include "muto_policy/onnx_policy.hpp"
#include "muto_policy/policy_io.hpp"

using namespace muto_policy;

int main(int argc, char ** argv) {
  if (argc != 3) {
    std::fprintf(stderr, "usage: muto_policy_selftest <policy.onnx> <test_vectors.json>\n");
    return 2;
  }
  OnnxPolicy policy(argv[1]);
  PolicyIO io(policy.interface_json);
  std::ifstream f(argv[2]);
  const auto tv = nlohmann::json::parse(f);

  const auto & steps = tv.at("steps");
  io.reset(steps.at(0).at("phase").get<double>());
  double worst_obs = 0, worst_act = 0;
  int servo_mismatch = 0;
  for (const auto & s : steps) {
    const auto q = s.at("imu_quat_wxyz").get<Quat>();
    const auto g = s.at("gyro_rad_s").get<Vec3>();
    const auto c = s.at("command").get<Vec3>();
    const auto obs = io.observation(q, g, c);
    const auto expected_obs = s.at("obs").get<std::vector<float>>();
    for (int i = 0; i < kObs; ++i) worst_obs = std::max(worst_obs, double(std::abs(obs[i] - expected_obs[i])));

    const auto action = policy(obs);
    const auto expected_act = s.at("action").get<std::vector<float>>();
    for (int i = 0; i < kAct; ++i) worst_act = std::max(worst_act, double(std::abs(action[i] - expected_act[i])));

    // servo degrees from the *expected* action, so a tiny ONNX difference can't flip a rounding
    const auto servo = io.servo_degrees(expected_act.data());
    if (servo != s.at("servo_deg").get<std::array<int, kAct>>()) ++servo_mismatch;
    io.advance(expected_act.data());
  }

  int heading_mismatch = 0;
  for (const auto & h : tv.at("heading_hold")) {
    io.target_yaw = h.at("target_yaw").get<double>();
    io.has_target_yaw = true;
    const auto out = io.hold_heading(h.at("command").get<Vec3>(), h.at("imu_quat_wxyz").get<Quat>());
    const auto expected = h.at("command_out").get<Vec3>();
    for (int i = 0; i < 3; ++i) heading_mismatch += std::abs(out[i] - expected[i]) > 1e-9;
  }

  const bool ok = worst_obs < 1e-5 && worst_act < 1e-4 && servo_mismatch == 0 && heading_mismatch == 0;
  std::printf("%zu steps: max input error %.1e (limit 1e-5), max action error %.1e (limit 1e-4), "
              "servo degree mismatches %d, heading-hold mismatches %d\n%s\n",
              steps.size(), worst_obs, worst_act, servo_mismatch, heading_mismatch, ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
