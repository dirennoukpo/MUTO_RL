/*
 * Documentation FR: src/muto_control/include/muto_control/safety_filter_node.hpp
 * Role: composant C++ du pipeline MUTO RS avec exigences de latence et surete.
 * Details: ce fichier implemente des fonctions critiques pour la perception, le controle ou le hardware.
 * Contraintes: eviter les regressions de timing, conserver les unites physiques et la coherence des interfaces.
 * Maintenance: documenter toute hypothese metier (seuils, conversions, heuristiques) lors des modifications.
 */

#pragma once

#include <array>

#include "muto_control/posture_validator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "muto_msgs/msg/commands.hpp"
#include "muto_msgs/msg/stamped_joint_state.hpp"

namespace muto_control {

class SafetyFilterNode : public rclcpp::Node {
public:
  SafetyFilterNode();

private:
  void on_raw_command(const muto_msgs::msg::Commands::SharedPtr msg);
  void on_joint_state(const muto_msgs::msg::StampedJointState::SharedPtr msg);
  muto_msgs::msg::Commands filter(const muto_msgs::msg::Commands &raw);
  rclcpp::QoS make_rt_qos() const;

  PostureValidator posture_validator_;
  std::array<float, 18> min_limits_{};
  std::array<float, 18> max_limits_{};
  std::array<float, 18> max_safe_velocity_{};
  std::array<float, 18> max_safe_current_{};
  std::array<float, 18> previous_output_{};
  std::array<float, 18> previous_delta_{};
  std::array<float, 18> current_velocity_{};
  std::array<float, 18> current_effort_{};

  float rate_limit_deg_{8.0F};
  float oscillation_threshold_{0.25F};
  float com_stability_threshold_{0.45F};
  float tau_decay_{0.5F};
  rclcpp::Time instability_start_;
  bool unstable_{false};

  rclcpp::Subscription<muto_msgs::msg::Commands>::SharedPtr raw_sub_;
  rclcpp::Subscription<muto_msgs::msg::StampedJointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<muto_msgs::msg::Commands>::SharedPtr filtered_pub_;
};

}  // namespace muto_control
