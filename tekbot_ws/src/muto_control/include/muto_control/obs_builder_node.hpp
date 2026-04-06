/*
 * Documentation FR: src/muto_control/include/muto_control/obs_builder_node.hpp
 * Role: composant C++ du pipeline MUTO RS avec exigences de latence et surete.
 * Details: ce fichier implemente des fonctions critiques pour la perception, le controle ou le hardware.
 * Contraintes: eviter les regressions de timing, conserver les unites physiques et la coherence des interfaces.
 * Maintenance: documenter toute hypothese metier (seuils, conversions, heuristiques) lors des modifications.
 */

#pragma once

#include <array>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "muto_msgs/msg/commands.hpp"
#include "muto_msgs/msg/observation.hpp"
#include "muto_msgs/msg/stamped_imu.hpp"
#include "muto_msgs/msg/stamped_joint_state.hpp"

namespace muto_control {

class ObsBuilderNode : public rclcpp::Node {
public:
  ObsBuilderNode();

private:
  static constexpr std::size_t kBufferSize = 256;
  static constexpr std::size_t kObsDim = 70;

  void on_imu(const muto_msgs::msg::StampedImu::SharedPtr msg);
  void on_joint(const muto_msgs::msg::StampedJointState::SharedPtr msg);
  void on_command(const muto_msgs::msg::Commands::SharedPtr msg);
  void on_goal_velocity(const geometry_msgs::msg::Twist::SharedPtr msg);
  void try_build(uint64_t cycle_id);
  void publish_stats();
  bool load_norm_stats();
  float estimate_contact_force(std::size_t leg_idx, const muto_msgs::msg::StampedImu& imu_msg) const;

  std::array<muto_msgs::msg::StampedImu, kBufferSize> imu_buffer_;
  std::array<muto_msgs::msg::StampedJointState, kBufferSize> joint_buffer_;
  std::array<bool, kBufferSize> imu_valid_{};
  std::array<bool, kBufferSize> joint_valid_{};
  std::array<float, 18> last_action_{};
  std::array<float, 3> goal_velocity_{};
  std::array<float, kObsDim> mean_{};
  std::array<float, kObsDim> std_{};
  float max_torque_{1.0F};

  uint64_t missed_cycles_{0};
  std::string norm_stats_path_;

  rclcpp::Subscription<muto_msgs::msg::StampedImu>::SharedPtr imu_sub_;
  rclcpp::Subscription<muto_msgs::msg::StampedJointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<muto_msgs::msg::Commands>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr goal_sub_;
  rclcpp::Publisher<muto_msgs::msg::Observation>::SharedPtr obs_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr missed_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

}  // namespace muto_control
