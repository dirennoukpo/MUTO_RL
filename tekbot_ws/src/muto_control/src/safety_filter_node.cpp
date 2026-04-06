/**
 * @file safety_filter_node.cpp
 * @brief Filtre de sécurité 7 étapes : traite les commandes brutes vers les servos avec multilayers safeguards.
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * ARCHITECTURE
 * ════════════════════════════════════════════════════════════════════════════════
 * Machine cible : Jetson Nano (ROBOT_ROLE=BRAIN)
 *
 * Ce nœud s'exécute sur la Jetson et applique des filtres de sécurité multicouches
 * aux sorties du modèle RL avant de les envoyer au Pi.
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * RÔLE EXACT DU FICHIER
 * ════════════════════════════════════════════════════════════════════════════════
 * Implémente un filtre 7 étapes pour transformer:
 *   Input:  /commands_raw [Commands] → sorties brutes du modèle RL
 *   Output: /commands     [Commands] → commandes filtrées et sécurisées
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * PIPELINE FILTRAGE 7 ÉTAPES
 * ════════════════════════════════════════════════════════════════════════════════
 * Step 1 : Sanitize infinity/NaN → force zéro (evite propagation de poison)
 * Step 2 : Hard clamp joint limits [-1.57, 1.57] rad (limites mécaniques)
 * Step 3 : Velocity saturation : Δangle ≤ max_safe_velocity * dt  (evite réniscences)
 * Step 4 : Current-based fallback : si effort > max_safe_current -> revert prev output
 * Step 5 : Explicit rate limiter : lissé graduel en rad/s (8 deg/s = 0.14 rad/s)
 * Step 6 : Oscillation detector & damping : si Δ²angle > threshold -> halve delta
 * Step 7 : COM/posture stability : PostureValidator avec contact forces, tau_decay exponential
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * CRC / CHECKSUMMING (prévu pour phase 2)
 * ════════════════════════════════════════════════════════════════════════════════
 * Pour valider l'intégrité du transport /commands entre Jetson et Pi,
 * ajouter checksum CRC-16 dans la prochaine version (validate_sim_real Phase 2).
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * MAINTENANCE
 * ════════════════════════════════════════════════════════════════════════════════
 * • L'ordre des 7 étapes est CRITIQUE: modification peut créer feedback oscillants
 * • Valider empiriquement tau_decay sur vrai hardware (actuellement 0.5 s)
 * • PostureValidator.valid() implémenté en muto_control/posturevalidator.hpp
 * • Documenter tout changement de seuil (rate_limit_deg, oscillation_threshold, etc)
 */


#include "muto_control/safety_filter_node.hpp"

#include <algorithm>
#include <cmath>

namespace muto_control {

SafetyFilterNode::SafetyFilterNode() : Node("safety_filter_node") {
  min_limits_.fill(-1.57F);
  max_limits_.fill(1.57F);
  max_safe_velocity_.fill(6.0F);
  max_safe_current_.fill(1.0F);

  rate_limit_deg_ = static_cast<float>(declare_parameter<double>("rate_limit_deg", 8.0));
  oscillation_threshold_ = static_cast<float>(declare_parameter<double>("oscillation_threshold", 0.25));
  com_stability_threshold_ = static_cast<float>(declare_parameter<double>("com_stability_threshold", 0.45));
  tau_decay_ = static_cast<float>(declare_parameter<double>("tau_decay", 0.5));

  const auto vel_param = declare_parameter<std::vector<double>>("max_safe_velocity", std::vector<double>(18, 6.0));
  const auto cur_param = declare_parameter<std::vector<double>>("max_safe_current", std::vector<double>(18, 1.0));
  for (size_t i = 0; i < 18 && i < vel_param.size(); ++i) {
    max_safe_velocity_[i] = static_cast<float>(vel_param[i]);
  }
  for (size_t i = 0; i < 18 && i < cur_param.size(); ++i) {
    max_safe_current_[i] = static_cast<float>(cur_param[i]);
  }

  instability_start_ = now();

  auto qos = make_rt_qos();

  raw_sub_ = create_subscription<muto_msgs::msg::Commands>(
      "/commands_raw", qos,
      std::bind(&SafetyFilterNode::on_raw_command, this, std::placeholders::_1));
  joint_sub_ = create_subscription<muto_msgs::msg::StampedJointState>(
      "/joint_states", qos,
      std::bind(&SafetyFilterNode::on_joint_state, this, std::placeholders::_1));
  filtered_pub_ = create_publisher<muto_msgs::msg::Commands>("/commands", qos);
}

rclcpp::QoS SafetyFilterNode::make_rt_qos() const {
  rclcpp::QoS qos(1);
  qos.best_effort();
  qos.durability_volatile();
  qos.history(rclcpp::HistoryPolicy::KeepLast);
  return qos;
}

void SafetyFilterNode::on_raw_command(const muto_msgs::msg::Commands::SharedPtr msg) {
  auto out = filter(*msg);
  filtered_pub_->publish(out);
}

void SafetyFilterNode::on_joint_state(const muto_msgs::msg::StampedJointState::SharedPtr msg) {
  for (size_t i = 0; i < 18 && i < msg->joint_state.velocity.size(); ++i) {
    current_velocity_[i] = static_cast<float>(msg->joint_state.velocity[i]);
  }
  for (size_t i = 0; i < 18 && i < msg->joint_state.effort.size(); ++i) {
    current_effort_[i] = static_cast<float>(msg->joint_state.effort[i]);
  }
}

muto_msgs::msg::Commands SafetyFilterNode::filter(const muto_msgs::msg::Commands &raw) {
  auto filtered = raw;

  // Step 1: sanitize non-finite values.
  for (std::size_t i = 0; i < filtered.angles.size(); ++i) {
    if (!std::isfinite(filtered.angles[i])) {
      filtered.angles[i] = 0.0F;
    }
  }

  // Step 2: hard clamp to joint limits.
  for (std::size_t i = 0; i < filtered.angles.size(); ++i) {
    filtered.angles[i] = std::clamp(filtered.angles[i], min_limits_[i], max_limits_[i]);
  }

  // Step 3: velocity saturation against max_safe_velocity.
  const float dt = 0.01F;
  for (std::size_t i = 0; i < filtered.angles.size(); ++i) {
    const float max_step = max_safe_velocity_[i] * dt;
    const float delta = filtered.angles[i] - previous_output_[i];
    filtered.angles[i] = previous_output_[i] + std::clamp(delta, -max_step, max_step);
  }

  // Step 4: current-based saturation fallback.
  for (std::size_t i = 0; i < filtered.angles.size(); ++i) {
    if (std::fabs(current_effort_[i]) > max_safe_current_[i]) {
      filtered.angles[i] = previous_output_[i];
    }
  }

  // Step 5: explicit rate limiter in rad from rate_limit_deg.
  const float rate_limit_rad = rate_limit_deg_ * static_cast<float>(M_PI / 180.0);
  for (std::size_t i = 0; i < filtered.angles.size(); ++i) {
    const float max_delta = rate_limit_rad * dt;
    const float delta = filtered.angles[i] - previous_output_[i];
    filtered.angles[i] = previous_output_[i] + std::clamp(delta, -max_delta, max_delta);
  }

  // Step 6: oscillation detector and damping.
  for (std::size_t i = 0; i < filtered.angles.size(); ++i) {
    const float delta = filtered.angles[i] - previous_output_[i];
    if (std::fabs(delta - previous_delta_[i]) > oscillation_threshold_) {
      filtered.angles[i] = previous_output_[i] + 0.5F * delta;
    }
    previous_delta_[i] = delta;
  }

  // Step 7: COM/posture stability with exponential decay.
  std::array<float, 6> contact_forces{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  for (size_t i = 0; i < 6; ++i) {
    contact_forces[i] = std::clamp(std::fabs(current_effort_[i]), 0.0F, 1.0F);
  }
  const bool stable = posture_validator_.valid(filtered.angles, contact_forces, com_stability_threshold_);
  if (!stable) {
    if (!unstable_) {
      unstable_ = true;
      instability_start_ = now();
    }
    const float t = static_cast<float>((now() - instability_start_).seconds());
    const float alpha = std::clamp(1.0F - std::exp(-t / std::max(tau_decay_, 1e-3F)), 0.05F, 0.3F);
    for (std::size_t i = 0; i < filtered.angles.size(); ++i) {
      filtered.angles[i] = previous_output_[i] + alpha * (filtered.angles[i] - previous_output_[i]);
    }
  } else {
    unstable_ = false;
  }

  for (std::size_t i = 0; i < filtered.angles.size(); ++i) {
    previous_output_[i] = filtered.angles[i];
  }

  return filtered;
}

}  // namespace muto_control

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<muto_control::SafetyFilterNode>());
  rclcpp::shutdown();
  return 0;
}
