/**
 * @file obs_builder_node.cpp
 * @brief Constructeur d'observations ("sensor fusion" du pipeline perception).
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * ARCHITECTURE
 * ════════════════════════════════════════════════════════════════════════════════
 * Machine cible : Jetson Nano (ROBOT_ROLE=BRAIN)
 *
 * Ce nœud tourne sur la Jetson et agrège les observations du Pi en une vecteur d'observation
 * standardisé pour le modèle RL (inference plugin).
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * RÔLE EXACT DU FICHIER
 * ════════════════════════════════════════════════════════════════════════════════
 * Implémente la "sensor fusion":
 *   Input 1: /joint_states [StampedJointState] → positions et vélocités des 18 servo
 *   Input 2: /imu/data [StampedImu] → orientation (quaternion), accel, gyro
 *   Input 3: /commands [Commands] → action précédente (pour feedback boucle fermée)
 *   Input 4: /navigation/goal_velocity [Twist] → consigne de naviguation du planificateur
 *
 *   Output: /observation [Observation] → vecteur [0...69] coordonnés pour le modèle RL
 *    Composé de:
 *      [0:4]     orientation quaternion (w,x,y,z)
 *      [4:7]     angular velocity gyro (rad/s)
 *      [7:10]    linear acceleration (m/s²)
 *      [10:28]   joint positions (rad) des 18 servos
 *      [28:46]   joint velocities (rad/s) des 18 servos
 *      [46:64]   last action angles (rad)
 *      [64:67]   goal velocity (m/s, rad/s)
 *      [67:70]   contact forces estimées (6 pattes) [0:1]
 *
 *   Normalisation (z-score) avec norm_stats_v3.json : (x - mean) / std
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * BUFFER LOCKFREE (ring buffer synchronisation à cycle_id)
 * ════════════════════════════════════════════════════════════════════════════════
 * Les messages /joint_states et /imu/data arrivent potentiellement avec décalage / desync.
 * Pour garantir la cohérence, try_build utilise cycle_id (64 bits, mod kBufferSize=4):
 *   idx = cycle_id % 4  → slot de buffer circulaire
 *   imu_valid_[idx] et joint_valid_[idx] tracent si les deux sont prêts au même ID
 *   Des que les deux sont syncs, obs est compilée et publiée (puis valid flags reset)
 * Compte missed_cycles_ si sync n'est pas atteint.
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * MAINTENANCE
 * ════════════════════════════════════════════════════════════════════════════════
 * • Tenir à jour [0..69] observation vector size = kObsDim
 * • Vérifier norm_stats_v3.json est chargé; fallback mean=0, std=1 si absent
 * • Contact forces (derniers champs) sont estimées grossièrement d'après IMU az
 * • Mesurer latence obs → inference pour identifier goulots
 * • Documenter les changements de layout observation si le modèle change
 */


#include "muto_control/obs_builder_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

using namespace std::chrono_literals;

namespace muto_control {

ObsBuilderNode::ObsBuilderNode() : Node("obs_builder_node") {
  // Résolution du chemin depuis la variable d'environnement WORKING_DIR
  // injectée par Docker via docker/config/.env.jetson_nano.
  const char* working_dir = std::getenv("WORKING_DIR");
  if (!working_dir || std::string(working_dir).empty()) {
    RCLCPP_FATAL(
        this->get_logger(),
        "WORKING_DIR non défini. Ce noeud doit tourner dans le container Docker. "
        "Vérifiez docker/config/.env.jetson_nano.");
    throw std::runtime_error("WORKING_DIR non défini");
  }
  std::string default_norm_stats = std::string(working_dir) + "/models/norm_stats_v3.json";
  norm_stats_path_ = declare_parameter<std::string>(
      "norm_stats_path", default_norm_stats);
  max_torque_ = static_cast<float>(declare_parameter<double>("max_torque", 1.0));
  if (!load_norm_stats()) {
    mean_.fill(0.0F);
    std_.fill(1.0F);
  }

  rclcpp::QoS qos(1);
  qos.best_effort();
  qos.durability_volatile();
  qos.history(rclcpp::HistoryPolicy::KeepLast);

  imu_sub_ = create_subscription<muto_msgs::msg::StampedImu>(
      "/imu/data", qos,
      std::bind(&ObsBuilderNode::on_imu, this, std::placeholders::_1));
  joint_sub_ = create_subscription<muto_msgs::msg::StampedJointState>(
      "/joint_states", qos,
      std::bind(&ObsBuilderNode::on_joint, this, std::placeholders::_1));
  cmd_sub_ = create_subscription<muto_msgs::msg::Commands>(
      "/commands", qos,
      std::bind(&ObsBuilderNode::on_command, this, std::placeholders::_1));
  goal_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/navigation/goal_velocity", qos,
      std::bind(&ObsBuilderNode::on_goal_velocity, this, std::placeholders::_1));
  obs_pub_ = create_publisher<muto_msgs::msg::Observation>("/observation", qos);
  missed_pub_ = create_publisher<std_msgs::msg::UInt64>("/sync/missed_cycles", qos);
  stats_timer_ = create_wall_timer(10s, std::bind(&ObsBuilderNode::publish_stats, this));
}

void ObsBuilderNode::on_imu(const muto_msgs::msg::StampedImu::SharedPtr msg) {
  const std::size_t idx = static_cast<std::size_t>(msg->cycle_id % kBufferSize);
  imu_buffer_[idx] = *msg;
  imu_valid_[idx] = true;
  try_build(msg->cycle_id);
}

void ObsBuilderNode::on_joint(const muto_msgs::msg::StampedJointState::SharedPtr msg) {
  const std::size_t idx = static_cast<std::size_t>(msg->cycle_id % kBufferSize);
  joint_buffer_[idx] = *msg;
  joint_valid_[idx] = true;
  try_build(msg->cycle_id);
}

void ObsBuilderNode::on_command(const muto_msgs::msg::Commands::SharedPtr msg) {
  last_action_ = msg->angles;
}

void ObsBuilderNode::on_goal_velocity(const geometry_msgs::msg::Twist::SharedPtr msg) {
  goal_velocity_[0] = static_cast<float>(msg->linear.x);
  goal_velocity_[1] = static_cast<float>(msg->linear.y);
  goal_velocity_[2] = static_cast<float>(msg->angular.z);
}

void ObsBuilderNode::try_build(uint64_t cycle_id) {
  const std::size_t idx = static_cast<std::size_t>(cycle_id % kBufferSize);
  if (!imu_valid_[idx] || !joint_valid_[idx]) {
    ++missed_cycles_;
    return;
  }

  const auto& imu_msg = imu_buffer_[idx];
  const auto& joint_msg = joint_buffer_[idx];
  if (imu_msg.cycle_id != cycle_id || joint_msg.cycle_id != cycle_id) {
    ++missed_cycles_;
    return;
  }

  muto_msgs::msg::Observation obs;
  obs.header.stamp = now();
  obs.cycle_id = cycle_id;
  std::fill(obs.values.begin(), obs.values.end(), 0.0F);

  obs.values[0] = static_cast<float>(imu_msg.imu.orientation.w);
  obs.values[1] = static_cast<float>(imu_msg.imu.orientation.x);
  obs.values[2] = static_cast<float>(imu_msg.imu.orientation.y);
  obs.values[3] = static_cast<float>(imu_msg.imu.orientation.z);
  obs.values[4] = static_cast<float>(imu_msg.imu.angular_velocity.x);
  obs.values[5] = static_cast<float>(imu_msg.imu.angular_velocity.y);
  obs.values[6] = static_cast<float>(imu_msg.imu.angular_velocity.z);
  obs.values[7] = static_cast<float>(imu_msg.imu.linear_acceleration.x);
  obs.values[8] = static_cast<float>(imu_msg.imu.linear_acceleration.y);
  obs.values[9] = static_cast<float>(imu_msg.imu.linear_acceleration.z);

  for (std::size_t i = 0; i < 18 && i < joint_msg.joint_state.position.size(); ++i) {
    obs.values[10 + i] = static_cast<float>(joint_msg.joint_state.position[i]);
  }
  for (std::size_t i = 0; i < 18 && i < joint_msg.joint_state.velocity.size(); ++i) {
    obs.values[28 + i] = static_cast<float>(joint_msg.joint_state.velocity[i]);
  }
  for (std::size_t i = 0; i < 18; ++i) {
    obs.values[46 + i] = last_action_[i];
  }
  obs.values[64] = goal_velocity_[0];
  obs.values[65] = goal_velocity_[1];
  obs.values[66] = goal_velocity_[2];
  obs.values[67] = estimate_contact_force(0, imu_msg);
  obs.values[68] = estimate_contact_force(1, imu_msg);
  obs.values[69] = estimate_contact_force(2, imu_msg);

  for (std::size_t i = 0; i < kObsDim; ++i) {
    const float stdev = std::max(std_[i], 1e-6F);
    const float normed = (obs.values[i] - mean_[i]) / stdev;
    obs.values[i] = std::clamp(normed, -5.0F, 5.0F);
  }

  obs_pub_->publish(obs);
  imu_valid_[idx] = false;
  joint_valid_[idx] = false;
}

void ObsBuilderNode::publish_stats() {
  std_msgs::msg::UInt64 msg;
  msg.data = missed_cycles_;
  missed_pub_->publish(msg);
}

float ObsBuilderNode::estimate_contact_force(std::size_t leg_idx, const muto_msgs::msg::StampedImu& imu_msg) const {
  (void)leg_idx;
  const float az_norm = std::clamp(static_cast<float>(imu_msg.imu.linear_acceleration.z) / 9.80665F, 0.0F, 1.0F);
  return az_norm;
}

bool ObsBuilderNode::load_norm_stats() {
  mean_.fill(0.0F);
  std_.fill(1.0F);

  std::ifstream in(norm_stats_path_);
  if (!in.good()) {
    return false;
  }

  const std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  auto parse_array = [&](const std::string& key, std::array<float, kObsDim>& out, float fallback) {
    const std::string needle = "\"" + key + "\"";
    const std::size_t kpos = content.find(needle);
    if (kpos == std::string::npos) {
      out.fill(fallback);
      return;
    }
    const std::size_t b = content.find('[', kpos);
    const std::size_t e = content.find(']', b);
    if (b == std::string::npos || e == std::string::npos || e <= b + 1) {
      out.fill(fallback);
      return;
    }
    std::stringstream ss(content.substr(b + 1, e - b - 1));
    std::string item;
    std::size_t idx = 0;
    while (std::getline(ss, item, ',') && idx < kObsDim) {
      out[idx++] = std::stof(item);
    }
    if (idx == 1) {
      out.fill(out[0]);
    } else {
      for (; idx < kObsDim; ++idx) {
        out[idx] = fallback;
      }
    }
  };

  parse_array("mean", mean_, 0.0F);
  parse_array("std", std_, 1.0F);
  return true;
}

}  // namespace muto_control

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<muto_control::ObsBuilderNode>());
  rclcpp::shutdown();
  return 0;
}
