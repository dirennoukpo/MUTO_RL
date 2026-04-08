/*
 * Documentation FR: src/muto_hardware/include/muto_hardware/watchdog_node.hpp
 * Role: composant C++ du pipeline MUTO RS avec exigences de latence et surete.
 * Details: ce fichier implemente des fonctions critiques pour la perception, le controle ou le hardware.
 * Contraintes: eviter les regressions de timing, conserver les unites physiques et la coherence des interfaces.
 * Maintenance: documenter toute hypothese metier (seuils, conversions, heuristiques) lors des modifications.
 */

/**
 * @file watchdog_node.hpp
 * @brief Supervision securite temps quasi-reel (timeouts, chute, validation dry-run).
 *
 * Le watchdog est la couche de defense independante du bridge hardware.
 * Il force SAFE/EMERGENCY si les invariants de communication ne sont plus respectes.
 */
#pragma once

#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "muto_msgs/msg/commands.hpp"
#include "muto_msgs/msg/stamped_imu.hpp"

namespace muto_hardware {

/**
 * @class WatchdogNode
 * @brief Noeud de surete qui surveille age commande, heartbeat et accelerations IMU.
 */
class WatchdogNode : public rclcpp::Node {
public:
  /** @brief Initialise les abonnements critiques et le timer de supervision. */
  WatchdogNode();

private:
  /** @brief Callback /commands : valide uniquement les commandes fraiches (<= 20 ms). */
  void on_command(const muto_msgs::msg::Commands::SharedPtr msg);
  /** @brief Callback /commands_dry_run : autorise l'entree dry-run si flux present. */
  void on_dry_run_command(const muto_msgs::msg::Commands::SharedPtr msg);
  /** @brief Callback /jetson/heartbeat : met a jour la derniere preuve de vie Jetson. */
  void on_heartbeat(const std_msgs::msg::String::SharedPtr msg);
  /** @brief Callback /imu/data : declenche EMERGENCY si norme accel > seuil de chute. */
  void on_imu(const muto_msgs::msg::StampedImu::SharedPtr msg);
  /** @brief Evaluation periodique des timeout et publication de mode degrade. */
  void check_safety();
  /** @brief QoS critique coherent avec les flux best_effort + volatile + keep_last(1). */
  rclcpp::QoS make_rt_qos() const;
  /** @brief Publie un mode systeme et trace la raison operateur. */
  void publish_mode(const std::string& mode, const std::string& reason);

  rclcpp::Time last_command_time_;  ///< Dernier instant commande valide.
  rclcpp::Time last_heartbeat_time_;  ///< Dernier instant heartbeat recu.
  rclcpp::Time last_dry_run_time_;  ///< Dernier instant commande dry-run valide.

  double command_max_age_ms_{20.0};  ///< Age max d'une commande acceptee (ms).
  double command_timeout_ms_{50.0};  ///< Timeout absence commande avant SAFE (ms).
  double heartbeat_timeout_ms_{500.0};  ///< Timeout heartbeat avant SAFE (ms).
  double fall_accel_threshold_ms2_{29.4};  ///< Seuil chute (3g) en m/s2.
  bool require_dry_run_validation_{true};  ///< Exige un flux /commands_dry_run avant d'autoriser NORMAL.
  bool require_commands_stream_{true};  ///< Exige un flux /commands frais avant d'autoriser NORMAL.
  bool require_heartbeat_stream_{true};  ///< Exige un heartbeat Jetson frais avant d'autoriser NORMAL.
  std::atomic<bool> dry_run_valid_{false};  ///< Vrai si le flux dry_run est effectivement observe.

  rclcpp::Subscription<muto_msgs::msg::Commands>::SharedPtr command_sub_;
  rclcpp::Subscription<muto_msgs::msg::Commands>::SharedPtr dry_run_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<muto_msgs::msg::StampedImu>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace muto_hardware
