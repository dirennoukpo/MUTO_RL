/**
 * @file watchdog_node.cpp
 * @brief Superviseur de sécurité : détecte les timeouts et anomalies capteurs.
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * ARCHITECTURE
 * ════════════════════════════════════════════════════════════════════════════════
 * Machine cible : Raspberry Pi (ROBOT_ROLE=DRIVER)
 *
 * Ce nœud tourne sur le Pi et supervise en PERMANENCE l'invété de sécurité du système.
 * Il agit comme un "watchdog" qui estime l'état global et force des transitions dégraderées (SAFE
 * ou EMERGENCY) si des conditions anormales sont détectées.
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * RÔLE EXACT DU FICHIER
 * ════════════════════════════════════════════════════════════════════════════════
 * Supervise les trois canaux critiques :
 *   1. /commands     : detection timeout > command_timeout_ms_
 *   2. /jetson/heartbeat : detection timeout > heartbeat_timeout_ms_ (Jetson distante)
 *   3. /commands_dry_run : validation préliminaire avant transition vers RL_ACTIVE
 * Dàs qu'une condition d'erreur est détectée, force une transition à l'état dégradé :
 *   • timeout commandes        → SAFE (arrêt moteurs, pas de nouvelle commande)
 *   • timeout heartbeat        → SAFE (Jetson déconnectée, perte du cerveau)
 *   • dry_run non validé       → SAFE (test préalable échoué)
 *   • chute détectée (accel) → EMERGENCY (escalade critique)
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * PAR AMÈTRES DE TIMEOUT (réglables via launch.py)
 * ════════════════════════════════════════════════════════════════════════════════
 * command_age_max_ms_       : 20.0 ms    (max age d'une trame /commands acceptée)
 * command_timeout_ms_       : 50.0 ms    (max delai sans /commands before SAFE)
 * heartbeat_timeout_ms_     : 500.0 ms   (max delai sans /jetson/heartbeat before SAFE)
 * fall_accel_threshold_ms2_ : 29.4 m/s²  (norme accel > seuil → chute détectée)
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * VÉRIFICATION ET DIAGNOSTIC (Heartbeat PÉRIODIQUE)
 * ════════════════════════════════════════════════════════════════════════════════
 * Timer periodié (10 ms) qui exécute check_safety():
 *   • Calcul du delta temps depuis dernier message /commands et /jetson/heartbeat
 *   • Comparaison vs seuils de timeout
 *   • Emission du diagnostic state et raison (throttled logs)
 *   • Publication de mode SAFE ou RL_ACTIVE selon les conditions
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * MAINTENANCE
 * ════════════════════════════════════════════════════════════════════════════════
 * • Les timeouts doivent être empiriquement validés sur le robot
 * • Augmenter heartbeat_timeout_ms_ si Jetson a besoin de plus de 500 ms pour répondre
 * • Documenter tout changement de threshold de chute (fall_accel_threshold_ms2_)
 * • Conh de /commands_dry_run nécessaire AVANT à chaque RL_ACTIVE (anti-regression)
 */


#include "muto_hardware/watchdog_node.hpp"

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace muto_hardware {

// Configure les seuils de surete et souscrit aux flux critiques de supervision.
WatchdogNode::WatchdogNode() : Node("watchdog_node") {
  command_max_age_ms_ = declare_parameter<double>("command_age_max_ms", 20.0);
  command_timeout_ms_ = declare_parameter<double>("command_timeout_ms", 50.0);
  heartbeat_timeout_ms_ = declare_parameter<double>("heartbeat_timeout_ms", 500.0);
  fall_accel_threshold_ms2_ = declare_parameter<double>("fall_accel_threshold_ms2", 29.4);

  last_command_time_ = now();
  last_heartbeat_time_ = now();
  last_dry_run_time_ = now();

  auto qos = make_rt_qos();

  command_sub_ = create_subscription<muto_msgs::msg::Commands>(
      "/commands", qos,
      std::bind(&WatchdogNode::on_command, this, std::placeholders::_1));

  dry_run_sub_ = create_subscription<muto_msgs::msg::Commands>(
      "/commands_dry_run", qos,
      std::bind(&WatchdogNode::on_dry_run_command, this, std::placeholders::_1));

  heartbeat_sub_ = create_subscription<std_msgs::msg::String>(
      "/jetson/heartbeat", qos,
      std::bind(&WatchdogNode::on_heartbeat, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", qos,
      std::bind(&WatchdogNode::on_imu, this, std::placeholders::_1));

  mode_pub_ = create_publisher<std_msgs::msg::String>("/system_mode", qos);
  // 10 ms: cadence suffisamment rapide pour detecter timeouts sans surcharger CPU.
  timer_ = create_wall_timer(10ms, std::bind(&WatchdogNode::check_safety, this));
}

// QoS best_effort/volatile pour rester coherent avec les topics temps reel.
rclcpp::QoS WatchdogNode::make_rt_qos() const {
  rclcpp::QoS qos(1);
  qos.best_effort();
  qos.durability_volatile();
  qos.history(rclcpp::HistoryPolicy::KeepLast);
  return qos;
}

// Accepte uniquement les commandes recentes; ignore les commandes stale.
void WatchdogNode::on_command(const muto_msgs::msg::Commands::SharedPtr msg) {
  const auto age_ms = (now() - rclcpp::Time(msg->timestamp_sent)).seconds() * 1000.0;
  if (age_ms <= command_max_age_ms_) {
    last_command_time_ = now();
  }
}

// Validation explicite du canal dry-run, indispensable avant ouverture RL.
void WatchdogNode::on_dry_run_command(const muto_msgs::msg::Commands::SharedPtr msg) {
  const auto age_ms = (now() - rclcpp::Time(msg->timestamp_sent)).seconds() * 1000.0;
  if (age_ms <= command_max_age_ms_) {
    last_dry_run_time_ = now();
    dry_run_valid_.store(true, std::memory_order_release);
  }
}

// Heartbeat minimal: le contenu n'est pas critique, seul l'age compte.
void WatchdogNode::on_heartbeat(const std_msgs::msg::String::SharedPtr msg) {
  (void)msg;
  last_heartbeat_time_ = now();
}

// Heuristique anti-chute: norme accel > seuil => EMERGENCY immediat.
void WatchdogNode::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  const double ax = msg->linear_acceleration.x;
  const double ay = msg->linear_acceleration.y;
  const double az = msg->linear_acceleration.z;
  const double norm = std::sqrt(ax * ax + ay * ay + az * az);
  if (norm > fall_accel_threshold_ms2_) {
    publish_mode("EMERGENCY", "fall_detected");
  }
}

// Publication du mode degrade avec trace throttlee pour eviter le spam logs.
void WatchdogNode::publish_mode(const std::string& mode, const std::string& reason) {
  std_msgs::msg::String msg;
  msg.data = mode;
  mode_pub_->publish(msg);
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "watchdog set mode=%s reason=%s", mode.c_str(), reason.c_str());
}

// Evaluation periodique des invariants de surete (dry-run, commandes, heartbeat).
void WatchdogNode::check_safety() {
  const auto dt_cmd_ms = (now() - last_command_time_).seconds() * 1000.0;
  const auto dt_hb_ms = (now() - last_heartbeat_time_).seconds() * 1000.0;

  if (!dry_run_valid_.load(std::memory_order_acquire)) {
    publish_mode("SAFE", "dry_run_not_validated");
    return;
  }
  if (dt_cmd_ms > command_timeout_ms_) {
    publish_mode("SAFE", "commands_timeout");
    return;
  }
  if (dt_hb_ms > heartbeat_timeout_ms_) {
    publish_mode("SAFE", "heartbeat_timeout");
  }
}

}  // namespace muto_hardware

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<muto_hardware::WatchdogNode>());
  rclcpp::shutdown();
  return 0;
}
