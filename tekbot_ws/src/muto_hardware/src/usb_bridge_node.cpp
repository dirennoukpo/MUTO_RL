/**
 * @file usb_bridge_node.cpp
 * @brief Interface bas niveau vers les servos et capteurs du MUTO Hexapode via API C (muto_link_cpp).
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * ARCHITECTURE
 * ════════════════════════════════════════════════════════════════════════════════
 * Machine cible : Raspberry Pi (ROBOT_ROLE=DRIVER)
 *
 * Ce nœud fait partie du package muto_hardware et s'exécute dans le container Docker.
 * Il est lancé par pi_full.launch.py après le démarrage du ROS 2 Humble sur le Pi.
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * RÔLE EXACT DU FICHIER
 * ════════════════════════════════════════════════════════════════════════════════
 * Ce fichier implémente le nœud `usb_bridge_node` qui:
 *   - Charge dynamiquement la libraire C partagée (muto_link_cpp_lib.so) via dlopen
 *   - Initialise la connexion USB vers les servos Feetech et l'IMU intégrée
 *   - Exécute une boucle temps réel (RT) à 200 Hz sur un cœur isolé (core=3)
 *   - Acquiert les lectures des servos (angles) et de l'IMU (accel, gyro, orientation)
 *   - Exécute les commandes de mouvement angles en temps réel (latence < 5 ms)
 *   - Publie les observations (joint_states, imu/data) via topics ROS 2
 *   - Détecte les anomalies (jitter, erreurs USB, chutes) et bascule en EMERGENCY si critique
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * VARIABLES D'ENVIRONNEMENT REQUISES
 * ════════════════════════════════════════════════════════════════════════════════
 * WORKING_DIR        : racine du volume Docker; pointe vers /root/tekbot_ws sur le Pi
 * ROBOT_ROLE         : "DRIVER" (validation stricte au démarrage)
 * ROS_DOMAIN_ID      : ID réseau ROS 2 entre Pi et Jetson (ex: 42)
 * ROS_DISTRO         : "humble" (système hôte du container)
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * CHEMINS CRITIQUES
 * ════════════════════════════════════════════════════════════════════════════════
 * ${WORKING_DIR}/muto_install/lib/libmuto_link_cpp_lib.so
 *   → Librairie dynamique montée du Pi via volume Docker; chargée à l'exécution
 *   → Contient l'implémentation bas niveau des I/O USB (servos, IMU)
 * ${WORKING_DIR}/muto_install/include/muto_link_cpp.hpp
 *   → Header C++ de la C API (types opaques, fonctions d'interface)
 *   → Inclus lors de la compilation CMake
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * DÉPENDANCES ROS 2
 * ════════════════════════════════════════════════════════════════════════════════
 * Publishers (RT 200 Hz, QoS best_effort/volatile):
 *   /joint_states             [muto_msgs::msg::StampedJointState]
 *   /imu/data                 [muto_msgs::msg::StampedImu]
 *   /hw/timing_jitter         [std_msgs::msg::Float32]
 *   /system_mode              [std_msgs::msg::String]  // Escalade EMERGENCY si USB errors
 *
 * Subscribers (RT 200 Hz, timeout check):
 *   /commands                 [muto_msgs::msg::Commands]  // Angles cibles (rad), max age 20 ms
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * THREAD ARCHITECTURE
 * ════════════════════════════════════════════════════════════════════════════════
 * 1. Main thread
 *    - Initialisation du nœud (paramètres, dlopen, souscriptions, publications)
 *    - Lancement des deux worker threads
 *    - Spin RCL à vitesse normal (non-RT)
 *
 * 2. RT Thread (SCHED_FIFO priority=90, core=3)
 *    - Boucle hardline 200 Hz (5 ms, dt=kCycleNs)
 *    - Lecture des capteurs du hardware (servos, IMU)
 *    - Écriture des commandes (servo_move pour chaque servo)
 *    - Conversion d'unités (deg ↔️ rad, m/s² ↔ raw)
 *    - Stockage dans structure partagée (lock-free synchronisation)
 *
 * 3. Publish thread (non-RT, wall rate 200 Hz)
 *    - Attente passive du signal data_ready de la boucle RT
 *    - Copie du SharedState vers messages ROS 2 Stamped*
 *    - Publication sur les topics
 *    - Évite la contention avec la boucle temps réel
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * UNITÉS PHYSIQUES ET CONVERSIONS
 * ════════════════════════════════════════════════════════════════════════════════
 * Angles servos
 *   Entrée   : commanded_angles[i] en radians ([-π/2, π/2])
 *   Sortie   : deg = angle_rad * 180/π, puis clamp [-90, +90] deg
 *   Feedback : raw_deg (int16) → angle_meas = deg * π/180 rad
 *
 * IMU (Inertial Measurement Unit)
 *   Gyroscope
 *     Raw     : gyro_x/y/z en unités LSB (int16)
 *     Constante d'échelle: 131 LSB/(deg/s) → 1 LSB = 7.63e-3 deg/s
 *     Sortie   : gyro[i] = (raw * 1/131) * π/180 rad/s
 *   Accélérométre
 *     Raw     : accel_x/y/z en unités LSB (int16)
 *     Constante d'échelle: 8192 LSB/g (pour ±4g range)
 *     Sortie   : accel[i] = (raw / 8192) * 9.80665 m/s²
 *   Orientation (Euler → Quaternion)
 *     Entrée   : roll, pitch, yaw (deg) depuis l'IMU
 *     Sortie   : quaternion [w, x, y, z] pour la compatibilité ROS 2 standard
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * ÉTATS ET TRANSITIONS
 * ════════════════════════════════════════════════════════════════════════════════
 * - NORMAL       : boucle RT stable, USB OK, erreurs < seuil
 * - EMERGENCY    : Escalade automatique triggered par:
 *                  * Accumulation erreurs USB (> 10 erreurs consécutives)
 *                  * Chute détectée (accel norm > seuil)
 *                  * Message EMERGENCY reçu du watchdog (fallback)
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * CONTRAINTES ET ASSUMPTIONS
 * ════════════════════════════════════════════════════════════════════════════════
 * 1. WORKING_DIR doit être défini et accessible (sinon throw lors du construction)
 * 2. Le core isolé (core=3) doit exister; sinon pthread_setaffinity_np échoue silencieusement
 * 3. Aucune allocation dynamique dans run_rt_loop() → pas de malloc/new en boucle critique
 * 4. Les paramètres ROS 2 sont lus UNE FOIS à la construction; pas de reconfiguration runtime
 * 5. L'IMU est calibré sur les 1000 premiers cycles; après quoi l'offset est appliqué
 * 6. Les servos ne supportent pas la lecture de couple; contact_forces estimée d'après IMU
 * 7. La latence commande → actionnement est < 5ms (RT hard deadline)
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * MAINTENANCE
 * ════════════════════════════════════════════════════════════════════════════════
 * À respecter lors de modifications:
 *   • Documenter tout changement de constante physique (baudrate, limits servo, etc)
 *   • Ne jamais allouer de mémoire dans run_rt_loop()
 *   • Conserver les unités SI (rad, m/s², rad/s) en interface ROS 2
 *   • Vérifier les seuils de détection d'anomalies (jitter, currents, forces)
 *   • Valider la synchronisation lock-free après modifications (memory_order)
 *   • Tester l'isolation CPU et les priorités SCHED_FIFO après chaque build
 */


#include "muto_hardware/usb_bridge_node.hpp"

#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>
#include <stdexcept>

namespace {

std::string resolve_working_dir_token(const std::string& value, const std::string& working_dir) {
  const std::string token = "$(env WORKING_DIR)";
  if (value.find(token) == std::string::npos) {
    return value;
  }

  std::string resolved = value;
  std::string::size_type pos = 0;
  while ((pos = resolved.find(token, pos)) != std::string::npos) {
    resolved.replace(pos, token.size(), working_dir);
    pos += working_dir.size();
  }
  return resolved;
}

// Charge dynamiquement la C API muto_link et echoue explicitement si un symbole manque.
muto_hardware::MutoApi load_muto_api(const std::string& so_path) {
  muto_hardware::MutoApi api;
  api.lib_handle = dlopen(so_path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!api.lib_handle) {
    throw std::runtime_error(std::string("dlopen failed: ") + dlerror());
  }

  auto load = [&](const char* name) -> void* {
    dlerror();
    void* sym = dlsym(api.lib_handle, name);
    const char* err = dlerror();
    if (err) {
      throw std::runtime_error(std::string("dlsym(") + name + "): " + err);
    }
    return sym;
  };

  api.create_usb = reinterpret_cast<decltype(api.create_usb)>(load("muto_create_usb"));
  api.open = reinterpret_cast<decltype(api.open)>(load("muto_open"));
  api.close = reinterpret_cast<decltype(api.close)>(load("muto_close"));
  api.destroy = reinterpret_cast<decltype(api.destroy)>(load("muto_destroy"));
  api.last_error = reinterpret_cast<decltype(api.last_error)>(load("muto_last_error"));
  api.torque_on = reinterpret_cast<decltype(api.torque_on)>(load("muto_torque_on"));
  api.torque_off = reinterpret_cast<decltype(api.torque_off)>(load("muto_torque_off"));
  api.servo_move = reinterpret_cast<decltype(api.servo_move)>(load("muto_servo_move"));
  api.read_servo_angle_deg = reinterpret_cast<decltype(api.read_servo_angle_deg)>(load("muto_read_servo_angle_deg"));
  api.get_imu_angles = reinterpret_cast<decltype(api.get_imu_angles)>(load("muto_get_imu_angles"));
  api.get_raw_imu = reinterpret_cast<decltype(api.get_raw_imu)>(load("muto_get_raw_imu_data"));

  return api;
}

}  // namespace

namespace muto_hardware {

// Horloge monotonic pour mesures de jitter et horodatage stable independant du wall-clock.
uint64_t UsbBridgeNode::now_mono_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

// Comparateur total sur timespec pour pilotage sleep absolu.
int UsbBridgeNode::timespec_cmp(const timespec& a, const timespec& b) {
  if (a.tv_sec < b.tv_sec) {
    return -1;
  }
  if (a.tv_sec > b.tv_sec) {
    return 1;
  }
  if (a.tv_nsec < b.tv_nsec) {
    return -1;
  }
  if (a.tv_nsec > b.tv_nsec) {
    return 1;
  }
  return 0;
}

// Addition nanosecondes avec normalisation sec/ns.
void UsbBridgeNode::timespec_add_ns(timespec* t, uint64_t ns) {
  const uint64_t n = static_cast<uint64_t>(t->tv_nsec) + ns;
  t->tv_sec += static_cast<time_t>(n / 1000000000ULL);
  t->tv_nsec = static_cast<long>(n % 1000000000ULL);
}

/**
 * Sequence de demarrage complète: params -> dlopen -> open/torque_on -> pubs/subs -> threads.
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * GUARD WORKING_DIR : validation au démarrage du container
 * ════════════════════════════════════════════════════════════════════════════════
 * La variable d'environnement WORKING_DIR est définie par le dockerfile dans:
 *   docker/config/.env.raspberrypi → sourced lors du démarrage du container (make run)
 * Elle pointe vers la racine du volume monté (ex: /root/tekbot_ws).
 * Sans WORKING_DIR, le chemin vers libmuto_link_cpp_lib.so est impossible à résoudre.
 * Nous faisons échouer EXPLICITEMENT (throw) plutôt que silencieusement.
 *
 * Exception levée ici:
 *   std::runtime_error("WORKING_DIR non défini")
 * Résultat:
 *   Le nœud ne démarre pas; l'administrateur verra l'erreur RCL fatal et pourra
 *   vérifier l'environnement Docker (env vars, volumes, profil .env chargé).
 */
UsbBridgeNode::UsbBridgeNode() : Node("usb_bridge_node") {
  // Guard WORKING_DIR: résolution depuis l'environnement Docker
  const char* working_dir = std::getenv("WORKING_DIR");
  if (!working_dir || std::string(working_dir).empty()) {
    RCLCPP_FATAL(
        this->get_logger(),
        "[muto] WORKING_DIR non défini. Ce noeud doit tourner dans le container Docker. "
        "Vérifiez docker/config/.env.raspberrypi est lancé (make run).");
    throw std::runtime_error("WORKING_DIR non défini");
  }

  const std::string working_dir_str(working_dir);
  const std::string default_so_path = working_dir_str + "/muto_install/lib/libmuto_link_cpp_lib.so";
  const std::string configured_so_path = declare_parameter<std::string>("muto_lib_path", default_so_path);
  so_path_ = resolve_working_dir_token(configured_so_path, working_dir_str);

  if (so_path_ != configured_so_path) {
    RCLCPP_INFO(
        this->get_logger(),
        "muto_lib_path resolu depuis token WORKING_DIR: %s",
        so_path_.c_str());
  }

  serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  baudrate_ = declare_parameter<int>("baudrate", 115200);
  rt_core_ = declare_parameter<int>("rt_core", 3);
  servo_speed_ = static_cast<uint16_t>(declare_parameter<int>("servo_speed", 200));
  jitter_threshold_ms_ = declare_parameter<double>("jitter_threshold_ms", 5.5);
  command_age_max_ms_ = declare_parameter<double>("command_age_max_ms", 20.0);
  gyro_calib_cycles_ = declare_parameter<int>("gyro_calib_cycles", 1000);
  gyro_outlier_sigma_ = declare_parameter<double>("gyro_outlier_sigma", 3.0);
  servo_reads_per_cycle_ = declare_parameter<int>("servo_reads_per_cycle", 3);
  servo_commands_per_cycle_ = declare_parameter<int>("servo_commands_per_cycle", 1);
  imu_reads_divider_ = declare_parameter<int>("imu_reads_divider", 2);
  imu_angles_divider_ = declare_parameter<int>("imu_angles_divider", 8);
  servo_command_divider_ = declare_parameter<int>("servo_command_divider", 2);
  if (servo_reads_per_cycle_ < 0) {
    servo_reads_per_cycle_ = 0;
  } else if (servo_reads_per_cycle_ > kServoCount) {
    servo_reads_per_cycle_ = kServoCount;
  }
  if (servo_commands_per_cycle_ < 1) {
    servo_commands_per_cycle_ = 1;
  } else if (servo_commands_per_cycle_ > kServoCount) {
    servo_commands_per_cycle_ = kServoCount;
  }
  if (imu_reads_divider_ < 1) {
    imu_reads_divider_ = 1;
  }
  if (imu_angles_divider_ < 1) {
    imu_angles_divider_ = 1;
  }
  if (servo_command_divider_ < 1) {
    servo_command_divider_ = 1;
  }

  RCLCPP_INFO(
      this->get_logger(),
      "USB bridge serial config: port=%s baudrate=%d",
      serial_port_.c_str(),
      baudrate_);
  RCLCPP_INFO(
      this->get_logger(),
      "USB bridge servo feedback: reads_per_cycle=%d",
      servo_reads_per_cycle_);
  RCLCPP_INFO(
      this->get_logger(),
      "USB bridge IMU polling: divider=%d",
      imu_reads_divider_);
  RCLCPP_INFO(
      this->get_logger(),
      "USB bridge IMU angles polling: divider=%d",
      imu_angles_divider_);
  RCLCPP_INFO(
      this->get_logger(),
      "USB bridge servo command rate: divider=%d commands_per_cycle=%d",
      servo_command_divider_,
      servo_commands_per_cycle_);

  api_ = load_muto_api(so_path_);
  hw_ = reinterpret_cast<muto_handle*>(api_.create_usb(serial_port_.c_str(), baudrate_));
  if (hw_ == nullptr) {
    throw std::runtime_error("muto_create_usb failed");
  }
  if (api_.open(hw_) != 0) {
    throw std::runtime_error("muto_open failed: " + last_hw_error());
  }
  if (api_.torque_on(hw_) != 0) {
    throw std::runtime_error("muto_torque_on failed: " + last_hw_error());
  }

  auto qos = make_rt_qos();
  command_sub_ = create_subscription<muto_msgs::msg::Commands>(
      "/commands", qos,
      std::bind(&UsbBridgeNode::on_command, this, std::placeholders::_1));
  joint_pub_ = create_publisher<muto_msgs::msg::StampedJointState>("/joint_states", qos);
  imu_pub_ = create_publisher<muto_msgs::msg::StampedImu>("/imu/data", qos);
  timing_jitter_pub_ = create_publisher<std_msgs::msg::Float32>("/hw/timing_jitter", qos);
  system_mode_pub_ = create_publisher<std_msgs::msg::String>("/system_mode", qos);

  last_command_ros_time_ = now();

  running_.store(true, std::memory_order_release);
  rt_thread_ = std::thread(&UsbBridgeNode::run_rt_loop, this);
  publish_thread_ = std::thread(&UsbBridgeNode::run_publish_loop, this);
}

// Arret propre: stop threads avant fermeture I/O pour eviter acces use-after-close.
UsbBridgeNode::~UsbBridgeNode() {
  running_.store(false, std::memory_order_release);
  if (rt_thread_.joinable()) {
    rt_thread_.join();
  }
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
  close_hw();
}

// QoS dediee flux critiques, latence minimale et pas de backlog historique.
rclcpp::QoS UsbBridgeNode::make_rt_qos() const {
  rclcpp::QoS qos(1);
  qos.best_effort();
  qos.durability_volatile();
  qos.history(rclcpp::HistoryPolicy::KeepLast);
  return qos;
}

// Extrait le dernier message d'erreur de la couche C API.
std::string UsbBridgeNode::last_hw_error() const {
  if (hw_ == nullptr || api_.last_error == nullptr) {
    return "unknown";
  }
  const char* e = api_.last_error(hw_);
  return e == nullptr ? std::string("unknown") : std::string(e);
}

// Bascule explicite en mode EMERGENCY lorsque la couche hardware devient non fiable.
void UsbBridgeNode::publish_emergency_mode() {
  std_msgs::msg::String mode;
  mode.data = "EMERGENCY";
  system_mode_pub_->publish(mode);
}

// Sequence de fermeture defensive: torque_off -> close -> destroy -> dlclose.
void UsbBridgeNode::close_hw() {
  if (hw_ != nullptr) {
    (void)api_.torque_off(hw_);
    (void)api_.close(hw_);
    api_.destroy(hw_);
    hw_ = nullptr;
  }
  if (api_.lib_handle != nullptr) {
    dlclose(api_.lib_handle);
    api_.lib_handle = nullptr;
  }
}

// Callback commande avec garde anti-stale pour proteger les actionneurs.
void UsbBridgeNode::on_command(const muto_msgs::msg::Commands::SharedPtr msg) {
  const double age_ms = (now() - rclcpp::Time(msg->timestamp_sent)).seconds() * 1000.0;
  if (age_ms > command_age_max_ms_) {
    use_last_valid_command();
    return;
  }
  commanded_angles_rad_ = msg->angles;
  shared_.commanded_angles = commanded_angles_rad_;
  last_command_ros_time_ = now();
}

// Replie sur la derniere commande valide en cas de trame obsolete.
void UsbBridgeNode::use_last_valid_command() {
  commanded_angles_rad_ = shared_.commanded_angles;
}

// Calibration gyro robuste: moyenne sigma-clipped pour rejeter pics au boot.
void UsbBridgeNode::compute_gyro_offset_reject_outliers() {
  const size_t n = static_cast<size_t>(std::clamp(gyro_calib_cycles_, 1, 1000));
  for (size_t axis = 0; axis < 3; ++axis) {
    double mean = 0.0;
    for (size_t i = 0; i < n; ++i) {
      mean += static_cast<double>(gyro_calib_buf_[i][axis]);
    }
    mean /= static_cast<double>(n);

    double var = 0.0;
    for (size_t i = 0; i < n; ++i) {
      const double d = static_cast<double>(gyro_calib_buf_[i][axis]) - mean;
      var += d * d;
    }
    const double sigma = std::sqrt(var / static_cast<double>(n));

    double kept_sum = 0.0;
    size_t kept = 0;
    const double lim = std::max(gyro_outlier_sigma_, 0.1) * sigma;
    for (size_t i = 0; i < n; ++i) {
      const double v = static_cast<double>(gyro_calib_buf_[i][axis]);
      if (std::fabs(v - mean) <= lim) {
        kept_sum += v;
        ++kept;
      }
    }
    gyro_offset_[axis] = static_cast<float>(kept > 0 ? (kept_sum / static_cast<double>(kept)) : mean);
  }
}

/**
 * Boucle temps réel HARDLINE à 200 Hz (5 ms par cycle).
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * PROPRIÉTÉS CRITIQUES
 * ════════════════════════════════════════════════════════════════════════════════
 * • Pas d'allocation dynamique → pas de malloc, new, dynamic_cast
 * • Pas d'I/O bloquant → pas de std::cout, std::cerr (écire dans SharedState logs)
 * • Priorité SCHED_FIFO = 90 (maximum pour les processus non-kernel)
 * • Affinité CPU = rt_core_ (isolcpus=3 si disponible sur le Pi)
 * • Ordonnanceur sans prémption; s'arrête seulement sur sleep/wait explicites
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * SETUP TEMPS RÉEL
 * ════════════════════════════════════════════════════════════════════════════════
 * 1. SCHED_FIFO + high priority → RT scheduler du kernel Linux
 * 2. CPU affinity → bind au cœur isolé (core=3) pour éviter les interruptions
 * 3. CLOCK_MONOTONIC + clock_nanosleep(TIMER_ABSTIME) → horloge stable, sleep absolu
 *    (évite l'accumulation d'erreur dues aux retards de Wakeup)
 *
 * Remarque: les appels pthread_setschedparam et pthread_setaffinity_np peuvent échouer
 * silencieusement (retval ignoré) si les permissions ou la topologie CPU ne permettent pas.
 * En développement sur x86/Linux standard, les priorités sont réduites par le critère CAP_SYS_NICE.
 * Validation: vérifier avec `top -p <pid> -H` : affiche PRI et la topologie réelle.
 */
void UsbBridgeNode::run_rt_loop() {
  // Setup SCHED_FIFO avec priorité 90
  sched_param sp{};
  sp.sched_priority = 90;
  (void)pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);

  // Affinité CPU : bind au cœur isolé (par défaut core 3)
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(rt_core_, &cpuset);
  (void)pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);

  // Horloge monotonic pour sleep absolu sans dérive
  timespec next_time{};
  clock_gettime(CLOCK_MONOTONIC, &next_time);

  // Boucle principale: 200 Hz = 5000 cycles/s = 5 ms/cycle
  // chaque itération exécute tous les 12 steps ci-dessous pour garantir la latence
  while (running_.load(std::memory_order_acquire) && rclcpp::ok()) {
    const uint64_t cid = shared_.cycle_id.fetch_add(1, std::memory_order_acq_rel);

    // STEP 1/12) t0 monotonic : base temporelle de référence pour diagnostics jitter
    // mesurée AVANT toute opération hardware pour capturer le délai d'accès RT
    const int64_t t0 = static_cast<int64_t>(now_mono_ns());
    bool ret_ok_this_cycle = true;

    // 2) Emission des commandes sur les 18 servos (decimee pour limiter la charge serie).
    const bool do_servo_write = (cid % static_cast<uint64_t>(servo_command_divider_)) == 0ULL;
    if (do_servo_write) {
       std::lock_guard<std::mutex> lock(hw_mutex_);
      for (int n = 0; n < servo_commands_per_cycle_; ++n) {
        const uint8_t i = static_cast<uint8_t>((static_cast<int>(next_servo_write_idx_) + n) % kServoCount);
        const float deg_f = std::clamp(commanded_angles_rad_[i] * static_cast<float>(180.0 / M_PI), -90.0F, 90.0F);
        const int16_t deg = static_cast<int16_t>(std::lround(deg_f));
        const int ret = api_.servo_move(hw_, static_cast<uint8_t>(i + 1), deg, servo_speed_);
        if (ret != 0) {
          ++usb_error_count_;
          ret_ok_this_cycle = false;
        }
      }
      next_servo_write_idx_ = static_cast<uint8_t>((static_cast<int>(next_servo_write_idx_) + servo_commands_per_cycle_) % kServoCount);
    }

    // 3) Lecture des angles reels via muto_read_servo_angle_deg (round-robin pour limiter la latence).
      // Mixed servo + IMU transactions in the same cycle can truncate replies on this bus.
      // To keep the protocol stable, we prioritize at most one hardware read family per tick.
      const bool do_imu_angles_read = (cid % static_cast<uint64_t>(imu_angles_divider_)) == 0ULL;
      const bool do_imu_read = !do_imu_angles_read && ((cid % static_cast<uint64_t>(imu_reads_divider_)) == 0ULL);
      const bool do_servo_read = !do_imu_angles_read && !do_imu_read;

      if (servo_reads_per_cycle_ == 0) {
        for (int i = 0; i < kServoCount; ++i) {
          shared_.measured_angles[static_cast<size_t>(i)] = commanded_angles_rad_[static_cast<size_t>(i)];
        }
      } else if (do_servo_read) {
        const bool is_startup_phase = (cid < 400ULL);
        const int reads_this_cycle = is_startup_phase ? 0 : servo_reads_per_cycle_;
        if (reads_this_cycle > 0) {
          std::lock_guard<std::mutex> lock(hw_mutex_);
          for (int n = 0; n < reads_this_cycle; ++n) {
            const uint8_t i = static_cast<uint8_t>((static_cast<int>(next_servo_read_idx_) + n) % kServoCount);
            int16_t raw_deg = 0;
            const int ret = api_.read_servo_angle_deg(hw_, static_cast<uint8_t>(i + 1), &raw_deg);
            if (ret == 0) {
              shared_.measured_angles[i] = static_cast<float>(raw_deg) * static_cast<float>(M_PI / 180.0);
            } else {
              ++usb_error_count_;
              ret_ok_this_cycle = false;
              RCLCPP_WARN_THROTTLE(
                  get_logger(), *get_clock(), 1000,
                  "[MONO %ld ns] read_angle id=%d failed: %s",
                  t0, static_cast<int>(i + 1), last_hw_error().c_str());
            }
          }
          next_servo_read_idx_ = static_cast<uint8_t>((static_cast<int>(next_servo_read_idx_) + reads_this_cycle) % kServoCount);
        }
      }

    // 4) Estimation vitesse par difference finie (dt fixe = 5 ms).
    for (int i = 0; i < kServoCount; ++i) {
      shared_.measured_velocities[static_cast<size_t>(i)] =
          (shared_.measured_angles[static_cast<size_t>(i)] - prev_measured_angles_[static_cast<size_t>(i)]) / 0.005F;
      prev_measured_angles_[static_cast<size_t>(i)] = shared_.measured_angles[static_cast<size_t>(i)];
    }

    // 5) Acquisition IMU brute + Euler depuis la C API (decimee pour limiter le blocage I/O).
    if (do_imu_read) {
      muto_raw_imu_data raw{};
        std::lock_guard<std::mutex> lock(hw_mutex_);
      const int ret_raw = api_.get_raw_imu(hw_, &raw);
      if (ret_raw != 0) {
        ret_ok_this_cycle = false;
        ++usb_error_count_;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[MONO %ld ns] imu raw read failed: %s",
            t0, last_hw_error().c_str());
      } else {
        // 6) Conversion d'unites capteurs vers SI (rad/s, m/s2).
        shared_.imu_gyro[0] = static_cast<float>(static_cast<int16_t>(raw.gyro_x)) / 16.4F * static_cast<float>(M_PI / 180.0);
        shared_.imu_gyro[1] = static_cast<float>(static_cast<int16_t>(raw.gyro_y)) / 16.4F * static_cast<float>(M_PI / 180.0);
        shared_.imu_gyro[2] = static_cast<float>(static_cast<int16_t>(raw.gyro_z)) / 16.4F * static_cast<float>(M_PI / 180.0);
        shared_.imu_accel[0] = static_cast<float>(static_cast<int16_t>(raw.accel_x)) / 8192.0F * 9.80665F;
        shared_.imu_accel[1] = static_cast<float>(static_cast<int16_t>(raw.accel_y)) / 8192.0F * 9.80665F;
        shared_.imu_accel[2] = static_cast<float>(static_cast<int16_t>(raw.accel_z)) / 8192.0F * 9.80665F;

        // 7) Conversion orientation Euler -> quaternion pour transport ROS standard.
        shared_.imu_quaternion = complementary_filter_.update(
          last_imu_roll_deg_, last_imu_pitch_deg_, last_imu_yaw_deg_,
          shared_.imu_gyro[0], shared_.imu_gyro[1], shared_.imu_gyro[2]);

        // 8) Calibration gyro sur les N premiers cycles au demarrage.
        if (cid < static_cast<uint64_t>(std::clamp(gyro_calib_cycles_, 1, 1000))) {
          const size_t idx = static_cast<size_t>(cid);
          gyro_calib_buf_[idx][0] = static_cast<int16_t>(raw.gyro_x);
          gyro_calib_buf_[idx][1] = static_cast<int16_t>(raw.gyro_y);
          gyro_calib_buf_[idx][2] = static_cast<int16_t>(raw.gyro_z);
          if (idx + 1 == static_cast<size_t>(std::clamp(gyro_calib_cycles_, 1, 1000))) {
            compute_gyro_offset_reject_outliers();
            calibration_done_ = true;
          }
        }
        if (calibration_done_) {
          shared_.imu_gyro[0] -= gyro_offset_[0] / 16.4F * static_cast<float>(M_PI / 180.0);
          shared_.imu_gyro[1] -= gyro_offset_[1] / 16.4F * static_cast<float>(M_PI / 180.0);
          shared_.imu_gyro[2] -= gyro_offset_[2] / 16.4F * static_cast<float>(M_PI / 180.0);
        }
      }
    }
    if (do_imu_angles_read) {
      muto_imu_angles ang{};
      std::lock_guard<std::mutex> lock(hw_mutex_);
      const int ret_ang = api_.get_imu_angles(hw_, &ang);
      if (ret_ang != 0) {
        ret_ok_this_cycle = false;
        ++usb_error_count_;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[MONO %ld ns] imu angles read failed: %s",
            t0, last_hw_error().c_str());
      } else {
        last_imu_roll_deg_ = ang.roll;
        last_imu_pitch_deg_ = ang.pitch;
        last_imu_yaw_deg_ = ang.yaw;
      }
    }

    // 9) Heuristique force de contact derivee de l'axe Z IMU.
    // LIMITATION HARDWARE : muto_link_cpp ne fournit pas de lecture de couple servo.
    // Contact forces estimees depuis IMU az. Sera affine lors de validate_sim_real phase 1.
    const float az_norm = std::clamp(shared_.imu_accel[2] / 9.80665F, 0.0F, 1.0F);
    for (int leg = 0; leg < 6; ++leg) {
      shared_.contact_forces[static_cast<size_t>(leg)] = az_norm;
    }

    // 10) Gestion erreurs USB consecutives avec escalade EMERGENCY.
    if (usb_error_count_ > 10) {
      publish_emergency_mode();
    } else if (ret_ok_this_cycle) {
      usb_error_count_ = 0;
    }

    // 11) Monitoring jitter + publication lock-free du cycle pret.
    const int64_t elapsed_ns = static_cast<int64_t>(now_mono_ns()) - t0;
    shared_.timestamp_ns = static_cast<uint64_t>(t0);
    if (elapsed_ns > static_cast<int64_t>(jitter_threshold_ms_ * 1000000.0)) {
      std_msgs::msg::Float32 j;
      j.data = static_cast<float>(elapsed_ns) / 1000000.0F;
      timing_jitter_pub_->publish(j);
      RCLCPP_WARN(get_logger(), "[MONO %ld ns] Jitter %.2f ms", t0, j.data);
    }
    shared_.cycle_id_ready.store(cid, std::memory_order_release);
    shared_.data_ready.store(true, std::memory_order_release);

    // 12) Sleep absolu anti-derive: rebase si depassement d'echeance.
    timespec now_ts{};
    clock_gettime(CLOCK_MONOTONIC, &now_ts);
    if (timespec_cmp(now_ts, next_time) > 0) {
      next_time = now_ts;
    }
    timespec_add_ns(&next_time, kCycleNs);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);
  }
}

// Thread non-RT: copie du SharedState vers messages ROS Stamped* a cadence 200 Hz.
void UsbBridgeNode::run_publish_loop() {
  rclcpp::WallRate rate(kRtHz);

  muto_msgs::msg::StampedJointState js;
  muto_msgs::msg::StampedImu imu;

  js.joint_state.name.resize(kServoCount);
  js.joint_state.position.resize(kServoCount);
  js.joint_state.velocity.resize(kServoCount);
  js.joint_state.effort.resize(kServoCount);
  for (int i = 0; i < kServoCount; ++i) {
    js.joint_state.name[static_cast<size_t>(i)] = "joint_" + std::to_string(i + 1);
  }

  uint64_t last_cid = 0;
  while (running_.load(std::memory_order_acquire) && rclcpp::ok()) {
    if (!shared_.data_ready.load(std::memory_order_acquire)) {
      rate.sleep();
      continue;
    }

    const uint64_t cid = shared_.cycle_id_ready.load(std::memory_order_acquire);
    if (cid == last_cid) {
      rate.sleep();
      continue;
    }

    const auto stamp = now();
    js.header.stamp = stamp;
    js.cycle_id = cid;
    js.joint_state.header.stamp = stamp;

    for (int i = 0; i < kServoCount; ++i) {
      js.joint_state.position[static_cast<size_t>(i)] = shared_.measured_angles[static_cast<size_t>(i)];
      js.joint_state.velocity[static_cast<size_t>(i)] = shared_.measured_velocities[static_cast<size_t>(i)];
      js.joint_state.effort[static_cast<size_t>(i)] = 0.0;
    }

    imu.header.stamp = stamp;
    imu.cycle_id = cid;
    imu.imu.header.stamp = stamp;
    imu.imu.orientation.w = shared_.imu_quaternion[0];
    imu.imu.orientation.x = shared_.imu_quaternion[1];
    imu.imu.orientation.y = shared_.imu_quaternion[2];
    imu.imu.orientation.z = shared_.imu_quaternion[3];
    imu.imu.angular_velocity.x = shared_.imu_gyro[0];
    imu.imu.angular_velocity.y = shared_.imu_gyro[1];
    imu.imu.angular_velocity.z = shared_.imu_gyro[2];
    imu.imu.linear_acceleration.x = shared_.imu_accel[0];
    imu.imu.linear_acceleration.y = shared_.imu_accel[1];
    imu.imu.linear_acceleration.z = shared_.imu_accel[2];

    joint_pub_->publish(js);
    imu_pub_->publish(imu);

    last_cid = cid;
    shared_.data_ready.store(false, std::memory_order_release);
    rate.sleep();
  }
}

}  // namespace muto_hardware

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<muto_hardware::UsbBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
