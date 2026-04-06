/*
 * Documentation FR: src/muto_hardware/include/muto_hardware/usb_bridge_node.hpp
 * Role: composant C++ du pipeline MUTO RS avec exigences de latence et surete.
 * Details: ce fichier implemente des fonctions critiques pour la perception, le controle ou le hardware.
 * Contraintes: eviter les regressions de timing, conserver les unites physiques et la coherence des interfaces.
 * Maintenance: documenter toute hypothese metier (seuils, conversions, heuristiques) lors des modifications.
 */

/**
 * @file usb_bridge_node.hpp
 * @brief Interface du noeud bridge hardware USB pour MUTO RS.
 *
 * RÔLE DANS L'ARCHITECTURE :
 *   Ce fichier décrit le contrat interne du nœud critique `usb_bridge_node`.
 *   Le bridge est la seule passerelle entre ROS 2 et la librairie propriétaire
 *   muto_link chargée dynamiquement via dlopen/dlsym.
 *
 *   Nœud responsable de :
 *   1. Chargement dynamique libmuto_link_cpp_lib.so (chemin via $(env WORKING_DIR))
 *   2. Communication USB série vers servos Dynamixel + IMU 6-axes
 *   3. Thread temps réel (SCHED_FIFO) à 200 Hz avec zero-jitter
 *   4. Publication ROS 2 topics : /imu/data, /joint_states (non-RT thread)
 *   5. Réception commands /commands (100 Hz) depuis controleur Jetson
 *
 * MACHINE CIBLE :
 *   Raspberry Pi (ROBOT_ROLE=DRIVER)
 *
 * FRÉQUENCE :
 *   - Boucle RT : 200 Hz (5 ms cycle absolu via clock_nanosleep)
 *   - Boucle publish : 100 Hz (asynchrone, event-driven)
 *
 * THREADS :
 *   - Thread RT (SCHED_FIFO priorité 90, core isolcpus) : loop 200 Hz sur USB/IMU
 *   - Thread ROS 2 (non-RT) : publication topics, gestion timeouts
 *
 * VARIABLES D'ENVIRONNEMENT (Docker) :
 *   WORKING_DIR     → Racine projet, utilisée pour construire chemin .so
 *                     Source : docker/config/.env.raspberrypi
 *                     Guard : RCLCPP_FATAL + exception si absent/vide
 *                     Chemin .so : $(WORKING_DIR)/muto_install/lib/libmuto_link_cpp_lib.so
 *
 *   ROBOT_ROLE      → Validation machine (doit être DRIVER sur Pi)
 *                     Source : docker/config/.env.raspberrypi
 *
 *   TARGET_IP       → IP Jetson (utilisée dans heartbeat vers BRAIN)
 *                     Source : docker/config/.env.raspberrypi
 *
 *   DOMAIN_ID       → ROS 2 DOMAIN_ID partagé (valeur : 33)
 *                     Source : docker/config/.env.base
 *
 * CONTRAINTES TEMPS RÉEL :
 *   - Zéro allocation dynamique dans la boucle 200 Hz
 *   - Zéro mutex dans le chemin critique (utiliser atomic + lock-free buffer)
 *   - clock_nanosleep en mode absolu avec rebase anti-dérive
 *   - Jitter maximal toléré : ~5 ms (seuil dans system_params.yaml)
 *
 * ARCHITECTURE INTERNE :
 *   - struct MutoApi : table de symboles dlsym pour libmuto_link
 *   - Filtre complémentaire : fusion accel+gyro → Euler angles
 *   - Buffer circulaire : synchronisation IMU (200 Hz) ↔ joint_states (200 Hz)
 *   - RingBuffer : pas de malloc dans le loop RT, pré-allocé au boot
 *
 * @see src/muto_bringup/config/system_params.yaml pour paramètres
 * @see src/muto_bringup/launch/pi_full.launch.py pour lancement
 * @see docker/config/.env.raspberrypi pour injection variables
 *
 * @author TEKBOT Robotics, Benin
 */
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include <dlfcn.h>

#include "muto_hardware/complementary_filter.hpp"
#include "muto_link/c_api.h"
#include "muto_msgs/msg/commands.hpp"
#include "muto_msgs/msg/stamped_imu.hpp"
#include "muto_msgs/msg/stamped_joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

namespace muto_hardware {

/**
 * @brief Table des symboles chargés dynamiquement depuis `libmuto_link_cpp_lib.so`.
 *
 * Cette structure contient exactement les 11 points d'entrée de la C API.
 * Le choix d'une table explicite simplifie la validation au démarrage et
 * permet une mise à jour de la .so sans relink du workspace ROS 2.
 *
 * Chaque pointeur est résolu via dlsym() au boot du nœud.
 * Si un symbole est absent : dlsym lance exception → container fatal stop.
 */
struct MutoApi {
  void* lib_handle{nullptr};

  // Pointeurs de fonctions C API — types explicites pour valeur et performance
  void* (*create_usb)(const char*, int){nullptr};           // crée handle USB
  int (*open)(void*){nullptr};                              // ouvre port série
  int (*close)(void*){nullptr};                             // ferme port série
  void (*destroy)(void*){nullptr};                          // libère ressources .so
  const char* (*last_error)(void*){nullptr};                // dernière erreur
  int (*torque_on)(void*){nullptr};                         // active servos
  int (*torque_off)(void*){nullptr};                        // coupe servos
  int (*servo_move)(void*, unsigned char, short, unsigned short){nullptr};     // mouvement servo
  int (*read_servo_angle_deg)(void*, unsigned char, short*){nullptr};          // lecture angle
  int (*get_imu_angles)(void*, void*){nullptr};             // Euler angles
  int (*get_raw_imu)(void*, void*){nullptr};                // données IMU brutes
};

/**
 * @brief État partagé lock-free entre thread RT et thread de publication.
 *
 * Ce buffer unique minimise les copies inter-threads. Le thread RT écrit,
 * le thread ROS 2 lit puis publie. Les champs atomiques assurent l'ordre
 * de publication sans introduire de mutex dans le chemin critique.
 */
struct alignas(64) SharedState {
  std::atomic<uint64_t> cycle_id{0};  ///< Compteur brut des cycles RT (200 Hz).
  std::atomic<uint64_t> cycle_id_ready{0};  ///< Dernier cycle completement calcule et publiable.
  std::array<float, 18> commanded_angles{};  ///< Commandes memorisees en radians.
  std::array<float, 18> measured_angles{};  ///< Angles mesures en radians (lus via muto_read_servo_angle_deg).
  std::array<float, 18> measured_velocities{};  ///< Vitesses en rad/s par difference finie (dt=5 ms).
  std::array<float, 3> imu_accel{};  ///< Acceleration IMU en m/s2.
  std::array<float, 3> imu_gyro{};  ///< Vitesse angulaire IMU en rad/s.
  std::array<float, 4> imu_quaternion{};  ///< Orientation estimatee (w,x,y,z).
  std::array<float, 6> contact_forces{};  ///< Heuristique de contact [0,1] par patte.
  std::atomic<bool> data_ready{false};  ///< Drapeau "donnees pretes" consomme par le thread non-RT.
  uint64_t timestamp_ns{0};  ///< Horodatage monotonic du debut de cycle, en ns.
};

/**
 * @class UsbBridgeNode
 * @brief Noeud hardware principal : commande servos + acquisition IMU + publication ROS.
 *
 * Design retenu :
 * - Couplage dynamique a la librairie hardware (MutoApi)
 * - SharedState lock-free pour reduire jitter et latence de synchronisation
 * - Separation RT/non-RT pour isoler DDS et serial du timing critique
 */
class UsbBridgeNode : public rclcpp::Node {
public:
  /** @brief Construit le noeud, charge la .so, ouvre le lien hardware et demarre les threads. */
  UsbBridgeNode();
  /** @brief Arrete proprement les threads puis coupe torque/connexion USB. */
  ~UsbBridgeNode() override;

private:
  static constexpr int kServoCount = 18;
  static constexpr int kRtHz = 200;
  static constexpr uint64_t kCycleNs = 5000000ULL;

  /** @brief Callback commande: applique le controle d'age puis met a jour la consigne active. */
  void on_command(const muto_msgs::msg::Commands::SharedPtr msg);
  /** @brief Restaure la derniere commande valide lorsque la commande courante est stale. */
  void use_last_valid_command();
  /** @brief Boucle critique 200 Hz (SCHED_FIFO), sans allocation dynamique. */
  void run_rt_loop();
  /** @brief Thread ROS non-RT qui publie les messages Stamped* a partir du SharedState. */
  void run_publish_loop();

  /** @brief Retourne l'horodatage monotonic courant en nanosecondes. */
  static uint64_t now_mono_ns();
  /** @brief Compare deux timespec absolues (-1, 0, +1). */
  static int timespec_cmp(const timespec& a, const timespec& b);
  /** @brief Ajoute un delta nanosecondes a un timespec absolu. */
  static void timespec_add_ns(timespec* t, uint64_t ns);

  /** @brief Calcule l'offset gyro via rejet d'outliers (mean +/- sigma). */
  void compute_gyro_offset_reject_outliers();
  /** @brief Construit le profil QoS critique (best_effort/volatile/keep_last(1)). */
  rclcpp::QoS make_rt_qos() const;
  /** @brief Retourne la derniere erreur textuelle remontee par la C API. */
  std::string last_hw_error() const;
  /** @brief Ferme proprement le hardware et decharge la librairie dynamique. */
  void close_hw();
  /** @brief Publie explicitement l'etat EMERGENCY sur /system_mode. */
  void publish_emergency_mode();

  MutoApi api_{};  ///< Table de fonctions resolvees par dlsym.
  muto_handle* hw_{nullptr};  ///< Handle opaque retourne par muto_create_usb.

  SharedState shared_{};  ///< Etat lock-free partage entre threads RT et publication.
  std::array<float, 18> commanded_angles_rad_{};  ///< Commandes courantes en radians.
  std::array<float, 18> prev_measured_angles_{};  ///< Angles du cycle precedent (difference finie vitesse).

  std::array<std::array<int16_t, 3>, 1000> gyro_calib_buf_{};  ///< Buffer brut calibration gyro [cycles][axes].
  std::array<float, 3> gyro_offset_{0.0F, 0.0F, 0.0F};  ///< Offset gyro calcule (LSB) applique apres calibration.
  bool calibration_done_{false};  ///< Vrai quand l'offset gyro est valide.

  std::atomic<bool> running_{false};  ///< Drapeau global de vie des threads.
  std::thread rt_thread_;  ///< Thread temps reel 200 Hz.
  std::thread publish_thread_;  ///< Thread non-RT de publication ROS.

  std::string so_path_;  ///< Chemin complet de la .so muto_link.
  std::string serial_port_;  ///< Port serie USB (ex: /dev/ttyUSB0).
  int baudrate_{115200};  ///< Baudrate de la liaison serie.
  int rt_core_{3};  ///< Coeur CPU reserve a la boucle RT.
  uint16_t servo_speed_{200};  ///< Vitesse Dynamixel [0-1023], param ROS `servo_speed`.
  int usb_error_count_{0};  ///< Compteur d'erreurs consecutives I/O USB.
  double jitter_threshold_ms_{5.5};  ///< Seuil jitter (5 ms nominal + 10% marge).
  double command_age_max_ms_{20.0};  ///< Age max commande acceptee, en millisecondes.
  int gyro_calib_cycles_{1000};  ///< Taille calibration gyro en cycles initiaux.
  double gyro_outlier_sigma_{3.0};  ///< Seuil statistique de rejet d'outliers (sigma).

  rclcpp::Time last_command_ros_time_;  ///< Date ROS de la derniere commande valide.
  ComplementaryFilter complementary_filter_;  ///< Filtre d'orientation (Euler -> quaternion).

  rclcpp::Subscription<muto_msgs::msg::Commands>::SharedPtr command_sub_;  ///< Entree commandes filtrees.
  rclcpp::Publisher<muto_msgs::msg::StampedJointState>::SharedPtr joint_pub_;  ///< Sortie etat articulations.
  rclcpp::Publisher<muto_msgs::msg::StampedImu>::SharedPtr imu_pub_;  ///< Sortie etat IMU.
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr timing_jitter_pub_;  ///< Alerte jitter boucle RT.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_mode_pub_;  ///< Publication d'etat EMERGENCY.
};

}  // namespace muto_hardware
