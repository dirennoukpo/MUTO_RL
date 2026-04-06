# Architecture Muto RS — Version finale complète
# C++ temps réel + perception + RL sim-to-real

---

## Décisions structurelles

Trois principes gouvernent l'ensemble.

Le chemin critique temps réel ne tolère aucune indirection : tout ce qui touche aux servos ou au watchdog est en C++ avec threads RT isolés. Le Python GIL est incompatible avec une réaction en moins de 1 ms.

La perception est physiquement séparée de la boucle de contrôle motrice. La caméra de profondeur et le LiDAR tournent dans leurs propres threads à leurs propres fréquences et ne peuvent jamais bloquer la boucle à 100 Hz.

Le contrat sim-to-real est un artefact versionné et signé. Aucun modèle ne tourne sur le robot sans validation explicite des deux phases : statique (distributions) et dynamique (corrélation croisée servo).

---

## Décision de langage par nœud

| Nœud | Machine | Langage | Raison |
|---|---|---|---|
| `usb_bridge_node` | Pi | C++ | Chemin critique. Thread RT, SyncWrite, BulkRead. Toute latence se répercute sur toute la boucle. |
| `watchdog_node` | Pi | C++ | Réaction < 1 ms requise. Python GIL interdit ici. |
| `mode_manager_node` | Pi | C++ | Transitions d'état temps réel, publie vers watchdog et bridge. |
| `robot_state_publisher` | Pi | Python | Nœud standard ROS 2, pas de chemin critique. |
| `diagnostics_node` | Pi | Python | Monitoring, pas temps réel. |
| `obs_builder_node` | Jetson | C++ | Synchronisation `cycle_id`, buffers lockfree, 100 Hz strict. |
| `safety_filter_node` | Jetson | C++ | Sur le chemin de `/commands`. Clamp, rate limit, posture check à chaque cycle. |
| `rl_policy_node` | Jetson | Python | TensorRT Python API est mature. L'inférence est en CUDA, le Python n'est qu'un wrapper. |
| `depth_processor_node` | Jetson | C++ | Traitement de nuage de points temps réel. PCL en C++ est 5x plus rapide qu'en Python. |
| `lidar_processor_node` | Jetson | C++ | SLAM et détection d'obstacles en C++. |
| `navigation_node` | Jetson | Python | Planification de trajectoire haute niveau. Pas de contrainte temps réel stricte. |

---

## Structure du workspace ROS 2

```
tekbot_ws/
├── src/
│   ├── muto_hardware/          C++ — usb_bridge, watchdog, mode_manager
│   ├── muto_control/           C++ — obs_builder, safety_filter
│   ├── muto_inference/         Python — rl_policy_node
│   ├── muto_perception/        C++ — depth_processor, lidar_processor
│   ├── muto_navigation/        Python — navigation_node
│   ├── muto_msgs/              Messages custom ROS 2
│   └── muto_bringup/           Launch files, configs
```

---

## Monde 1 — Isaac Lab (entraînement)

### Vecteur d'observation (v3, 70 valeurs)

```
observation_spec.py — NE PAS MODIFIER sans incrémenter la version

Version : v3
Dimensions : 70

Index 00-03 : quaternion orientation corps (w, x, y, z)
Index 04-06 : vitesse angulaire corps rad/s (roll_rate, pitch_rate, yaw_rate)
Index 07-09 : accélération linéaire corps m/s² (ax, ay, az)
Index 10-27 : angles 18 articulations en radians (ordre URDF strict)
Index 28-45 : vitesses 18 articulations en rad/s
Index 46-63 : dernières 18 actions envoyées (t-1) en radians
Index 64-66 : goal_velocity cible (vx, vy, yaw_rate) normalisé
Index 67-69 : charge estimée par patte (6 pattes, scalaire normalisé 0-1)
```

Les index 67-69 sont critiques. Ils permettent au posture validator de pondérer le centroïde des pattes au sol par les pattes réellement en contact, et à la politique d'apprendre à anticiper les phases de swing. Sans cette information, la politique ne peut pas gérer correctement les transitions de contact.

**Roadmap vers l'option B (terrain) :** une fois la politique de locomotion stable sur terrain plat, ajouter une grille de hauteur 11×11 centrée sur le robot (121 valeurs scalaires). Le trunk proprioceptif du réseau reste identique ; on ajoute un encodeur visuel. Le `depth_processor_node` construit déjà cette height map à chaque cycle sur `/depth/height_map` pour valider l'option B avant de ré-entraîner.

### Domain randomization (v3)

**Randomization temporelle :**
- Délai d'observation : uniforme [0, 10] ms par épisode
- Délai d'action : uniforme [0, 10] ms par épisode
- Phase servo : jitter ±2 ms par articulation (uniforme) — simule les erreurs de corrélation temporelle détectées par la validation sim-to-real
- Aléatoires et indépendants à chaque reset

**Simulation de paquets perdus (v3 — avec mode burst) :**

Déclenchement global à 2.5% par step. Distribution interne :
- 65% → répéter la dernière observation (simule retard DDS)
- 20% → bruit gaussien fort std=0.3 normalisé (simule glitch USB partiel)
- 10% → vecteur de zéros (simule dropout complet)
- 5% → burst de 3 à 8 pertes consécutives

Le mode burst est la correction critique. Sans lui, la politique apprend à gérer des pertes indépendantes et diverge sur une vraie perte USB prolongée. Dans la réalité, les pertes USB arrivent en rafales.

**Randomization physique :**
- Masse segments : ±20%
- Friction pattes : [0.4, 1.2]
- Raideur articulations : ±30%
- Amortissement articulations : ±30%
- Hauteur terrain : perturbation ±1 cm
- Biais IMU : offset aléatoire [−0.05, +0.05] rad/s sur gyroscope par épisode

### model_card.json (v3)

```json
{
  "model_id": "muto_rs_v003",
  "obs_spec_version": "v3",
  "obs_dim": 70,
  "action_dim": 18,
  "norm_stats_file": "norm_stats_v3.json",
  "perception_mode": "proprioceptive_only",
  "trained_on": "isaac_lab_v1.2",
  "reward_version": "v1",
  "domain_rand": {
    "latency_obs_ms": [0, 10],
    "latency_action_ms": [0, 10],
    "servo_phase_jitter_ms": 2,
    "packet_loss_pct": 2.5,
    "packet_loss_burst_pct": 0.5,
    "packet_loss_burst_len": [3, 8],
    "mass_variation": 0.2,
    "imu_bias_rad_s": 0.05
  },
  "training_date": "",
  "best_reward": 0.0,
  "dry_run_validated": false,
  "sim_real_validated": false,
  "sim_real_cross_corr_validated": false,
  "notes": ""
}
```

`sim_real_validated` passe à `true` après la phase 1 de `validate_sim_real.py` (distributions statiques).
`sim_real_cross_corr_validated` passe à `true` après la phase 2 (corrélation croisée servo).
Le `rl_policy_node` refuse de passer en `RL_ACTIVE` si l'un de ces deux champs est `false`.

### Reward function (v1)

```python
def compute_reward(obs, action, prev_action, contact_forces):

    # Terme 1 : suivi de vitesse cible (dominant)
    goal_vel = obs[64:67]
    actual_vel = compute_body_velocity(obs)
    r_velocity = -torch.norm(goal_vel - actual_vel, dim=-1)

    # Terme 2 : stabilité angulaire (éviter tangage et roulis)
    angular_vel = obs[4:7]
    r_stability = -0.3 * torch.norm(angular_vel[:, :2], dim=-1)

    # Terme 3 : efficacité énergétique (minimiser les couples)
    r_energy = -0.01 * torch.sum(action ** 2, dim=-1)

    # Terme 4 : douceur des transitions (minimiser les à-coups)
    r_smooth = -0.05 * torch.sum((action - prev_action) ** 2, dim=-1)

    # Terme 5 : contact au sol (récompenser les pattes en contact)
    r_contact = 0.1 * torch.sum(contact_forces > 0.5, dim=-1).float()

    # Terme 6 : pénalité chute (terminaison)
    r_fall = -10.0 * (obs[:, 0] < 0.15)  # hauteur corps < 15 cm

    return r_velocity + r_stability + r_energy + r_smooth + r_contact + r_fall
```

`r_velocity` domine intentionnellement : la locomotion suit la commande de vitesse, le reste contraint le style. `r_fall` est une pénalité terminale — dès qu'elle se déclenche, l'épisode s'arrête. Les poids sont des points de départ à calibrer empiriquement.

---

## Monde 2 — Robot réel

### `muto_hardware` — Raspberry Pi, package C++ critique

#### `usb_bridge_node` (C++, RT)

Ce nœud est le cœur hardware. Il tourne à 200 Hz avec un thread RT épinglé sur le core 3 isolé.

**Configuration système :** dans `/boot/cmdline.txt`, ajouter `isolcpus=3` pour dédier le core 3 exclusivement au thread RT. Sans ça, le scheduler Linux peut interrompre le thread RT pour des tâches système.

**Structure de données partagée entre threads (lockfree) :**

```cpp
struct alignas(64) SharedState {
    std::atomic<uint64_t> cycle_id{0};
    std::atomic<uint64_t> cycle_id_ready{0};
    std::array<float, 18> commanded_angles;
    std::array<float, 18> measured_angles;
    std::array<float, 18> measured_velocities;
    std::array<float, 18> measured_torques;   // pour charge pattes (index 67-69)
    std::array<float, 3>  imu_accel;
    std::array<float, 3>  imu_gyro;
    std::array<float, 4>  imu_quaternion;
    std::atomic<bool>     data_ready{false};
    uint64_t              timestamp_ns;
};
```

**Thread RT (SCHED_FIFO, priorité 90) :**

```cpp
void rt_loop() {
    struct sched_param sp = {.sched_priority = 90};
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);

    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    while (running_) {
        auto t_start = std::chrono::steady_clock::now();

        uint64_t cid = shared_state_.cycle_id.fetch_add(1);

        // 1. SyncWrite angles commandés vers les 18 servos
        protocol_.sync_write(shared_state_.commanded_angles);

        // 2. BulkRead états et couples des 18 servos
        if (!protocol_.bulk_read(
                shared_state_.measured_angles,
                shared_state_.measured_velocities,
                shared_state_.measured_torques)) {
            usb_error_count_++;
            if (usb_error_count_ > 10)
                mode_manager_.request_mode(Mode::EMERGENCY);
        } else {
            usb_error_count_ = 0;
        }

        // 3. Lecture IMU + filtre complémentaire
        imu_.read(shared_state_.imu_accel, shared_state_.imu_gyro);
        complementary_filter_.update(
                shared_state_.imu_accel,
                shared_state_.imu_gyro,
                shared_state_.imu_quaternion);

        // 4. Calibration offset gyro — 5 secondes (1000 cycles) avec rejet outliers ±3σ
        if (cid < 1000) {
            gyro_calibration_.accumulate(shared_state_.imu_gyro);
            if (cid == 999) {
                gyro_offset_ = gyro_calibration_.mean_reject_outliers(3.0f);
                calibration_done_ = true;
            }
        }

        // 5. Moniteur de dérive de période
        auto t_elapsed = std::chrono::steady_clock::now() - t_start;
        timing_stats_.record(t_elapsed.count());
        if (t_elapsed.count() > 5'500'000) {  // > 5.5 ms
            timing_jitter_pub_->publish(Float32(data=t_elapsed.count() / 1e6f));
        }

        // 6. Signaler les données prêtes
        shared_state_.cycle_id_ready.store(cid);
        shared_state_.data_ready.store(true);

        // 7. Attente précise jusqu'au prochain cycle (5 ms)
        next_time.tv_nsec += 5'000'000;
        if (next_time.tv_nsec >= 1'000'000'000) {
            next_time.tv_sec++;
            next_time.tv_nsec -= 1'000'000'000;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);
    }
}
```

**Publisher thread (non-RT, 200 Hz) :**

```cpp
void publish_loop() {
    while (running_) {
        if (shared_state_.data_ready.exchange(false)) {
            uint64_t cid = shared_state_.cycle_id_ready.load();

            auto js_msg = sensor_msgs::msg::JointState();
            js_msg.header.stamp = this->now();
            js_msg.header.frame_id = std::to_string(cid);  // cycle_id encodé
            // remplir positions, vitesses, couples
            joint_pub_->publish(js_msg);

            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp = js_msg.header.stamp;
            imu_msg.header.frame_id = std::to_string(cid);
            // remplir orientation, angular_velocity, linear_acceleration
            imu_pub_->publish(imu_msg);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
}
```

#### `watchdog_node` (C++, RT)

Trois niveaux de réponse + validation subscriber dry-run.

```cpp
class WatchdogNode : public rclcpp::Node {
    rclcpp::Time last_command_time_;
    rclcpp::Time last_heartbeat_time_;
    float imu_accel_norm_{0.0f};

    void on_command(const muto_msgs::msg::Commands::SharedPtr msg) {
        // Vérifier l'âge de la commande — ignorer si > 20 ms
        auto command_age_ms = (this->now() - msg->timestamp_sent).nanoseconds() / 1e6;
        if (command_age_ms > 20.0) {
            RCLCPP_WARN(this->get_logger(),
                "Commande ignorée : age %.1f ms", command_age_ms);
            return;
        }
        last_command_time_ = this->now();
    }

    void check_timer_callback() {
        // Niveau 1 : timeout /commands → SAFE
        auto dt_cmd = (this->now() - last_command_time_).seconds() * 1000;
        if (dt_cmd > 50.0)
            mode_manager_client_->request_mode(Mode::SAFE);

        // Niveau 2 : timeout heartbeat Jetson → SAFE
        auto dt_hb = (this->now() - last_heartbeat_time_).seconds() * 1000;
        if (dt_hb > 500.0) {
            RCLCPP_ERROR(this->get_logger(), "Jetson non répondante");
            mode_manager_client_->request_mode(Mode::SAFE);
        }

        // Niveau 3 : chute détectée (3g) → EMERGENCY
        if (imu_accel_norm_ > 29.4f)
            mode_manager_client_->request_mode(Mode::EMERGENCY);
    }

    // Validation avant activation dry-run
    void validate_dry_run_mode() {
        auto count = dry_run_pub_->get_subscription_count();
        if (count == 0) {
            RCLCPP_WARN(this->get_logger(),
                "DRY_RUN refusé : aucun subscriber sur /commands_dry_run");
            mode_manager_client_->request_mode(Mode::IDLE);
        }
    }
};
```

#### `mode_manager_node` (C++, FSM)

Expose un service ROS 2 `/mode_request` (pas un topic) pour garantir que les transitions sont acquittées. Implémenté avec `enum class` et matrice de transitions explicites.

```cpp
enum class SystemMode {
    INIT, IDLE, DRY_RUN, RL_ACTIVE, SAFE, MANUAL, EMERGENCY
};

const std::map<SystemMode, std::set<SystemMode>> valid_transitions = {
    {SystemMode::INIT,      {SystemMode::IDLE}},
    {SystemMode::IDLE,      {SystemMode::RL_ACTIVE, SystemMode::DRY_RUN,
                             SystemMode::MANUAL, SystemMode::EMERGENCY}},
    {SystemMode::DRY_RUN,   {SystemMode::IDLE, SystemMode::SAFE}},
    {SystemMode::RL_ACTIVE, {SystemMode::SAFE, SystemMode::MANUAL,
                             SystemMode::EMERGENCY}},
    {SystemMode::SAFE,      {SystemMode::IDLE}},
    {SystemMode::MANUAL,    {SystemMode::IDLE, SystemMode::SAFE}},
    {SystemMode::EMERGENCY, {}}  // sortie uniquement par reboot
};

bool can_enter_dry_run() {
    return model_card_.dry_run_validated &&
           watchdog_.dry_run_subscriber_active();
}

bool can_enter_rl_active() {
    return model_card_.dry_run_validated &&
           model_card_.sim_real_validated &&
           model_card_.sim_real_cross_corr_validated;
}
```

---

### `muto_control` — Jetson Nano, package C++ chemin critique

#### `obs_builder_node` (C++, 100 Hz)

Synchronise les messages IMU et JointState par `cycle_id`, construit le vecteur d'observation à 70 valeurs, normalise, publie.

```cpp
class ObsBuilderNode : public rclcpp::Node {
    std::unordered_map<uint64_t, sensor_msgs::msg::Imu>        imu_buffer_;
    std::unordered_map<uint64_t, sensor_msgs::msg::JointState> joint_buffer_;
    std::mutex buffer_mutex_;
    uint64_t missed_cycles_{0};
    uint64_t built_observations_{0};

    void try_build_obs(uint64_t cid) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);

        if (imu_buffer_.count(cid) && joint_buffer_.count(cid)) {
            build_and_publish(imu_buffer_.at(cid), joint_buffer_.at(cid));
            imu_buffer_.erase(cid);
            joint_buffer_.erase(cid);
            built_observations_++;
        } else {
            missed_cycles_++;
        }

        // Moniteur de cohérence
        if (std::abs((int64_t)imu_buffer_.size() -
                     (int64_t)joint_buffer_.size()) > 3) {
            RCLCPP_WARN(this->get_logger(), "Désync buffers détectée");
        }

        // Purge des cycles trop vieux (> 5 cycles)
        purge_old_cycles(cid - 5);
    }

    // Timer 10s — publication cumulative (détecte dérives lentes)
    void publish_sync_stats() {
        auto msg = std_msgs::msg::UInt64();
        msg.data = missed_cycles_;
        missed_cycles_pub_->publish(msg);
        // Ne pas remettre à zéro : cumulatif depuis le démarrage
    }

    // Charge estimée par patte (index 67-69 du vecteur)
    float estimate_contact_force(int leg_index) {
        float torque = shared_state_.measured_torques[leg_tip_servo[leg_index]];
        return std::clamp(torque / max_contact_torque_, 0.0f, 1.0f);
    }
};
```

**Normalisation (70 valeurs) :**

```cpp
struct NormStats {
    std::array<float, 70> mean;
    std::array<float, 70> std_dev;
};

void normalize_observation(std::array<float, 70>& obs) {
    for (size_t i = 0; i < 70; ++i) {
        obs[i] = (obs[i] - norm_stats_.mean[i]) / norm_stats_.std_dev[i];
        obs[i] = std::clamp(obs[i], -5.0f, 5.0f);
    }
}
```

#### `safety_filter_node` (C++, 100 Hz)

Filtre toutes les commandes brutes avant envoi aux servos. Pipeline de validation en 6 étapes.

```cpp
class SafetyFilterNode : public rclcpp::Node {
    static constexpr int WINDOW_SIZE = 10;  // 100 ms à 100 Hz
    std::array<std::array<float, 18>, WINDOW_SIZE> action_history_;
    int skip_count_{0};
    int history_idx_{0};

    muto_msgs::msg::Commands filter(
            const muto_msgs::msg::Commands& raw_cmd,
            const std::array<float, 6>& contact_forces) {

        auto filtered = raw_cmd;

        for (int i = 0; i < 18; ++i) {
            // 1. Détection NaN / Inf
            if (!std::isfinite(filtered.angles[i]))
                return last_valid_action_;

            // 2. Clamp angulaire (limites URDF)
            filtered.angles[i] = std::clamp(
                filtered.angles[i],
                joint_limits_[i].min,
                joint_limits_[i].max);

            // 3. Rate limiting
            float delta = filtered.angles[i] - last_valid_action_.angles[i];
            float max_delta = rate_limit_deg_ * M_PI / 180.0f;
            filtered.angles[i] = last_valid_action_.angles[i] +
                std::clamp(delta, -max_delta, max_delta);
        }

        // 4. Protection oscillations RL (fenêtre 10 cycles = 100 ms)
        float variance = compute_action_variance(action_history_);
        if (variance > oscillation_threshold_) {
            RCLCPP_WARN(this->get_logger(), "Oscillation détectée");
            mode_manager_client_->request_mode(Mode::SAFE);
        }

        // 5. Posture validator — COM pondéré par les pattes en contact
        if (!posture_valid(filtered, contact_forces)) {
            skip_count_++;
            if (skip_count_ > 3)
                mode_manager_client_->request_mode(Mode::SAFE);
            return last_valid_action_;
        }

        // 6. Decay progressif vers safe pose si trop de skips consécutifs
        if (skip_count_ > 5) {
            float alpha = std::min(0.05f + 0.01f * (float)skip_count_, 0.3f);
            filtered = blend_toward_safe_pose(last_valid_action_, alpha);
        }

        action_history_[history_idx_++ % WINDOW_SIZE] = filtered.angles;
        last_valid_action_ = filtered;
        skip_count_ = 0;
        return filtered;
    }

    // COM pondéré par les pattes réellement en contact (seuil 0.1)
    bool posture_valid(const muto_msgs::msg::Commands& cmd,
                       const std::array<float, 6>& contact_forces) {
        float cx = 0, cy = 0, total_weight = 0;
        for (int i = 0; i < 6; ++i) {
            if (contact_forces[i] > 0.1f) {
                cx += foot_position_x(cmd, i) * contact_forces[i];
                cy += foot_position_y(cmd, i) * contact_forces[i];
                total_weight += contact_forces[i];
            }
        }
        if (total_weight < 0.1f) return false;  // toutes les pattes en l'air
        cx /= total_weight;
        cy /= total_weight;
        float dist = std::hypot(com_x_ - cx, com_y_ - cy);
        return dist < com_stability_threshold_;
    }
};
```

---

### `muto_inference` — Jetson Nano, Python + TensorRT

#### `rl_policy_node` (Python)

L'inférence est en CUDA via TensorRT. Le Python n'est qu'un orchestrateur. Warmup à 500 inférences pour stabiliser le cache GPU sur Jetson Nano (RAM unifiée partagée avec la perception).

```python
class RLPolicyNode(Node):
    def __init__(self):
        super().__init__('rl_policy_node')
        self.dry_run = self.declare_parameter('dry_run', False).value
        self.obs_dim = self.declare_parameter('obs_dim', 70).value
        self.warmup_done = False
        self.inference_skip_count = 0
        self.last_valid_action = SAFE_POSE.copy()

    def warmup(self):
        dummy = np.zeros(self.obs_dim, dtype=np.float16)
        for _ in range(500):
            self.engine.infer(dummy)
        self.get_logger().info("Warmup TensorRT terminé (500 inférences)")
        self.warmup_done = True

    def inference_callback(self, obs_msg):
        if not self.warmup_done:
            return

        obs = np.array(obs_msg.values, dtype=np.float16)

        t_start = time.perf_counter()
        raw_action = self.engine.infer(obs)
        t_elapsed = (time.perf_counter() - t_start) * 1000

        if t_elapsed > 5.0:
            # Publier le dépassement pour corrélation avec tegrastats
            self.timing_warn_pub.publish(Float32(data=t_elapsed))
            self.inference_skip_count += 1
            # Decay progressif vers safe pose
            alpha = min(0.05 + 0.01 * (self.inference_skip_count - 5), 0.3)
            if self.inference_skip_count > 5:
                raw_action = blend(self.last_valid_action, SAFE_POSE, alpha)
            else:
                raw_action = self.last_valid_action
        else:
            self.last_valid_action = raw_action
            self.inference_skip_count = 0

        if self.dry_run:
            self.dry_run_pub.publish(action_to_msg(raw_action))
            return

        self.commands_raw_pub.publish(action_to_msg(raw_action))
```

---

### `muto_perception` — Jetson Nano, package C++ perception

**Principe clé :** la perception ne bloque jamais la boucle de contrôle motrice. Les deux flux tournent dans leurs propres threads et publient de façon asynchrone. TensorRT garde la priorité GPU en toutes circonstances.

```
Jetson Nano — threads de perception :

Thread 1 (15-30 Hz) :  depth_processor_node
    → RealSense D435i ou similaire
    → /depth/pointcloud     (nuage de points filtré)
    → /depth/obstacles      (obstacles < 50 cm)
    → /depth/height_map     (grille 11×11 pour future option B)

Thread 2 (10 Hz) :      lidar_processor_node
    → LiDAR 2D ou 3D
    → /lidar/scan_filtered
    → /lidar/obstacles
    → Alimente slam_toolbox (CPU uniquement)

Thread 3 (5 Hz) :       navigation_node (Python)
    → Fusionne /depth/obstacles + /lidar/obstacles
    → /navigation/goal_velocity (vx, vy, yaw_rate)
```

#### `depth_processor_node` (C++, 15-30 Hz avec dégradation thermique)

```cpp
class DepthProcessorNode : public rclcpp::Node {
    void process_depth(const sensor_msgs::msg::Image::SharedPtr& msg) {
        // 1. Depth image → pointcloud (PCL)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        depth_to_pointcloud(msg, cloud);

        // 2. VoxelGrid 2 cm — réduit la densité avant traitement
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.02f, 0.02f, 0.02f);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        vg.filter(*filtered);

        // 3. Retirer le sol (RANSAC plane fitting, ~3 ms)
        remove_ground_plane(filtered);

        // 4. Détecter obstacles < 50 cm
        detect_nearby_obstacles(filtered);

        // 5. Construire height map 11×11 (pour future option B)
        build_height_map(filtered);

        // 6. Publier
        pc_pub_->publish(cloud_to_ros2(*filtered));
    }

    // Moniteur thermique — dégradation gracieuse si GPU throttle
    void thermal_monitor_callback() {
        float gpu_freq_ratio = read_gpu_freq() / nominal_gpu_freq_;
        if (gpu_freq_ratio < 0.8f) {
            thermal_throttle_count_++;
            if (thermal_throttle_count_ > 25) {  // 5 secondes à 5 Hz
                set_processing_rate(15.0);
                health_pub_->publish("depth_throttled");
                RCLCPP_WARN(this->get_logger(), "Throttling GPU — 30Hz → 15Hz");
            }
        } else {
            thermal_throttle_count_ = 0;
            set_processing_rate(30.0);
        }
    }
};
```

**Performance Jetson Nano :** VoxelGrid sur 30 000 points ≈ 8 ms. RANSAC sol ≈ 3 ms. Total ≈ 11 ms → 30 Hz confortable. Avec throttling thermique : 15 Hz. Refroidissement actif recommandé (ventilateur 5V) — sans ça, le throttling commence après 10 minutes de fonctionnement combiné TensorRT + perception.

**Allocation GPU :**
- TensorRT : priorité CUDA stream maximale, ~400 MB RAM unifiée
- depth_processor : CPU principalement, CUDA uniquement si TensorRT n'est pas en inférence
- slam_toolbox : CPU exclusivement

#### `lidar_processor_node` (C++, 10 Hz)

```cpp
class LidarProcessorNode : public rclcpp::Node {
    void process_scan(const sensor_msgs::msg::LaserScan::SharedPtr& msg) {
        // 1. Filtrer les points aberrants (min/max range)
        auto filtered = filter_scan(msg);

        // 2. Clustering pour détecter les segments d'obstacles
        auto clusters = cluster_scan(filtered);

        // 3. Publier obstacles < 1 m
        muto_msgs::msg::ObstacleList obs_list;
        for (auto& c : clusters) {
            if (c.min_distance < 1.0f)
                obs_list.obstacles.push_back(cluster_to_obstacle(c));
        }
        obs_pub_->publish(obs_list);

        // 4. Publier vers slam_toolbox
        filtered_scan_pub_->publish(*filtered);
    }
};
```

---

### `validate_sim_real.py` (deux phases obligatoires)

Aucun modèle ne passe en `RL_ACTIVE` sans avoir réussi les deux phases.

**Phase 1 — validation statique :**

Comparer les distributions de chaque dimension du vecteur d'observation entre simulation et robot réel en pose statique. Test de Kolmogorov-Smirnov avec seuil p > 0.05. Si une dimension échoue, logger laquelle et arrêter. Corriger avant de continuer.

**Phase 2 — validation dynamique (corrélation croisée) :**

```python
def validate_cross_correlation(commanded, measured, model_card):
    """
    Oscillations manuelles 10 secondes.
    Vérifie que le délai de réponse de chaque servo
    est dans la fenêtre simulée dans model_card.
    Un servo hors fenêtre = ID Dynamixel mal assigné ou firmware obsolète.
    """
    results = {}
    for i in range(18):
        corr = np.correlate(measured[:, i], commanded[:, i], mode='full')
        lag = np.argmax(corr) - len(commanded)
        lag_ms = lag * 10  # 10 ms par sample à 100 Hz
        in_window = (model_card['latency_action_ms'][0] <= lag_ms
                     <= model_card['latency_action_ms'][1])
        results[i] = {'lag_ms': lag_ms, 'valid': in_window}
        if not in_window:
            print(f"ÉCHEC servo {i} : délai mesuré {lag_ms} ms, "
                  f"fenêtre attendue {model_card['latency_action_ms']}")

    all_valid = all(r['valid'] for r in results.values())
    return all_valid, results
```

---

## Topics ROS 2 — liste finale complète

```
Hardware (Pi, 200 Hz)
  /imu/data                   sensor_msgs/Imu           (cycle_id dans header.frame_id)
  /joint_states               sensor_msgs/JointState    (cycle_id dans header.frame_id)
  /hw/timing_jitter           std_msgs/Float32          (sur événement, si cycle > 5.5 ms)

Contrôle (Jetson, 100 Hz)
  /observation                muto_msgs/Observation     (70 floats + cycle_id)
  /commands_raw               muto_msgs/Commands        (sortie brute RL + timestamp_sent)
  /commands                   muto_msgs/Commands        (après safety filter)
  /commands_dry_run           muto_msgs/Commands        (dry-run uniquement)

Perception (Jetson, async)
  /depth/pointcloud           sensor_msgs/PointCloud2   (15-30 Hz)
  /depth/obstacles            muto_msgs/ObstacleList    (15-30 Hz)
  /depth/height_map           muto_msgs/HeightMap       (15-30 Hz, future option B)
  /lidar/scan_filtered        sensor_msgs/LaserScan     (10 Hz)
  /lidar/obstacles            muto_msgs/ObstacleList    (10 Hz)
  /map                        nav_msgs/OccupancyGrid    (SLAM, 1 Hz)

Navigation (Jetson, 5 Hz)
  /navigation/goal_velocity   geometry_msgs/Twist

Système
  /system_mode                std_msgs/String                  (1 Hz)
  /system_health              muto_msgs/SystemHealth           (10 Hz)
  /system_health_score        std_msgs/Float32                 (10 Hz)
  /sync/missed_cycles         std_msgs/UInt64                  (10 Hz cumulatif)
  /inference/timing_warn      std_msgs/Float32                 (sur événement)
  /hw/timing_jitter           std_msgs/Float32                 (sur événement)
  /network/latency_ms         std_msgs/Float32                 (10 Hz)
  /diagnostics                diagnostic_msgs/DiagnosticArray  (10 Hz)
  /jetson/heartbeat           std_msgs/Bool                    (10 Hz)
  /robot_status               std_msgs/String                  (1 Hz)
```

---

## Budget de latence final

| Étape | Nominal | Pire cas | Mitigation |
|---|---|---|---|
| Lecture IMU + calibration (C++) | 0.5 ms | 1 ms | Thread RT, core 3 isolé |
| BulkRead 18 servos + couples (C++) | 0.5 ms | 1 ms | Timeout 2 ms + fallback EMERGENCY |
| Transport Pi → Jetson (DDS) | 1.5 ms | 2.5 ms | DDS profilé, IP fixes |
| Sync cycle_id + charge pattes (C++) | 0.1 ms | 0.2 ms | unordered_map lockfree |
| Normalisation obs 70 valeurs (C++) | 0.05 ms | 0.1 ms | SIMD possible |
| Inférence TensorRT FP16 | 2 ms | 6 ms | Fallback + decay progressif si > 5 ms |
| Safety filter + COM pondéré (C++) | 0.3 ms | 0.5 ms | Heuristique O(n) |
| Transport Jetson → Pi (DDS) | 1.5 ms | 2.5 ms | DDS profilé |
| SyncWrite 18 servos (C++) | 0.5 ms | 1 ms | Trame groupée |
| **Total nominal** | **7 ms** | **14.8 ms** | **Fallback 60 Hz** |

Le fallback à 60 Hz se déclenche si le pire cas est atteint deux cycles consécutifs. En dessous de 60 Hz, le watchdog passe en `SAFE`. La perception n'impacte pas ce budget — elle tourne dans des threads séparés. Le seul risque est la contention mémoire GPU entre TensorRT et le depth processor, géré par la priorité CUDA stream.

---

## Ordre de développement

### Phase 1 — Hardware Pi seul (sans Jetson)

1. `usb_bridge_node` C++ : BulkRead avec lecture des couples, calibration gyro 1000 cycles avec rejet outliers, moniteur de jitter > 5.5 ms. Vérifier `ros2 topic hz` sur `/imu/data` et `/joint_states`.
2. `watchdog_node` C++ : tester le timeout de commande (50 ms), le timeout heartbeat (500 ms), la détection de chute (3g), la validation du subscriber dry-run.
3. `mode_manager_node` C++ : vérifier toutes les transitions manuellement, en particulier les deux prérequis de `RL_ACTIVE`.
4. Figer `observation_spec.py` v3. Ne plus modifier sans incrémenter la version.

### Phase 2 — Contrôle Jetson sans IA

5. `obs_builder_node` C++ : logger le vecteur d'observation 70 valeurs en CSV. Vérifier la cohérence du `cycle_id`. Vérifier les index 67-69 (charge pattes). Vérifier la publication cumulative de `/sync/missed_cycles` toutes les 10 secondes.
6. `safety_filter_node` C++ : tester le clamp, le rate limit, le posture validator pondéré, le decay progressif (alpha 0.05→0.3). Calibrer `com_stability_threshold_` et `oscillation_threshold_` empiriquement.
7. Exécuter `validate_sim_real.py` phase 1 (KS statique). Corriger les divergences avant de continuer. Mettre `sim_real_validated = true` dans le model_card.

### Phase 3 — Perception Jetson sans IA

8. `depth_processor_node` C++ : vérifier le nuage de points filtré dans RViz2. Vérifier le moniteur thermique (déclencher en throttlant la Jetson manuellement). Vérifier la height map 11×11 sur `/depth/height_map`.
9. `lidar_processor_node` C++ : vérifier la carte SLAM dans RViz2. Vérifier les obstacles publiés sur `/lidar/obstacles`.
10. Charger un modèle TensorRT factice et vérifier que la perception ne dépasse jamais 15% de la RAM GPU quand TensorRT est actif.

### Phase 4 — IA Jetson

11. `rl_policy_node` Python en dry-run avec modèle trivial (sorties aléatoires bornées), 500 inférences de warmup. Vérifier que le pipeline complet tourne à 100 Hz stable sur 10 minutes.
12. Exécuter `validate_sim_real.py` phase 2 (corrélation croisée). Corriger les servos hors fenêtre. Mettre `sim_real_cross_corr_validated = true` dans le model_card.
13. Isaac Lab — premier spawn URDF. Vérifier la physique (masse, friction, limites articulaires).
14. Premier entraînement avec reward v1. Objectif minimal : tenir debout 10 secondes.
15. Premier sim-to-real : robot surélevé, dry-run uniquement, observer `/commands_dry_run` en continu.

### Phase 5 — Navigation et intégration

16. `navigation_node` Python : tester la publication de `/navigation/goal_velocity` à partir de `/depth/obstacles` et `/lidar/obstacles`.
17. Intégrer le goal velocity dans la reward function Isaac Lab. Ré-entraîner.
18. Passer en `RL_ACTIVE` uniquement après les deux validations et le dry-run validé sur terrain surélevé.
