# Plan détaillé final — Architecture Muto RS (C++ temps réel + perception)

---

## Analyse des derniers retours

Pertinent et intégré :
- Le moniteur de cohérence `cycle_id` avec publication de `/sync/missed_cycles`. Correction réelle, pas cosmétique.
- Le decay vers la pose safe au lieu du freeze sur la dernière action. Critique pour éviter un robot bloqué dans une posture instable.
- La randomization de paquets en trois cas (repeat / bruit / zéro). Plus réaliste que la simple répétition.
- L'ajout du `command_age` côté Pi pour ignorer les commandes trop vieilles. Simple à implémenter, protège contre les rafales DDS retardées.
- La protection contre les oscillations RL (variance des actions sur une fenêtre glissante). Couvre un cas que le posture validator ne couvre pas.
- Le dry-run mode. Indispensable pour les premiers tests.
- La validation dynamique sim-to-real (dérivées d'observation). Complète la validation statique.

Pertinent mais reformulé :
- Le COM check simplifié remplace l'heuristique "pattes en l'air" seule. Intégré dans le safety filter.

Superflu ou déjà couvert :
- Seed fixe Isaac Lab — standard dans tous les frameworks RL, pas besoin de le traiter comme une amélioration.
- Health score global — utile mais cosmétique par rapport aux vrais problèmes. Implémenté en une ligne une fois les autres métriques en place.

**Décision de langage :** les nœuds directement sur le chemin critique temps réel sont en C++. Les nœuds qui traitent de la logique, de la supervision ou de la perception lourde restent en Python. Détail par nœud ci-dessous.

**Décision perception :** la caméra de profondeur et le LiDAR sont sur la Jetson. Ils ajoutent une couche de perception qui alimente la politique RL et la navigation. Cette couche est traitée séparément des commandes motrices pour ne pas impacter la boucle de contrôle à 100 Hz.

---

## Monde 1 — Isaac Lab (entraînement)

### Vecteur d'observation avec perception

L'ajout de la caméra de profondeur et du LiDAR change le vecteur d'observation. Il faut décider maintenant comment la perception entre dans la politique, parce que ce choix impacte l'architecture réseau de neurones entière.

**Deux options :**

Option A — Observation proprioceptive seule (recommandée pour commencer) :
La politique RL ne reçoit que les 64 valeurs proprioceptives (IMU + angles + vitesses + dernière action). La perception est utilisée en couche au-dessus pour la navigation (planification de trajectoire, détection d'obstacles), mais pas directement dans la boucle de contrôle à 100 Hz. C'est l'approche de la majorité des robots quadrupèdes modernes (ANYmal, Spot).

Option B — Observation avec hauteur du terrain :
Ajouter une représentation compacte de l'environnement local dans le vecteur d'observation. Typiquement une grille de hauteur 11×11 centrée sur le robot (121 valeurs scalaires). La politique apprend à adapter sa démarche au terrain. Plus puissant, mais entraînement beaucoup plus long et plus difficile.

**Recommandation pour Muto RS :** commencer avec l'option A. Une fois la politique de locomotion stable sur terrain plat, passer à l'option B en ajoutant la height map. Les deux options peuvent partager la même architecture réseau (le trunk proprioceptif reste identique, on ajoute un encodeur visuel).

### Vecteur d'observation final (option A, 64 valeurs)

```
observation_spec.py — NE PAS MODIFIER sans incrémenter la version

Version : v2
Dimensions : 64

Index 00-03 : quaternion orientation corps (w, x, y, z)
Index 04-06 : vitesse angulaire corps rad/s (roll_rate, pitch_rate, yaw_rate)
Index 07-09 : accélération linéaire corps m/s² (ax, ay, az)
Index 10-27 : angles 18 articulations en radians (ordre URDF strict)
Index 28-45 : vitesses 18 articulations en rad/s
Index 46-63 : dernières 18 actions envoyées (t-1) en radians
```

### Domain Randomization — version finale complète

**Randomization temporelle :**
- Délai d'observation : uniforme [0, 10] ms par épisode
- Délai d'action : uniforme [0, 10] ms par épisode
- Aléatoires et indépendants à chaque reset

**Simulation de paquets perdus — version à trois cas :**

À chaque step de simulation, avec les probabilités suivantes :
- 70% de probabilité → répéter la dernière observation (simule un retard DDS)
- 20% de probabilité → ajouter un bruit gaussien fort (std=0.3 normalisé) sur toutes les dimensions (simule glitch USB partiel)
- 10% de probabilité → vecteur de zéros (simule dropout complet)

Cette distribution se déclenche globalement avec une probabilité de 2% par step. Les 98% restants reçoivent l'observation normale.

**Randomization physique :**
- Masse segments : ±20%
- Friction pattes : [0.4, 1.2]
- Raideur articulations : ±30%
- Amortissement articulations : ±30%
- Hauteur terrain : perturbation ±1 cm
- Biais IMU : offset aléatoire [−0.05, +0.05] rad/s sur gyroscope par épisode

### Versioning modèle

Chaque `.onnx` livré au robot réel doit avoir son `model_card.json` :

```json
{
  "model_id": "muto_rs_v003",
  "obs_spec_version": "v2",
  "obs_dim": 64,
  "action_dim": 18,
  "norm_stats_file": "norm_stats_v2.json",
  "perception_mode": "proprioceptive_only",
  "trained_on": "isaac_lab_v1.2",
  "reward_version": "v5",
  "domain_rand": {
    "latency_obs_ms": [0, 10],
    "latency_action_ms": [0, 10],
    "packet_loss_pct": 2,
    "mass_variation": 0.2,
    "imu_bias_rad_s": 0.05
  },
  "training_date": "2025-01-15",
  "best_reward": 847.3,
  "dry_run_validated": true,
  "sim_real_validated": false,
  "notes": ""
}
```

Le champ `sim_real_validated` passe à `true` uniquement après que `validate_sim_real.py` confirme la cohérence des distributions. Le `rl_policy_node` refuse de passer en `RL_ACTIVE` si ce champ est `false`, sauf en dry-run.

---

## Monde 2 — Robot réel

### Décision de langage par nœud

| Nœud | Machine | Langage | Raison |
|---|---|---|---|
| `usb_bridge_node` | Pi | **C++** | Chemin critique. Thread RT, SyncWrite, BulkRead. Toute latence ici se répercute sur toute la boucle. |
| `watchdog_node` | Pi | **C++** | Réaction < 1 ms requise. Python GIL interdit ici. |
| `mode_manager_node` | Pi | **C++** | Transitions d'état temps réel, publie vers watchdog et bridge. |
| `robot_state_publisher` | Pi | Python | Nœud standard ROS 2, pas de chemin critique. |
| `diagnostics_node` | Pi | Python | Monitoring, pas temps réel. |
| `obs_builder_node` | Jetson | **C++** | Synchronisation `cycle_id`, buffers lockfree, 100 Hz strict. |
| `safety_filter_node` | Jetson | **C++** | Sur le chemin de `/commands`. Clamp, rate limit, posture check à chaque cycle. |
| `rl_policy_node` | Jetson | Python | TensorRT Python API est mature. L'inférence elle-même est en CUDA, pas en Python. Le Python n'est qu'un wrapper. |
| `depth_processor_node` | Jetson | **C++** | Traitement de nuage de points temps réel. PCL en C++ est 5x plus rapide qu'en Python. |
| `lidar_processor_node` | Jetson | **C++** | Même raison. SLAM et détection d'obstacles en C++. |
| `navigation_node` | Jetson | Python | Planification de trajectoire haute niveau. Pas de contrainte temps réel stricte. |

### Structure du workspace ROS 2

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

### `muto_hardware` — package C++ critique

#### `usb_bridge_node` en C++

Ce nœud est le cœur hardware. Voici l'architecture C++ complète.

**Thread principal ROS 2 (non-RT) :**
- Initialise le nœud, les publishers, les subscribers
- Lance le thread RT
- Tourne à la fréquence normale ROS 2 pour la gestion des callbacks non-critiques

**Thread RT (`SCHED_FIFO`, priorité 90) :**

```cpp
// Structure de données partagée entre threads (lockfree)
struct alignas(64) SharedState {
    std::atomic<uint64_t> cycle_id{0};
    std::array<float, 18> commanded_angles;
    std::array<float, 18> measured_angles;
    std::array<float, 18> measured_velocities;
    std::array<float, 3>  imu_accel;
    std::array<float, 3>  imu_gyro;
    std::array<float, 4>  imu_quaternion;
    std::atomic<bool>     data_ready{false};
    uint64_t              timestamp_ns;
};

void rt_loop() {
    // Configurer le thread RT
    struct sched_param sp = {.sched_priority = 90};
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);

    // Épingler sur CPU core 3 (isolé du reste du système)
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);

    // Boucle à 200 Hz (5 ms période)
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    while (running_) {
        auto t_start = std::chrono::steady_clock::now();

        uint64_t cid = shared_state_.cycle_id.fetch_add(1);

        // 1. SyncWrite angles commandés vers les 18 servos
        protocol_.sync_write(shared_state_.commanded_angles);

        // 2. BulkRead états des 18 servos
        if (!protocol_.bulk_read(
                shared_state_.measured_angles,
                shared_state_.measured_velocities)) {
            usb_error_count_++;
            if (usb_error_count_ > 10)
                mode_manager_.request_mode(Mode::EMERGENCY);
        } else {
            usb_error_count_ = 0;
        }

        // 3. Lecture IMU
        imu_.read(shared_state_.imu_accel,
                  shared_state_.imu_gyro);
        complementary_filter_.update(
                shared_state_.imu_accel,
                shared_state_.imu_gyro,
                shared_state_.imu_quaternion);

        // 4. Calibration offset gyro (3 premières secondes)
        if (cid < 600) {
            gyro_calibration_.accumulate(shared_state_.imu_gyro);
        }

        // 5. Mesure de timing
        auto t_elapsed = std::chrono::steady_clock::now() - t_start;
        timing_stats_.record(t_elapsed.count());

        // 6. Signaler que les données sont prêtes pour le publisher
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

            // Joint states avec cycle_id
            auto js_msg = sensor_msgs::msg::JointState();
            js_msg.header.stamp = this->now();
            js_msg.header.frame_id = std::to_string(cid); // cycle_id encodé
            // ... remplir les positions et vitesses
            joint_pub_->publish(js_msg);

            // IMU avec cycle_id
            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp = js_msg.header.stamp;
            imu_msg.header.frame_id = std::to_string(cid);
            // ... remplir orientation, angular_velocity, linear_acceleration
            imu_pub_->publish(imu_msg);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
}
```

**Isolation CPU pour le thread RT :**

Dans `/boot/cmdline.txt` sur le Pi, ajouter `isolcpus=3` pour dédier le core 3 exclusivement au thread RT. Sans ça, le scheduler Linux peut interrompre le thread RT pour des tâches système.

#### `watchdog_node` en C++

```cpp
class WatchdogNode : public rclcpp::Node {
    rclcpp::Time last_command_time_;
    rclcpp::Time last_heartbeat_time_;
    int skip_count_{0};

    void on_command(const muto_msgs::msg::Commands::SharedPtr msg) {
        // Vérifier l'âge de la commande
        auto command_age_ms = (this->now() - msg->timestamp_sent).nanoseconds() / 1e6;
        if (command_age_ms > 20.0) {
            RCLCPP_WARN(this->get_logger(),
                "Commande ignorée : age %.1f ms", command_age_ms);
            return;
        }
        last_command_time_ = this->now();
        skip_count_ = 0;
    }

    void check_timer_callback() {
        // Niveau 1 : timeout /commands
        auto dt_cmd = (this->now() - last_command_time_).seconds() * 1000;
        if (dt_cmd > 50.0) {
            mode_manager_client_->request_mode(Mode::SAFE);
        }

        // Niveau 2 : timeout heartbeat Jetson
        auto dt_hb = (this->now() - last_heartbeat_time_).seconds() * 1000;
        if (dt_hb > 500.0) {
            RCLCPP_ERROR(this->get_logger(), "Jetson non répondante");
            mode_manager_client_->request_mode(Mode::SAFE);
        }

        // Niveau 3 : détection de chute (3g)
        if (imu_accel_norm_ > 29.4f) {  // 3 * 9.81
            mode_manager_client_->request_mode(Mode::EMERGENCY);
        }
    }
};
```

#### `mode_manager_node` en C++

Implémenté comme une machine à états finis (`std::variant` ou enum class avec transitions explicites). Expose un service ROS 2 `/mode_request` (pas un topic) pour garantir que les transitions sont acquittées.

```cpp
enum class SystemMode {
    INIT, IDLE, DRY_RUN, RL_ACTIVE, SAFE, MANUAL, EMERGENCY
};

// Transitions autorisées (matrice)
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
```

---

### `muto_control` — package C++ chemin critique

#### `obs_builder_node` en C++ avec moniteur de cohérence

```cpp
class ObsBuilderNode : public rclcpp::Node {
    // Buffers lockfree indexés par cycle_id
    std::unordered_map<uint64_t, sensor_msgs::msg::Imu> imu_buffer_;
    std::unordered_map<uint64_t, sensor_msgs::msg::JointState> joint_buffer_;
    std::mutex buffer_mutex_;

    // Compteurs de cohérence
    uint64_t missed_cycles_{0};
    uint64_t built_observations_{0};

    void try_build_obs(uint64_t cid) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);

        if (imu_buffer_.count(cid) && joint_buffer_.count(cid)) {
            build_and_publish(imu_buffer_.at(cid), joint_buffer_.at(cid));
            imu_buffer_.erase(cid);
            joint_buffer_.erase(cid);
            built_observations_++;
        }

        // Moniteur de cohérence
        if (std::abs((int64_t)imu_buffer_.size() -
                     (int64_t)joint_buffer_.size()) > 3) {
            RCLCPP_WARN(this->get_logger(), "Désync buffers détectée");
            missed_cycles_pub_->publish(/* count */);
        }

        // Purge des cycles trop vieux (> 5 cycles)
        purge_old_cycles(cid - 5);
    }
};
```

**Normalisation en C++ :**

```cpp
// Charger norm_stats.json au démarrage
struct NormStats {
    std::array<float, 64> mean;
    std::array<float, 64> std_dev;
};

void normalize_observation(std::array<float, 64>& obs) {
    for (size_t i = 0; i < 64; ++i) {
        obs[i] = (obs[i] - norm_stats_.mean[i]) / norm_stats_.std_dev[i];
        // Clamp pour éviter valeurs extrêmes hors distribution
        obs[i] = std::clamp(obs[i], -5.0f, 5.0f);
    }
}
```

#### `safety_filter_node` en C++ avec posture validator

```cpp
class SafetyFilterNode : public rclcpp::Node {
    static constexpr int WINDOW_SIZE = 5;
    std::array<std::array<float, 18>, WINDOW_SIZE> action_history_;
    int skip_count_{0};
    int history_idx_{0};

    muto_msgs::msg::Commands filter(
            const muto_msgs::msg::Commands& raw_cmd) {

        auto filtered = raw_cmd;

        for (int i = 0; i < 18; ++i) {
            // 1. Détection NaN / Inf
            if (!std::isfinite(filtered.angles[i])) {
                return last_valid_action_;
            }

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

        // 4. Protection oscillations RL
        float variance = compute_action_variance(action_history_);
        if (variance > oscillation_threshold_) {
            RCLCPP_WARN(this->get_logger(), "Oscillation détectée");
            mode_manager_client_->request_mode(Mode::SAFE);
        }

        // 5. Posture validator — COM simplifié
        if (!posture_valid(filtered)) {
            skip_count_++;
            if (skip_count_ > 3) {
                mode_manager_client_->request_mode(Mode::SAFE);
            }
            return last_valid_action_;
        }

        // 6. Decay vers pose safe si trop de skips consécutifs
        if (skip_count_ > 5) {
            filtered = blend_toward_safe_pose(last_valid_action_, 0.1f);
        }

        action_history_[history_idx_++ % WINDOW_SIZE] = filtered.angles;
        last_valid_action_ = filtered;
        skip_count_ = 0;
        return filtered;
    }

    bool posture_valid(const muto_msgs::msg::Commands& cmd) {
        // Heuristique COM simplifié :
        // projeter le centre du corps sur le plan XY
        // vérifier que la projection est à moins de D cm
        // du centroïde des pattes au sol
        float com_x = 0.0f, com_y = 0.0f;
        float feet_cx = compute_feet_centroid_x(cmd);
        float feet_cy = compute_feet_centroid_y(cmd);
        float dist = std::hypot(com_x - feet_cx, com_y - feet_cy);
        return dist < com_stability_threshold_;
    }
};
```

---

### `muto_perception` — package C++ perception

C'est la couche qui était absente du plan précédent. La caméra de profondeur et le LiDAR sont sur la Jetson Nano. Ils alimentent deux flux parallèles et indépendants de la boucle de contrôle à 100 Hz.

#### Architecture de la perception

**Principe clé :** la perception ne doit jamais bloquer la boucle de contrôle motrice. Les deux flux (depth camera, LiDAR) tournent dans leurs propres threads avec leurs propres fréquences, et publient leurs résultats sur des topics que le `navigation_node` consomme de façon asynchrone.

```
Jetson Nano — threads de perception :

Thread 1 (30 Hz) :  depth_processor_node
    → RealSense D435i ou similaire
    → Publie /depth/pointcloud (nuage de points filtré)
    → Publie /depth/height_map (grille 11×11 pour future option B)
    → Publie /depth/obstacles (liste d'obstacles proches)

Thread 2 (10 Hz) :  lidar_processor_node
    → LiDAR 2D ou 3D
    → Publie /lidar/scan_filtered
    → Publie /lidar/obstacles
    → Alimente le SLAM (slam_toolbox ou cartographer)

Thread 3 (5 Hz) :   navigation_node (Python)
    → Fusionne /depth/obstacles + /lidar/obstacles
    → Publie /navigation/goal_velocity (vecteur vitesse cible pour le robot)
    → Le rl_policy_node peut consommer ce vecteur pour orienter la locomotion
```

#### `depth_processor_node` en C++

```cpp
class DepthProcessorNode : public rclcpp::Node {
public:
    DepthProcessorNode() : Node("depth_processor_node") {
        // Publisher nuage de points filtré
        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/depth/pointcloud", rclcpp::SensorDataQoS());

        // Publisher obstacles proches (< 50 cm)
        obs_pub_ = this->create_publisher<muto_msgs::msg::ObstacleList>(
            "/depth/obstacles", rclcpp::SensorDataQoS());

        // Subscriber caméra raw
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_rect_raw",
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::SharedPtr msg) {
                process_depth(msg);
            });
    }

private:
    void process_depth(const sensor_msgs::msg::Image::SharedPtr& msg) {
        // 1. Convertir depth image → pointcloud (PCL)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        depth_to_pointcloud(msg, cloud);

        // 2. Filtrage voxel grid (réduit la densité, accélère le traitement)
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.02f, 0.02f, 0.02f); // 2 cm resolution
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
            new pcl::PointCloud<pcl::PointXYZ>);
        vg.filter(*filtered);

        // 3. Retirer le sol (RANSAC plane fitting)
        remove_ground_plane(filtered);

        // 4. Détecter obstacles < 50 cm
        detect_nearby_obstacles(filtered);

        // 5. Construire height map 11×11 pour future option B
        build_height_map(filtered);

        // 6. Publier
        pc_pub_->publish(cloud_to_ros2(*filtered));
    }
};
```

**Performance sur Jetson Nano :** PCL avec VoxelGrid sur un nuage de 30 000 points tourne en environ 8 ms sur la Jetson Nano en C++. Le RANSAC pour le sol ajoute 3 ms. Total 11 ms, soit environ 30 Hz confortablement.

#### `lidar_processor_node` en C++

```cpp
class LidarProcessorNode : public rclcpp::Node {
    void process_scan(const sensor_msgs::msg::LaserScan::SharedPtr& msg) {
        // 1. Filtrer les points aberrants (min/max range)
        auto filtered = filter_scan(msg);

        // 2. Détecter segments d'obstacles (clustering)
        auto clusters = cluster_scan(filtered);

        // 3. Estimer vitesse obstacles (si tracking activé)
        // tracker_.update(clusters);

        // 4. Publier obstacles formatés
        muto_msgs::msg::ObstacleList obs_list;
        for (auto& c : clusters) {
            if (c.min_distance < 1.0f) {  // obstacles < 1 m
                obs_list.obstacles.push_back(cluster_to_obstacle(c));
            }
        }
        obs_pub_->publish(obs_list);

        // 5. Publier vers slam_toolbox
        filtered_scan_pub_->publish(*filtered);
    }
};
```

#### Intégration perception dans la politique RL

Pour la version initiale (option A, proprioceptive only), le `navigation_node` publie un vecteur vitesse cible `/navigation/goal_velocity` (vx, vy, yaw_rate). Ce vecteur remplace la direction de marche codée en dur dans la reward function d'Isaac Lab.

Cela permet au robot de se déplacer vers un objectif défini par la perception, tout en laissant la politique RL gérer le bas niveau (comment bouger les 18 servos pour marcher). Cette séparation est propre architecturalement et évite de re-entraîner la politique chaque fois que la logique de navigation change.

Dans `obs_builder_node`, ajouter optionnellement le vecteur vitesse cible aux observations si la politique le supporte :

```
Index 64-66 : goal_velocity cible (vx, vy, yaw_rate) normalisé
```

Ce qui porte le vecteur à 67 valeurs pour les modèles avec navigation. Le `model_card.json` spécifie `obs_dim`.

#### Gestion de la charge GPU sur Jetson Nano

La Jetson Nano a 4 GB de RAM unifiée (CPU + GPU). Trois processus vont se disputer le GPU :
- TensorRT (inférence politique) : ~400 MB
- PCL + OpenCV (depth processing) : utilise CPU principalement
- slam_toolbox : CPU uniquement

**Règle d'allocation :**
- TensorRT a la priorité GPU maximale (CUDA stream prioritaire)
- Le depth processing peut utiliser CUDA pour la conversion depth → pointcloud uniquement si TensorRT n'est pas en cours d'inférence (sinon CPU fallback)
- Surveiller `tegrastats` en continu pendant les tests pour détecter le throttling thermique

**Refroidissement actif recommandé :** un petit ventilateur 5V sur le dissipateur de la Jetson Nano. Sans ça, le throttling thermique commence après 10 minutes de fonctionnement combiné TensorRT + perception.

---

### `rl_policy_node` — Python avec TensorRT

Reste en Python car l'API TensorRT Python est mature et l'inférence elle-même est en CUDA. Le Python n'est qu'un orchestrateur.

**Ajout du dry-run mode :**

```python
class RLPolicyNode(Node):
    def __init__(self):
        super().__init__('rl_policy_node')
        self.dry_run = self.declare_parameter('dry_run', False).value
        self.warmup_done = False

    def warmup(self):
        # 200 inférences à vide pour pré-charger le cache GPU TensorRT
        dummy = np.zeros(self.obs_dim, dtype=np.float16)
        for _ in range(200):
            self.engine.infer(dummy)
        self.get_logger().info("Warmup TensorRT terminé")
        self.warmup_done = True

    def inference_callback(self, obs_msg):
        if not self.warmup_done:
            return

        obs = np.array(obs_msg.values, dtype=np.float16)

        t_start = time.perf_counter()
        raw_action = self.engine.infer(obs)
        t_elapsed = (time.perf_counter() - t_start) * 1000

        if t_elapsed > 5.0:
            # Fallback + decay progressif vers pose safe
            self.inference_skip_count += 1
            if self.inference_skip_count > 5:
                alpha = min(0.1 * (self.inference_skip_count - 5), 1.0)
                raw_action = blend(self.last_valid_action, SAFE_POSE, alpha)
            else:
                raw_action = self.last_valid_action
        else:
            self.last_valid_action = raw_action
            self.inference_skip_count = 0

        if self.dry_run:
            # Publier sur topic séparé, pas sur /commands_raw
            self.dry_run_pub.publish(action_to_msg(raw_action))
            return

        self.commands_raw_pub.publish(action_to_msg(raw_action))
```

---

### Topics ROS 2 — liste finale complète

```
Topics hardware (Pi, 200 Hz) :
  /imu/data                   sensor_msgs/Imu         (cycle_id dans header.frame_id)
  /joint_states               sensor_msgs/JointState  (cycle_id dans header.frame_id)

Topics contrôle (Pi→Jetson, 100 Hz) :
  /observation                muto_msgs/Observation   (vecteur 64 floats + cycle_id)

Topics commandes (Jetson→Pi, 100 Hz) :
  /commands_raw               muto_msgs/Commands      (sortie brute IA + timestamp_sent)
  /commands                   muto_msgs/Commands      (après safety filter)
  /commands_dry_run           muto_msgs/Commands      (dry-run uniquement)

Topics perception (Jetson, async) :
  /depth/pointcloud           sensor_msgs/PointCloud2   (30 Hz)
  /depth/obstacles            muto_msgs/ObstacleList    (30 Hz)
  /depth/height_map           muto_msgs/HeightMap       (30 Hz, pour future option B)
  /lidar/scan_filtered        sensor_msgs/LaserScan     (10 Hz)
  /lidar/obstacles            muto_msgs/ObstacleList    (10 Hz)
  /map                        nav_msgs/OccupancyGrid    (SLAM, 1 Hz)

Topics navigation (Jetson, 5 Hz) :
  /navigation/goal_velocity   geometry_msgs/Twist

Topics système (partout) :
  /system_mode                std_msgs/String           (1 Hz)
  /system_health              muto_msgs/SystemHealth    (10 Hz)
  /system_health_score        std_msgs/Float32          (10 Hz)
  /sync/missed_cycles         std_msgs/UInt64           (sur événement)
  /network/latency_ms         std_msgs/Float32          (10 Hz)
  /diagnostics                diagnostic_msgs/DiagnosticArray (10 Hz)
  /jetson/heartbeat           std_msgs/Bool             (10 Hz)
  /robot_status               std_msgs/String           (1 Hz)
```

---

### Budget de latence — version finale avec perception

| Étape | Budget | Pire cas | Mitigation C++ |
|---|---|---|---|
| Lecture IMU + filtre (C++) | 0.5 ms | 1 ms | Thread RT, core isolé |
| BulkRead 18 servos (C++) | 0.5 ms | 1 ms | Timeout 2 ms + fallback |
| Transport Pi → Jetson | 1.5 ms | 2.5 ms | DDS profilé, IP fixes |
| Sync cycle_id (C++) | 0.05 ms | 0.1 ms | Unordered_map lockfree |
| Normalisation obs (C++) | 0.05 ms | 0.1 ms | SIMD possible |
| Inférence TensorRT FP16 | 2 ms | 6 ms | Fallback + decay si > 5 ms |
| Safety filter + COM (C++) | 0.2 ms | 0.4 ms | Heuristique O(n) |
| Transport Jetson → Pi | 1.5 ms | 2.5 ms | DDS profilé |
| SyncWrite 18 servos (C++) | 0.5 ms | 1 ms | Trame groupée |
| **Total nominal** | **6.8 ms** | **14.6 ms** | **Fallback 60 Hz** |

**Impact de la perception sur le budget :** nul en conditions normales. La perception tourne dans des threads séparés sans bloquer la boucle de contrôle. Le seul impact est la contention mémoire GPU si TensorRT et le depth processing utilisent le GPU simultanément. La solution est la priorité CUDA stream sur TensorRT.

---

### Ordre de développement final

**Phase 1 — Hardware C++ (Pi seul, sans Jetson)**
1. `usb_bridge_node` C++ : publier `/imu/data` et `/joint_states` avec `cycle_id`. Vérifier avec `ros2 topic hz`.
2. `watchdog_node` C++ : tester le timeout et le decay vers pose safe.
3. `mode_manager_node` C++ : vérifier toutes les transitions d'état manuellement.
4. Écrire `observation_spec.py` et le figer.

**Phase 2 — Contrôle C++ (Jetson + Pi, sans IA)**
5. `obs_builder_node` C++ : logger le vecteur d'observation en CSV. Vérifier le `cycle_id` sync.
6. `safety_filter_node` C++ : tester le clamp, le rate limit, le posture validator, le decay.
7. Lancer `validate_sim_real.py` en statique. Corriger les divergences.

**Phase 3 — Perception C++ (Jetson, sans IA)**
8. `depth_processor_node` C++ : vérifier le nuage de points dans RViz2.
9. `lidar_processor_node` C++ : vérifier la carte SLAM dans RViz2.
10. Vérifier que la perception ne dépasse jamais 15% de la RAM GPU quand TensorRT tourne.

**Phase 4 — IA (Jetson)**
11. `rl_policy_node` Python en dry-run avec un modèle trivial. Vérifier que le pipeline complet tourne à 100 Hz stable.
12. Validation dynamique `validate_sim_real.py` avec oscillations manuelles.
13. Isaac Lab — premier spawn URDF, vérification physique.
14. Premier entraînement avec reward minimale (tenir debout).
15. Premier sim-to-real avec politique triviale, robot surélevé.

**Phase 5 — Intégration navigation**
16. `navigation_node` Python : tester la publication de `/navigation/goal_velocity`.
17. Intégrer le goal velocity dans la reward function Isaac Lab.
18. Ré-entraîner avec navigation dirigée.
