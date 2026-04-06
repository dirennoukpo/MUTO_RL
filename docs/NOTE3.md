Voici l'architecture finale consolidée de Muto RS, intégrant toutes les corrections et améliorations identifiées.

---

## Décisions structurelles

Trois principes gouvernent l'ensemble. Le chemin critique temps réel ne tolère aucune indirection : tout ce qui touche aux servos ou au watchdog est en C++ avec threads RT isolés. La perception est physiquement séparée de la boucle de contrôle et ne peut jamais la bloquer. Le contrat sim-to-real est un artefact versionné et signé — aucun modèle ne tourne sur le robot sans validation explicite.

---

## Monde 1 — Isaac Lab

### Vecteur d'observation (v3, 70 valeurs)

```
Index 00-03 : quaternion orientation corps (w, x, y, z)
Index 04-06 : vitesse angulaire corps rad/s
Index 07-09 : accélération linéaire corps m/s²
Index 10-27 : angles 18 articulations en radians (ordre URDF strict)
Index 28-45 : vitesses 18 articulations en rad/s
Index 46-63 : dernières 18 actions envoyées (t-1)
Index 64-66 : goal_velocity cible (vx, vy, yaw_rate) normalisé
Index 67-69 : charge estimée par patte (6 pattes, scalaire par patte)
```

Les index 67-69 (charge par patte) sont nouveaux. Ils permettent au posture validator de pondérer le centroïde des pattes au sol par les pattes réellement en contact, et à la politique d'apprendre à anticiper les phases de swing. Sans cette information dans le vecteur d'entraînement, la politique ne peut pas apprendre à gérer les transitions de contact correctement.

### Domain randomization (v3)

La distribution des paquets perdus intègre maintenant le mode burst. À chaque step, avec une probabilité globale de 2.5% :
- 65% → répéter la dernière observation
- 20% → bruit gaussien fort (std=0.3 normalisé)
- 10% → vecteur de zéros
- 5% → déclencher un burst de 3 à 8 pertes consécutives

Le burst est la correction critique. Sans lui, la politique apprend à gérer des pertes indépendantes et diverge sur une vraie perte USB prolongée.

La randomization physique reste identique sauf pour les délais de réponse servo : ajouter une randomization de phase de ±2 ms par articulation (uniforme) pour simuler les erreurs de corrélation temporelle détectées par la validation sim-to-real.

### model_card.json (v3)

```json
{
  "model_id": "muto_rs_v003",
  "obs_spec_version": "v3",
  "obs_dim": 70,
  "action_dim": 18,
  "norm_stats_file": "norm_stats_v3.json",
  "perception_mode": "proprioceptive_only",
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
  "sim_real_validated": false,
  "sim_real_cross_corr_validated": false,
  "dry_run_validated": false
}
```

Le champ `sim_real_cross_corr_validated` est nouveau. Il passe à `true` uniquement après que la corrélation croisée entre `commanded_angles` et `measured_angles` de chaque servo est cohérente avec la fenêtre de délai simulée. Sans ça, le `rl_policy_node` refuse de passer en `RL_ACTIVE`.

### Reward function (v1)

Elle n'était pas définie dans le plan précédent. La voici.

```python
def compute_reward(obs, action, prev_action, contact_forces):

    # Terme 1 : suivi de vitesse cible (dominant)
    goal_vel = obs[64:67]
    actual_vel = compute_body_velocity(obs)
    r_velocity = -torch.norm(goal_vel - actual_vel, dim=-1)

    # Terme 2 : stabilité angulaire (éviter le tangage et le roulis)
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

Les poids sont des points de départ. `r_velocity` domine intentionnellement — la locomotion suit la commande de vitesse, le reste contraint le style. `r_fall` est une pénalité terminale : dès qu'elle se déclenche, l'épisode s'arrête.

---

## Monde 2 — Robot réel

### Raspberry Pi — `muto_hardware`

#### `usb_bridge_node` (C++, RT)

Deux corrections par rapport à la version précédente.

Le thread RT utilise maintenant `clock_nanosleep` avec `TIMER_ABSTIME` sur `CLOCK_MONOTONIC` pour la précision de timing. C'était déjà le cas. Ce qui est ajouté : un compteur de dérive de période. Si deux cycles consécutifs sont séparés de plus de 5.5 ms au lieu de 5 ms, le nœud publie un event sur `/hw/timing_jitter` sans bloquer la boucle. C'est ce qui permet de détecter une contention CPU avant qu'elle devienne un problème.

La calibration du biais gyro passe de 3 secondes (600 cycles à 200 Hz) à 5 secondes (1000 cycles) avec rejet des outliers à ±3 sigma. La calibration courte donnait des offsets instables si le robot bougeait légèrement au démarrage.

```cpp
// Calibration gyro avec rejet outliers
if (cid < 1000) {
    gyro_calibration_.accumulate(shared_state_.imu_gyro);
    if (cid == 999) {
        gyro_offset_ = gyro_calibration_.mean_reject_outliers(3.0f);
        calibration_done_ = true;
    }
}
```

#### `watchdog_node` (C++, RT)

Trois niveaux de réponse, inchangés dans leur logique, mais avec un ajout : la vérification que le topic `/commands_dry_run` a au moins un subscriber actif avant de valider le passage en mode `DRY_RUN`. Sans ça, le dry-run tourne en silence sans que personne ne regarde les sorties — bug silencieux difficile à détecter.

```cpp
void validate_dry_run_mode() {
    auto count = dry_run_pub_->get_subscription_count();
    if (count == 0) {
        RCLCPP_WARN(this->get_logger(),
            "DRY_RUN refusé : aucun subscriber sur /commands_dry_run");
        mode_manager_client_->request_mode(Mode::IDLE);
    }
}
```

Le timeout de commande reste à 50 ms. Le timeout heartbeat Jetson reste à 500 ms. La détection de chute reste à 3g. Ces valeurs sont correctes.

#### `mode_manager_node` (C++, FSM)

La matrice de transitions est identique. Ce qui change : le mode `DRY_RUN` exige maintenant explicitement `dry_run_validated = true` dans le `model_card.json` ET au moins un subscriber sur `/commands_dry_run`. Le mode `RL_ACTIVE` exige `sim_real_validated = true` ET `sim_real_cross_corr_validated = true`.

```cpp
bool can_enter_rl_active() {
    return model_card_.dry_run_validated &&
           model_card_.sim_real_validated &&
           model_card_.sim_real_cross_corr_validated;
}
```

### Jetson Nano — `muto_control`

#### `obs_builder_node` (C++, 100 Hz)

Le moniteur de cohérence publie maintenant en cumulatif toutes les 10 secondes, même si le count est zéro. C'est ce qui détecte une dérive lente sur une session longue. La version précédente ne publiait que sur événement ponctuel.

```cpp
// Timer 10s pour publication cumulative
void publish_sync_stats() {
    auto msg = std_msgs::msg::UInt64();
    msg.data = missed_cycles_;
    missed_cycles_pub_->publish(msg);
    // Ne pas remettre à zéro : cumulatif depuis le démarrage
}
```

Les index 67-69 du vecteur d'observation (charge par patte) sont construits ici à partir des couples mesurés sur les servos de patte. La charge estimée est un scalaire par patte, normalisé entre 0 (aucun contact) et 1 (charge maximale). Le seuil de contact est calibré pendant la phase de validation statique.

```cpp
float estimate_contact_force(int leg_index) {
    float torque = shared_state_.measured_torques[leg_tip_servo[leg_index]];
    return std::clamp(torque / max_contact_torque_, 0.0f, 1.0f);
}
```

#### `safety_filter_node` (C++, 100 Hz)

Trois corrections par rapport à la version précédente.

Le posture validator pondère maintenant le centroïde uniquement par les pattes en contact, en utilisant les index 67-69 du vecteur d'observation. Le seuil de contact est 0.1 (normalisé) — en dessous, la patte est considérée en swing et exclue du calcul.

```cpp
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
    if (total_weight < 0.1f) return false; // toutes les pattes en l'air
    cx /= total_weight;
    cy /= total_weight;
    float dist = std::hypot(com_x_ - cx, com_y_ - cy);
    return dist < com_stability_threshold_;
}
```

Le decay vers safe pose utilise maintenant un alpha progressif au lieu d'un alpha fixe, pour éviter un mouvement brutal si le robot est loin de la safe pose au moment du déclenchement.

```cpp
float alpha = std::min(0.05f + 0.01f * (float)skip_count_, 0.3f);
filtered = blend_toward_safe_pose(last_valid_action_, alpha);
```

La fenêtre glissante de variance passe de 5 à 10 cycles. Sur 5 cycles à 100 Hz, la fenêtre couvre 50 ms — trop courte pour distinguer une oscillation RL d'une transition rapide volontaire. Sur 10 cycles (100 ms), la distinction est plus fiable. Le seuil est à calibrer empiriquement lors des premières sessions physiques.

#### `rl_policy_node` (Python, TensorRT)

Le warmup passe de 200 à 500 inférences. Sur Jetson Nano, TensorRT a besoin de plus de cycles pour stabiliser le cache GPU, surtout si la mémoire unifiée est partagée avec les nœuds de perception.

Le fallback en cas de dépassement du budget (5 ms) déclenche maintenant une publication sur `/inference/timing_warn` avec le temps mesuré. Cela permet de corréler les dépassements avec `tegrastats` pour identifier si c'est du throttling thermique ou une contention mémoire.

```python
if t_elapsed > 5.0:
    self.timing_warn_pub.publish(Float32(data=t_elapsed))
    self.inference_skip_count += 1
    alpha = min(0.05 + 0.01 * (self.inference_skip_count - 5), 0.3)
    if self.inference_skip_count > 5:
        raw_action = blend(self.last_valid_action, SAFE_POSE, alpha)
    else:
        raw_action = self.last_valid_action
```

### Jetson Nano — `muto_perception`

#### `depth_processor_node` (C++, 30 Hz)

Un mécanisme de dégradation gracieuse est ajouté. Si `tegrastats` indique que la fréquence GPU est sous 80% de sa valeur nominale pendant plus de 5 secondes, le nœud réduit automatiquement sa fréquence de 30 Hz à 15 Hz et publie un warning sur `/system_health`. TensorRT garde la priorité GPU en toutes circonstances.

```cpp
void thermal_monitor_callback() {
    float gpu_freq_ratio = read_gpu_freq() / nominal_gpu_freq_;
    if (gpu_freq_ratio < 0.8f) {
        thermal_throttle_count_++;
        if (thermal_throttle_count_ > 25) { // 5 secondes à 5 Hz
            set_processing_rate(15.0);
            health_pub_->publish("depth_throttled");
        }
    } else {
        thermal_throttle_count_ = 0;
        set_processing_rate(30.0);
    }
}
```

La height map 11×11 pour l'option B (perception dans la politique) est construite à chaque cycle mais publiée sur `/depth/height_map` sans être consommée pour l'instant. Cela permet de logger les données pour valider l'option B avant de ré-entraîner.

#### `lidar_processor_node` (C++, 10 Hz)

Inchangé par rapport au plan précédent. Le SLAM via `slam_toolbox` reste sur CPU uniquement.

### `validate_sim_real.py` (version finale)

Deux phases de validation obligatoires avant `RL_ACTIVE`.

Phase 1 — validation statique : comparer les distributions de chaque dimension du vecteur d'observation en pose statique. Test de Kolmogorov-Smirnov avec seuil p > 0.05. Si une dimension échoue, logger laquelle et arrêter.

Phase 2 — validation dynamique : oscillations manuelles de 10 secondes. Calculer la corrélation croisée entre `commanded_angles[t]` et `measured_angles[t+k]` pour k de 0 à 20 ms. Le délai mesuré doit se trouver dans la fenêtre `[latency_action_ms_min, latency_action_ms_max]` du `model_card.json`. Si un servo est en dehors de cette fenêtre, il est probable que son ID Dynamixel est mal assigné ou que son firmware est obsolète.

```python
def validate_cross_correlation(commanded, measured, model_card):
    max_lag_samples = int(0.020 * 100)  # 20 ms à 100 Hz
    results = {}
    for i in range(18):
        corr = np.correlate(measured[:, i], commanded[:, i], mode='full')
        lag = np.argmax(corr) - len(commanded)
        lag_ms = lag * 10  # 10 ms par sample à 100 Hz
        in_window = (model_card['latency_action_ms'][0] <= lag_ms
                     <= model_card['latency_action_ms'][1])
        results[i] = {'lag_ms': lag_ms, 'valid': in_window}
    all_valid = all(r['valid'] for r in results.values())
    return all_valid, results
```

---

## Topics ROS 2 — liste finale

```
Hardware (Pi, 200 Hz)
  /imu/data                   sensor_msgs/Imu
  /joint_states               sensor_msgs/JointState
  /hw/timing_jitter           std_msgs/Float32        (sur événement)

Contrôle (Jetson, 100 Hz)
  /observation                muto_msgs/Observation   (70 floats + cycle_id)
  /commands_raw               muto_msgs/Commands
  /commands                   muto_msgs/Commands
  /commands_dry_run           muto_msgs/Commands

Perception (Jetson, async)
  /depth/pointcloud           sensor_msgs/PointCloud2   (15-30 Hz)
  /depth/obstacles            muto_msgs/ObstacleList    (15-30 Hz)
  /depth/height_map           muto_msgs/HeightMap       (15-30 Hz)
  /lidar/scan_filtered        sensor_msgs/LaserScan     (10 Hz)
  /lidar/obstacles            muto_msgs/ObstacleList    (10 Hz)
  /map                        nav_msgs/OccupancyGrid    (1 Hz)

Navigation (Jetson, 5 Hz)
  /navigation/goal_velocity   geometry_msgs/Twist

Système
  /system_mode                std_msgs/String           (1 Hz)
  /system_health              muto_msgs/SystemHealth    (10 Hz)
  /system_health_score        std_msgs/Float32          (10 Hz)
  /sync/missed_cycles         std_msgs/UInt64           (10 Hz cumulatif)
  /inference/timing_warn      std_msgs/Float32          (sur événement)
  /network/latency_ms         std_msgs/Float32          (10 Hz)
  /diagnostics                diagnostic_msgs/DiagnosticArray (10 Hz)
  /jetson/heartbeat           std_msgs/Bool             (10 Hz)
```

---

## Budget latence final

| Étape | Nominal | Pire cas |
|---|---|---|
| Lecture IMU + calibration (C++) | 0.5 ms | 1 ms |
| BulkRead 18 servos (C++) | 0.5 ms | 1 ms |
| Transport Pi → Jetson (DDS) | 1.5 ms | 2.5 ms |
| Sync cycle_id + charge pattes (C++) | 0.1 ms | 0.2 ms |
| Normalisation obs 70 valeurs (C++) | 0.05 ms | 0.1 ms |
| Inférence TensorRT FP16 | 2 ms | 6 ms |
| Safety filter + COM pondéré (C++) | 0.3 ms | 0.5 ms |
| Transport Jetson → Pi (DDS) | 1.5 ms | 2.5 ms |
| SyncWrite 18 servos (C++) | 0.5 ms | 1 ms |
| **Total nominal** | **7 ms** | **14.8 ms** |

Le fallback à 60 Hz se déclenche si le pire cas est atteint deux cycles consécutifs. En dessous de 60 Hz, le watchdog passe en `SAFE`.

---

## Ordre de développement

**Phase 1 — Hardware Pi seul**
Implémenter `usb_bridge_node` avec la calibration gyro à 1000 cycles et le moniteur de jitter. Vérifier `ros2 topic hz` sur `/imu/data` et `/joint_states`. Implémenter `watchdog_node` avec la validation du subscriber dry-run. Implémenter `mode_manager_node` avec les deux nouveaux prérequis pour `RL_ACTIVE`. Figer `observation_spec.py` v3.

**Phase 2 — Contrôle Jetson sans IA**
Implémenter `obs_builder_node` avec les index 67-69 et la publication cumulative de `/sync/missed_cycles`. Implémenter `safety_filter_node` avec le posture validator pondéré et l'alpha progressif. Exécuter `validate_sim_real.py` phase 1 (statique). Corriger les divergences avant de continuer.

**Phase 3 — Perception Jetson sans IA**
Implémenter `depth_processor_node` avec le moniteur thermique et la dégradation gracieuse. Implémenter `lidar_processor_node`. Vérifier dans RViz2. Vérifier que la perception ne dépasse jamais 15% de la RAM GPU quand TensorRT est actif (simuler avec un modèle factice).

**Phase 4 — IA Jetson**
`rl_policy_node` en dry-run avec modèle trivial, 500 inférences de warmup. Vérifier pipeline complet à 100 Hz stable. Exécuter `validate_sim_real.py` phase 2 (corrélation croisée). Premier entraînement Isaac Lab avec reward v1 et obs v3. Premier sim-to-real, robot surélevé, dry-run uniquement.

**Phase 5 — Navigation**
`navigation_node` Python, tester `/navigation/goal_velocity`. Intégrer le goal velocity dans la reward function. Ré-entraîner. Passer en `RL_ACTIVE` uniquement après les deux validations.