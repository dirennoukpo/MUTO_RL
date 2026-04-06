# muto_control

## 1. Rôle dans l'architecture

Agrégation observations brutes (IMU + joints) en vecteur 70D normalisé, synchronisation par cycle_id ring-buffer. Application du pipeline sécurité en 7 étapes (sanitization, clamp vitesse/courant, rate-limit, stabilité posture). Fournit l'observation à la politique RL et les commandes sécurisées au hardware. Cœur du contrôle Jetson (100 Hz).

## 2. Machine cible

| Machine | Rôle |
|---------|------|
| Jetson (BRAIN) | Agrégation observation + filtrage sécurité 100 Hz |

## 3. Nœuds ROS 2

| Nom nœud | Langage | Fréquence | Rôle |
|----------|---------|-----------|------|
| obs_builder_node | C++ | 100 Hz | Synchronisation IMU/joints → observation 70D |
| safety_filter_node | C++ | 100 Hz | Pipeline sécurité 7 étapes → commandes |

## 4. Topics

| Nom topic | Type | Fréquence | Direction | Description |
|-----------|------|-----------|-----------|-------------|
| /observation | muto_msgs/Observation | 100 Hz | Pub | Vecteur 70D normalisé (IMU + joints + état) |
| /sync/missed_cycles | std_msgs/UInt64 | 100 Hz | Pub | Compteur cycles manqués (sync) |
| /imu/data | muto_msgs/StampedImu | 200 Hz | Sub | IMU depuis Pi |
| /joint_states | muto_msgs/StampedJointState | 200 Hz | Sub | États articulations depuis Pi |
| /commands_dry_run | muto_msgs/Commands | 100 Hz | Sub | Consignes simulation (avant safety filter) |
| /navigation/goal_velocity | geometry_msgs/Twist | 5 Hz | Sub | Vitesse objectif navigation |
| /commands | muto_msgs/Commands | 100 Hz | Pub | Commandes sécurisées → Pi |

## 5. Variables d'environnement

| Variable | Valeur/Format | Description |
|----------|---------------|-------------|
| OBS_BUILDER_CPU | 1 | Affinité CPU core obs_builder_node |
| SAFETY_FILTER_CPU | 2 | Affinité CPU core safety_filter_node |

## 6. Paramètres ROS 2

| Nom paramètre | Type | Valeur défaut | Description |
|---------------|------|---------------|-------------|
| obs_dim | int | 70 | Dimension observation normalisée |
| ring_buffer_size | int | 256 | Taille ring buffer cycle_id |
| imu_acc_scale | float | 1.0 | Facteur normalisation accéléromètre |
| joint_pos_scale | float | 1.0 | Facteur normalisation position articulaire |
| safety_velocity_limit | float | 2.0 | Limite vitesse maximale (rad/s) |
| safety_current_limit | float | 3.0 | Limite courant maximal (A) |
| rate_limiter_threshold | float | 0.5 | Seuil variation rate-limiter |

## 7. Commandes de compilation

```bash
colcon build --symlink-install --packages-select muto_control
```

## 8. Lancement + Dépendances

**Via jetson_full.launch.py:**
```bash
ros2 launch muto_bringup jetson_full.launch.py
```

Dépendances amont :
- muto_msgs (Observation, Commands, interfaces)
- muto_hardware (Pi) pour /imu/data, /joint_states
- muto_navigation pour /navigation/goal_velocity
- muto_inference (consomme /observation)

Notes de design :
- Ring buffer 256 (cycle_id % 256) sans mutex pour éviter allocation dynamique
- Normalisation par-vecteur pour stabilité inférence
- Safety filter découplé en obs_builder/safety_filter (séparation concerns)
