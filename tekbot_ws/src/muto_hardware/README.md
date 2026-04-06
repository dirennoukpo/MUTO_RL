# muto_hardware

## 1. Rôle dans l'architecture

Couche temps réel, responsable de l'acquisition capteurs (IMU, encodeurs) et commande actionneurs (18 servos) sur Raspberry Pi. Pilote la boucle critique 200 Hz, maintient FSM mode/état système (watchdog, supervision sécurité). Charge dynamiquement la bibliothèque hardware-specific (muto_link) via dlopen/dlsym pour découpler le time-critical du compile-time.

## 2. Machine cible

| Machine | Rôle |
|---------|------|
| Raspberry Pi (DRIVER) | Acquisition I/O USB + temps réel critique |

## 3. Nœuds ROS 2

| Nom nœud | Langage | Fréquence | Rôle |
|----------|---------|-----------|------|
| usb_bridge_node | C++ | 200 Hz | Acquisition IMU/joints + servo commande |
| watchdog_node | C++ | 100 Hz | Supervision sécurité (timeouts, chute) |
| mode_manager_node | C++ | Service | FSM transitions mode système |

## 4. Topics

| Nom topic | Type | Fréquence | Direction | Description |
|-----------|------|-----------|-----------|-------------|
| /joint_states | muto_msgs/StampedJointState | 200 Hz | Pub | État articulations (18 servos) + cycle_id |
| /imu/data | muto_msgs/StampedImu | 200 Hz | Pub | IMU (accel, gyro) + cycle_id |
| /hw/timing_jitter | std_msgs/Float32 | À la demande | Pub | Dépassement latence boucle >1ms |
| /commands | muto_msgs/Commands | 100 Hz | Sub | Consignes servo depuis Jetson |
| /commands_dry_run | muto_msgs/Commands | 100 Hz | Sub | Consignes simulation (mode apprentissage) |
| /jetson/heartbeat | std_msgs/Header | ~10 Hz | Sub | Heartbeat Jetson (timeout watchdog) |
| /system_mode | std_msgs/String | Événementiel | Pub | Mode système (IDLE, READY, RL_ACTIVE, FAULT) |

## 5. Variables d'environnement

| Variable | Valeur/Format | Description |
|----------|---------------|-------------|
| MUTO_LINK_SO | /path/to/libmuto_link.so | Chemin bibliothèque hardware (dlopen) |
| RT_PRIORITY | 90 | Priorité SCHED_FIFO pour usb_bridge_node |
| USB_BRIDGE_CPU | 0 | Affinité CPU core usb_bridge_node |

## 6. Paramètres ROS 2

| Nom paramètre | Type | Valeur défaut | Description |
|---------------|------|---------------|-------------|
| command_timeout_ms | int | 200 | Timeout réception commande sans expiration |
| heartbeat_timeout_ms | int | 300 | Timeout heartbeat Jetson avant FAULT |
| fall_threshold_g | float | 3.0 | Seuil accélération chute (IMU) |
| servo_count | int | 18 | Nombre de servos PID |
| watchdog_period_ms | int | 100 | Période watchdog (100 Hz) |

## 7. Commandes de compilation

```bash
colcon build --symlink-install --packages-select muto_hardware
```

## 8. Lancement + Dépendances

**Via pi_full.launch.py:**
```bash
ros2 launch muto_bringup pi_full.launch.py
```

Dépendances amont : muto_msgs (interfaces), muto_utils (diagnostics)

Dépendances runtime :
- libmuto_link.so (bibliothèque hardware, chargée dynamiquement)
- Drivers USB servo/IMU disponibles sur Pi

Notes de design :
- dlopen/dlsym évite couplage compile-time hardware
- SCHED_FIFO + affinité CPU pour latence déterministe
- Ring buffer 256 entrées (cycle_id % 256) sans mutex
