# Workspace ROS 2 MUTO RS — Jetson Nano + Raspberry Pi

## Vue d'ensemble

Implémentation complète de la pile MUTO RS (robotique temps réel) séparant le hardware (Pi, temps réel) et la décision (Jetson, IA/perception). Le système tourne entièrement dans containers Docker avec injection de configuration par variables d'environnement.

### Caractéristiques clés
- **Architecture distribuée** : Pi (hardware/temps réel) ↔ Jetson (IA/perception)
- **ROS 2 Humble** : communication inter-machines via DOMAIN_ID=33
- **Docker-first** : déploiement identique, configuration par profils .env
- **Zéro hardcoding** : tous chemins/IPs résolus à l'exécution
- **Temps réel** : boucle 200 Hz sur Pi (SCHED_FIFO, core isolé)
- **Validation complète** : protocole terrain 4 phases + audit build

---

## 1. Architecture fonctionnelle

```
┌─ RASPBERRY PI (ROBOT_ROLE=DRIVER) ──────────────────┐
│                                                      │
│ [muto_hardware] 200 Hz boucle RT                    │
│  ├─ usb_bridge_node      → USB/Dynamixel/IMU       │
│  ├─ watchdog_node         → Timeouts + fallback    │
│  └─ mode_manager_node     → FSM états             │
│                                                      │
│ Publie: /imu/data (200 Hz), /joint_states (200 Hz) │
│ Écoute: /commands (100 Hz depuis Jetson)           │
└──────────────────┬─────────────────────────────────┘
                   │ ROS 2 DOMAIN_ID=33 réseau
                   │
┌──────────────────┬─────────────────────────────────┐
│ JETSON NANO (ROBOT_ROLE=BRAIN)                     │
│                                                     │
│ [muto_control]      [muto_perception]              │
│  ├─ obs_builder     ├─ depth_processor            │
│  └─ safety_filter   └─ lidar_processor            │
│                                                     │
│ [muto_inference]    [muto_navigation]              │
│  ├─ rl_policy_node  └─ navigation_node            │
│  └─ TensorRT .trt                                  │
│                                                     │
│ Publie: /commands (100 Hz vers Pi)                 │
│ Écoute: /imu/data, /joint_states (200 Hz)          │
└─────────────────────────────────────────────────────┘
```

### Topics principal

| Topic | Fréquence | Publieur | Abonné |
|---|---|---|---|
| `/imu/data` | 200 Hz | usb_bridge (Pi) | obs_builder (Jetson) |
| `/joint_states` | 200 Hz | usb_bridge (Pi) | obs_builder (Jetson) |
| `/observation` | 100 Hz | obs_builder (Jetson) | safety_filter, rl_policy (Jetson) |
| `/commands` | 100 Hz | safety_filter (Jetson) | usb_bridge (Pi) |
| `/commands_raw` | 100 Hz | rl_policy (Jetson) | safety_filter (Jetson) |
| `/system_mode` | 1 Hz | mode_manager (Pi) | rl_policy (Jetson) |

---

## 2. Packages

| Package | Description | Machine |
|---|---|---|
| **muto_msgs** | Interfaces ROS 2 (msg/srv) | Both |
| **muto_utils** | Utilities + diagnostics | Both |
| **muto_hardware** | USB bridge, watchdog, mode FSM | Pi |
| **muto_control** | Observation builder, safety filter | Jetson |
| **muto_perception** | Depth/LiDAR processing | Jetson |
| **muto_inference** | TensorRT policy + safe pose | Jetson |
| **muto_navigation** | Trajectory planning | Jetson |
| **muto_bringup** | Launch scripts, configs, tests | Both |

---

## 3. Installation

### Pré-requis
- Docker 20.10+ (ou podman)
- docker-compose
- ROS 2 Humble (dans les containers)

### Clonage workspace
```bash
git clone <repo> /home/pi/TEKBOT_ROBOTICS/tekbot_ws
cd /home/pi/TEKBOT_ROBOTICS/tekbot_ws
```

### Configuration Docker
```bash
# Copier templates .env
cp docker/config/.env.base.template docker/config/.env.base
cp docker/config/.env.raspberrypi.template docker/config/.env.raspberrypi

# Adapter les chemins/IPs si nécessaire
# Voir section 2 dans DOCKER_ARCHITECTURE.md
```

---

## 4. Build (après bootstrap uniquement)

### Build global
```bash
cd tekbot_ws
source /opt/ros/humble/setup.bash
export WORKING_DIR=$(pwd)/..
colcon build --symlink-install
source install/setup.bash
```

### Build incrémental par package
```bash
colcon build --packages-select muto_msgs
colcon build --packages-select muto_hardware
colcon build --packages-select muto_control
colcon build --packages-select muto_inference
```

---

## 5. Exécution

### sur Raspberry Pi (1ère fois : bootstrap)
```bash
# Dans le container Docker muto_driver_pi
source /opt/ros/humble/setup.bash
export WORKING_DIR=/home/pi/TEKBOT_ROBOTICS
bash setup_pi.sh  # UNE FOIS seulement, ~5-10 min
```

### Sur Raspberry Pi (lancement régulier)
```bash
source /opt/ros/humble/setup.bash
export WORKING_DIR=/home/pi/TEKBOT_ROBOTICS
export ROS_DOMAIN_ID=33
ros2 launch muto_bringup pi_full.launch.py
```

### Sur Jetson Nano (1ère fois : bootstrap)
```bash
# Dans le container Docker muto_brain_jetson
source /opt/ros/humble/setup.bash
export WORKING_DIR=/home/jetson/TEKBOT_ROBOTICS
bash setup_jetson.sh  # UNE FOIS seulement, ~15-25 min
```

### Sur Jetson Nano (lancement régulier)
```bash
source /opt/ros/humble/setup.bash
export WORKING_DIR=/home/jetson/TEKBOT_ROBOTICS
export ROS_DOMAIN_ID=33
ros2 launch muto_bringup jetson_full.launch.py
```

---

## 6. Validation terrain (par phases)

### Phase 1 : Hardware Pi
Tests USB bridge, IMU, watchdog  
`ros2 launch muto_bringup test_phase1_pi.launch.py`

### Phase 2 : Control Jetson
Tests observation builder, safety filter  
`ros2 launch muto_bringup test_phase2_jetson.launch.py`

### Phase 3 : Perception
Tests depth/LiDAR processing (sans RL)  
`ros2 launch muto_bringup test_phase3_perception.launch.py`

### Phase 4 : AI/RL
Tests dry-run RL policy, cadence pipeline  
`ros2 launch muto_bringup test_phase4_ai.launch.py`

### Checklist opérationnelle
Voir: `src/muto_bringup/test/checklist_terrain.md`

---

## 7. Variables d'environnement critiques

| Variable | Source | Pi | Jetson | Gardes |
|---|---|---|---|---|
| `WORKING_DIR` | .env.raspberrypi/.env.jetson_nano | `/home/pi/...` | `/home/jetson/...` | RuntimeError si vide |
| `ROBOT_ROLE` | .env.raspberrypi/.env.jetson_nano | `DRIVER` | `BRAIN` | Exit setup si mismatch |
| `ROS_DOMAIN_ID` | .env.base | `33` | `33` | Doit correspondre pour comms |
| `ROS_LOCALHOST_ONLY` | .env.base | `0` | `0` | 0=inter-machine, 1=localhost |

Voir section 3 dans `DOCKER_ARCHITECTURE.md` pour tableau complet.

---

## 8. Architecture Docker

### Containers
- **muto_driver_pi** → Raspberry Pi (hardware/temps réel)
- **muto_brain_jetson** → Jetson Nano (IA/perception)

### Injection configuration
```
.env.raspberrypi/jetson → docker-compose → Container env variables
                        ↓
        src/muto_bringup/config/system_params.yaml $(env VAR)
                        ↓
                     Nœuds ROS 2
```

Voir `DOCKER_ARCHITECTURE.md` section 6 pour flux détaillé.

---

## 9. Documentation détaillée

- **DOCKER_ARCHITECTURE.md** : Configuration Docker, variables, dépannage
- **src/muto_bringup/README.md** : Launch files et paramètres
- **src/muto_hardware/README.md** : Details Pi (RT, USB)
- **src/muto_inference/README.md** : Details Jetson (TensorRT, RL)
- Tous fichiers source commentés en français (role, variables, guards)

---

## 10. Règles importantes

### ✅ À faire
1. Source ROS 2 setup.bash avant toute commande ROS
2. Exporter WORKING_DIR avant lancer system
3. Exécuter setup_pi.sh une seule fois par Pi (bootstrap)
4. Exécuter setup_jetson.sh une seule fois par Jetson (bootstrap)
5. Garder .env.base synchronisé (DOMAIN_ID)

### ❌ À ne jamais faire
1. Commiter docker/config/.env.user  
2. Hardcoder chemins absolus ou IPs dans code
3. Modifier WORKING_DIR après bootstrap
4. Lancer le system sans docker-compose (isolation)

---

## 11. Support

### Erreurs courantes

**"WORKING_DIR non défini"**
→ Vérifier `.env.raspberrypi` ou `.env.jetson_nano` contient WORKING_DIR

**"dlopen failed"**
→ Vérifier libmuto_link_cpp_lib.so bien compilée sur le Pi

**"Pas de communication Pi↔Jetson"**
→ Vérifier ROS_DOMAIN_ID=33 sur les deux machines  
→ Tester ping entre machines hôte

**"Jitter > 5 ms"**
→ Pi only : vérifier isolcpus=3 dans /boot/cmdline.txt

Voir `DOCKER_ARCHITECTURE.md` section 10 pour guide dépannage complet.
