# Architecture Docker — MUTO RS Jetson + Raspberry Pi

## Vue d'ensemble

MUTO RS tourne désormais dans **deux containers Docker distincts** (l'un par machine), chacun avec son propre environnement isolé mais connecté au même réseau ROS 2 (DOMAIN_ID=33). L'architecture élimine complètement les chemins absolus codés en dur : tout est piloté par **injection de variables d'environnement** au démarrage des containers.

---

## 1. Prérequis

### Composants obligatoires
- **Docker** installé et configurable (ou podman)
- **docker-compose** pour orchestration multi-container
- **Fichiers configuration** dans `docker/config/.env.*` présents
- **Code source** dans le volume bind mount du host

### Versions minimales
- Docker : 20.10+, ROS 2 Humble, Python 3.10+, C++17, CMake 3.8+

---

## 2. Fichiers .env — Configuration centralisée

### docker/config/.env.base (commun)
```bash
ROS_DOMAIN_ID=33
ROS_LOCALHOST_ONLY=0
ROS_DDS_PROFILE=cyclonedds
```

### docker/config/.env.raspberrypi (Pi)
```bash
ROBOT_ROLE=DRIVER
WORKING_DIR=/home/pi/TEKBOT_ROBOTICS
USER_HOME=/home/pi
TARGET_IP=10.0.0.2
CONTAINER_NAME=muto_driver_pi
```

### docker/config/.env.jetson_nano (Jetson)
```bash
ROBOT_ROLE=BRAIN
WORKING_DIR=/home/jetson/TEKBOT_ROBOTICS
USER_HOME=/home/jetson
TARGET_IP=10.0.0.1
CONTAINER_NAME=muto_brain_jetson
```

---

## 3. Variables d'environnement — Tableau de référence

| Variable | Source .env | Pi | Jetson | Utilisé par |
|----------|-------------|---|----|---|
| **WORKING_DIR** | .env.raspberrypi / .env.jetson_nano | `/home/pi/TEKBOT_ROBOTICS` | `/home/jetson/TEKBOT_ROBOTICS` | Tous nœuds, CMakeLists, launch |
| **ROBOT_ROLE** | .env.raspberrypi / .env.jetson_nano | `DRIVER` | `BRAIN` | setup scripts, validation |
| **TARGET_IP** | .env.raspberrypi / .env.jetson_nano | Jetson (10.0.0.2) | Pi (10.0.0.1) | system_params.yaml, heartbeat |
| **USER_HOME** | .env.raspberrypi / .env.jetson_nano | `/home/pi` | `/home/jetson` | Docker volumes |
| **DOMAIN_ID** | .env.base | `33` | `33` | ROS 2 communication |
| **LOCALHOST_ONLY** | .env.base | `0` | `0` | ROS 2 DDS config |

---

## 4. Structure du workspace

```
/home/pi/TEKBOT_ROBOTICS/                (ou /home/jetson)
├── tekbot_ws/
│   ├── src/
│   │   ├── muto_msgs/                  (both)
│   │   ├── muto_utils/                 (both)
│   │   ├── muto_hardware/              (Pi only)
│   │   │   ├── launch/pi_full.launch.py
│   │   │   └── config/system_params.yaml
│   │   ├── muto_control/               (Jetson only)
│   │   ├── muto_perception/            (Jetson only)
│   │   ├── muto_inference/             (Jetson only)
│   │   │   ├── rl_policy_node.py
│   │   │   └── models/model_card_v003.json
│   │   ├── muto_navigation/            (Jetson only)
│   │   └── muto_bringup/               (both)
│   │       ├── launch/jetson_full.launch.py
│   │       └── test/
│   ├── setup_pi.sh
│   ├── setup_jetson.sh
│   └── install/                        (après colcon build)
├── muto_install/                       (Pi only)
│   └── lib/libmuto_link_cpp_lib.so
└── models/                             (both)
    ├── muto_rs_v003.trt               (Jetson only)
    ├── norm_stats_v3.json
    └── model_card_v003.json
```

---

## 5. Règle "Zéro valeur codée en dur" — Exemples AVANT/APRÈS

### Python (launch file)
```python
# ❌ ANCIEN
params_file = "/home/pi/TEKBOT_ROBOTICS/tekbot_ws/src/muto_bringup/config/system_params.yaml"

# ✅ NOUVEAU
working_dir = os.environ.get('WORKING_DIR', '')
if not working_dir:
    raise RuntimeError("WORKING_DIR non défini")
params_file = os.path.join(working_dir, 'tekbot_ws', 'src', 'muto_bringup', 'config', 'system_params.yaml')
```

### C++ (nœud)
```cpp
// ❌ ANCIEN
std::string so_path = "/home/pi/TEKBOT_ROBOTICS/muto_install/lib/libmuto_link_cpp_lib.so";

// ✅ NOUVEAU
const char* working_dir = std::getenv("WORKING_DIR");
if (!working_dir || std::string(working_dir).empty()) {
    RCLCPP_FATAL(this->get_logger(), "WORKING_DIR non défini");
    throw std::runtime_error("WORKING_DIR non défini");
}
std::string so_path = std::string(working_dir) + "/muto_install/lib/libmuto_link_cpp_lib.so";
```

### YAML (paramètres)
```yaml
# ❌ ANCIEN
usb_bridge_node:
  ros__parameters:
    muto_lib_path: "/home/pi/TEKBOT_ROBOTICS/muto_install/lib/libmuto_link_cpp_lib.so"

# ✅ NOUVEAU
usb_bridge_node:
  ros__parameters:
    muto_lib_path: "$(env WORKING_DIR)/muto_install/lib/libmuto_link_cpp_lib.so"
```

---

## 6. Flux d'injection des variables

```
.env.raspberrypi (WORKING_DIR=/home/pi/...)
       ↓
docker-compose.yml (env_file: .env.raspberrypi)
       ↓
Container startup → Variables injectées ($WORKING_DIR)
       ↓
launch files (os.environ.get('WORKING_DIR'))
       ↓
system_params.yaml ($(env WORKING_DIR) interprété par ROS 2)
       ↓
Nœuds ROS 2 (lisent paramètres → démarrage avec chemins résolus)
```

---

## 7. Commandes de démarrage

### Raspberry Pi

**Démarrer le container:**
```bash
docker-compose -f docker/docker-compose.yml --profile raspberrypi up -d
docker exec -it muto_driver_pi /bin/bash
```

**Bootstrap (une fois):**
```bash
source /opt/ros/humble/setup.bash
cd tekbot_ws
export WORKING_DIR=/home/pi/TEKBOT_ROBOTICS
bash setup_pi.sh
```

**Lancer le système:**
```bash
source /opt/ros/humble/setup.bash
export WORKING_DIR=/home/pi/TEKBOT_ROBOTICS
export ROS_DOMAIN_ID=33
export ROS_LOCALHOST_ONLY=0
ros2 launch muto_bringup pi_full.launch.py
```

### Jetson Nano

**Démarrer le container:**
```bash
docker-compose -f docker/docker-compose.yml --profile jetson up -d
docker exec -it muto_brain_jetson /bin/bash
```

**Bootstrap (une fois):**
```bash
source /opt/ros/humble/setup.bash
cd tekbot_ws
export WORKING_DIR=/home/jetson/TEKBOT_ROBOTICS
bash setup_jetson.sh
```

**Lancer le système:**
```bash
source /opt/ros/humble/setup.bash
export WORKING_DIR=/home/jetson/TEKBOT_ROBOTICS
export ROS_DOMAIN_ID=33
export ROS_LOCALHOST_ONLY=0
ros2 launch muto_bringup jetson_full.launch.py
```

---

## 8. Bootstrap — setup_pi.sh et setup_jetson.sh

### setup_pi.sh
- [1/5] Validate ROBOT_ROLE != DRIVER → exit
- [2/5] Compile muto_link_cpp → $(WORKING_DIR)/muto_install
- [3/5] Verify symbol exports (dlsym validation)
- [4/5] Build ROS 2 workspace (muto_msgs + muto_hardware)
- [5/5] Verify isolcpus=3 in /boot/cmdline.txt

**Temps:** 5-10 minutes

### setup_jetson.sh
- [1/4] Create $(WORKING_DIR)/models directory
- [2/4] Build entire ROS 2 workspace
- [3/4] Verify TensorRT availability
- [4/4] Check RAM > 500 MB free

**Temps:** 15-25 minutes

---

## 9. Validation — État du workspace

| Vérification | État | Details |
|---|---|---|
| Chemins absolus résiduels | ✅ PASS (0) | grep -rn "/home/pi\|/home/jetson" = 0 |
| IPs codées en dur | ✅ PASS (0) | grep -rn IP patterns = 0 |
| Fallbacks concrets | ✅ PASS (0) | os.environ.get defaults = empty '' |
| additional_env Pi | ✅ 3/3 | 100% nodes |
| additional_env Jetson | ✅ 6/6 | 100% nodes |
| Build packages | ✅ 8/8 | Zero errors/warnings |

---

## 10. Dépannage

### "WORKING_DIR non défini"
- Vérifier `.env.raspberrypi` ou `.env.jetson_nano` existe
- Vérifier docker-compose.yml liste .env en `env_file:`
- Dans container: `echo $WORKING_DIR`

### "dlopen failed: libcuda.so"
- Vérifier CUDA/cuDNN dans image Docker (Jetson only)
- `ls /usr/lib/*/libcuda.so*`

### "Pas de communication Pi↔Jetson"
- Vérifier ROS_DOMAIN_ID=33 sur les deux
- Tester ping entre machines hôte
- Vérifier réseau Docker : `docker network ls`

### "Jitter > 5 ms"
- Pi: Vérifier /boot/cmdline.txt a `isolcpus=3`
- Reboot Pi pour activer
- Vérifier taskset : `taskset -c 3 bash`

---

## 11. Maintenance — À faire et À ne JAMAIS faire

### ✅ À faire
- Lancer bootstrap **une seule fois** par machine
- Vérifier `ROBOT_ROLE` au démarrage
- Garder `.env.base` synchronisé (DOMAIN_ID)
- Exécuter scripts dans le bon container

### ❌ À ne jamais faire
- Commiter `docker/config/.env.user` (secrets)
- Hardcoder chemins absolus ou IPs
- Modifier WORKING_DIR après bootstrap
- Lancer système sans sourcer ROS 2 setup.bash
