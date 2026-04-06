# muto_inference

## 1. Rôle dans l'architecture

Exécution de la politique RL (modèle TensorRT) sur GPU Jetson, 100 Hz. Gère les modes dry-run (simulation) et RL_ACTIVE (déploiement), warmup GPU (500 itérations stabilisation latence CUDA), supervision latence inférence. Decay progressif vers pose sûre en cas de dépassements répétés >5ms. Cœur apprentissage/inférence du système.

## 2. Machine cible

| Machine | Rôle |
|---------|------|
| Jetson (BRAIN) | Inférence GPU TensorRT 100 Hz |

## 3. Nœuds ROS 2

| Nom nœud | Langage | Fréquence | Rôle |
|----------|---------|-----------|------|
| rl_policy_node | Python | 100 Hz | Inférence politique RL + modes dry-run/RL_ACTIVE |

## 4. Topics

| Nom topic | Type | Fréquence | Direction | Description |
|-----------|------|-----------|-----------|-------------|
| /observation | muto_msgs/Observation | 100 Hz | Sub | Vecteur 70D depuis control obs_builder |
| /system_mode | std_msgs/String | Événementiel | Sub | Mode système (IDLE, READY, RL_ACTIVE, FAULT) |
| /commands_raw | muto_msgs/Commands | 100 Hz | Pub | Actions RL brutes [-1, 1] clippées |
| /commands_dry_run | muto_msgs/Commands | 100 Hz | Pub | Actions RL en mode apprentissage |
| /inference/timing_warn | std_msgs/Float32 | À la demande | Pub | Latence inférence (warn si >5ms) |
| /robot_status | std_msgs/String | 1 Hz | Pub | État robot (READY, RUNNING, DEGRADED, STOPPED) |

## 5. Variables d'environnement

| Variable | Valeur/Format | Description |
|----------|---------------|-------------|
| CUDA_DEVICE | 0 | Index GPU CUDA |
| TENSORRT_CACHE | /tmp/trt_cache | Répertoire cache TensorRT |
| RL_WARMUP_ITER | 500 | Itérations warmup stabilisation latence |

## 6. Paramètres ROS 2

| Nom paramètre | Type | Valeur défaut | Description |
|---------------|------|---------------|-------------|
| obs_dim | int | 70 | Dimension entrée réseau RL |
| action_dim | int | 18 | Nombre actions (18 servos) |
| action_scale_rad | float | 1.57 | Facteur scaling actions [-1,1] → [-π/2, π/2] |
| model_card_path | string | models/model_card_v003.json | Chemin fichier model_card |
| freeze_policy_after_step | int | 1000000 | Pas d'apprentissage on-device après N steps |
| dry_run_mode | bool | false | Mode simulation (pas de vrai robot) |
| inference_latency_warn_ms | float | 5.0 | Seuil alerte latence inférence |
| decay_rate | float | 0.95 | Taux decay vers safe_pose si latence dépassée |

## 7. Commandes de compilation

```bash
colcon build --symlink-install --packages-select muto_inference
```

## 8. Lancement + Dépendances

**Via jetson_full.launch.py:**
```bash
ros2 launch muto_bringup jetson_full.launch.py
```

Dépendances amont :
- muto_msgs (Observation, Commands, interfaces)
- muto_control (produit /observation via obs_builder)
- Modèle TensorRT compilé (model_card_v003.json + .engine)
- CUDA/cuDNN/TensorRT installés Jetson

Notes de design :
- Clip [-1,1] avant scaling pour robustesse
- Warmup 500 iter stabilise latence GPU ~2-3ms
- Decay progressif vers pose sûre si latence > seuil répété
