# muto_bringup

## 1. Rôle dans l'architecture

Package d'orchestration du système MUTO RS. Centralize les fichiers de lancement (launch files) pour Pi et Jetson, les paramètres système partagés (system_params.yaml), et les outils de validation terrain. Ce package ne contient aucun nœud ROS 2 exécutable, mais pilote l'ensemble du middleware via des fichiers de configuration déclaratifs.

## 2. Machine cible

| Machine | Rôle |
|---------|------|
| Workstation / Orchestration | Centralisation configuration + lancement |

## 3. Nœuds ROS 2

| Nom nœud | Langage | Fréquence | Rôle |
|----------|---------|-----------|------|
| (Aucun) | N/A | N/A | Package launch pur (pas de nœuds) |

## 4. Topics

| Nom topic | Type | Fréquence | Direction | Description |
|-----------|------|-----------|-----------|-------------|
| (Aucun) | N/A | N/A | N/A | Package launch — aucune publication directe |

## 5. Variables d'environnement

| Variable | Valeur/Format | Description |
|----------|---------------|-------------|
| ROS_DOMAIN_ID | 0 | Domaine ROS 2 par défaut |
| ROBOT_ROLE | DRIVER / BRAIN | Mode exécution : Pi (DRIVER) ou Jetson (BRAIN) |
| MUTO_WS | /home/[user]/TEKBOT_ROBOTICS_BENIN/MUTO_RL/tekbot_ws | Chemin workspace (source depuis setup.bash) |

## 6. Paramètres ROS 2

| Nom paramètre | Type | Valeur défaut | Description |
|---------------|------|---------------|-------------|
| command_timeout_ms | int | 200 | Timeout maximum pour réception commande |
| heartbeat_timeout_ms | int | 300 | Timeout heartbeat Jetson (watchdog) |
| fall_threshold_g | float | 3.0 | Seuil accélération chute en g |

Paramètres complets définis dans `config/system_params.yaml`.

## 7. Commandes de compilation

```bash
colcon build --symlink-install --packages-select muto_bringup
```

## 8. Lancement + Dépendances

**Pi (DRIVER):**
```bash
ros2 launch muto_bringup pi_full.launch.py
```
Dépendances : muto_hardware, muto_msgs, muto_utils

**Jetson (BRAIN):**
```bash
ros2 launch muto_bringup jetson_full.launch.py
```
Dépendances : muto_control, muto_inference, muto_perception, muto_navigation, muto_msgs, muto_utils

Contenu du package :
- `launch/pi_full.launch.py` : pipeline Pi (hardware + sécurité)
- `launch/jetson_full.launch.py` : pipeline Jetson (contrôle + perception + RL + navigation)
- `config/system_params.yaml` : paramètres centraux par nœud
- `test/` : suite de validation terrain phasée
