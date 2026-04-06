# muto_navigation

## 1. Rôle dans l'architecture

Fusion d'obstacles perception (depth + LiDAR), génération consigne vitesse cible /navigation/goal_velocity 5 Hz compatible avec stabilité robot. Simple, déterministe, robuste. Alimente obs_builder (indices 64-66 du vecteur 70D). Assure compatibilité sécurité contrôle Jetson.

## 2. Machine cible

| Machine | Rôle |
|---------|------|
| Jetson (BRAIN) | Fusion obstacles + génération goal_velocity 5 Hz |

## 3. Nœuds ROS 2

| Nom nœud | Langage | Fréquence | Rôle |
|----------|---------|-----------|------|
| navigation_node | Python | 5 Hz | Fusion obstacles → goal_velocity |

## 4. Topics

| Nom topic | Type | Fréquence | Direction | Description |
|-----------|------|-----------|-----------|-------------|
| /depth/obstacles | muto_msgs/ObstacleList | 30 Hz | Sub | Obstacles détectés profondeur |
| /lidar/obstacles | muto_msgs/ObstacleList | 20 Hz | Sub | Obstacles détectés LiDAR |
| /navigation/goal_velocity | geometry_msgs/Twist | 5 Hz | Pub | Consigne vitesse (linear.x, angular.z) |

## 5. Variables d'environnement

| Variable | Valeur/Format | Description |
|----------|---------------|-------------|
| NAVIGATION_CPU | 3 | Affinité CPU core navigation_node |

## 6. Paramètres ROS 2

| Nom paramètre | Type | Valeur défaut | Description |
|---------------|------|---------------|-------------|
| goal_velocity_linear_max | float | 0.5 | Vitesse linéaire max (m/s) |
| goal_velocity_angular_max | float | 1.0 | Vitesse angulaire max (rad/s) |
| obstacle_fusion_ratio | float | 0.6 | Ratio poids fusion depth/lidar |
| avoidance_enabled | bool | true | Activation évitement d'obstacle |
| safety_margin_m | float | 0.2 | Marge sécurité autour obstacles |

## 7. Commandes de compilation

```bash
colcon build --symlink-install --packages-select muto_navigation
```

## 8. Lancement + Dépendances

**Via jetson_full.launch.py:**
```bash
ros2 launch muto_bringup jetson_full.launch.py
```

Dépendances amont :
- muto_msgs (ObstacleList, Twist)
- muto_perception (produit /depth/obstacles, /lidar/obstacles)
- muto_control (consomme /navigation/goal_velocity → obs_builder)

Notes de design :
- Fusion déterministe (pas ML) → robustesse sûreté
- 5 Hz suffit (observation 100 Hz utilise goal_velocity)
- Indices 64-66 observation = goal_velocity normalized
