# muto_perception

## 1. Rôle dans l'architecture

Traitement perception capteurs (caméra profondeur, LiDAR) sur Jetson. Extraction obstacles locaux, height maps 2.5D, filtrages bas-bruit. Dégradation dynamique fréquence si throttling thermique détecté. Feeds navigation + safety checks. Isolé CPU pour ne pas perturber contrôle 100 Hz.

## 2. Machine cible

| Machine | Rôle |
|---------|------|
| Jetson (BRAIN) | Traitement capteurs perception + extraction obstacles |

## 3. Nœuds ROS 2

| Nom nœud | Langage | Fréquence | Rôle |
|----------|---------|-----------|------|
| depth_processor_node | C++ | 15-30 Hz | Pipeline profondeur → obstacles + height map |
| lidar_processor_node | C++ | 10-20 Hz | Filtrage LiDAR + clustering obstacles <1m |

## 4. Topics

| Nom topic | Type | Fréquence | Direction | Description |
|-----------|------|-----------|-----------|-------------|
| /camera/depth/image_raw | sensor_msgs/Image | 30 Hz | Sub | Image profondeur brute (capteur) |
| /scan | sensor_msgs/LaserScan | 20 Hz | Sub | Scan LiDAR brut |
| /depth/pointcloud | sensor_msgs/PointCloud2 | 15-30 Hz | Pub | Nuage points profondeur filtré |
| /depth/obstacles | muto_msgs/ObstacleList | 15-30 Hz | Pub | Obstacles détectés profondeur |
| /depth/height_map | muto_msgs/HeightMap | 15-30 Hz | Pub | Carte hauteur 2.5D locale |
| /lidar/scan_filtered | sensor_msgs/LaserScan | 10-20 Hz | Pub | Scan LiDAR filtré bruit |
| /lidar/obstacles | muto_msgs/ObstacleList | 10-20 Hz | Pub | Obstacles clustering LiDAR <1m |
| /perception/thermal_throttle | std_msgs/Bool | 1 Hz | Pub | Flag dégradation thermique |

## 5. Variables d'environnement

| Variable | Valeur/Format | Description |
|----------|---------------|-------------|
| DEPTH_PROC_CPU | 4 | Affinité CPU core depth_processor |
| LIDAR_PROC_CPU | 5 | Affinité CPU core lidar_processor |
| THERMAL_LIMIT_C | 75 | Seuil thermique dégradation fréquence |

## 6. Paramètres ROS 2

| Nom paramètre | Type | Valeur défaut | Description |
|---------------|------|---------------|-------------|
| depth_fps_nominal | int | 30 | FPS caméra profondeur nominal |
| depth_fps_degraded | int | 15 | FPS minimal dégradation thermique |
| lidar_fps_nominal | int | 20 | FPS LiDAR nominal |
| lidar_fps_degraded | int | 10 | FPS minimal dégradation thermique |
| obstacle_min_height_m | float | 0.05 | Hauteur minimale obstacle détecté |
| obstacle_max_range_m | float | 2.0 | Portée maximale obstacles |
| height_map_resolution | float | 0.05 | Résolution grid height map (m/cell) |
| thermal_check_interval_s | float | 5.0 | Intervalle vérification thermique |

## 7. Commandes de compilation

```bash
colcon build --symlink-install --packages-select muto_perception
```

## 8. Lancement + Dépendances

**Via jetson_full.launch.py:**
```bash
ros2 launch muto_bringup jetson_full.launch.py
```

Dépendances amont :
- muto_msgs (ObstacleList, HeightMap)
- Drivers caméra profondeur (ROS 2 node sources /camera/depth/image_raw)
- Drivers LiDAR (ROS 2 node sources /scan)

Dépendances aval :
- muto_navigation (consomme /depth/obstacles, /lidar/obstacles)

Notes de design :
- Affinité CPU isolée (cores 4-5) pour ne pas perturber contrôle (cores 1-2)
- Dégradation dynamique fréquence si throttling détecté
- Filtrage robuste bruit (voxel downsampling, outlier removal)
