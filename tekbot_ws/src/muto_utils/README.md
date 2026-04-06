# muto_utils

## 1. Rôle dans l'architecture

Utilities partagées, diagnostiques robot, outils communs. Fourniture de fonctionnalités transversales (logging, profiling, health check) utilisées par tous les packages (Pi et Jetson). Node diagnostics agrège santé CPU/mémoire/thermique globale 1 Hz pour supervision système.

## 2. Machine cible

| Machine | Rôle |
|---------|------|
| Toutes (Pi + Jetson) | Utilities partagées + diagnostics sytème communs |

## 3. Nœuds ROS 2

| Nom nœud | Langage | Fréquence | Rôle |
|----------|---------|-----------|------|
| diagnostics_node | C++ | 1 Hz | Agrégation santé CPU/mémoire/thermique |

## 4. Topics

| Nom topic | Type | Fréquence | Direction | Description |
|-----------|------|-----------|-----------|-------------|
| /diagnostics | muto_msgs/SystemHealth | 1 Hz | Pub | État sytème (CPU%, mémoire, temp CPU, heartbeat) |

## 5. Variables d'environnement

| Variable | Valeur/Format | Description |
|----------|---------------|-------------|
| MUTO_LOG_LEVEL | DEBUG / INFO / WARN / ERROR | Niveau verbosité logging |
| MUTO_DIAGNOSTICS_INTERVAL_S | 1.0 | Intervalle collecte diagnostics (secondes) |

## 6. Paramètres ROS 2

| Nom paramètre | Type | Valeur défaut | Description |
|---------------|------|---------------|-------------|
| cpu_threshold_percent | float | 80.0 | Seuil alerte CPU% |
| memory_threshold_percent | float | 80.0 | Seuil alerte mémoire% |
| temp_threshold_c | float | 75.0 | Seuil alerte température CPU |
| heartbeat_timeout_s | float | 5.0 | Timeout heartbeat sytème global |
| diagnostics_enabled | bool | true | Activation node diagnostics |

## 7. Commandes de compilation

```bash
colcon build --symlink-install --packages-select muto_utils
```

## 8. Lancement + Dépendances

**Lancement via packages dépendants :**

Utilities compilées comme dépendance build pour tous les packages (muto_bringup, muto_hardware, muto_control, muto_inference, etc.).

**Node diagnostics autonome :**
```bash
ros2 run muto_utils diagnostics_node
```

Dépendances amont :
- muto_msgs (SystemHealth interface)
- ROS 2 Humble système

Dépendances aval :
- Tous les packages (dépendent muto_utils comme library)

Notes de design :
- Légère (minimale overhead CPU)
- Isolation thermique/CPU monitoring pour prévention throttling
- Heartbeat global aggrégé tous nœuds
