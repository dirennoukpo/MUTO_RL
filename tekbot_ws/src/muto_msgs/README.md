# muto_msgs

## 1. Rôle dans l'architecture

Package d'interfaces ROS 2 centralisé. Définit les messages et services custom pour synchronisation tight des cycles temps réel (200 Hz Pi, 100 Hz Jetson) entre Raspberry Pi et Jetson. Stockage unique évite duplication/inconsistance. Compilé une fois, utilisé par tous les nœuds.

## 2. Machine cible

| Machine | Rôle |
|---------|------|
| Toutes (Pi + Jetson) | Définitions compilées + partagées globalement |

## 3. Nœuds ROS 2

| Nom nœud | Langage | Fréquence | Rôle |
|----------|---------|-----------|------|
| (Aucun) | N/A | N/A | Package interfaces (pas de nœuds) |

## 4. Topics

| Nom topic | Type | Fréquence | Direction | Description |
|-----------|------|-----------|-----------|-------------|
| (Aucun) | N/A | N/A | N/A | Définitions génériques (pas de topics) |

## 5. Variables d'environnement

| Variable | Valeur/Format | Description |
|----------|---------------|-------------|
| N/A | N/A | Pas de variables d'environnement spécifiques |

## 6. Paramètres ROS 2

| Nom paramètre | Type | Valeur défaut | Description |
|---------------|------|---------------|-------------|
| N/A | N/A | N/A | Pas de paramètres (interfaces statiques) |

## 7. Commandes de compilation

```bash
colcon build --symlink-install --packages-select muto_msgs
```

## 8. Lancement + Dépendances

Ce package est compilé en tant que dépendance build pour tous les autres packages. Pas de lancement direct.

**Messages définis :**
- `StampedImu.msg` : IMU encapsulée avec cycle_id explicite (Pi 200 Hz)
- `StampedJointState.msg` : État articulaire (18 servos) + cycle_id (Pi 200 Hz)
- `Commands.msg` : Commande 18 articulations + timestamp émission
- `Observation.msg` : Vecteur observation 70D normalisé (Jetson → RL)
- `Obstacle.msg`, `ObstacleList.msg` : Objets détectés perception
- `HeightMap.msg` : Carte locale 2.5D profondeur
- `SystemHealth.msg` : État système (mémoire, heartbeat, score)

**Services définis :**
- `ModeRequest.srv` : Transition mode système (IDLE → READY → RL_ACTIVE)

**Design :**
- cycle_id champ dédié (jamais via header.frame_id) → synchronisation explicite
- Types encapsulés évitent ambiguïtés multi-topics
