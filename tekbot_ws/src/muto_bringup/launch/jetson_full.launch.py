#!/usr/bin/env python3
"""
════════════════════════════════════════════════════════════════
jetson_full.launch.py — Orchestration contrôle + IA + perception sur Jetson Nano
════════════════════════════════════════════════════════════════

Machine cible : Jetson Nano (ROBOT_ROLE=BRAIN)

Variables d'environnement requises (injectées par Docker) :
  WORKING_DIR   → chemin racine du projet
                  Source : docker/config/.env.jetson_nano
                  Exemple Jetson : /home/jetson/TEKBOT_ROBOTICS
                  À l'exécution du launch : validation + construction chemin params_file

  ROBOT_ROLE    → rôle de la machine (DRIVER ou BRAIN)
                  Source : docker/config/.env.jetson_nano
                  Valeur sur Jetson : BRAIN
                  Guard : RuntimeError si absent ou valeur invalide

  TARGET_IP     → IP du Raspberry Pi (machine distante)
                  Source : docker/config/.env.jetson_nano
                  Exemple : 10.0.0.1
                  Guard : RuntimeError si absent

  DOMAIN_ID     → ROS_DOMAIN_ID partagé entre Pi et Jetson
                  Source : docker/config/.env.base
                  Valeur configurée : 33
                  Guard : RuntimeError si absent

  LOCALHOST_ONLY → booléen pour mode intra-machine
                  Source : docker/config/.env.base
                  Valeur configurée : 0 (communication inter-machines)
                  Guard : RuntimeError si absent

Si une variable requise est absente :
  → RuntimeError levée immédiatement au démarrage du nœud
  → Le container s'arrête proprement avec un message explicite
  → Ne jamais continuer avec une valeur silencieusement vide

Nœuds lancés (6 en total) :
  [1] obs_builder_node        (C++)     Publie : /observation (100 Hz)
                                        Souscrit : /imu/data, /joint_states
                                        Rôle : agrégation état (IMU + j_states) → vecteur normalisation

  [2] safety_filter_node      (C++)     Publie : /commands (100 Hz)
                                        Souscrit : /commands_raw, /observation
                                        Rôle : filtrage + limites vitesse/courant/jerk

  [3] depth_processor_node    (C++)     Publie : /depth/processed
                                        Souscrit : /camera/depth
                                        Affinity : taskset -c 0,1
                                        Rôle : traitement RGB-D parallélisé

  [4] lidar_processor_node    (C++)     Publie : /scan/processed
                                        Souscrit : /lidar/scan
                                        Affinity : taskset -c 2
                                        Rôle : traitement LiDAR agrégé

  [5] rl_policy_node          (Python)  Publie : /commands_raw (100 Hz), /robot_status
                                        Souscrit : /observation, /system_mode
                                        Rôle : inference TensorRT, pilotage RL
                                        Model : muto_rs_v003.trt (compilé pour Jetson)

  [6] navigation_node         (Python)  Publie : /trajectory
                                        Souscrit : /commands_raw, /depth/processed, /scan/processed
                                        Rôle : planner haut niveau, évitement obstacles

Fichier de configuration :
  system_params.yaml : centralize les paramètres pour les 6 nœuds
  Résolu via WORKING_DIR → $(WORKING_DIR}/tekbot_ws/src/muto_bringup/config/system_params.yaml
  Valeurs dynamiques : $(env WORKING_DIR), $(env TARGET_IP) résolues par ROS 2 au lancement

════════════════════════════════════════════════════════════════
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    # ── Guard variables d'environnement ───────────────────────────────
    # Chaque variable est validée séquentiellement.
    # Si une est absente ou vide → RuntimeError immédiate.
    # Cette vérification s'exécute avant la construction des Node().

    # WORKING_DIR : chemin racine du projet sur cette machine.
    # Injecté par Docker depuis docker/config/.env.jetson_nano.
    # Exemple Jetson : /home/jetson/TEKBOT_ROBOTICS
    # Guard : RuntimeError si absent — le nœud refuse de démarrer.
    working_dir = os.environ.get('WORKING_DIR', '')
    if not working_dir:
        raise RuntimeError(
            "[muto] WORKING_DIR non défini. Ce launch doit tourner dans le container Docker. "
            "Vérifiez docker/config/.env.jetson_nano."
        )
    
    # DOMAIN_ID : identifiant de domaine ROS 2 partagé entre Pi et Jetson.
    # Injecté par Docker depuis docker/config/.env.base.
    # Valeur configurée : 33
    # Guard : RuntimeError si absent.
    domain_id = os.environ.get('DOMAIN_ID', '')
    if not domain_id:
        raise RuntimeError("[muto] DOMAIN_ID non défini. Vérifiez docker/config/.env.base.")
    
    # LOCALHOST_ONLY : booléen pour mode intra-machine (1) ou inter-machines (0).
    # Injecté par Docker depuis docker/config/.env.base.
    # Valeur configurée : 0 (permet communication Pi↔Jetson)
    # Guard : RuntimeError si absent.
    localhost_only = os.environ.get('LOCALHOST_ONLY', '')
    if not localhost_only:
        raise RuntimeError("[muto] LOCALHOST_ONLY non défini. Vérifiez docker/config/.env.base.")
    
    # ROBOT_ROLE : rôle de la machine (DRIVER pour Pi, BRAIN pour Jetson).
    # Injecté par Docker depuis docker/config/.env.jetson_nano.
    # Valeur sur Jetson : BRAIN
    # Guard : RuntimeError si absent.
    robot_role = os.environ.get('ROBOT_ROLE', '')
    if not robot_role:
        raise RuntimeError("[muto] ROBOT_ROLE non défini. Vérifiez le profil .env machine.")
    
    # TARGET_IP : IP de la machine distante (le Raspberry Pi).
    # Injecté par Docker depuis docker/config/.env.jetson_nano.
    # Exemple : 10.0.0.1
    # Guard : RuntimeError si absent.
    target_ip = os.environ.get('TARGET_IP', '')
    if not target_ip:
        raise RuntimeError("[muto] TARGET_IP non défini. Vérifiez le profil .env machine.")
    
    # ── Construction du chemin système_params.yaml ───────────────────
    # system_params.yaml contient les paramètres ROS 2 pour les 6 nœuds.
    # Chemin : $(WORKING_DIR}/tekbot_ws/src/muto_bringup/config/system_params.yaml
    params_file = os.path.join(working_dir, 'tekbot_ws', 'src',
                               'muto_bringup', 'config', 'system_params.yaml')
    
    # ── Propagation des variables d'environnement aux nœuds enfants ─────
    # Les nœuds lancés héritent le processus parent, mais ROS 2 ne propage
    # pas automatiquement les variables d'environnement.
    # additional_env : force la propagation manuelle.
    # Chaque clé doit exister dans le parent (os.environ).
    additional_env = {
        'ROS_DOMAIN_ID': os.environ.get('DOMAIN_ID', ''),
        'ROS_LOCALHOST_ONLY': os.environ.get('LOCALHOST_ONLY', ''),
        'ROBOT_ROLE': os.environ.get('ROBOT_ROLE', ''),
        'TARGET_IP': os.environ.get('TARGET_IP', ''),
        'WORKING_DIR': os.environ.get('WORKING_DIR', ''),
    }
    
    return LaunchDescription(
        [
            # ── Nœud 1 : obs_builder_node ────────────────────────────────
            # obs_builder_node : agrégation de l'état du système.
            # Rôle : combine IMU (200 Hz) + joint_states (200 Hz) → observation 70D
            # Normalise avec norm_stats_v3.json (min/max/mean/std par dimension).
            # Buffer circulaire : cicatrisation IMU/joints → synchronisation parfaite.
            # Publie : /observation (100 Hz, sync avec safety_filter et RL)
            # Souscrit : /imu/data (200 Hz), /joint_states (200 Hz), /system_mode
            # Paramètres : norm_stats_path résolue depuis WORKING_DIR via system_params.yaml
            # additional_env : propage WORKING_DIR, DOMAIN_ID, ROBOT_ROLE, TARGET_IP.
            Node(
                package="muto_control",
                executable="obs_builder_node",
                output="screen",
                parameters=[params_file],
                additional_env=additional_env,
            ),
            
            # ── Nœud 2 : safety_filter_node ──────────────────────────────
            # safety_filter_node : couche de validation et limitation.
            # Rôle : filtre commandes brutes du RL avant transmission au Pi.
            # Limitations :
            #   - Vitesse maxi par articulation (rad/s)
            #   - Courant maxi par articulation (Ampères)
            #   - Rate limit : limitation delta vitesse par cycle (degrés/cycle)
            #   - Détection oscillation : freeze si amplitude diverge
            #   - Détection perte équilibre : via seuil CoM
            # Publie : /commands (100 Hz, envoyée au Pi via topic ROS 2)
            # Souscrit : /commands_raw (100 Hz depuis RL), /observation
            # Fallback : si /commands_raw absent > timeout → SAFE_POSE envoyée.
            # Paramètres : max_safe_velocity, max_safe_current, etc. dans system_params.yaml
            # additional_env : même propagation.
            Node(
                package="muto_control",
                executable="safety_filter_node",
                output="screen",
                parameters=[params_file],
                additional_env=additional_env,
            ),
            
            # ── Nœud 3 : depth_processor_node ────────────────────────────
            # depth_processor_node : traitement RGB-D parallélisé.
            # Rôle : extraction features (edges, obstacles) des images depth.
            # Affinity : taskset -c 0,1 (cores 0-1 dédiés, isolcpus compatible).
            # Publie : /depth/processed (15 Hz throttled), /depth/debug (30 Hz)
            # Souscrit : /camera/depth (30 Hz brut)
            # Rate limit : 30 Hz input, 15 Hz output (réduction réseau).
            # Paramètres : rate_hz=30, throttle_rate_hz=15 dans system_params.yaml
            # additional_env : même propagation.
            Node(
                package="muto_perception",
                executable="depth_processor_node",
                output="screen",
                prefix="taskset -c 0,1",
                parameters=[params_file],
                additional_env=additional_env,
            ),
            
            # ── Nœud 4 : lidar_processor_node ────────────────────────────
            # lidar_processor_node : traitement LiDAR 2D/3D.
            # Rôle : détection obstacles, clustering, extraction zone libre.
            # Affinity : taskset -c 2 (core 2 dédié, isolation).
            # Publie : /scan/processed (10 Hz), /lidar/obstacles
            # Souscrit : /lidar/scan (10 Hz rotation complète)
            # Paramètres : rate_hz=10 dans system_params.yaml
            # additional_env : même propagation.
            Node(
                package="muto_perception",
                executable="lidar_processor_node",
                output="screen",
                prefix="taskset -c 2",
                parameters=[params_file],
                additional_env=additional_env,
            ),
            
            # ── Nœud 5 : rl_policy_node ──────────────────────────────────
            # rl_policy_node : moteur d'inference TensorRT.
            # Rôle : exécute le modèle RL compilé (muto_rs_v003.trt) sur observation 70D.
            # Modèle : TensorRT engine optimisé pour Jetson Nano (INT8 quantization).
            # Entrée : observation normalisée (70D)
            # Sortie : commandes brutes (18D articulaires)
            # Réchauffage : warmup cycles avant premier appel inference
            # Validation model_card : flags sim_real_validated, dry_run_validated.
            # Mode dry_run : si dry_run=true → commandes non envoyées au Pi.
            # Publie : /commands_raw (100 Hz), /inference/timing_warn (événement), /robot_status
            # Souscrit : /observation (100 Hz), /system_mode
            # Paramètres : model_path, model_card_path, norm_stats_path résolues depuis WORKING_DIR
            # additional_env : même propagation.
            Node(
                package="muto_inference",
                executable="rl_policy_node",
                output="screen",
                parameters=[params_file],
                additional_env=additional_env,
            ),
            
            # ── Nœud 6 : navigation_node ─────────────────────────────────
            # navigation_node : planner trajectoire haut niveau.
            # Rôle : trajectoire autonome basée sur observations perception.
            # Entrée : commandes brutes RL + obstacles perception
            # Sorties : trajectoire filtrée
            # Boucle : 5 Hz planning (asynchrone avec 100 Hz cycle RL)
            # Évitement : fusion depth + lidar pour zone libre 3D.
            # Publie : /trajectory, /navigation/status (5 Hz)
            # Souscrit : /commands_raw, /depth/processed, /scan/processed, /observation
            # Paramètres : loop_hz=5 dans system_params.yaml
            # additional_env : même propagation.
            Node(
                package="muto_navigation",
                executable="navigation_node",
                output="screen",
                parameters=[params_file],
                additional_env=additional_env,
            ),
        ]
    )
