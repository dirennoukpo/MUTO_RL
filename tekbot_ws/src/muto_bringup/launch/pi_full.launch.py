#!/usr/bin/env python3
"""
════════════════════════════════════════════════════════════════
pi_full.launch.py — Orchestration hardware temps réel sur Raspberry Pi
════════════════════════════════════════════════════════════════

Machine cible : Raspberry Pi (ROBOT_ROLE=DRIVER)

Variables d'environnement requises (injectées par Docker) :
  WORKING_DIR   → chemin racine du projet
                  Source : docker/config/.env.raspberrypi
                  Exemple Pi : /home/pi/TEKBOT_ROBOTICS
                  À l'exécution du launch : validation + construction chemin params_file

  ROBOT_ROLE    → rôle de la machine (DRIVER ou BRAIN)
                  Source : docker/config/.env.raspberrypi
                  Valeur sur Pi : DRIVER
                  Guard : RuntimeError si absent ou valeur invalide

  TARGET_IP     → IP de la Jetson (machine distante)
                  Source : docker/config/.env.raspberrypi
                  Exemple : 10.0.0.2
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

Nœuds lancés :
  [1] usb_bridge_node  (C++)     Publie : /imu/data (200 Hz), /joint_states (200 Hz)
                                 Souscrit : /commands (100 Hz)
                                 Rôle : communication USB—Dynamixel, lecture IMU, temps réel
                                 Affinity : thread RT SCHED_FIFO priorité 90, CPU core 3

  [2] watchdog_node    (C++)     Publie : /system_status, /hw/timing_jitter
                                 Souscrit : /commands, /heartbeat
                                 Rôle : supervision timeouts, détection chute, fallback sécurisé

  [3] mode_manager_node (C++)    Publie : /system_mode, /transition_logs
                                 Souscrit : /system_events
                                 Rôle : FSM pour états (INIT, IDLE, DRY_RUN, RL_ACTIVE, SAFE)
                                 Validation : flags model_card avant transitions

Fichier de configuration :
  system_params.yaml : centralize les paramètres pour les 3 nœuds
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
    # Injecté par Docker depuis docker/config/.env.raspberrypi.
    # Exemple Pi : /home/pi/TEKBOT_ROBOTICS
    # Guard : RuntimeError si absent — le nœud refuse de démarrer.
    working_dir = os.environ.get('WORKING_DIR', '')
    if not working_dir:
        raise RuntimeError(
            "[muto] WORKING_DIR non défini. Ce launch doit tourner dans le container Docker. "
            "Vérifiez docker/config/.env.raspberrypi."
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
    # Injecté par Docker depuis docker/config/.env.raspberrypi.
    # Valeur sur Pi : DRIVER
    # Guard : RuntimeError si absent.
    robot_role = os.environ.get('ROBOT_ROLE', '')
    if not robot_role:
        raise RuntimeError("[muto] ROBOT_ROLE non défini. Vérifiez le profil .env machine.")
    
    # TARGET_IP : IP de la machine distante (la Jetson).
    # Injecté par Docker depuis docker/config/.env.raspberrypi.
    # Exemple : 10.0.0.2
    # Guard : RuntimeError si absent.
    target_ip = os.environ.get('TARGET_IP', '')
    if not target_ip:
        raise RuntimeError("[muto] TARGET_IP non défini. Vérifiez le profil .env machine.")
    
    # ── Construction du chemin système_params.yaml ───────────────────
    # system_params.yaml contient les paramètres ROS 2 pour les 3 nœuds.
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
            # ── Nœud 1 : usb_bridge_node ─────────────────────────────────
            # usb_bridge_node : nœud critique temps réel sur le Pi.
            # Rôle : communication USB↔Dynamixel, lecture IMU 6-axe, timing RT.
            # Thread RT : SCHED_FIFO priorité 90, épinglé sur CPU core 3 (isolcpus=3).
            # Publie : /imu/data (200 Hz), /joint_states (200 Hz), /hw/timing_jitter (événement)
            # Souscrit : /commands (100 Hz, timeout watchdog 50ms)
            # additional_env : propage TARGET_IP, DOMAIN_ID, WORKING_DIR, ROBOT_ROLE
            # au processus enfant depuis les variables Docker.
            # Parameters : lues depuis system_params.yaml via ROS 2 declare_parameter.
            Node(
                package="muto_hardware",
                executable="usb_bridge_node",
                output="screen",
                parameters=[params_file],
                additional_env=additional_env,
            ),
            
            # ── Nœud 2 : watchdog_node ───────────────────────────────────
            # watchdog_node : supervision temps réel et détection perte comms.
            # Rôle : timeouts commandes, détection chute via accel, fallback sécurisé SAFE_POSE.
            # Publie : /system_status (événement urgence), /heartbeat (1 Hz vers Jetson)
            # Souscrit : /commands (100 Hz), /heartbeat (depuis Jetson détectée si absent)
            # Fallback : si /commands absent > 50ms ou accel > threshold → SAFE_POSE envoyée.
            # additional_env : même propagation qu'usb_bridge_node.
            # Parameters : lues depuis system_params.yaml.
            Node(
                package="muto_hardware",
                executable="watchdog_node",
                output="screen",
                parameters=[params_file],
                additional_env=additional_env,
            ),
            
            # ── Nœud 3 : mode_manager_node ───────────────────────────────
            # mode_manager_node : machine à états (FSM) pour contraintes système.
            # Rôle : transitions INIT→IDLE→DRY_RUN→RL_ACTIVE avec validation flags.
            # États :
            #   INIT        → bootstrap, test séquence Dynamixel
            #   IDLE        → watchdog actif, pas d'inference
            #   DRY_RUN     → inference RL active mais commandes non exécutées au Pi
            #   RL_ACTIVE   → inference RL + commande → Pi en temps réel
            #   SAFE        → fallback urgence après timeout/chute
            # Consulte : model_card_v003.json pour flags sim_real_validated, dry_run_validated
            # Publie : /system_mode (topic String, 1 Hz), /transition_logs (debug)
            # Souscrit : paramètres ROS 2 pour gestion transitions
            # additional_env : même propagation.
            # Parameters : model_card_path résolue depuis WORKING_DIR via system_params.yaml.
            Node(
                package="muto_hardware",
                executable="mode_manager_node",
                output="screen",
                parameters=[params_file],
                additional_env=additional_env,
            ),
        ]
    )
