#!/usr/bin/env python3
"""
════════════════════════════════════════════════════════════════
rl_policy_node.py — Moteur d'inférence TensorRT pour MUTO RS
════════════════════════════════════════════════════════════════

RÔLE DANS L'ARCHITECTURE :
  Nœud Python ROS 2 qui pilote le moteur d'inférence TensorRT.
  Exécute le modèle RL compilé (muto_rs_v003.trt) à 100 Hz.
  Consomme un vecteur d'observation (70 dimensions) normalisé.
  Produit des actions (18 dimensions articulaires) avec validation et fallback.

MACHINE CIBLE :
  Jetson Nano (ROBOT_ROLE=BRAIN)

FRÉQUENCE :
  - Souscription /observation : 100 Hz (callback-driven)
  - Publication /commands_raw : 100 Hz (si RL_ACTIVE) ou /commands_dry_run (sinon)
  - Publication /robot_status : 100 Hz (status RL_ACTIVE ou DRY_RUN)

VARIABLES D'ENVIRONNEMENT (Docker) :
  WORKING_DIR     → Racine projet, utilisée pour construire chemins modèles
                    Source : docker/config/.env.jetson_nano
                    Exemple : /home/jetson/TEKBOT_ROBOTICS
                    Guard : RuntimeError levée à l'import si absent/vide
                    Chemins modèles :
                      $(WORKING_DIR)/models/muto_rs_v003.trt (TensorRT engine)
                      $(WORKING_DIR)/models/model_card_v003.json (metadata)
                      $(WORKING_DIR)/models/norm_stats_v3.json (normalisation)

  ROBOT_ROLE      → Validation machine (doit être BRAIN sur Jetson)
                    Source : docker/config/.env.jetson_nano

  TARGET_IP       → IP Raspberry Pi (pas utilisée directement ici)
                    Source : docker/config/.env.jetson_nano

  DOMAIN_ID       → ROS 2 DOMAIN_ID partagé (valeur : 33)
                    Source : docker/config/.env.base

TOPICS PUBLIÉS :
  /inference/timing_warn  (std_msgs/Float32)
                          Publié si inference > 5 ms.
                          Valeur : temps latence (ms).

  /commands_raw           (muto_msgs/Commands)  SÉCURISÉ
                          Publié SEULEMENT si :
                            • RL_ACTIVE (pas dry_run)
                            • model_card flags OK
                            • system_mode == "RL_ACTIVE"
                          Utilisé par safety_filter → /commands → Pi → servos

  /commands_dry_run       (muto_msgs/Commands)
                          Publié si NOT RL_ACTIVE.
                          À fin de debug, NE PAS envoyer au Pi.

  /robot_status           (std_msgs/String)
                          Données : "RL_ACTIVE" ou "DRY_RUN"

TOPICS SOUSCRITS :
  /observation            (muto_msgs/Observation)    100 Hz
                          Vecteur normalisé (70D) depuis obs_builder_node.
                          Callback d'inférence synchrone.

  /system_mode            (std_msgs/String)
                          Reçoit transitions FSM (INIT→DRY_RUN→RL_ACTIVE→SAFE).
                          Stockée dans self.system_mode pour gating logique.

PARAMÈTRES ROS 2 (système_params.yaml) :
  obs_dim                 Dimension vecteur observation (défaut : 70)
  action_scale_rad        Facteur scale actions (défaut : 1.0) — en radians
  dry_run                 Booléen mode sécurisé (défaut : true)
  model_path              Chemin modèle TensorRT (défaut : $(WORKING_DIR)/models/muto_rs_v003.trt)
  model_card_path         Chemin carte identité modèle (défaut : $(WORKING_DIR)/models/model_card_v003.json)
  norm_stats_path         Chemin stats normalisation (défaut : $(WORKING_DIR)/models/norm_stats_v3.json)

MODELCARD VALIDATION :
  Fichier JSON : $(WORKING_DIR)/models/model_card_v003.json
  Champs de validation (booléens, tous doivent être true avant RL_ACTIVE) :
    • dry_run_validated         : tests sim avec commandes bloquées OK
    • sim_real_validated        : synchronisation sim↔real établie
    • sim_real_cross_corr_validated : corrélation cross-domaine validée
  
  Si any=false lors du boot :
    → Basculer automatiquement à dry_run=true
    → Refuser transition RL_ACTIVE depuis FSM

FALLBACK EN CAS DE TIMEOUT/DIVERGENCE :
  Si inference prend > 5 ms :
    → Incrémente compteur skip
    → Publie avertissement sur /inference/timing_warn

  Si 5+ cycles consécutifs > 5 ms :
    → Blender vers SAFE_POSE : last_valid_action * (1-alpha) + SAFE_POSE * alpha
    → Alpha = 1 - exp(-t/10) clippé [0.05, 0.3]
    → Protège le robot contre oscillations

NORMALISATION OBSERVATION :
  1. Vecteur brut (70D) reçu sur /observation
  2. Déjà normalisé par obs_builder_node via norm_stats_v3.json
  3. Converti float16 pour TensorRT (économie latence)
  4. Passé au engine.infer()

ACTION SCALING :
  1. Sortie TensorRT : range [-1.0, 1.0]
  2. Clip strict (défense en profondeur)
  3. Multiplication action_scale_rad → valeur en radians
  4. Exemple : action_scale_rad=1.0 → [-1.0, 1.0] rad

SÉQUENCE STARTUP :
  1. Vérifie WORKING_DIR non-vide → RuntimeError sinon
  2. Charge model_card_v003.json
  3. Valide flags (dry_run, sim_real, sim_real_cross_corr)
  4. Crée TensorRTEngine(action_dim=18)
  5. Warmup 500 inferences (setup GPU)
  6. Subscribe /observation + /system_mode
  7. Spin ROS 2

════════════════════════════════════════════════════════════════
"""

import json
import os
import time
from pathlib import Path
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String

from muto_msgs.msg import Commands, Observation
from .safe_pose import SAFE_POSE
from .tensorrt_engine import TensorRTEngine

# ── Guard WORKING_DIR – résolution chemin modèle ────────────────────────────
# WORKING_DIR est injecté par Docker via docker/config/.env.jetson_nano.
# Utilisé pour construire les chemins vers les modèles TensorRT + metadata.
# Si absent ou vide au moment de l'import → RuntimeError immédiate.
# Le nœud refuse de refuser intentionnellement plutôt que de fail silencieusement.
_WORKING_DIR = os.environ.get('WORKING_DIR', '')
if not _WORKING_DIR:
    raise RuntimeError(
        "[muto] WORKING_DIR non défini. Ce nœud doit tourner dans le container Docker. "
        "Vérifiez docker/config/.env.jetson_nano."
    )


class RLPolicyNode(Node):
    def __init__(self) -> None:
        super().__init__("rl_policy_node")

        self.obs_dim = int(self.declare_parameter("obs_dim", 70).value)
        self.action_scale = float(self.declare_parameter("action_scale_rad", 1.0).value)
        self.dry_run = bool(self.declare_parameter("dry_run", True).value)
        
        # Chemins par défaut résolus depuis WORKING_DIR
        default_model_path = os.path.join(_WORKING_DIR, 'models', 'muto_rs_v003.trt')
        default_model_card = os.path.join(_WORKING_DIR, 'models', 'model_card_v003.json')
        default_norm_stats = os.path.join(_WORKING_DIR, 'models', 'norm_stats_v3.json')
        
        self.model_path = self.declare_parameter('model_path', default_model_path).value
        self.model_card_path = str(
            self.declare_parameter("model_card_path", default_model_card).value
        )
        self.norm_stats_path = self.declare_parameter('norm_stats_path', default_norm_stats).value

        self.model_card = self._load_model_card(self.model_card_path)
        self.flags_ok = self._flags_ok(self.model_card)

        if not self.dry_run and not self.flags_ok:
            self.get_logger().warn("Flags model_card invalides: bascule automatique en dry_run")
            self.dry_run = True

        self.engine = TensorRTEngine(action_dim=18)
        self.last_valid_action = np.array(SAFE_POSE, dtype=np.float32)
        self.inference_skip_count = 0
        self.warmup_done = False
        self.system_mode = "DRY_RUN"

        qos_rt = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.timing_warn_pub = self.create_publisher(Float32, "/inference/timing_warn", qos_rt)
        self.raw_pub = self.create_publisher(Commands, "/commands_raw", qos_rt)
        self.dry_pub = self.create_publisher(Commands, "/commands_dry_run", qos_rt)
        self.mode_pub = self.create_publisher(String, "/robot_status", qos_rt)

        self.obs_sub = self.create_subscription(Observation, "/observation", self.inference_callback, qos_rt)
        self.mode_sub = self.create_subscription(String, "/system_mode", self.on_mode, qos_rt)

        self.warmup()

    def _load_model_card(self, path: str) -> dict:
        """Charge le model_card et résout les variables d'environnement dans les chemins."""
        p = Path(path)
        if not p.exists():
            self.get_logger().warn(f"Model card absent: {path}")
            return {}
        try:
            card = json.loads(p.read_text(encoding="utf-8"))
            # Résoudre ${WORKING_DIR} dans tous les champs string
            working_dir = os.environ.get('WORKING_DIR', '')
            for key, value in card.items():
                if isinstance(value, str):
                    card[key] = value.replace('${WORKING_DIR}', working_dir)
            return card
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"Lecture model_card echouee: {exc}")
            return {}

    @staticmethod
    def _flags_ok(model_card: dict) -> bool:
        return bool(
            model_card.get("dry_run_validated", False)
            and model_card.get("sim_real_validated", False)
            and model_card.get("sim_real_cross_corr_validated", False)
        )

    def warmup(self) -> None:
        dummy = np.zeros(self.obs_dim, dtype=np.float16)
        for _ in range(500):
            self.engine.infer(dummy)
        self.warmup_done = True
        self.get_logger().info("Warmup TensorRT termine (500 inferences)")

    def on_mode(self, msg: String) -> None:
        self.system_mode = msg.data.strip()

    def inference_callback(self, obs_msg: Observation) -> None:
        if not self.warmup_done:
            return

        obs = np.array(obs_msg.values, dtype=np.float16)

        start = time.perf_counter()
        raw_action = self.engine.infer(obs).astype(np.float32)
        elapsed_ms = (time.perf_counter() - start) * 1000.0

        if elapsed_ms > 5.0:
            self.inference_skip_count += 1
            warn = Float32()
            warn.data = float(elapsed_ms)
            self.timing_warn_pub.publish(warn)

            if self.inference_skip_count > 5:
                t = float(self.inference_skip_count - 5)
                tau = 10.0
                alpha = float(np.clip(1.0 - np.exp(-t / tau), 0.05, 0.3))
                raw_action = (1.0 - alpha) * self.last_valid_action + alpha * np.array(
                    SAFE_POSE, dtype=np.float32
                )
            else:
                raw_action = self.last_valid_action
        else:
            self.last_valid_action = raw_action
            self.inference_skip_count = 0

        # Regle absolue: clip AVANT scaling radians.
        raw_action = np.clip(raw_action, -1.0, 1.0)
        scaled_action = raw_action * self.action_scale

        out = Commands()
        out.header.stamp = self.get_clock().now().to_msg()
        out.timestamp_sent = out.header.stamp
        out.angles = [float(v) for v in scaled_action.tolist()]

        rl_allowed = (not self.dry_run) and self.flags_ok and self.system_mode == "RL_ACTIVE"
        status = String()
        status.data = "RL_ACTIVE" if rl_allowed else "DRY_RUN"
        self.mode_pub.publish(status)

        if rl_allowed:
            self.raw_pub.publish(out)
        else:
            self.dry_pub.publish(out)


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RLPolicyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
