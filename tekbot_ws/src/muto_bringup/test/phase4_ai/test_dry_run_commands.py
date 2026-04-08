"""
Documentation FR: src/muto_bringup/test/phase4_ai/test_dry_run_commands.py

ROLE EXACT
════════════════════════════════════════════════════════════════════════════
Test d'intégration Phase 4: validation DRY_RUN end-to-end (Jetson Nano).
Vérifie pipeline inference → commandes_dry_run sans armement robot.

SCÉNARIOS TESTÉS [1/4] à [4/4]
════════════════════════════════════════════════════════════════════════════
[1/4] /commands_dry_run: valeurs finies, dans limites [-π/2, π/2]
[2/4] Mode DRY_RUN: /commands (réel) ne publie pas pendant 5s
[3/4] Clip+scaling coherent: max_abs <= π/2 + ε
[4/4] Latence obs → commands_dry_run < 10 ms nominal

Role: module du pipeline MUTO RS (robotique temps reel ROS2).
Details: ce fichier participe a la chaine capteurs->observation->decision->commande.
Contraintes: garder la coherence QoS, les unites SI et la robustesse aux timeouts.
Maintenance: toute evolution doit conserver la compatibilite des topics, services et parametres.
"""

#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import List

import rclpy
from muto_msgs.msg import Commands, Observation
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

ANGLE_ABS_MAX = math.pi / 2.0
INFER_LATENCY_MAX_MS = 10.0

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
RESET = "\033[0m"
# Machine cible : Jetson Nano
# Exécuter dans le container Docker avec le profil Jetson
# Variable injectée par Docker : WORKING_DIR
_WORKING_DIR = os.environ.get('WORKING_DIR', '')
if not _WORKING_DIR:
    raise RuntimeError(
        "WORKING_DIR non défini. Ce script doit tourner dans le container Docker. "
        "Vérifiez le profil .env chargé."
    )
MODEL_CARD_PATH = Path(os.path.join(_WORKING_DIR, 'models', 'model_card_v003.json'))


class Reporter:
    def __init__(self) -> None:
        self.fail_count = 0

    def _emit(self, level: str, color: str, text: str) -> None:
        sys.stdout.write(f"{color}{level}{RESET} - {text}\n")
        sys.stdout.flush()

    def pass_(self, text: str) -> None:
        self._emit("PASS", GREEN, text)

    def warn(self, text: str) -> None:
        self._emit("WARN", YELLOW, text)

    def fail(self, text: str) -> None:
        self.fail_count += 1
        self._emit("FAIL", RED, text)


class DryRunProbe(Node):
    def __init__(self) -> None:
        super().__init__("test_dry_run_commands")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.obs: List[Observation] = []
        self.dry: List[Commands] = []
        self.cmd: List[Commands] = []
        self.create_subscription(Observation, "/observation", self.on_obs, qos)
        self.create_subscription(Commands, "/commands_dry_run", self.on_dry, qos)
        self.create_subscription(Commands, "/commands", self.on_cmd, qos)

    def on_obs(self, msg: Observation) -> None:
        self.obs.append(msg)

    def on_dry(self, msg: Commands) -> None:
        self.dry.append(msg)

    def on_cmd(self, msg: Commands) -> None:
        self.cmd.append(msg)


def ms_between(obs: Observation, cmd: Commands) -> float:
    to_ns = lambda t: int(t.sec) * 1_000_000_000 + int(t.nanosec)
    return abs(to_ns(cmd.timestamp_sent) - to_ns(obs.header.stamp)) / 1e6


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    rep = Reporter()
    role = os.environ.get("ROBOT_ROLE", "").upper()
    if role and role != "BRAIN":
        rep.warn(f"SKIP test_dry_run_commands: ROBOT_ROLE={role}, test reserve Jetson/BRAIN")
        return 0

    rclpy.init()
    node = DryRunProbe()

    t0 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t0 < min(args.timeout, 8.0):
        rclpy.spin_once(node, timeout_sec=0.05)

    if node.dry:
        vals = [float(v) for m in node.dry for v in m.angles]
        finite = all(math.isfinite(v) for v in vals)
        in_range = all(-ANGLE_ABS_MAX <= v <= ANGLE_ABS_MAX for v in vals)
        if finite and in_range:
            rep.pass_("/commands_dry_run valeurs finies et dans limites")
        else:
            rep.fail("/commands_dry_run contient NaN/Inf ou hors limites")
    else:
        rep.fail("aucun message /commands_dry_run")

    c_before = len(node.cmd)
    t1 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t1 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    c_after = len(node.cmd)
    if c_after == c_before:
        rep.pass_("/commands non publie en DRY_RUN (5s)")
    else:
        rep.fail("/commands publie alors que DRY_RUN attendu")

    if node.dry:
        max_abs = max(abs(float(v)) for m in node.dry for v in m.angles)
        if max_abs <= ANGLE_ABS_MAX + 1e-3:
            rep.pass_(f"clip+scaling coherent max_abs={max_abs:.3f}")
        else:
            rep.fail(f"clip+scaling incoherent max_abs={max_abs:.3f}")

    if MODEL_CARD_PATH.exists():
        model = json.loads(MODEL_CARD_PATH.read_text(encoding="utf-8"))
        ok = bool(model.get("sim_real_validated", False))
        if ok:
            rep.warn("sim_real_validated=true: test refus publication flags=false non declenchable a chaud")
        else:
            rep.pass_("sim_real_validated=false confirme dans model_card")
    else:
        rep.warn("model_card introuvable")

    if node.obs and node.dry:
        lat = ms_between(node.obs[-1], node.dry[-1])
        if lat < INFER_LATENCY_MAX_MS:
            rep.pass_(f"latence obs->commands_dry_run={lat:.2f} ms")
        else:
            rep.fail(f"latence obs->commands_dry_run trop elevee={lat:.2f} ms")
    else:
        rep.fail("impossible de mesurer latence obs->dry")

    node.destroy_node()
    rclpy.shutdown()
    return 1 if rep.fail_count > 0 else 0


if __name__ == "__main__":
    raise SystemExit(main())
