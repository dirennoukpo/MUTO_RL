"""
Documentation FR: src/muto_bringup/test/phase1_hardware_pi/test_mode_transitions.py

ROLE EXACT
════════════════════════════════════════════════════════════════════════════
Test d'intégration Phase 1: valide la matrice de transitions FSM sur Pi.
Exécute chaque transition valide/invalide du mode_manager_node.

SCÉNARIOS TESTÉS [1/5] à [5/5]
════════════════════════════════════════════════════════════════════════════
[1/5] RL_ACTIVE guard-rails: 3 flags doivent être true dans model_card
[2/5] Transition valide IDLE -> DRY_RUN
[3/5] Transition valide IDLE -> RL_ACTIVE (avec flags true)
[4/5] Transition invalide bien refusée
[5/5] Nettoyage: retour à IDLE

Role: module du pipeline MUTO RS (robotique temps reel ROS2).
Details: ce fichier participe a la chaine capteurs->observation->decision->commande.
Contraintes: garder la coherence QoS, les unites SI et la robustesse aux timeouts.
Maintenance: toute evolution doit conserver la compatibilite des topics, services et parametres.
"""

#!/usr/bin/env python3
import argparse
import json
import os
import sys
from pathlib import Path
from typing import List, Tuple

import rclpy
from muto_msgs.srv import ModeRequest
from rclpy.node import Node

# ════════════════════════════════════════════════════════════════════════════════
# COULEURS POUR RAPPORTS
# ════════════════════════════════════════════════════════════════════════════════
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
RESET = "\033[0m"

# ════════════════════════════════════════════════════════════════════════════════
# MATRICE DE TRANSITIONS VALIDES ET INVALIDES
# ════════════════════════════════════════════════════════════════════════════════
# Des que le mode_manager_node doit changer, la transition doit être dans cette matrice.
# Ces listes sont testées exhaustivement pour garantir la cohérence de la FSM.
VALID_TRANSITIONS: List[Tuple[str, str]] = [
    ("INIT", "IDLE"),
    ("IDLE", "DRY_RUN"),
    ("IDLE", "RL_ACTIVE"),
    ("IDLE", "MANUAL"),
    ("IDLE", "EMERGENCY"),
    ("DRY_RUN", "IDLE"),
    ("DRY_RUN", "SAFE"),
    ("RL_ACTIVE", "SAFE"),
    ("RL_ACTIVE", "MANUAL"),
    ("RL_ACTIVE", "EMERGENCY"),
    ("SAFE", "IDLE"),
    ("MANUAL", "IDLE"),
    ("MANUAL", "SAFE"),
]

# Transitions directement INVALIDES et qui doivent être refusées par le service.
INVALID_TRANSITIONS: List[Tuple[str, str]] = [
    ("IDLE", "SAFE"),
    ("DRY_RUN", "RL_ACTIVE"),
    ("DRY_RUN", "MANUAL"),
    ("DRY_RUN", "EMERGENCY"),
    ("RL_ACTIVE", "IDLE"),
    ("RL_ACTIVE", "DRY_RUN"),
    ("SAFE", "RL_ACTIVE"),
    ("SAFE", "DRY_RUN"),
    ("SAFE", "MANUAL"),
    ("SAFE", "EMERGENCY"),
    ("MANUAL", "RL_ACTIVE"),
    ("MANUAL", "DRY_RUN"),
    ("MANUAL", "EMERGENCY"),
    ("EMERGENCY", "IDLE"),
    ("EMERGENCY", "SAFE"),
    ("EMERGENCY", "DRY_RUN"),
    ("EMERGENCY", "RL_ACTIVE"),
    ("EMERGENCY", "MANUAL"),
]

# Machine cible : Raspberry Pi
# Exécuter dans le container Docker avec le profil Pi
# Variable injectée par Docker : WORKING_DIR
_WORKING_DIR = os.environ.get('WORKING_DIR', '')
if not _WORKING_DIR:
    raise RuntimeError(
        "WORKING_DIR non défini. Ce script doit tourner dans le container Docker. "
        "Vérifiez le profil .env chargé."
    )
MODEL_CARD = Path(os.path.join(_WORKING_DIR, 'models', 'model_card_v003.json'))


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


class ModeClient(Node):
    def __init__(self) -> None:
        super().__init__("test_mode_transitions")
        self.cli = self.create_client(ModeRequest, "/mode_request")

    def request(self, mode: str, timeout: float) -> ModeRequest.Response | None:
        if not self.cli.wait_for_service(timeout_sec=timeout):
            return None
        req = ModeRequest.Request()
        req.requested_mode = mode
        fut = self.cli.call_async(req)
        end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if fut.done():
                return fut.result()
        return None


def set_flags(path: Path, value: bool) -> None:
    data = json.loads(path.read_text(encoding="utf-8"))
    data["dry_run_validated"] = value
    data["sim_real_validated"] = value
    data["sim_real_cross_corr_validated"] = value
    path.write_text(json.dumps(data, indent=2), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    rep = Reporter()
    rclpy.init()
    node = ModeClient()

    original = MODEL_CARD.read_text(encoding="utf-8") if MODEL_CARD.exists() else ""
    try:
        if not MODEL_CARD.exists():
            rep.warn("model_card introuvable: test RL_ACTIVE flags saute")
        else:
            set_flags(MODEL_CARD, False)
            resp = node.request("RL_ACTIVE", args.timeout)
            if resp is not None and not resp.success:
                rep.pass_("RL_ACTIVE refuse quand flags=false")
            else:
                rep.fail("RL_ACTIVE non refuse avec flags=false")

            set_flags(MODEL_CARD, True)
            node.request("IDLE", args.timeout)
            resp2 = node.request("RL_ACTIVE", args.timeout)
            if resp2 is not None and resp2.success:
                rep.pass_("RL_ACTIVE accepte quand flags=true")
            else:
                rep.fail("RL_ACTIVE non accepte avec flags=true")
            node.request("SAFE", args.timeout)
            node.request("IDLE", args.timeout)

        for src, dst in VALID_TRANSITIONS:
            if src == "INIT":
                rep.warn("transition INIT->IDLE non rejouable a chaud (INIT non reachable)")
                continue
            node.request(src, args.timeout)
            resp = node.request(dst, args.timeout)
            if resp is not None and resp.success:
                rep.pass_(f"transition valide {src}->{dst}")
            else:
                msg = "none" if resp is None else resp.message
                rep.fail(f"transition valide echouee {src}->{dst}: {msg}")

        for src, dst in INVALID_TRANSITIONS:
            if src == "EMERGENCY":
                node.request("MANUAL", args.timeout)
                node.request("EMERGENCY", args.timeout)
            else:
                node.request(src, args.timeout)
            resp = node.request(dst, args.timeout)
            if resp is not None and (not resp.success) and bool(resp.message.strip()):
                rep.pass_(f"transition invalide bien refusee {src}->{dst}")
            else:
                msg = "none" if resp is None else resp.message
                rep.fail(f"transition invalide non refusee {src}->{dst}: {msg}")

    finally:
        if original:
            MODEL_CARD.write_text(original, encoding="utf-8")
        node.request("IDLE", args.timeout)
        node.destroy_node()
        rclpy.shutdown()

    return 1 if rep.fail_count > 0 else 0


if __name__ == "__main__":
    raise SystemExit(main())
