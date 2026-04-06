"""
Documentation FR: src/muto_bringup/test/phase2_control_jetson/test_obs_builder_vector.py
Role: module du pipeline MUTO RS (robotique temps reel ROS2).
Details: ce fichier participe a la chaine capteurs->observation->decision->commande.
Contraintes: garder la coherence QoS, les unites SI et la robustesse aux timeouts.
Maintenance: toute evolution doit conserver la compatibilite des topics, services et paramètres.
"""

#!/usr/bin/env python3
import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import List

import rclpy
from muto_msgs.msg import Observation
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt64

OBS_DIM = 70
QUAT_NORM_MIN = 0.99
QUAT_NORM_MAX = 1.01
GYRO_ABS_MAX = 35.0
ACC_NORM_MIN = 7.0
ACC_NORM_MAX = 12.0
JOINT_ABS_MAX = math.pi / 2.0
VEL_STATIC_MAX = 0.5
CLAMP_MAX = 5.0
MISSED_MAX_RATIO = 0.01

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
RESET = "\033[0m"


@dataclass
class ObsSample:
    ts: float
    msg: Observation


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


class ObsProbe(Node):
    def __init__(self) -> None:
        super().__init__("test_obs_builder_vector")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.samples: List[ObsSample] = []
        self.last_missed = 0
        self.desync_warn = 0

        self.create_subscription(Observation, "/observation", self.on_obs, qos)
        self.create_subscription(UInt64, "/sync/missed_cycles", self.on_missed, qos)
        self.create_subscription(Log, "/rosout", self.on_log, qos)

    def on_obs(self, msg: Observation) -> None:
        self.samples.append(ObsSample(time.monotonic(), msg))

    def on_missed(self, msg: UInt64) -> None:
        self.last_missed = int(msg.data)

    def on_log(self, msg: Log) -> None:
        if "Desync buffers" in msg.msg:
            self.desync_warn += 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    rep = Reporter()
    rclpy.init()
    node = ObsProbe()

    start = time.monotonic()
    while rclpy.ok() and time.monotonic() - start < min(args.timeout, 12.0):
        rclpy.spin_once(node, timeout_sec=0.05)

    if not node.samples:
        rep.fail("aucun message /observation")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    o = node.samples[-1].msg
    if len(o.values) == OBS_DIM:
        rep.pass_(f"dimension observation={len(o.values)}")
    else:
        rep.fail(f"dimension observation invalide={len(o.values)}")

    qn = math.sqrt(sum(float(o.values[i]) ** 2 for i in range(4)))
    if QUAT_NORM_MIN <= qn <= QUAT_NORM_MAX:
        rep.pass_(f"quaternion norme={qn:.4f}")
    else:
        rep.fail(f"quaternion norme hors plage={qn:.4f}")

    gyro_ok = all(-GYRO_ABS_MAX <= float(o.values[i]) <= GYRO_ABS_MAX for i in range(4, 7))
    rep.pass_("gyro dans plage") if gyro_ok else rep.fail("gyro hors plage")

    an = math.sqrt(sum(float(o.values[i]) ** 2 for i in range(7, 10)))
    if ACC_NORM_MIN <= an <= ACC_NORM_MAX:
        rep.pass_(f"accel norme={an:.3f}")
    else:
        rep.fail(f"accel norme hors plage={an:.3f}")

    joints_ok = all(abs(float(o.values[i])) <= JOINT_ABS_MAX for i in range(10, 28))
    rep.pass_("angles joints dans limites") if joints_ok else rep.fail("angles joints hors limites")

    vel_ok = all(abs(float(o.values[i])) < VEL_STATIC_MAX for i in range(28, 46))
    rep.pass_("vitesses statiques <0.5 rad/s") if vel_ok else rep.warn("vitesses >0.5 rad/s detectees")

    la_ok = all(abs(float(o.values[i])) < 1e-6 for i in range(46, 64))
    rep.pass_("last_action initial ~0") if la_ok else rep.warn("last_action non nul")

    gv_ok = all(abs(float(o.values[i])) < 1e-6 for i in range(64, 67))
    rep.pass_("goal_velocity initial ~0") if gv_ok else rep.warn("goal_velocity non nul")

    cf_ok = all(0.0 <= float(o.values[i]) <= 1.0 for i in range(67, 70))
    rep.pass_("contact_forces [0,1]") if cf_ok else rep.fail("contact_forces hors plage")

    clamp_ok = all(abs(float(v)) <= CLAMP_MAX for v in o.values)
    rep.pass_("normalisation clamp +/-5 active") if clamp_ok else rep.fail("valeurs hors clamp +/-5")

    if len(node.samples) >= 200:
        seq_ok = True
        base = node.samples[-200].msg.cycle_id
        for s in node.samples[-199:]:
            if s.msg.cycle_id != base + 1:
                seq_ok = False
                break
            base += 1
        rep.pass_("cycle_id strict croissant") if seq_ok else rep.fail("cycle_id non strict")
    else:
        rep.fail("echantillons observation insuffisants pour cycle_id")

    missed_start = node.last_missed
    t0 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t0 < min(10.0, args.timeout):
        rclpy.spin_once(node, timeout_sec=0.05)
    missed_delta = max(0, node.last_missed - missed_start)
    ratio = missed_delta / 1000.0
    if ratio < MISSED_MAX_RATIO:
        rep.pass_(f"missed_cycles ratio={ratio*100:.2f}%")
    else:
        rep.fail(f"missed_cycles ratio trop eleve={ratio*100:.2f}%")

    if node.desync_warn == 0:
        rep.pass_("aucun log 'Desync buffers'")
    else:
        rep.warn(f"logs 'Desync buffers' detectes={node.desync_warn}")

    node.destroy_node()
    rclpy.shutdown()
    return 1 if rep.fail_count > 0 else 0


if __name__ == "__main__":
    raise SystemExit(main())
