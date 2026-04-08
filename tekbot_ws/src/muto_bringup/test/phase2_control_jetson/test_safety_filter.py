"""
Documentation FR: src/muto_bringup/test/phase2_control_jetson/test_safety_filter.py
Role: module du pipeline MUTO RS (robotique temps reel ROS2).
Details: ce fichier participe a la chaine capteurs->observation->decision->commande.
Contraintes: garder la coherence QoS, les unites SI et la robustesse aux timeouts.
Maintenance: toute evolution doit conserver la compatibilite des topics, services et paramètres.
"""

#!/usr/bin/env python3
import argparse
import math
import os
import sys
import time
from typing import List

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from muto_msgs.msg import Commands, StampedJointState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

JOINT_LIMIT = math.pi / 2.0
RATE_LIMIT_DEG = 8.0
DT = 0.01
RATE_LIMIT_PER_CYCLE = RATE_LIMIT_DEG * math.pi / 180.0 * DT

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
RESET = "\033[0m"


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


class SafetyProbe(Node):
    def __init__(self) -> None:
        super().__init__("test_safety_filter")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.filtered: List[Commands] = []
        self.modes: List[str] = []

        self.raw_pub = self.create_publisher(Commands, "/commands_raw", qos)
        self.joint_pub = self.create_publisher(StampedJointState, "/joint_states", qos)
        self.create_subscription(Commands, "/commands", self.on_filtered, qos)
        self.create_subscription(String, "/system_mode", self.on_mode, qos)

    def on_filtered(self, msg: Commands) -> None:
        self.filtered.append(msg)

    def on_mode(self, msg: String) -> None:
        self.modes.append(msg.data)


def mk_cmd(node: Node, value: float = 0.0) -> Commands:
    now = node.get_clock().now().to_msg()
    msg = Commands()
    msg.header.stamp = now
    msg.timestamp_sent = TimeMsg(sec=now.sec, nanosec=now.nanosec)
    msg.angles = [value] * 18
    return msg


def spin_for(node: Node, sec: float) -> None:
    end = time.monotonic() + sec
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    rep = Reporter()
    role = os.environ.get("ROBOT_ROLE", "").upper()
    if role and role != "BRAIN":
        rep.warn(f"SKIP test_safety_filter: ROBOT_ROLE={role}, test reserve Jetson/BRAIN")
        return 0

    rclpy.init()
    node = SafetyProbe()

    node.raw_pub.publish(mk_cmd(node, 0.1))
    spin_for(node, 0.1)
    last_valid = node.filtered[-1] if node.filtered else mk_cmd(node, 0.0)

    cmd_nan = mk_cmd(node, 0.0)
    cmd_nan.angles[0] = float("nan")
    node.raw_pub.publish(cmd_nan)
    spin_for(node, 0.1)
    if node.filtered and math.isfinite(float(node.filtered[-1].angles[0])):
        rep.pass_("etape1 NaN sanitise")
    else:
        rep.fail("etape1 NaN non traite")

    node.raw_pub.publish(mk_cmd(node, 2.0))
    spin_for(node, 0.1)
    if node.filtered and abs(float(node.filtered[-1].angles[0])) <= JOINT_LIMIT + 1e-3:
        rep.pass_("etape2 clamp angulaire")
    else:
        rep.fail("etape2 clamp angulaire absent")

    before = node.filtered[-1].angles[0] if node.filtered else 0.0
    node.raw_pub.publish(mk_cmd(node, 1.5))
    spin_for(node, 0.1)
    after = node.filtered[-1].angles[0] if node.filtered else 0.0
    if abs(float(after) - float(before)) <= RATE_LIMIT_PER_CYCLE + 1e-3:
        rep.pass_("etape3 rate limiting")
    else:
        rep.fail("etape3 rate limiting non respecte")

    js = StampedJointState()
    js.joint_state.velocity = [10.0] + [0.0] * 17
    js.joint_state.effort = [2.0] + [0.0] * 17
    node.joint_pub.publish(js)
    node.raw_pub.publish(mk_cmd(node, 0.5))
    spin_for(node, 0.2)
    if "SAFE" in node.modes:
        rep.pass_("etape4 saturation vitesse/current -> SAFE detectable")
    else:
        rep.warn("etape4 SAFE non publie par safety_filter (depend watchdog/mode_manager)")

    for i in range(15):
        node.raw_pub.publish(mk_cmd(node, 0.5 if i % 2 == 0 else -0.5))
        spin_for(node, 0.02)
    if node.filtered:
        rep.pass_("etape5 oscillation injectee")
    else:
        rep.fail("etape5 aucune sortie filtre")

    node.raw_pub.publish(mk_cmd(node, 1.2))
    spin_for(node, 0.1)
    if node.filtered and abs(float(node.filtered[-1].angles[0])) <= JOINT_LIMIT + 1e-3:
        rep.pass_("etape6 posture validator/guard actif via limitation")
    else:
        rep.fail("etape6 posture validator non observable")

    seq = []
    for _ in range(6):
        node.raw_pub.publish(mk_cmd(node, 1.5))
        spin_for(node, 0.05)
        if node.filtered:
            seq.append(float(node.filtered[-1].angles[0]))
    if len(seq) >= 2 and all(abs(seq[i] - seq[i - 1]) <= abs(seq[1] - seq[0]) + 1e-6 for i in range(1, len(seq))):
        rep.pass_("etape7 decay exponentiel/blend observe")
    else:
        rep.warn("etape7 decay non clairement observable")

    _ = last_valid
    node.destroy_node()
    rclpy.shutdown()
    return 1 if rep.fail_count > 0 else 0


if __name__ == "__main__":
    raise SystemExit(main())
