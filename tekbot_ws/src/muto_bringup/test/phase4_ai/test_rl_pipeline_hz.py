"""
Documentation FR: src/muto_bringup/test/phase4_ai/test_rl_pipeline_hz.py
Role: module du pipeline MUTO RS (robotique temps reel ROS2).
Details: ce fichier participe a la chaine capteurs->observation->decision->commande.
Contraintes: garder la coherence QoS, les unites SI et la robustesse aux timeouts.
Maintenance: toute evolution doit conserver la compatibilite des topics, services et paramètres.
"""

#!/usr/bin/env python3
import argparse
import os
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import List

import rclpy
from muto_msgs.msg import Commands, Observation
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32

HZ_MIN = 98.0
HZ_MAX = 102.0
TIMING_WARN_MAX = 60
DRIFT_TOL = 0.02

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
RESET = "\033[0m"


@dataclass
class Sample:
    t: float
    cid: int


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


class RLPipelineProbe(Node):
    def __init__(self) -> None:
        super().__init__("test_rl_pipeline_hz")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.obs: List[Sample] = []
        self.cmd: List[Sample] = []
        self.warn_count = 0
        self.warmup_seen = False
        self.create_subscription(Observation, "/observation", self.on_obs, qos)
        self.create_subscription(Commands, "/commands_raw", self.on_cmd, qos)
        self.create_subscription(Float32, "/inference/timing_warn", self.on_warn, qos)
        self.create_subscription(Log, "/rosout", self.on_log, qos)

    def on_obs(self, msg: Observation) -> None:
        self.obs.append(Sample(time.monotonic(), int(msg.cycle_id)))

    def on_cmd(self, _: Commands) -> None:
        self.cmd.append(Sample(time.monotonic(), -1))

    def on_warn(self, _: Float32) -> None:
        self.warn_count += 1

    def on_log(self, msg: Log) -> None:
        if "Warmup TensorRT termine (500 inferences)" in msg.msg:
            self.warmup_seen = True


def hz(samples: List[Sample], window: float) -> float:
    now = time.monotonic()
    xs = [s.t for s in samples if now - s.t <= window]
    if len(xs) < 2:
        return 0.0
    return (len(xs) - 1) / max(xs[-1] - xs[0], 1e-6)


def ros2_node_list() -> List[str]:
    if shutil.which("ros2") is None:
        return []
    out = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True, check=False)
    return [x.strip() for x in out.stdout.splitlines() if x.strip()]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=700.0)
    parser.add_argument("--duration", type=float, default=600.0)
    args = parser.parse_args()

    rep = Reporter()
    role = os.environ.get("ROBOT_ROLE", "").upper()
    if role and role != "BRAIN":
        rep.warn(f"SKIP test_rl_pipeline_hz: ROBOT_ROLE={role}, test reserve Jetson/BRAIN")
        return 0

    rclpy.init()
    node = RLPipelineProbe()

    warmup_deadline = time.monotonic() + min(60.0, args.timeout)
    while rclpy.ok() and time.monotonic() < warmup_deadline and not node.warmup_seen:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.warmup_seen:
        rep.pass_("warmup TensorRT detecte")
    else:
        rep.fail("warmup TensorRT non detecte sous 60s")

    start = time.monotonic()
    obs_start = node.obs[-1].cid if node.obs else -1
    while rclpy.ok() and time.monotonic() - start < min(args.duration, args.timeout):
        rclpy.spin_once(node, timeout_sec=0.1)

    cmd_hz = hz(node.cmd, min(args.duration, 600.0))
    obs_hz = hz(node.obs, min(args.duration, 600.0))
    if HZ_MIN <= cmd_hz <= HZ_MAX:
        rep.pass_(f"/commands_raw hz={cmd_hz:.2f}")
    else:
        rep.fail(f"/commands_raw hors plage hz={cmd_hz:.2f}")

    if HZ_MIN <= obs_hz <= HZ_MAX:
        rep.pass_(f"/observation hz={obs_hz:.2f}")
    else:
        rep.fail(f"/observation hors plage hz={obs_hz:.2f}")

    if node.warn_count < TIMING_WARN_MAX:
        rep.pass_(f"timing_warn count={node.warn_count}")
    else:
        rep.fail(f"timing_warn trop eleve count={node.warn_count}")

    nodes = ros2_node_list()
    needed = {"/rl_policy_node", "/obs_builder_node", "/safety_filter_node"}
    if needed.issubset(set(nodes)):
        rep.pass_("nodes critiques actifs apres test")
    else:
        rep.fail(f"nodes manquants: {sorted(list(needed - set(nodes)))}")

    if node.obs and obs_start >= 0:
        obs_end = node.obs[-1].cid
        elapsed = max(time.monotonic() - start, 1e-6)
        rate = (obs_end - obs_start) / elapsed
        rel = abs(rate - 100.0) / 100.0
        if rel <= DRIFT_TOL:
            rep.pass_(f"drift cycle_id ok rate={rate:.2f}Hz")
        else:
            rep.fail(f"drift cycle_id trop fort rate={rate:.2f}Hz")
    else:
        rep.fail("cycle_id observation indisponible")

    node.destroy_node()
    rclpy.shutdown()
    return 1 if rep.fail_count > 0 else 0


if __name__ == "__main__":
    raise SystemExit(main())
