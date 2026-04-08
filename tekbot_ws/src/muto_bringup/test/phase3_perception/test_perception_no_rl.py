"""
Documentation FR: src/muto_bringup/test/phase3_perception/test_perception_no_rl.py
Role: module du pipeline MUTO RS (robotique temps reel ROS2).
Details: ce fichier participe a la chaine capteurs->observation->decision->commande.
Contraintes: garder la coherence QoS, les unites SI et la robustesse aux timeouts.
Maintenance: toute evolution doit conserver la compatibilite des topics, services et paramètres.
"""

#!/usr/bin/env python3
import argparse
import os
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import List

import rclpy
from muto_msgs.msg import HeightMap
from muto_msgs.msg import ObstacleList
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Float32

DEPTH_MIN_HZ = 15.0
DEPTH_MAX_HZ = 30.0
LIDAR_MIN_HZ = 8.0
LIDAR_MAX_HZ = 12.0
JITTER_LIMIT_MS = 5.5
RAM_MAX_RATIO = 0.85

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
RESET = "\033[0m"


@dataclass
class Stamp:
    t: float


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


class PerceptionProbe(Node):
    def __init__(self) -> None:
        super().__init__("test_perception_no_rl")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.depth: List[Stamp] = []
        self.lidar: List[Stamp] = []
        self.hm_count = 0
        self.obs_count = 0
        self.jitter: List[float] = []
        self.create_subscription(PointCloud2, "/depth/pointcloud", lambda _: self.depth.append(Stamp(time.monotonic())), qos)
        self.create_subscription(LaserScan, "/lidar/scan_filtered", lambda _: self.lidar.append(Stamp(time.monotonic())), qos)
        self.create_subscription(HeightMap, "/depth/height_map", self.on_hm, qos)
        self.create_subscription(ObstacleList, "/lidar/obstacles", self.on_obs, qos)
        self.create_subscription(Float32, "/hw/timing_jitter", self.on_jitter, qos)

    def on_hm(self, _: ObstacleList) -> None:
        self.hm_count += 1

    def on_obs(self, _: ObstacleList) -> None:
        self.obs_count += 1

    def on_jitter(self, msg: Float32) -> None:
        self.jitter.append(float(msg.data))


def hz(stamps: List[Stamp], window: float = 5.0) -> float:
    now = time.monotonic()
    xs = [s.t for s in stamps if now - s.t <= window]
    if len(xs) < 2:
        return 0.0
    return (len(xs) - 1) / max(xs[-1] - xs[0], 1e-6)


def parse_tegrastats() -> tuple[float, float] | None:
    if shutil.which("tegrastats") is None:
        return None
    try:
        proc = subprocess.run(["tegrastats", "--interval", "1000"], capture_output=True, text=True, timeout=12)
    except subprocess.TimeoutExpired:
        return None
    lines = proc.stdout.splitlines()
    patt = re.compile(r"RAM\s+(\d+)/(\d+)MB")
    vals = []
    for line in lines:
        m = patt.search(line)
        if m:
            used = float(m.group(1))
            total = float(m.group(2))
            vals.append((used, total))
    if not vals:
        return None
    used = max(v[0] for v in vals)
    total = vals[-1][1]
    return used, total


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=40.0)
    args = parser.parse_args()

    rep = Reporter()
    role = os.environ.get("ROBOT_ROLE", "").upper()
    if role and role != "BRAIN":
        rep.warn(f"SKIP test_perception_no_rl: ROBOT_ROLE={role}, test reserve Jetson/BRAIN")
        return 0

    rclpy.init()
    node = PerceptionProbe()

    start = time.monotonic()
    while rclpy.ok() and time.monotonic() - start < min(args.timeout, 8.0):
        rclpy.spin_once(node, timeout_sec=0.05)

    dhz = hz(node.depth)
    lhz = hz(node.lidar)
    if DEPTH_MIN_HZ <= dhz <= DEPTH_MAX_HZ:
        rep.pass_(f"/depth/pointcloud hz={dhz:.2f}")
    else:
        rep.fail(f"/depth/pointcloud hors plage hz={dhz:.2f}")

    if LIDAR_MIN_HZ <= lhz <= LIDAR_MAX_HZ:
        rep.pass_(f"/lidar/scan_filtered hz={lhz:.2f}")
    else:
        rep.fail(f"/lidar/scan_filtered hors plage hz={lhz:.2f}")

    rep.pass_(f"/depth/height_map messages={node.hm_count}") if node.hm_count > 0 else rep.fail("/depth/height_map absent")
    rep.pass_(f"/lidar/obstacles messages={node.obs_count}") if node.obs_count > 0 else rep.fail("/lidar/obstacles absent")

    t0 = time.monotonic()
    before = len(node.jitter)
    while rclpy.ok() and time.monotonic() - t0 < min(30.0, args.timeout):
        rclpy.spin_once(node, timeout_sec=0.05)
    spikes = [v for v in node.jitter[before:] if v > JITTER_LIMIT_MS]
    if not spikes:
        rep.pass_("aucun spike jitter > 5.5 ms")
    else:
        ratio = len(spikes) / max(len(node.jitter[before:]), 1)
        if ratio < 0.01:
            rep.warn(f"spikes jitter limites ratio={ratio*100:.2f}%")
        else:
            rep.fail(f"spikes jitter trop frequents ratio={ratio*100:.2f}%")

    mem = parse_tegrastats()
    if mem is None:
        rep.warn("tegrastats indisponible, test RAM GPU saute")
    else:
        used, total = mem
        ratio = used / total
        if ratio < RAM_MAX_RATIO:
            rep.pass_(f"RAM {used:.0f}/{total:.0f}MB ({ratio*100:.1f}%)")
        else:
            rep.fail(f"RAM trop elevee {used:.0f}/{total:.0f}MB ({ratio*100:.1f}%)")

    node.destroy_node()
    rclpy.shutdown()
    return 1 if rep.fail_count > 0 else 0


if __name__ == "__main__":
    raise SystemExit(main())
