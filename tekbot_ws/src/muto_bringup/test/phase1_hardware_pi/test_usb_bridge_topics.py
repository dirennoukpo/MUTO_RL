"""
Documentation FR: src/muto_bringup/test/phase1_hardware_pi/test_usb_bridge_topics.py
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
from muto_msgs.msg import StampedImu, StampedJointState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32

FREQ_MIN_HZ = 190.0
FREQ_MAX_HZ = 210.0
GYRO_MIN = -35.0
GYRO_MAX = 35.0
ACC_NORM_MIN = 7.0
ACC_NORM_MAX = 12.0
QUAT_NORM_MIN = 0.99
QUAT_NORM_MAX = 1.01
JOINT_MIN = -math.pi / 2.0
JOINT_MAX = math.pi / 2.0

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
RESET = "\033[0m"


@dataclass
class Sample:
    ts: float
    msg: object


class Reporter:
    def __init__(self) -> None:
        self.fail_count = 0
        self.warn_count = 0
        self.lines: List[str] = []

    def _emit(self, level: str, color: str, text: str) -> None:
        line = f"{color}{level}{RESET} - {text}"
        self.lines.append(line)
        sys.stdout.write(line + "\n")
        sys.stdout.flush()

    def pass_(self, text: str) -> None:
        self._emit("PASS", GREEN, text)

    def fail(self, text: str) -> None:
        self.fail_count += 1
        self._emit("FAIL", RED, text)

    def warn(self, text: str) -> None:
        self.warn_count += 1
        self._emit("WARN", YELLOW, text)


class UsbBridgeProbe(Node):
    def __init__(self) -> None:
        super().__init__("test_usb_bridge_topics")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.imu_samples: List[Sample] = []
        self.joint_samples: List[Sample] = []
        self.jitter_msgs: List[float] = []

        self.create_subscription(StampedImu, "/imu/data", self.on_imu, qos)
        self.create_subscription(StampedJointState, "/joint_states", self.on_joint, qos)
        self.create_subscription(Float32, "/hw/timing_jitter", self.on_jitter, qos)

    def on_imu(self, msg: StampedImu) -> None:
        self.imu_samples.append(Sample(time.monotonic(), msg))

    def on_joint(self, msg: StampedJointState) -> None:
        self.joint_samples.append(Sample(time.monotonic(), msg))

    def on_jitter(self, msg: Float32) -> None:
        self.jitter_msgs.append(float(msg.data))


def measure_frequency(samples: List[Sample], window_sec: float) -> float:
    now = time.monotonic()
    data = [s for s in samples if now - s.ts <= window_sec]
    if len(data) < 2:
        return 0.0
    dt = data[-1].ts - data[0].ts
    if dt <= 0.0:
        return 0.0
    return (len(data) - 1) / dt


def assert_cycle_strict(samples: List[Sample], reporter: Reporter, name: str) -> None:
    if len(samples) < 201:
        reporter.fail(f"{name}: echantillons insuffisants ({len(samples)} < 201)")
        return
    last = samples[-201].msg.cycle_id
    for i in range(200):
        cur = samples[-200 + i].msg.cycle_id
        if cur != last + 1:
            reporter.fail(f"{name}: cycle_id non strict ({last} -> {cur})")
            return
        last = cur
    reporter.pass_(f"{name}: cycle_id strict sur 200 messages")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    reporter = Reporter()
    rclpy.init()
    node = UsbBridgeProbe()

    start = time.monotonic()
    while rclpy.ok() and time.monotonic() - start < min(args.timeout, 12.0):
        rclpy.spin_once(node, timeout_sec=0.05)

    imu_hz = measure_frequency(node.imu_samples, 5.0)
    js_hz = measure_frequency(node.joint_samples, 5.0)
    if FREQ_MIN_HZ <= imu_hz <= FREQ_MAX_HZ:
        reporter.pass_(f"/imu/data frequence={imu_hz:.2f} Hz")
    else:
        reporter.fail(f"/imu/data frequence hors plage={imu_hz:.2f} Hz")

    if FREQ_MIN_HZ <= js_hz <= FREQ_MAX_HZ:
        reporter.pass_(f"/joint_states frequence={js_hz:.2f} Hz")
    else:
        reporter.fail(f"/joint_states frequence hors plage={js_hz:.2f} Hz")

    if node.imu_samples:
        reporter.pass_("type /imu/data = StampedImu")
    else:
        reporter.fail("aucun message /imu/data")
    if node.joint_samples:
        reporter.pass_("type /joint_states = StampedJointState")
    else:
        reporter.fail("aucun message /joint_states")

    assert_cycle_strict(node.imu_samples, reporter, "/imu/data")
    assert_cycle_strict(node.joint_samples, reporter, "/joint_states")

    if node.imu_samples:
        bad = 0
        for s in node.imu_samples[-200:]:
            if s.msg.header.frame_id == str(s.msg.cycle_id):
                bad += 1
        if bad == 0:
            reporter.pass_("header.frame_id ne contient jamais cycle_id")
        else:
            reporter.fail(f"header.frame_id contient cycle_id ({bad} cas)")

    if node.imu_samples:
        latest = node.imu_samples[-1].msg
        q = latest.imu.orientation
        qn = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
        if QUAT_NORM_MIN <= qn <= QUAT_NORM_MAX:
            reporter.pass_(f"norme quaternion={qn:.4f}")
        else:
            reporter.fail(f"norme quaternion hors plage={qn:.4f}")

        g = latest.imu.angular_velocity
        gy_ok = all(GYRO_MIN <= v <= GYRO_MAX for v in (g.x, g.y, g.z))
        if gy_ok:
            reporter.pass_(f"gyro rad/s dans [{GYRO_MIN},{GYRO_MAX}]")
        else:
            reporter.fail(f"gyro hors plage=({g.x:.2f},{g.y:.2f},{g.z:.2f})")

        a = latest.imu.linear_acceleration
        an = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        if ACC_NORM_MIN <= an <= ACC_NORM_MAX:
            reporter.pass_(f"norme accel={an:.3f} m/s2")
        else:
            reporter.fail(f"norme accel hors plage={an:.3f} m/s2")

    if node.joint_samples:
        latest_js = node.joint_samples[-1].msg
        if latest_js.joint_state.position:
            ok = all(JOINT_MIN <= float(v) <= JOINT_MAX for v in latest_js.joint_state.position)
            if ok:
                reporter.pass_("angles joints dans [-pi/2, pi/2]")
            else:
                reporter.fail("angles joints hors limites")
        else:
            reporter.fail("joint_state.position vide")

    jitter_start = time.monotonic()
    before = len(node.jitter_msgs)
    while rclpy.ok() and time.monotonic() - jitter_start < min(10.0, args.timeout):
        rclpy.spin_once(node, timeout_sec=0.05)
    after = len(node.jitter_msgs)
    if after == before:
        reporter.pass_("/hw/timing_jitter silencieux sur 10s")
    else:
        vals = node.jitter_msgs[before:after]
        reporter.warn(f"/hw/timing_jitter actif ({len(vals)} msgs, max={max(vals):.3f} ms)")

    node.destroy_node()
    rclpy.shutdown()
    return 1 if reporter.fail_count > 0 else 0


if __name__ == "__main__":
    raise SystemExit(main())
