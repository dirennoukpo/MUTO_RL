"""
Documentation FR: src/muto_bringup/test/phase1_hardware_pi/test_watchdog_timeouts.py
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
from typing import List, Optional

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from muto_msgs.msg import Commands
from muto_msgs.srv import ModeRequest
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String

SAFE_WAIT_SEC = 0.7
STALE_MS = 25

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
RESET = "\033[0m"


@dataclass
class ModeEvent:
    ts: float
    mode: str


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


class WatchdogTester(Node):
    def __init__(self) -> None:
        super().__init__("test_watchdog_timeouts")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.mode_events: List[ModeEvent] = []
        self.jitter: List[float] = []
        self.create_subscription(String, "/system_mode", self.on_mode, qos)
        self.create_subscription(Float32, "/hw/timing_jitter", self.on_jitter, qos)

        self.cmd_pub = self.create_publisher(Commands, "/commands", qos)
        self.dry_pub = self.create_publisher(Commands, "/commands_dry_run", qos)
        self.hb_pub = self.create_publisher(String, "/jetson/heartbeat", qos)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", qos)
        self.mode_cli = self.create_client(ModeRequest, "/mode_request")

    def on_mode(self, msg: String) -> None:
        self.mode_events.append(ModeEvent(time.monotonic(), msg.data))

    def on_jitter(self, msg: Float32) -> None:
        self.jitter.append(float(msg.data))

    def request_mode(self, mode: str, timeout: float = 2.0) -> Optional[ModeRequest.Response]:
        if not self.mode_cli.wait_for_service(timeout_sec=timeout):
            return None
        req = ModeRequest.Request()
        req.requested_mode = mode
        fut = self.mode_cli.call_async(req)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if fut.done():
                return fut.result()
        return None


def mk_cmd(node: Node, age_ms: float = 0.0) -> Commands:
    now = node.get_clock().now()
    ts = now - rclpy.duration.Duration(seconds=age_ms / 1000.0)
    msg = Commands()
    msg.header.stamp = now.to_msg()
    msg.timestamp_sent = TimeMsg(sec=ts.to_msg().sec, nanosec=ts.to_msg().nanosec)
    msg.angles = [0.0] * 18
    return msg


def wait_mode(node: WatchdogTester, target: str, timeout: float) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        if node.mode_events and node.mode_events[-1].mode == target:
            return True
    return False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    reporter = Reporter()
    rclpy.init()
    node = WatchdogTester()

    try:
        resp = node.request_mode("DRY_RUN")
        if resp is None:
            reporter.fail("service /mode_request indisponible")
            return 1

        node.dry_pub.publish(mk_cmd(node, 0.0))
        node.cmd_pub.publish(mk_cmd(node, 0.0))
        node.hb_pub.publish(String(data="ok"))
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.02)

        if wait_mode(node, "SAFE", 0.15):
            reporter.pass_("timeout commande -> SAFE detecte")
        else:
            reporter.fail("timeout commande -> SAFE non detecte")

        node.dry_pub.publish(mk_cmd(node, 0.0))
        node.cmd_pub.publish(mk_cmd(node, 0.0))
        for _ in range(40):
            rclpy.spin_once(node, timeout_sec=0.02)
        if wait_mode(node, "SAFE", SAFE_WAIT_SEC):
            reporter.pass_("timeout heartbeat -> SAFE detecte")
        else:
            reporter.fail("timeout heartbeat -> SAFE non detecte")

        before = len(node.jitter)
        node.cmd_pub.publish(mk_cmd(node, STALE_MS))
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.02)
        after = len(node.jitter)
        if after == before:
            reporter.pass_("commande stale 25ms ignoree sans spike jitter")
        else:
            reporter.warn("commande stale: messages jitter detectes")

        resp_dry = node.request_mode("DRY_RUN")
        if resp_dry is not None and not resp_dry.success:
            reporter.pass_("DRY_RUN refuse sans subscriber actif /commands_dry_run")
        else:
            reporter.fail("DRY_RUN non refuse (comportement inattendu)")

        imu = Imu()
        imu.linear_acceleration.x = 35.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 0.0
        node.imu_pub.publish(imu)
        if wait_mode(node, "EMERGENCY", 0.05):
            reporter.pass_("chute >3g -> EMERGENCY detecte")
        else:
            reporter.fail("chute >3g -> EMERGENCY non detecte")

    finally:
        node.request_mode("IDLE")
        node.destroy_node()
        rclpy.shutdown()

    return 1 if reporter.fail_count > 0 else 0


if __name__ == "__main__":
    raise SystemExit(main())
