"""Reference implementation of the robot side of the policy interface (pure numpy, no MuJoCo).

This is what the ROS 2 policy node must do every 20 ms. The C++ node is a line-by-line port of it,
and deploy/test_vectors.json checks both. Every constant comes from the ONNX metadata
("muto_interface"), so the model and the robot code can't disagree.

One control cycle (50 Hz):
    obs    = io.observation(imu_quat_wxyz, gyro_rad_s, command)   # latest IMU sample (IMU runs at 25 Hz)
    action = onnx_policy(obs)                                    # 18 numbers in [-1, 1]
    servo  = io.servo_degrees(action)                            # whole degrees, servo IDs 1..18
    io.advance()                                                 # remember the action, move the clock
"""
import json

import numpy as np


def quat_to_rot(q):
    """Rotation matrix (body -> world) of a unit quaternion (w, x, y, z)."""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def yaw_of(q):
    w, x, y, z = q
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class PolicyIO:
    def __init__(self, interface):
        self.cfg = interface
        self.cmd_max = np.array(interface["cmd_max"])
        self.home = np.array(interface["home_rad"])
        self.lo, self.hi = (np.array(r) for r in interface["joint_range_rad"])
        self.sign = np.tile(interface["servo_sign"], 6)
        self.offset = np.tile(interface["servo_offset_deg"], 6)
        self.phase_step = interface["gait_hz"] / interface["control_hz"]  # 2 Hz / 50 Hz = 0.04 per cycle
        self.heading_gain = interface.get("heading_gain", 2.0)
        self.reset()

    @classmethod
    def from_onnx(cls, path):
        import onnx
        meta = {p.key: p.value for p in onnx.load(str(path)).metadata_props}
        return cls(json.loads(meta["muto_interface"]))

    def reset(self, phase=0.0):
        self.last_action = np.zeros(18)
        self.phase = phase  # in [0, 1); any start value is fine (training starts at random)
        self.target_yaw = None

    # ── inputs ────────────────────────────────────────────────────────────────
    def observation(self, imu_quat_wxyz, gyro_rad_s, command):
        """29 inputs. imu_quat: body orientation (w, x, y, z); gyro: body rates [rad/s]; command: vx, vy, yaw rate."""
        gravity = quat_to_rot(np.asarray(imu_quat_wxyz, float)).T @ np.array([0.0, 0.0, -1.0])
        command = np.asarray(command, float)
        if np.any(command != 0):
            angle = 2 * np.pi * self.phase
            clock = [np.sin(angle), np.cos(angle)]
        else:
            clock = [0.0, 0.0]  # stopped clock = stand still
        return np.concatenate([gravity, self.cfg["gyro_scale"] * np.asarray(gyro_rad_s, float),
                               self.last_action, command / self.cmd_max, clock]).astype(np.float32)

    # ── outputs ───────────────────────────────────────────────────────────────
    def joint_targets(self, action):
        """URDF joint targets [rad], servo-ID order, before rounding."""
        action = np.clip(np.asarray(action, float), -1.0, 1.0)
        return np.clip(self.home + self.cfg["action_scale_rad"] * action, self.lo, self.hi)

    def servo_degrees(self, action):
        """Whole servo degrees for servo IDs 1..18 (what goes on the bus)."""
        servo = np.round(self.sign * np.degrees(self.joint_targets(action)) + self.offset)
        return np.clip(servo, -90, 90).astype(int)

    def advance(self, action):
        """End of the cycle: the action becomes 'previous action', the clock moves one step."""
        self.last_action = np.clip(np.asarray(action, float), -1.0, 1.0)
        self.phase = (self.phase + self.phase_step) % 1.0

    # ── heading hold (outside the policy) ─────────────────────────────────────
    def hold_heading(self, command, imu_quat_wxyz):
        """When moving with zero turn command, add a turn command that keeps the current heading."""
        command = np.array(command, float)
        yaw = yaw_of(np.asarray(imu_quat_wxyz, float))
        if command[2] != 0 or not np.any(command):
            self.target_yaw = yaw  # turning or standing: the current heading becomes the one to hold
            return command
        if self.target_yaw is None:
            self.target_yaw = yaw
        err = (self.target_yaw - yaw + np.pi) % (2 * np.pi) - np.pi
        command[2] = np.clip(self.heading_gain * err, -self.cmd_max[2], self.cmd_max[2])
        return command
