"""Facts about the real MUTO RS shared by the model builder and the RL environment.

Sources and derivations: see ROBOT_NOTES.md (sections 2, 6c).
"""

import math
import pathlib

MODEL_DIR = pathlib.Path(__file__).resolve().parents[1] / "model"
SCENE_XML = MODEL_DIR / "scene.xml"

# Leg prefixes in real servo-ID order: leg1 = right front (IDs 1-3) ... leg6 = left front (IDs 16-18).
LEGS_BY_SERVO_ID = ["yq", "yz", "yh", "zh", "zz", "zq"]
JOINT_NAMES = [f"{leg}{k}_Joint" for leg in LEGS_BY_SERVO_ID for k in (1, 2, 3)]

CONTROL_HZ = 50  # real ros2_control loop rate
IMU_HZ = 25  # real IMU publish rate (/muto/imu)

# Yahboom servo command (deg) <-> URDF joint angle, derived from MutoLib leg.py + the CAD:
#   servo1 = +q1,  servo2 = -q2 + FEMUR_DROOP_DEG,  servo3 = -q3 + FEMUR_DROOP_DEG + TIBIA_LEAN_DEG
FEMUR_DROOP_DEG = 10.75  # at URDF zero the femur (axis to axis) points 10.75 deg below horizontal
TIBIA_LEAN_DEG = 4.07  # at URDF zero the tibia leans 4.07 deg outward from vertical
SERVO_SIGN = (1.0, -1.0, -1.0)
SERVO_OFFSET_DEG = (0.0, FEMUR_DROOP_DEG, FEMUR_DROOP_DEG + TIBIA_LEAN_DEG)
SERVO_LIMIT_DEG = 90  # Yahboom motor() accepts [-90, 90]
HOME_SERVO_DEG = (0, -30, -15)  # MutoLib standby: femur 30 deg up, tibia 15 deg out


def servo_to_urdf(s1, s2, s3):
    """One leg's Yahboom servo command (deg) -> URDF joint angles (rad)."""
    return tuple(math.radians((s - off) / sign) for s, sign, off in zip((s1, s2, s3), SERVO_SIGN, SERVO_OFFSET_DEG))


def urdf_to_servo(q1, q2, q3):
    """One leg's URDF joint angles (rad) -> Yahboom servo command (deg, not rounded)."""
    return tuple(sign * math.degrees(q) + off for q, sign, off in zip((q1, q2, q3), SERVO_SIGN, SERVO_OFFSET_DEG))
