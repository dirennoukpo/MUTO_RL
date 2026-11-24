# The sim video sequence (tools/video.py SCRIPT), with the two turns ending at exactly 90 degrees (IMU yaw),
# so the robot is back on its starting heading for the backward walk.
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

rclpy.init()
n = rclpy.create_node("video_seq")
pub = n.create_publisher(Twist, "/cmd_vel", 10)
st = {"yaw": None, "turned": 0.0}


def on_imu(m):
    q = m.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    if st["yaw"] is not None:
        st["turned"] += (yaw - st["yaw"] + math.pi) % (2 * math.pi) - math.pi
    st["yaw"] = yaw


n.create_subscription(Imu, "/muto/imu", on_imu, qos_profile_sensor_data)


def send(vx=0.0, vy=0.0, wz=0.0):
    m = Twist()
    m.linear.x, m.linear.y, m.angular.z = float(vx), float(vy), float(wz)
    pub.publish(m)
    rclpy.spin_once(n, timeout_sec=0.05)


def hold(label, secs, vx=0.0, vy=0.0, wz=0.0):
    t0 = time.time()
    while time.time() - t0 < secs:
        send(vx, vy, wz)
    print(f"{label:18s} {secs:.1f} s", flush=True)


def turn(label, degrees, wz):
    st["turned"] = 0.0
    t0 = time.time()
    while abs(math.degrees(st["turned"])) < abs(degrees) and time.time() - t0 < 6.0:
        send(wz=wz)
    print(f"{label:18s} {math.degrees(st['turned']):+.0f} deg in {time.time() - t0:.1f} s", flush=True)


while st["yaw"] is None:
    rclpy.spin_once(n, timeout_sec=0.1)
hold("Standing", 2.0)
hold("Walking forward", 4.0, vx=0.15)
turn("Turning left", 90, 0.6)
hold("Walking sideways", 3.0, vy=0.10)
turn("Turning right", 90, -0.6)
hold("Walking backward", 3.0, vx=-0.15)
hold("Stop", 2.5)
