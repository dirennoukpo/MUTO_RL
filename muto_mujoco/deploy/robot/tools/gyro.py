# Prints, every 0.5 s, the gravity direction (raw IMU frame) and the largest gyro value on each axis in that window.
#   python3 /tools/gyro.py SECONDS
import sys, time, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
rclpy.init(); n = rclpy.create_node("gyro"); buf = []; st = {}
def cb(m):
    st["q"] = m.orientation; buf.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
n.create_subscription(Imu, "/muto/imu", cb, qos_profile_sensor_data)
t_end = time.time() + float(sys.argv[1]); last = time.time()
while time.time() < t_end:
    rclpy.spin_once(n, timeout_sec=0.02)
    if time.time() - last > 0.5 and buf and "q" in st:
        last = time.time(); q = st["q"]
        peak = [max((b[i] for b in buf), key=abs) for i in range(3)]; buf.clear()
        print("gravity (%+.2f %+.2f %+.2f)  gyro peak x %+.2f y %+.2f z %+.2f" % (
            -2*(q.x*q.z-q.w*q.y), -2*(q.y*q.z+q.w*q.x), -(1-2*(q.x*q.x+q.y*q.y)), *peak), flush=True)
