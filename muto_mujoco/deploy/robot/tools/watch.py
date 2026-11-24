# Prints joint angles (URDF degrees) that moved since start, and the gravity direction in the body frame.
#   python3 /tools/watch.py SECONDS      (passive mode: move legs by hand, tilt the robot)
import math, sys, time, rclpy
from sensor_msgs.msg import JointState, Imu
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n = rclpy.create_node("watch"); st = {"j0": None, "j": None, "q": None}
n.create_subscription(JointState, "/joint_states", lambda m: st.update(j=dict(zip(m.name, m.position))), 10)
n.create_subscription(Imu, "/muto/imu", lambda m: st.update(q=m.orientation), qos_profile_sensor_data)
t_end = time.time() + float(sys.argv[1]); last = 0
while time.time() < t_end:
    rclpy.spin_once(n, timeout_sec=0.05)
    if st["j"] and st["j0"] is None: st["j0"] = dict(st["j"])
    if time.time() - last > 1.0 and st["j"]:
        last = time.time(); moved = {k: math.degrees(v - st["j0"][k]) for k, v in st["j"].items() if abs(v - st["j0"][k]) > math.radians(4)}
        g = ""
        if st["q"]:
            w, x, y, z = st["q"].w, st["q"].x, st["q"].y, st["q"].z
            g = "gravity (%+.2f %+.2f %+.2f)" % (-2*(x*z-w*y), -2*(y*z+w*x), -(1-2*(x*x+y*y)))
        print("%s | moved: %s" % (g, ", ".join("%s %+.0f" % (k.replace("_Joint", ""), v) for k, v in sorted(moved.items())) or "none"), flush=True)
