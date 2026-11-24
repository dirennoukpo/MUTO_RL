# Prints the policy node inputs 1-3 (gravity, body frame) and 4-6 (0.25 x gyro) every 0.5 s.
#   python3 /tools/obs.py SECONDS
import sys, time, rclpy
from std_msgs.msg import Float32MultiArray
rclpy.init(); n = rclpy.create_node("obs_watch"); st = {}
n.create_subscription(Float32MultiArray, "/muto_policy/obs", lambda m: st.update(o=list(m.data)), 10)
t_end = time.time() + float(sys.argv[1]); last = 0
while time.time() < t_end:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() - last > 0.5 and "o" in st:
        last = time.time(); o = st["o"]
        print("gravity %+.2f %+.2f %+.2f | gyro/4 %+.2f %+.2f %+.2f" % tuple(o[:6]), flush=True)
