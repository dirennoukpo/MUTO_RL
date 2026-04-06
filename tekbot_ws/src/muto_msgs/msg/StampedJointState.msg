# Header ROS standard du paquet articulaire.
std_msgs/Header header
# Identifiant de cycle synchronise avec IMU et observation.
uint64 cycle_id
# Etats articulaires: position (rad), vitesse (rad/s), effort (approximation selon hardware).
sensor_msgs/JointState joint_state
