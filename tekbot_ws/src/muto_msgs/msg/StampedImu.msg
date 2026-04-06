# Header ROS standard (temps/repere) pour transport middleware.
std_msgs/Header header
# Identifiant atomique du cycle hardware 200 Hz. Ne doit jamais etre encode dans frame_id.
uint64 cycle_id
# Donnees IMU complete: orientation quaternion, vitesse angulaire (rad/s), acceleration lineaire (m/s2).
sensor_msgs/Imu imu
