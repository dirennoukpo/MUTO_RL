# Header ROS de l'observation publiee vers la politique RL.
std_msgs/Header header
# Identifiant du cycle source (doit correspondre au couple IMU/joints synchronise).
uint64 cycle_id
# Vecteur d'observation normalise de dimension 70 (spec v3).
float32[70] values
# Header ROS de l'observation publiee vers la politique RL.
# Identifiant du cycle source (doit correspondre au couple IMU/joints synchronise).
# Vecteur d'observation normalise de dimension 70 (spec v3).
