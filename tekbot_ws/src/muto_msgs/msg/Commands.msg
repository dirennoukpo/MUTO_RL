# Horodatage ROS du message de commande (temps logique de publication).
std_msgs/Header header
# Timestamp d'emission reelle de la commande, utilise pour le controle d'age (20 ms max).
builtin_interfaces/Time timestamp_sent
# Consigne articulaire cible pour 18 servos, en radians.
float32[18] angles
