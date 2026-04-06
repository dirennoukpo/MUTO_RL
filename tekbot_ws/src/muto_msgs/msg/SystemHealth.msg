# Header ROS du diagnostic systeme.
std_msgs/Header header
# Score global de sante (plus eleve = meilleur etat).
float32 score
# Vrai si la profondeur est degradee thermiquement (throttling actif).
bool depth_throttled
# Vrai si heartbeat Jetson/Pi est valide.
bool heartbeat_ok
# Vrai si la consommation memoire reste dans la plage de securite.
bool memory_ok
# Champ libre pour details operateur et logs explicatifs.
string details
