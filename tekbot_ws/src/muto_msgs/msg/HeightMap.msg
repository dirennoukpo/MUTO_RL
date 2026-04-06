# Header ROS de la carte 2.5D locale.
std_msgs/Header header
# Largeur de grille (nombre de colonnes).
uint32 width
# Hauteur de grille (nombre de lignes).
uint32 height
# Taille d'une cellule en metres.
float32 resolution
# Donnees de hauteur (m) linearisees ligne par ligne.
float32[] data
