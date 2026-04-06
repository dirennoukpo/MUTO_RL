"""
Documentation FR: src/muto_navigation/muto_navigation/obstacle_fusion.py
Role: module du pipeline MUTO RS (robotique temps reel ROS2).
Details: ce fichier participe a la chaine capteurs->observation->decision->commande.
Contraintes: garder la coherence QoS, les unites SI et la robustesse aux timeouts.
Maintenance: toute evolution doit conserver la compatibilite des topics, services et paramètres.
"""

from muto_msgs.msg import ObstacleList


def fuse_obstacles(depth: ObstacleList, lidar: ObstacleList) -> ObstacleList:
    out = ObstacleList()
    out.header = depth.header
    out.obstacles = list(depth.obstacles) + list(lidar.obstacles)
    return out
