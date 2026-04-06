"""
Documentation FR: src/muto_hardware/launch/pi_bringup.launch.py
Role: module du pipeline MUTO RS (robotique temps reel ROS2).
Details: ce fichier participe a la chaine capteurs->observation->decision->commande.
Contraintes: garder la coherence QoS, les unites SI et la robustesse aux timeouts.
Maintenance: toute evolution doit conserver la compatibilite des topics, services et paramètres.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(package="muto_hardware", executable="usb_bridge_node", output="screen"),
        Node(package="muto_hardware", executable="watchdog_node", output="screen"),
        Node(package="muto_hardware", executable="mode_manager_node", output="screen"),
    ])
