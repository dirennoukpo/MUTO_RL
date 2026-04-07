"""
Documentation FR: src/muto_navigation/muto_navigation/navigation_node.py
Role: module du pipeline MUTO RS (robotique temps reel ROS2).
Details: ce fichier participe a la chaine capteurs->observation->decision->commande.
Contraintes: garder la coherence QoS, les unites SI et la robustesse aux timeouts.
Maintenance: toute evolution doit conserver la compatibilite des topics, services et paramètres.
"""

from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from muto_msgs.msg import ObstacleList
from .obstacle_fusion import fuse_obstacles


class NavigationNode(Node):
    def __init__(self) -> None:
        super().__init__("navigation_node")
        self.depth_obs = ObstacleList()
        self.lidar_obs = ObstacleList()

        self.max_vx = float(self.declare_parameter("max_vx", 0.20).value)
        self.max_yaw = float(self.declare_parameter("max_yaw", 0.8).value)
        self.stop_distance = float(self.declare_parameter("obstacle_stop_distance_m", 0.5).value)

        qos_obstacles = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.goal_pub = self.create_publisher(Twist, "/navigation/goal_velocity", 10)
        self.depth_sub = self.create_subscription(ObstacleList, "/depth/obstacles", self.on_depth, qos_obstacles)
        self.lidar_sub = self.create_subscription(ObstacleList, "/lidar/obstacles", self.on_lidar, qos_obstacles)
        self.timer = self.create_timer(0.2, self.publish_goal)

    def on_depth(self, msg: ObstacleList) -> None:
        self.depth_obs = msg

    def on_lidar(self, msg: ObstacleList) -> None:
        self.lidar_obs = msg

    def publish_goal(self) -> None:
        merged = fuse_obstacles(self.depth_obs, self.lidar_obs)

        nearest = 10.0
        left_count = 0
        right_count = 0
        for obs in merged.obstacles:
            d = float((obs.center.x**2 + obs.center.y**2 + obs.center.z**2) ** 0.5)
            nearest = min(nearest, d)
            if obs.center.y >= 0.0:
                left_count += 1
            else:
                right_count += 1

        cmd = Twist()
        if nearest < self.stop_distance:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = -self.max_yaw if left_count > right_count else self.max_yaw
        else:
            cmd.linear.x = self.max_vx
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0

        self.goal_pub.publish(cmd)


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
