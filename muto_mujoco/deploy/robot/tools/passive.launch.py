# Passive mode: hardware driver with torque OFF (servos limp), only reads servo angles + IMU.
# No leg controller, no policy. The URDF with torque_enabled=false is written to /tmp/muto_passive.urdf
# by the docker run command (see README.md).
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    urdf = open("/tmp/muto_passive.urdf").read()
    desc = {"robot_description": urdf}
    ctrl = "/muto_ws/install/muto_description/share/muto_description/config/controllers.yaml"
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[desc]),
        Node(package="controller_manager", executable="ros2_control_node", parameters=[desc, ctrl], output="screen"),
        TimerAction(period=3.0, actions=[Node(package="controller_manager", executable="spawner",
                                              arguments=["joint_state_broadcaster"])]),
    ])
