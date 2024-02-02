from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mujoco_ros_sim",
                executable="franka_block",
                name="mujoco_ros_sim",
                output="screen",
            )
        ]
    )
