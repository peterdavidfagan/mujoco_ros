from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mujoco_ros_sim",
                executable="franka",
                name="mujoco_ros_sim",
                output="screen",
                parameters=[
                    {"model_file": get_package_share_directory("mujoco_ros_sim") + "/models/franka.mjb"}]
                )
        ]
    )
