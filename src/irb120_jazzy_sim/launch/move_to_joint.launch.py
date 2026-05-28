from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="irb120_jazzy_sim",
            executable="move_to_joint",
            output="screen",
        ),
    ])
