from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("irb120_jazzy_sim")

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg, "launch", "moveit_irb120.launch.py"])
        ])
    )

    obstacle_node = Node(
        package="irb120_jazzy_sim",
        executable="add_obstacle",
        output="screen",
    )

    return LaunchDescription([
        moveit_launch,
        # Espera a que move_group este listo antes de publicar el obstaculo
        TimerAction(period=5.0, actions=[obstacle_node]),
    ])
