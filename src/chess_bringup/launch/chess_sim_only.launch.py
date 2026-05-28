"""Launch only Gazebo + MoveIt for the dual IRB120 chess scene (no AI nodes)."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("chess_gazebo"), "launch",
                                  "chess_gazebo.launch.py"])
        ])
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("chess_moveit_config"), "launch",
                                  "moveit_chess.launch.py"])
        ])
    )
    return LaunchDescription([
        gazebo,
        TimerAction(period=8.0, actions=[moveit]),
    ])
