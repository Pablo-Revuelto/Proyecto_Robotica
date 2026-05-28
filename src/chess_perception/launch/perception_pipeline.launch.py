"""Launch the perception pipeline (overhead camera → board state estimator).

All parameters are loaded from `config/params.yaml` (edit that file to
configure YOLO weights, camera intrinsics, board geometry, etc.).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("chess_perception")

    default_params_file = PathJoinSubstitution([pkg_share, "config", "params.yaml"])
    default_model_path  = PathJoinSubstitution([pkg_share, "models", "best.pt"])

    params_file = LaunchConfiguration("params_file")

    args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Full path to the ROS 2 parameters YAML file for chess_perception nodes.",
        ),
    ]

    board_state_estimator = Node(
        package="chess_perception",
        executable="board_state_estimator",
        name="chess_board_state_estimator",
        output="screen",
        parameters=[
            params_file,
            {"yolo_weights": default_model_path},   # ruta absoluta resuelta al instalar
        ],
    )

    return LaunchDescription(args + [board_state_estimator])
