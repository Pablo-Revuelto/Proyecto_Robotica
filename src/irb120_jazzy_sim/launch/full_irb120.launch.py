from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("irb120_jazzy_sim")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg, "launch", "gz_irb120.launch.py"])
        ])
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg, "launch", "moveit_irb120.launch.py"])
        ]),
        launch_arguments=[
            ("use_fake_hardware", "false"),
            ("use_sim_time", "true"),
        ],
    )

    return LaunchDescription([
        gazebo_launch,
        TimerAction(period=6.0, actions=[moveit_launch]),
    ])
