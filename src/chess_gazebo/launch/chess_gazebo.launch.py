"""Launch Gazebo Harmonic with the dual-arm chess world, both IRB120s,
controllers, ros_gz bridges and the chess piece spawner."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_gz   = FindPackageShare("chess_gazebo")
    pkg_desc = FindPackageShare("chess_description")

    world_file = PathJoinSubstitution([pkg_gz, "worlds", "chess_world.sdf"])
    urdf_xacro = PathJoinSubstitution([pkg_desc, "urdf", "dual_irb120_chess.urdf.xacro"])
    layout_file = str(Path(get_package_share_directory("chess_gazebo"))
                      / "config" / "board_layout.yaml")
    piece_xacro = str(Path(get_package_share_directory("chess_description"))
                      / "urdf" / "chess_piece.urdf.xacro")

    robot_description = ParameterValue(
        Command(["xacro ", urdf_xacro, " use_gazebo:=true"]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py",
            ])
        ]),
        launch_arguments={"gz_args": ["-r ", world_file]}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
        output="screen",
    )

    spawn_robots = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "dual_irb120_chess",
            "-topic", "robot_description",
            "-x", "0", "-y", "0", "-z", "0.0",
        ],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    overhead_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/overhead_camera/image"],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    white_controller = Node(
        package="controller_manager", executable="spawner",
        arguments=["white_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    black_controller = Node(
        package="controller_manager", executable="spawner",
        arguments=["black_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    piece_spawner = Node(
        package="chess_gazebo",
        executable="spawn_chess_pieces",
        output="screen",
        parameters=[{
            "world":         "chess_world",
            "layout_file":   layout_file,
            "piece_xacro":   piece_xacro,
            "initial_delay": 6.0,
        }],
    )

    return LaunchDescription([
        gazebo,
        clock_bridge,
        overhead_image_bridge,
        robot_state_publisher,
        spawn_robots,
        TimerAction(period=3.0, actions=[joint_state_broadcaster]),
        TimerAction(period=4.0, actions=[white_controller, black_controller]),
        TimerAction(period=6.0, actions=[piece_spawner]),
    ])
