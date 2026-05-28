from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("irb120_jazzy_sim")

    world = PathJoinSubstitution([
        pkg,
        "worlds",
        "irb120_world.sdf",
    ])

    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([
                pkg,
                "urdf",
                "irb120.urdf.xacro",
            ]),
            " use_gazebo:=true",
        ]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            ])
        ]),
        launch_arguments={
            "gz_args": ["-r ", world],
        }.items(),
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

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "irb120",
            "-topic", "robot_description",
            "-x", "0",
            "-y", "0",
            "-z", "0.01",
        ],
        output="screen",
    )

    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "irb120_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,

        TimerAction(
            period=3.0,
            actions=[spawn_joint_state_broadcaster],
        ),

        TimerAction(
            period=4.0,
            actions=[spawn_arm_controller],
        ),
    ])