from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("irb120_jazzy_sim"))
    urdf_xacro = str(pkg_share / "urdf" / "irb120.urdf.xacro")

    # use_fake_hardware:=true  -> arranca ros2_control_node con mock_components
    #                            (modo demo, sin Gazebo). RViz/MoveIt usables.
    # use_fake_hardware:=false -> asume que Gazebo (full_irb120.launch.py)
    #                            ya esta corriendo y aporta el controller_manager.
    use_fake_hw = LaunchConfiguration("use_fake_hardware")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ", urdf_xacro,
                " use_gazebo:=false",
                " use_mock_hardware:=", use_fake_hw,
            ]),
            value_type=str,
        )
    }

    moveit_config = (
        MoveItConfigsBuilder("irb120", package_name="irb120_jazzy_sim")
        .robot_description(file_path=urdf_xacro)
        .robot_description_semantic(file_path=str(pkg_share / "config" / "irb120.srdf"))
        .trajectory_execution(file_path=str(pkg_share / "config" / "moveit_controllers.yaml"))
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .pilz_cartesian_limits(file_path=str(pkg_share / "config" / "pilz_cartesian_limits.yaml"))
        .joint_limits(file_path=str(pkg_share / "config" / "joint_limits.yaml"))
        .robot_description_kinematics(file_path=str(pkg_share / "config" / "kinematics.yaml"))
        .to_moveit_configs()
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            str(pkg_share / "config" / "ros2_controllers.yaml"),
        ],
        output="screen",
        condition=IfCondition(use_fake_hw),
    )

    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
        condition=IfCondition(use_fake_hw),
    )

    spawn_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["irb120_arm_controller", "-c", "/controller_manager"],
        output="screen",
        condition=IfCondition(use_fake_hw),
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", str(pkg_share / "config" / "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        rsp,
        ros2_control_node,
        TimerAction(period=2.0, actions=[spawn_jsb]),
        TimerAction(period=3.0, actions=[spawn_arm]),
        move_group,
        rviz,
    ])
