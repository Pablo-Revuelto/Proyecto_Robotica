"""Launch MoveIt 2 (move_group + RViz) configured for the dual IRB120 chess scene."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    desc_share = Path(get_package_share_directory("chess_description"))
    cfg_share = Path(get_package_share_directory("chess_moveit_config"))

    urdf_xacro = str(desc_share / "urdf" / "dual_irb120_chess.urdf.xacro")
    srdf_file = str(cfg_share / "config" / "chess.srdf")

    # When run standalone (no Gazebo) we must publish TF and joint states
    # ourselves; when included by chess_full, Gazebo already provides both, so
    # the caller sets use_rsp:=false to avoid duplicate publishers.
    use_rsp = LaunchConfiguration("use_rsp")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", urdf_xacro, " use_gazebo:=false"]),
            value_type=str,
        )
    }

    moveit_config = (
        MoveItConfigsBuilder("dual_irb120_chess", package_name="chess_moveit_config")
        .robot_description(file_path=urdf_xacro, mappings={"use_gazebo": "false"})
        .robot_description_semantic(file_path=srdf_file)
        .trajectory_execution(file_path=str(cfg_share / "config" / "moveit_controllers.yaml"))
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path=str(cfg_share / "config" / "joint_limits.yaml"))
        .robot_description_kinematics(file_path=str(cfg_share / "config" / "kinematics.yaml"))
        .to_moveit_configs()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        condition=IfCondition(use_rsp),
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        condition=IfCondition(use_rsp),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz_config = str(cfg_share / "config" / "chess.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.to_dict(),
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rsp", default_value="true",
            description="Start robot_state_publisher + joint_state_publisher. "
                        "Set false when an external source (e.g. Gazebo) already "
                        "publishes TF and joint states."),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use the simulation clock. Set true when running with Gazebo."),
        robot_state_publisher,
        joint_state_publisher,
        move_group,
        rviz,
    ])
