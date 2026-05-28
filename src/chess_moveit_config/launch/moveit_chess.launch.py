"""Launch MoveIt 2 (move_group + RViz) configured for the dual IRB120 chess scene."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    desc_share = Path(get_package_share_directory("chess_description"))
    cfg_share = Path(get_package_share_directory("chess_moveit_config"))

    urdf_xacro = str(desc_share / "urdf" / "dual_irb120_chess.urdf.xacro")
    srdf_file = str(cfg_share / "config" / "chess.srdf")

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

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            robot_description,
            {"use_sim_time": True},
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
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([move_group, rviz])
