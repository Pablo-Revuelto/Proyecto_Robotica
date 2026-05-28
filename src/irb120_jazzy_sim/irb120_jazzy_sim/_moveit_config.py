"""Construye la moveit_config completa del IRB-120 para MoveItPy."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_config_dict():
    pkg_share = Path(get_package_share_directory("irb120_jazzy_sim"))
    urdf_xacro = str(pkg_share / "urdf" / "irb120.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder("irb120", package_name="irb120_jazzy_sim")
        .robot_description(
            file_path=urdf_xacro,
            mappings={"use_gazebo": "false", "use_mock_hardware": "true"},
        )
        .robot_description_semantic(file_path=str(pkg_share / "config" / "irb120.srdf"))
        .trajectory_execution(file_path=str(pkg_share / "config" / "moveit_controllers.yaml"))
        .joint_limits(file_path=str(pkg_share / "config" / "joint_limits.yaml"))
        .robot_description_kinematics(file_path=str(pkg_share / "config" / "kinematics.yaml"))
        .planning_pipelines(pipelines=["ompl"])
        .moveit_cpp(file_path=str(pkg_share / "config" / "moveit_cpp.yaml"))
        .to_moveit_configs()
    )
    return moveit_config.to_dict()
