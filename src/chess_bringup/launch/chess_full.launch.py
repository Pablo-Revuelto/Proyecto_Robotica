"""Launch the entire dual-arm chess stack.

All node tuning (LLM on/off, LLM model, YOLO weights, Whisper model/device,
camera + board geometry, ...) lives in each package's `config/params.yaml`:
  - chess_voice/config/params.yaml       (voice_parser, whisper_asr, audio_capture)
  - chess_perception/config/params.yaml  (board_state_estimator)
Edit those files to configure the system; no command-line arguments needed.

The only launch-level switches are which subsystems to start at all:
  enable_voice:=false   skip mic / whisper / audio capture (text-only testing)
  enable_vision:=false  skip the YOLO board estimator
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_llm        = LaunchConfiguration("use_llm")
    llm_model_id   = LaunchConfiguration("llm_model_id")
    yolo_weights   = LaunchConfiguration("yolo_weights")
    whisper_model  = LaunchConfiguration("whisper_model")
    whisper_device = LaunchConfiguration("whisper_device")
    enable_voice   = LaunchConfiguration("enable_voice")
    enable_vision  = LaunchConfiguration("enable_vision")

    args = [
        DeclareLaunchArgument("use_llm",        default_value="true"),
        DeclareLaunchArgument("llm_model_id",
                              default_value="meta-llama/Meta-Llama-3-8B-Instruct"),
        # Empty by default (matches README §6.2): perception then uses the
        # NullDetector and publishes an empty board. An empty value cannot be
        # passed on the CLI as `yolo_weights:=""` (ros2 launch rejects it), so
        # the default must be empty here.
        DeclareLaunchArgument("yolo_weights",   default_value=""),
        DeclareLaunchArgument("whisper_model",  default_value="openai/whisper-small"),
        DeclareLaunchArgument("whisper_device", default_value="cpu"),
        DeclareLaunchArgument("enable_voice",   default_value="true"),
        DeclareLaunchArgument("enable_vision",  default_value="true"),
    ]

    # Central per-package parameter files (the single source of truth).
    chess_voice_params = PathJoinSubstitution([
        FindPackageShare("chess_voice"), "config", "params.yaml"
    ])
    chess_perception_params = PathJoinSubstitution([
        FindPackageShare("chess_perception"), "config", "params.yaml"
    ])

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
        ]),
        # Gazebo already publishes TF (robot_state_publisher) and joint states,
        # so MoveIt must not start its own to avoid duplicate /tf publishers.
        launch_arguments={"use_rsp": "false", "use_sim_time": "true"}.items(),
    )

    # Keep the engine (game_manager), the piece registry (move_executor) and the
    # spawned scene (chess_gazebo) on the SAME position by reading the spawner's
    # initial_fen from board_layout.yaml and feeding it to both nodes.
    _board_layout = (Path(get_package_share_directory("chess_gazebo"))
                     / "config" / "board_layout.yaml")
    _initial_fen = yaml.safe_load(_board_layout.read_text()).get("initial_fen", "")

    # move_executor is now a *client* of the running move_group (see
    # chess_motion/move_executor_node.py), so it needs no MoveIt config of its own.
    move_executor = Node(
        package="chess_motion", executable="move_executor",
        output="screen",
        parameters=[{"use_sim_time": True, "initial_fen": _initial_fen}],
    )

    game_manager = Node(
        package="chess_brain", executable="game_manager",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "initial_fen": _initial_fen,
            # Advisory perception validator (silent by default; never blocks).
            "perception_validation_enabled": True,
            "perception_validation_confidence": 0.80,
            "perception_min_detections": 2,
            "perception_warn_cooldown_sec": 10.0,
            "perception_warn_on_empty_engine": False,
            "perception_log_matches": False,
        }],
    )

    # voice_parser is always-on: the regex fallback works even without an LLM.
    # use_llm / llm_model_id / token come from chess_voice/config/params.yaml.
    voice_parser = Node(
        package="chess_voice", executable="voice_parser",
        output="screen",
        parameters=[chess_voice_params, {"use_sim_time": True}],
    )

    # Mic + Whisper only when enable_voice:=true (model/device from params.yaml).
    whisper = Node(
        package="chess_voice", executable="whisper_asr",
        output="screen",
        condition=IfCondition(enable_voice),
        parameters=[chess_voice_params, {"use_sim_time": True}],
    )

    mic = Node(
        package="chess_voice", executable="audio_capture",
        output="screen",
        condition=IfCondition(enable_voice),
        parameters=[chess_voice_params, {"use_sim_time": True}],
    )

    # Vision node only when enable_vision:=true (yolo_weights from params.yaml).
    vision = Node(
        package="chess_perception", executable="board_state_estimator",
        output="screen",
        condition=IfCondition(enable_vision),
        parameters=[chess_perception_params, {"use_sim_time": True}],
    )

    return LaunchDescription(args + [
        gazebo,
        TimerAction(period=8.0,  actions=[moveit]),
        TimerAction(period=12.0, actions=[move_executor, game_manager, voice_parser]),
        TimerAction(period=14.0, actions=[whisper, mic, vision]),
    ])
