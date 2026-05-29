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

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    enable_voice  = LaunchConfiguration("enable_voice")
    enable_vision = LaunchConfiguration("enable_vision")

    args = [
        DeclareLaunchArgument("enable_voice",  default_value="true"),
        DeclareLaunchArgument("enable_vision", default_value="true"),
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

    move_executor = Node(
        package="chess_motion", executable="move_executor",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    game_manager = Node(
        package="chess_brain", executable="game_manager",
        output="screen",
        parameters=[{"use_sim_time": True}],
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
