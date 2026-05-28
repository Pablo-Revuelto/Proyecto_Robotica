"""Launch the entire dual-arm chess stack.

Modes controlled by launch args:
  enable_voice:=false   skip mic / whisper / audio capture (text-only testing)
  enable_vision:=false  skip the YOLO board estimator
  use_llm:=false        use regex fallback parser instead of LLM (fast dev)
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
        DeclareLaunchArgument("yolo_weights",
                              default_value="~/Robotica_inteligente/ros2_ws/best.pt"),
        DeclareLaunchArgument("whisper_model",  default_value="openai/whisper-small"),
        DeclareLaunchArgument("whisper_device", default_value="cpu"),
        DeclareLaunchArgument("enable_voice",   default_value="true"),
        DeclareLaunchArgument("enable_vision",  default_value="true"),
    ]

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
        ])
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

    # voice_parser is always-on: the regex fallback works without an LLM token.
    voice_parser = Node(
        package="chess_voice", executable="voice_parser",
        output="screen",
        parameters=[{
            "use_llm": use_llm,
            "llm_model_id": llm_model_id,
            "use_sim_time": True,
        }],
    )

    # Mic + Whisper only when enable_voice:=true
    whisper = Node(
        package="chess_voice", executable="whisper_asr",
        output="screen",
        condition=IfCondition(enable_voice),
        parameters=[{
            "model": whisper_model,
            "device": whisper_device,
            "use_sim_time": True,
        }],
    )

    mic = Node(
        package="chess_voice", executable="audio_capture",
        output="screen",
        condition=IfCondition(enable_voice),
        parameters=[{"use_sim_time": True}],
    )

    # Vision node only when enable_vision:=true
    vision = Node(
        package="chess_perception", executable="board_state_estimator",
        output="screen",
        condition=IfCondition(enable_vision),
        parameters=[{
            "yolo_weights": yolo_weights,
            "use_sim_time": True,
        }],
    )

    return LaunchDescription(args + [
        gazebo,
        TimerAction(period=8.0,  actions=[moveit]),
        TimerAction(period=12.0, actions=[move_executor, game_manager, voice_parser]),
        TimerAction(period=14.0, actions=[whisper, mic, vision]),
    ])
