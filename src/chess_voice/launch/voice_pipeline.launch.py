"""Launch the voice pipeline (audio capture + whisper ASR + voice parser).

By default, loads parameters from `config/params.yaml`.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("chess_voice")
    
    default_params_file = PathJoinSubstitution([
        pkg_share,
        "config",
        "params.yaml"
    ])

    params_file = LaunchConfiguration("params_file")
    enable_voice = LaunchConfiguration("enable_voice")

    args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Full path to the ROS 2 parameters YAML file for chess_voice nodes."
        ),
        DeclareLaunchArgument(
            "enable_voice",
            default_value="true",
            description="Whether to launch mic capture and Whisper ASR nodes (set to false for text-only testing)."
        ),
    ]

    # Always-on voice parser node (exposes `/chess/voice/parse` service, regex fallback works offline)
    voice_parser = Node(
        package="chess_voice",
        executable="voice_parser",
        name="chess_voice_parser",
        output="screen",
        parameters=[params_file],
    )

    # Audio Capture node (records mic data when enable_voice is true)
    audio_capture = Node(
        package="chess_voice",
        executable="audio_capture",
        name="chess_audio_capture",
        output="screen",
        parameters=[params_file],
        condition=IfCondition(enable_voice),
    )

    # Whisper ASR node (speech-to-text when enable_voice is true)
    whisper_asr = Node(
        package="chess_voice",
        executable="whisper_asr",
        name="chess_whisper_asr",
        output="screen",
        parameters=[params_file],
        condition=IfCondition(enable_voice),
    )

    return LaunchDescription(args + [
        voice_parser,
        audio_capture,
        whisper_asr,
    ])
