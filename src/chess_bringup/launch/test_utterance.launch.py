"""One-shot helper: publish a text utterance on /chess/voice/utterance.

Use case: integration-test the move pipeline without a microphone.
Usage:
  ros2 launch chess_bringup test_utterance.launch.py utterance:='peon de e2 a e4'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    utt = LaunchConfiguration("utterance")
    return LaunchDescription([
        DeclareLaunchArgument("utterance",
                              default_value="peon de e2 a e4"),
        ExecuteProcess(
            cmd=["ros2", "topic", "pub", "-1",
                 "/chess/voice/utterance", "std_msgs/msg/String",
                 ["data: '", utt, "'"]],
            output="screen",
        ),
    ])
