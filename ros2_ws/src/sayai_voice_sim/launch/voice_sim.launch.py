from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_whisper_bridge",
                default_value="false",
                description="Start the bridge from /whisper/transcription to /voice/transcript.",
            ),
            DeclareLaunchArgument(
                "require_wake_word",
                default_value="true",
                description="Require a wake word before forwarding Whisper text.",
            ),
            DeclareLaunchArgument(
                "wake_window_sec",
                default_value="30.0",
                description="Seconds to accept commands after a wake word.",
            ),
            Node(
                package="sayai_voice_sim",
                executable="mock_robot_task_server",
                name="mock_robot_task_server",
                output="screen",
            ),
            Node(
                package="sayai_voice_sim",
                executable="voice_command_manager",
                name="voice_command_manager",
                output="screen",
            ),
            Node(
                package="sayai_voice_sim",
                executable="whisper_transcript_bridge",
                name="whisper_transcript_bridge",
                output="screen",
                parameters=[
                    {
                        "require_wake_word": LaunchConfiguration("require_wake_word"),
                        "wake_window_sec": LaunchConfiguration("wake_window_sec"),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("use_whisper_bridge")),
            ),
        ]
    )
