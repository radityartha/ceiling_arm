import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    whisper_launch = os.path.join(
        get_package_share_directory("whisper_bringup"),
        "launch",
        "whisper.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_whisper_pipeline",
                default_value="true",
                description="Start microphone capture, Silero VAD, and Whisper streaming.",
            ),
            DeclareLaunchArgument(
                "use_whisper_bridge",
                default_value="true",
                description="Start the bridge from /whisper/transcription to /voice/transcript.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel",
                description="Velocity command topic for the mobile base simulation.",
            ),
            DeclareLaunchArgument(
                "linear_speed_mps",
                default_value="0.15",
                description="Safe linear speed used by move_forward and move_backward.",
            ),
            DeclareLaunchArgument(
                "move_duration_sec",
                default_value="2.5",
                description="Duration for each forward/backward command.",
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
            DeclareLaunchArgument(
                "use_web_ui",
                default_value="true",
                description="Start a small local browser UI for monitoring and manual override.",
            ),
            DeclareLaunchArgument(
                "web_ui_port",
                default_value="8080",
                description="HTTP port for the local browser UI.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(whisper_launch),
                condition=IfCondition(LaunchConfiguration("use_whisper_pipeline")),
            ),
            Node(
                package="sayai_voice_sim",
                executable="mobile_robot_task_server",
                name="mobile_robot_task_server",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "linear_speed_mps": LaunchConfiguration("linear_speed_mps"),
                        "move_duration_sec": LaunchConfiguration("move_duration_sec"),
                    }
                ],
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
            Node(
                package="sayai_voice_sim",
                executable="voice_web_ui",
                name="voice_web_ui",
                output="screen",
                parameters=[
                    {
                        "port": LaunchConfiguration("web_ui_port"),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("use_web_ui")),
            ),
        ]
    )
