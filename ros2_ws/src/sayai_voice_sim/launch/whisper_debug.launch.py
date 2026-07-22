"""Whisper launch + print every transcription to the terminal.

Use this instead of `ros2 launch whisper_bringup whisper.launch.py` when you
want to see the raw text whisper hears printed in the same terminal.

    ros2 launch sayai_voice_sim whisper_debug.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    whisper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("whisper_bringup"),
                "launch",
                "whisper.launch.py",
            )
        )
    )

    # Wait 3 s for the whisper node to start before echoing
    print_transcript = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "topic", "echo", "/whisper/transcription"],
                output="screen",
            )
        ],
    )

    return LaunchDescription([whisper_launch, print_transcript])
