"""Voice control for the real workcell sequences.

Run the Whisper driver separately first, e.g.
`ros2 launch whisper_bringup whisper.launch.py`. The flow is:

    /whisper/transcription
        -> whisper_transcript_bridge  (wake-word gate)
        -> /voice/transcript
        -> voice_command_manager      (match words -> /task/* Trigger)
        -> real_robot_task_server     (launch the matching sequence)

The sequences need my_workcell.launch.py and a dual_table_controller already
running, just like starting the demos by hand.

    ros2 launch sayai_voice_sim workcell_voice.launch.py
    # dry-run the speech part without moving the robot:
    ros2 launch sayai_voice_sim workcell_voice.launch.py use_mock:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_mock = LaunchConfiguration("use_mock")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_mock", default_value="false",
            description="Use the fake task server (logs only) instead of running "
                        "real sequences. For testing speech without moving the robot."),
        DeclareLaunchArgument(
            "require_wake_word", default_value="true",
            description="Require a wake word before forwarding Whisper text."),
        DeclareLaunchArgument(
            "wake_window_sec", default_value="30.0",
            description="Seconds to accept commands after a wake word."),

        Node(
            package="sayai_voice_sim",
            executable="whisper_transcript_bridge",
            name="whisper_transcript_bridge",
            output="screen",
            parameters=[{
                "require_wake_word": LaunchConfiguration("require_wake_word"),
                "wake_window_sec": LaunchConfiguration("wake_window_sec"),
            }],
        ),
        Node(
            package="sayai_voice_sim",
            executable="voice_command_manager",
            name="voice_command_manager",
            output="screen",
        ),
        Node(
            package="sayai_voice_sim",
            executable="real_robot_task_server",
            name="real_robot_task_server",
            output="screen",
            condition=UnlessCondition(use_mock),
        ),
        Node(
            package="sayai_voice_sim",
            executable="mock_robot_task_server",
            name="mock_robot_task_server",
            output="screen",
            condition=IfCondition(use_mock),
        ),
        Node(
            package="sayai_voice_sim",
            executable="voice_web_ui",
            name="voice_web_ui",
            output="screen",
        ),
    ])
