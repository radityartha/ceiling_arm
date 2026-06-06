"""Single-RViz workcell bringup — switch the planning group, not the launch.

Brings up all four real Kinova arms + both tables under ONE move_group and ONE
RViz, using the workcell SRDF that already defines the arm_1..arm_4 groups. To
move a different arm you just pick its group in the RViz MotionPlanning panel —
no need to stop and relaunch with a different IP.

Why this is a new file rather than a tweak to my_workcell.launch.py:
  * my_workcell.launch.py loads trailer_workcell.urdf.xacro, which forces mock
    hardware (it never forwards the IP/fake args and adds a FakeSystem block).
    It is left untouched so that mock-only flow keeps working.
  * This launch loads config/real_workcell.urdf.xacro, which forwards
    use_fake_hardware + the four IPs into workcell.urdf.xacro so the arms come
    up on the real KortexMultiInterfaceHardware driver.

Per-arm controllers (arm_1..arm_4) are spawned and registered with MoveIt via
config/moveit_controllers_per_arm.yaml, so each SRDF group executes on its own
FollowJointTrajectory controller.

Arm IPs:
  arm_1 (t1_a1) = 192.168.2.13
  arm_2 (t1_a2) = 192.168.2.12
  arm_3 (t2_a1) = 192.168.2.11
  arm_4 (t2_a2) = 192.168.2.10

Usage:
  ros2 launch workcell_moveit_config single_rviz_workcell.launch.py
  ros2 launch workcell_moveit_config single_rviz_workcell.launch.py use_fake_hardware:=true

Prefer scripts/start_single_rviz.sh, which strips the broken rviz2_ws/moveit2_ws
overlays from the environment first (see project-single-arm-bringup memory).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _strip_broken_overlays():
    """Drop rviz2_ws / moveit2_ws from every env-var so every spawned node uses
    the system /opt/ros/humble install instead of the broken VNC-injected ones."""
    for var in ("PATH", "LD_LIBRARY_PATH", "AMENT_PREFIX_PATH",
                "CMAKE_PREFIX_PATH", "PYTHONPATH"):
        val = os.environ.get(var)
        if not val:
            continue
        os.environ[var] = os.pathsep.join(
            p for p in val.split(os.pathsep)
            if "rviz2_ws" not in p and "moveit2_ws" not in p
        )


def generate_launch_description():
    _strip_broken_overlays()

    args = [
        DeclareLaunchArgument("use_fake_hardware", default_value="false",
                              description="Mock all arms (true = no robots needed)"),
        DeclareLaunchArgument("arm1_ip", default_value="192.168.2.13",
                              description="Arm 1 (t1_a1) IP"),
        DeclareLaunchArgument("arm2_ip", default_value="192.168.2.12",
                              description="Arm 2 (t1_a2) IP"),
        DeclareLaunchArgument("arm3_ip", default_value="192.168.2.11",
                              description="Arm 3 (t2_a1) IP"),
        DeclareLaunchArgument("arm4_ip", default_value="192.168.2.10",
                              description="Arm 4 (t2_a2) IP"),
        # Per-arm mock: default to the global use_fake_hardware, but any single
        # arm can be mocked (e.g. armN_fake:=true) so an unreachable/powered-off
        # arm doesn't abort the whole ros2_control_node.
        DeclareLaunchArgument("arm1_fake", default_value=LaunchConfiguration("use_fake_hardware"),
                              description="Mock arm 1 only"),
        DeclareLaunchArgument("arm2_fake", default_value=LaunchConfiguration("use_fake_hardware"),
                              description="Mock arm 2 only"),
        DeclareLaunchArgument("arm3_fake", default_value=LaunchConfiguration("use_fake_hardware"),
                              description="Mock arm 3 only"),
        DeclareLaunchArgument("arm4_fake", default_value=LaunchConfiguration("use_fake_hardware"),
                              description="Mock arm 4 only"),
    ]

    moveit_config = (
        MoveItConfigsBuilder("trailer_workcell", package_name="workcell_moveit_config")
        .robot_description(
            file_path="config/real_workcell.urdf.xacro",
            mappings={
                "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                "arm1_ip":           LaunchConfiguration("arm1_ip"),
                "arm2_ip":           LaunchConfiguration("arm2_ip"),
                "arm3_ip":           LaunchConfiguration("arm3_ip"),
                "arm4_ip":           LaunchConfiguration("arm4_ip"),
                "arm1_fake":         LaunchConfiguration("arm1_fake"),
                "arm2_fake":         LaunchConfiguration("arm2_fake"),
                "arm3_fake":         LaunchConfiguration("arm3_fake"),
                "arm4_fake":         LaunchConfiguration("arm4_fake"),
                "use_sim_time":      "false",
            },
        )
        .trajectory_execution(file_path="config/moveit_controllers_per_arm.yaml")
        .to_moveit_configs()
    )

    config_dict = moveit_config.to_dict()
    rviz_config = str(
        moveit_config.package_path / "config" / "moveit.rviz"
    )

    # to_dict() may carry the default moveit_controllers.yaml when called with
    # unresolved LaunchConfiguration substitutions. Load the per-arm YAML directly
    # so it explicitly overrides whatever the controller manager key says.
    import yaml as _yaml
    per_arm_controllers_yaml = _yaml.safe_load(
        (moveit_config.package_path / "config" / "moveit_controllers_per_arm.yaml")
        .read_text()
    )

    nodes = [
        # Robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[moveit_config.robot_description],
        ),

        # move_group — one instance, with per-arm controller config
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                config_dict,
                per_arm_controllers_yaml,   # explicit override for controller list
                {
                    "publish_robot_description_semantic": True,
                    "allow_trajectory_execution": True,
                    "publish_planning_scene": True,
                    "publish_geometry_updates": True,
                    "publish_state_updates": True,
                    "publish_transforms_updates": True,
                    "monitor_dynamics": False,
                    # Give the Kortex arms more time to finish trajectories.
                    # Default 1.2x is too tight at low vel_scale; 3.0x avoids
                    # mid-trajectory cancellation that crashes the cyclic driver.
                    "allowed_execution_duration_scaling": 3.0,
                    "allowed_goal_duration_margin": 5.0,
                },
            ],
        ),

        # RViz — one instance
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=["-d", rviz_config],
            parameters=[config_dict],
        ),

        # ros2_control_node — loads Kortex hardware interfaces
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config" / "ros2_controllers.yaml"),
            ],
        ),

        # Static TF for the LIDAR (overhead, pointing down)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="livox_static_tf",
            arguments=["2.3", "0", "1.9", "3.14159", "3.14159", "0",
                       "world", "livox_frame"],
            parameters=[{"use_sim_time": False}],
        ),
    ]

    # Per-arm + gripper controllers only (no table joints → no velocity-interface crash)
    spawners = [
        Node(package="controller_manager", executable="spawner",
             arguments=[c], output="screen")
        for c in [
            "joint_state_broadcaster",
            "arm_1_controller", "arm_2_controller",
            "arm_3_controller", "arm_4_controller",
            "gripper_1_controller", "gripper_2_controller",
            "gripper_3_controller", "gripper_4_controller",
        ]
    ]

    # arm_1 Cartesian-velocity controller for the compliant curtain pull.
    # Loaded inactive (--inactive): it claims nothing until run_close_curtain
    # switches arm_1 from arm_1_controller to this for the pull, then back.
    spawners.append(
        Node(package="controller_manager", executable="spawner",
             arguments=["t1_a1_twist_controller", "--inactive"], output="screen")
    )

    return LaunchDescription(args + nodes + spawners)
