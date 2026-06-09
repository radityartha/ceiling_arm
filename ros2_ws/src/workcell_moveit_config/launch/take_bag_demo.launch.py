"""Launch the take-the-bag demo sequence runner.

Assumes my_workcell.launch.py is already running (MoveIt, arm controllers)
AND a dual_table_controller is already running (its own terminal, table_keyboard.py, etc.).
Only the one-shot sequence runner is started by default.

    # Normal case: a dual_table_controller is already running.
    ros2 launch workcell_moveit_config take_bag_demo.launch.py
    # Let this launch spawn the controller too (only if nothing else owns the serial ports):
    ros2 launch workcell_moveit_config take_bag_demo.launch.py start_table_controller:=true
    ros2 launch workcell_moveit_config take_bag_demo.launch.py start_table_controller:=true use_fake_tables:=true

NOTE: never run two dual_table_controllers at once -- the second cannot lock
/dev/ttyUSB* and comes up with table1/table2 = None, which the runner rejects with
"Table 'table2' is not initialized or available."
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("start_table_controller", default_value="false",
                              description="Set true to spawn dual_table_controller; leave false if one "
                                          "is already running (the normal case). Two controllers fight "
                                          "for the serial ports -> table1/table2 come up uninitialized."),
        DeclareLaunchArgument("use_fake_tables", default_value="false",
                              description="Use fake hardware for table motors"),
        DeclareLaunchArgument("gripper_grip_deg", default_value="45.0",
                              description="Bottom-finger joint angle (deg) when gripping (max 46° = 0.81 rad kortex limit)"),
        DeclareLaunchArgument("gripper_open_deg", default_value="0.0",
                              description="Bottom-finger joint angle (deg) when open/loosened"),
        DeclareLaunchArgument("gripper_max_effort", default_value="50.0",
                              description="Max effort for GripperCommand goals"),
        DeclareLaunchArgument("skip_grippers", default_value="false",
                              description="Skip all gripper steps (arm+table only)"),
        DeclareLaunchArgument("linear_speed", default_value="3000",
                              description="Table linear speed (pulses/s)"),
        DeclareLaunchArgument("rotate_speed", default_value="1000",
                              description="Table rotation speed (pulses/s)"),
        DeclareLaunchArgument("planning_time", default_value="10.0",
                              description="MoveIt allowed planning time (s) per arm/gripper goal"),
        DeclareLaunchArgument("vel_scale", default_value="0.2",
                              description="Max velocity scaling (0-1] for arm/gripper moves"),
        DeclareLaunchArgument("acc_scale", default_value="0.2",
                              description="Max acceleration scaling (0-1] for arm/gripper moves"),
        DeclareLaunchArgument("table_timeout_s", default_value="120.0",
                              description="Max wait (s) for a table move to reach target"),
        DeclareLaunchArgument("table_tol_mm", default_value="5.0",
                              description="Linear tolerance (mm) for table completion"),
        DeclareLaunchArgument("table_tol_deg", default_value="2.0",
                              description="Rotation tolerance (deg) for table completion"),
        DeclareLaunchArgument("table_stable_samples", default_value="6",
                              description="Consecutive in-tolerance joint_states samples required "
                                          "before a table counts as in position (settled, not just "
                                          "passing through). ~6 @ ~0.2s ≈ 1.2s."),
        DeclareLaunchArgument("startup_delay_s", default_value="3.0",
                              description="Delay (s) before the first command"),
        DeclareLaunchArgument("motor_settle_s", default_value="1.0",
                              description="Extra settle time (s) after table joints reach tolerance"),
        DeclareLaunchArgument("gripper_pre_delay_s", default_value="1.5",
                              description="Delay (s) before each gripper goal — allows Kortex hardware "
                                          "to transition from high-level to low-level servoing mode "
                                          "after an arm trajectory (prevents WRONG_SERVOING_MODE)"),
    ]

    table_controller = Node(
        package="moving_table_pkg",
        executable="dual_table_controller",
        name="dual_table_controller",
        output="screen",
        parameters=[{
            "use_fake_hardware": LaunchConfiguration("use_fake_tables"),
        }],
        condition=IfCondition(LaunchConfiguration("start_table_controller")),
    )

    runner = Node(
        package="workcell_description",
        executable="run_take_bag.py",
        name="take_bag_runner",
        output="screen",
        parameters=[{
            "gripper_grip_deg": LaunchConfiguration("gripper_grip_deg"),
            "gripper_open_deg": LaunchConfiguration("gripper_open_deg"),
            "gripper_max_effort": LaunchConfiguration("gripper_max_effort"),
            "skip_grippers": LaunchConfiguration("skip_grippers"),
            "linear_speed": LaunchConfiguration("linear_speed"),
            "rotate_speed": LaunchConfiguration("rotate_speed"),
            "planning_time": LaunchConfiguration("planning_time"),
            "vel_scale": LaunchConfiguration("vel_scale"),
            "acc_scale": LaunchConfiguration("acc_scale"),
            "table_timeout_s": LaunchConfiguration("table_timeout_s"),
            "table_tol_mm": LaunchConfiguration("table_tol_mm"),
            "table_tol_deg": LaunchConfiguration("table_tol_deg"),
            "table_stable_samples": LaunchConfiguration("table_stable_samples"),
            "startup_delay_s": LaunchConfiguration("startup_delay_s"),
            "motor_settle_s": LaunchConfiguration("motor_settle_s"),
            "gripper_pre_delay_s": LaunchConfiguration("gripper_pre_delay_s"),
        }],
    )

    return LaunchDescription(args + [table_controller, runner])
