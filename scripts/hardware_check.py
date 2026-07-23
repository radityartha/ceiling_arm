#!/usr/bin/env python3
"""
Moonshot Workcell - Hardware Check Script

Systematically verifies every piece of hardware:
  Stage 1 (--preflight)  : USB serial ports, arm network pings, ROS2 env  [no ROS needed]
  Stage 2 (--tables)     : Table-1 and Table-2 motor move-and-return test  [needs dual_table_controller]
  Stage 3 (--arms)       : All 4 arms move to home position via MoveIt      [needs my_workcell.launch.py]
  Stage 4 (--grippers)   : All 4 grippers open then close                   [needs my_workcell.launch.py]

Usage examples:
  # Check cables and network ONLY (no ROS needed)
  python3 scripts/hardware_check.py --preflight --arm-ips 192.168.1.10 192.168.1.11 192.168.1.12 192.168.1.13

  # Run full check (start all launch files first — see instructions printed by this script)
  python3 scripts/hardware_check.py --all --arm-ips 192.168.1.10 192.168.1.11 192.168.1.12 192.168.1.13

  # Individual stages
  python3 scripts/hardware_check.py --tables
  python3 scripts/hardware_check.py --arms
  python3 scripts/hardware_check.py --grippers
"""

import argparse
import os
import subprocess
import sys
import time
import threading

# ──────────────────────────────────────────────────────────────────────────────
# Terminal colours
# ──────────────────────────────────────────────────────────────────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

PASS_COUNT = 0
FAIL_COUNT = 0
WARN_COUNT = 0


def ok(msg):
    global PASS_COUNT
    PASS_COUNT += 1
    print(f"  {GREEN}[PASS]{RESET} {msg}")


def fail(msg):
    global FAIL_COUNT
    FAIL_COUNT += 1
    print(f"  {RED}[FAIL]{RESET} {msg}")


def warn(msg):
    global WARN_COUNT
    WARN_COUNT += 1
    print(f"  {YELLOW}[WARN]{RESET} {msg}")


def info(msg):
    print(f"  {CYAN}[INFO]{RESET} {msg}")


def header(msg):
    bar = "═" * 62
    print(f"\n{BOLD}{CYAN}{bar}{RESET}")
    print(f"{BOLD}  {msg}{RESET}")
    print(f"{BOLD}{CYAN}{bar}{RESET}")


def section(msg):
    print(f"\n{BOLD}▸ {msg}{RESET}")


def summary():
    bar = "─" * 40
    print(f"\n{BOLD}{bar}{RESET}")
    print(f"{BOLD}  Results: "
          f"{GREEN}{PASS_COUNT} passed{RESET}  "
          f"{RED}{FAIL_COUNT} failed{RESET}  "
          f"{YELLOW}{WARN_COUNT} warnings{RESET}")
    print(f"{BOLD}{bar}{RESET}\n")


# ──────────────────────────────────────────────────────────────────────────────
# Stage 1 – Pre-flight (no ROS)
# ──────────────────────────────────────────────────────────────────────────────

def check_serial_port(port: str):
    if os.path.exists(port):
        ok(f"{port} exists")
        if os.access(port, os.R_OK | os.W_OK):
            ok(f"{port} is readable/writable")
        else:
            fail(f"{port} permission denied  →  run:  sudo chmod a+rw {port}")
            info("Or add yourself to dialout:  sudo usermod -aG dialout $USER  (then log out/in)")
    else:
        fail(f"{port} NOT found — check USB-to-RS485 cable for the table controller")


def ping_host(ip: str, label: str) -> bool:
    result = subprocess.run(
        ["ping", "-c", "1", "-W", "2", ip],
        capture_output=True, text=True
    )
    if result.returncode == 0:
        ok(f"{label} ({ip}) — reachable")
        return True
    else:
        fail(f"{label} ({ip}) — NOT reachable (check Ethernet cable and arm power)")
        return False


def run_preflight(arm_ips: list):
    header("STAGE 1 · Pre-flight System Check")

    section("USB serial ports (table motors)")
    check_serial_port("/dev/ttyUSB0")   # Table 1
    check_serial_port("/dev/ttyUSB1")   # Table 2

    section("Network connectivity (Kinova arms)")
    if arm_ips:
        labels = [
            "Arm 1 — Table-1 Left  (t1_a1)",
            "Arm 2 — Table-1 Right (t1_a2)",
            "Arm 3 — Table-2 Left  (t2_a1)",
            "Arm 4 — Table-2 Right (t2_a2)",
        ]
        for i, ip in enumerate(arm_ips):
            lbl = labels[i] if i < len(labels) else f"Arm {i+1}"
            ping_host(ip, lbl)
    else:
        warn("No arm IPs provided — pass  --arm-ips IP1 IP2 IP3 IP4  to test network")
        info("Kinova Gen3 Lite factory default IP:  192.168.1.10")

    section("ROS 2 environment")
    result = subprocess.run(["ros2", "--help"], capture_output=True, text=True)
    if result.returncode == 0:
        ok("ros2 CLI available")
    else:
        fail("ros2 CLI not found — source /opt/ros/humble/setup.bash")

    ament_path = os.environ.get("AMENT_PREFIX_PATH", "")
    if "workcell_moveit_config" in ament_path:
        ok("Workspace is sourced (workcell_moveit_config visible)")
    elif "workcell_description" in ament_path:
        ok("Workspace is sourced (workcell_description visible)")
    else:
        warn("Workspace may not be sourced")
        info("Run:  source ~/Documents/ceiling_arm/ros2_ws/install/setup.bash")

    print()
    info("To test real hardware next, start the table controller in a NEW terminal:")
    info("  ros2 run moving_table_pkg dual_table_controller --ros-args -p use_fake_hardware:=false")
    info("Then in another terminal run:  python3 scripts/hardware_check.py --tables")
    info("")
    info("To test arms/grippers, first update arm IPs in the URDF (see README) then:")
    info("  ros2 launch workcell_moveit_config my_workcell.launch.py use_sim_time:=false")
    info("Then run:  python3 scripts/hardware_check.py --arms --grippers")


# ──────────────────────────────────────────────────────────────────────────────
# Stage 2 – Table motor test
# ──────────────────────────────────────────────────────────────────────────────

def _call_table_service(node, client, table_id, distance_mm, angle_deg, label):
    from moving_table_interfaces.srv import MovingTable

    req = MovingTable.Request()
    req.table_id       = table_id
    req.distance_mm    = float(distance_mm)
    req.angle_deg      = float(angle_deg)
    req.linear_speed   = 30000
    req.rotate_speed   = 10000
    req.operation_type = 1   # 1=absolute (go_to_table always uses absolute internally)

    info(f"{label}: sending {distance_mm:+.0f} mm, {angle_deg:+.0f}°  …")
    future = client.call_async(req)

    # Spin until the future completes (with timeout)
    deadline = time.time() + 30.0
    while not future.done():
        import rclpy
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() > deadline:
            fail(f"{label}: service call timed out (30 s)")
            return False

    if future.result().success:
        ok(f"{label}: {future.result().message.strip()}")
        return True
    else:
        fail(f"{label}: {future.result().message.strip()}")
        return False


def run_tables_test():
    header("STAGE 2 · Table Motor Test")
    info("Prerequisite: dual_table_controller must be running with use_fake_hardware:=false")
    info("Command: ros2 run moving_table_pkg dual_table_controller --ros-args -p use_fake_hardware:=false")

    try:
        import rclpy
        from rclpy.node import Node
        from moving_table_interfaces.srv import MovingTable
    except ImportError as e:
        fail(f"ROS2 import failed: {e}  — source your workspace first")
        return

    rclpy.init()
    node = Node("hw_check_table_tester")

    try:
        client = node.create_client(MovingTable, "move_dual_table")
        info("Waiting for move_dual_table service (10 s) …")
        if not client.wait_for_service(timeout_sec=10.0):
            fail("Service /move_dual_table not available — is dual_table_controller running?")
            return

        ok("Service /move_dual_table is available")

        for table_id, label in [("table1", "Table 1"), ("table2", "Table 2")]:
            section(f"{label} movement test")
            # Move forward 50 mm
            moved = _call_table_service(node, client, table_id, +50, 0, f"{label} forward")
            if moved:
                time.sleep(35.0)  # wait for motor to finish (30s timeout + buffer)
                # Return: send -50mm incremental to go back
                _call_table_service(node, client, table_id, -50, 0, f"{label} return -50mm")
                time.sleep(35.0)

        section("Table joint states")
        from sensor_msgs.msg import JointState
        received = {"msg": None}

        def js_cb(msg):
            received["msg"] = msg

        sub = node.create_subscription(JointState, "/joint_states", js_cb, 10)
        deadline = time.time() + 5.0
        while received["msg"] is None and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        if received["msg"]:
            table_joints = [j for j in received["msg"].name if j.startswith("t1_") or j.startswith("t2_")]
            if table_joints:
                ok(f"/joint_states includes table joints: {table_joints}")
            else:
                warn("/joint_states present but no t1_/t2_ table joints found")
        else:
            fail("/joint_states not being published")

    finally:
        node.destroy_node()
        rclpy.shutdown()


# ──────────────────────────────────────────────────────────────────────────────
# Stage 3 – Arm test via MoveIt
# ──────────────────────────────────────────────────────────────────────────────

# Named home positions from trailer_workcell.srdf
ARM_HOME_POSITIONS = {
    "arm_1": {
        "t1_a1_joint_1": 0.0,
        "t1_a1_joint_2": -0.2792,
        "t1_a1_joint_3":  1.309,
        "t1_a1_joint_4":  0.0,
        "t1_a1_joint_5": -1.0471,
        "t1_a1_joint_6":  0.0,
    },
    "arm_2": {
        "t1_a2_joint_1": 0.0,
        "t1_a2_joint_2": -0.2792,
        "t1_a2_joint_3":  1.309,
        "t1_a2_joint_4":  0.0,
        "t1_a2_joint_5": -1.0471,
        "t1_a2_joint_6":  0.0,
    },
    "arm_3": {
        "t2_a1_joint_1": 0.0,
        "t2_a1_joint_2":  0.0,
        "t2_a1_joint_3":  1.309,
        "t2_a1_joint_4":  0.0,
        "t2_a1_joint_5": -1.0471,
        "t2_a1_joint_6":  0.0,
    },
    "arm_4": {
        "t2_a2_joint_1": 0.0,
        "t2_a2_joint_2": -0.2792,
        "t2_a2_joint_3":  1.309,
        "t2_a2_joint_4":  0.0,
        "t2_a2_joint_5": -1.0471,
        "t2_a2_joint_6":  0.0,
    },
}


def _move_group_to_joints(action_client, node, group_name, joint_positions, label):
    import rclpy
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes

    goal = MoveGroup.Goal()
    goal.request.group_name              = group_name
    goal.request.num_planning_attempts   = 5
    goal.request.allowed_planning_time   = 15.0
    goal.request.max_velocity_scaling_factor     = 0.15  # 15 % speed — safe for testing
    goal.request.max_acceleration_scaling_factor = 0.15

    c = Constraints()
    for jname, jval in joint_positions.items():
        jc = JointConstraint()
        jc.joint_name     = jname
        jc.position       = jval
        jc.tolerance_above = 0.02
        jc.tolerance_below = 0.02
        jc.weight         = 1.0
        c.joint_constraints.append(jc)
    goal.request.goal_constraints.append(c)

    info(f"{label}: planning and executing to home position …")
    future = action_client.send_goal_async(goal)

    deadline = time.time() + 20.0
    while not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() > deadline:
            fail(f"{label}: goal send timed out")
            return False

    goal_handle = future.result()
    if not goal_handle.accepted:
        fail(f"{label}: goal REJECTED by MoveGroup")
        return False

    result_future = goal_handle.get_result_async()
    deadline = time.time() + 60.0
    while not result_future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() > deadline:
            fail(f"{label}: execution timed out (60 s)")
            return False

    result = result_future.result().result
    if result.error_code.val == MoveItErrorCodes.SUCCESS:
        ok(f"{label}: moved to home position successfully")
        return True
    else:
        fail(f"{label}: MoveIt error code {result.error_code.val}  "
             f"(1=SUCCESS, -1=FAILURE, -4=PLANNING_FAILED, -5=INVALID_MOTION_PLAN)")
        return False


def run_arms_test():
    header("STAGE 3 · Arm Test via MoveIt")
    info("Prerequisite: ros2 launch workcell_moveit_config my_workcell.launch.py use_sim_time:=false")
    info("NOTE: In workcell.urdf.xacro set use_fake_hardware=false and real robot IPs before testing real hardware")

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from moveit_msgs.action import MoveGroup
    except ImportError as e:
        fail(f"ROS2/MoveIt import failed: {e}  — source your workspace first")
        return

    rclpy.init()
    node = Node("hw_check_arm_tester")

    try:
        ac = ActionClient(node, MoveGroup, "move_action")
        info("Waiting for MoveGroup action server (15 s) …")
        if not ac.wait_for_server(timeout_sec=15.0):
            fail("MoveGroup /move_action not available — is my_workcell.launch.py running?")
            return

        ok("MoveGroup action server is available")

        for arm_name, joints in ARM_HOME_POSITIONS.items():
            section(f"{arm_name.upper()} home test")
            _move_group_to_joints(ac, node, arm_name, joints, arm_name.upper())
            time.sleep(1.0)

    finally:
        node.destroy_node()
        rclpy.shutdown()


# ──────────────────────────────────────────────────────────────────────────────
# Stage 4 – Gripper test
# ──────────────────────────────────────────────────────────────────────────────

GRIPPER_CONTROLLERS = [
    ("gripper_1_controller", "t1_a1_right_finger_bottom_joint", "Gripper 1 (Arm 1)"),
    ("gripper_2_controller", "t1_a2_right_finger_bottom_joint", "Gripper 2 (Arm 2)"),
    ("gripper_3_controller", "t2_a1_right_finger_bottom_joint", "Gripper 3 (Arm 3)"),
    ("gripper_4_controller", "t2_a2_right_finger_bottom_joint", "Gripper 4 (Arm 4)"),
]

GRIPPER_OPEN  = 0.0    # radians — fully open
GRIPPER_CLOSE = 0.78   # radians — fully closed (~45 deg for gen3_lite_2f)


def _send_gripper_goal(action_client, node, position: float, label: str) -> bool:
    import rclpy
    from control_msgs.action import GripperCommand
    from control_msgs.msg import GripperCommand as GripperCommandMsg

    goal = GripperCommand.Goal()
    goal.command.position    = position
    goal.command.max_effort  = 50.0

    future = action_client.send_goal_async(goal)
    deadline = time.time() + 15.0
    while not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() > deadline:
            fail(f"{label}: goal send timed out")
            return False

    goal_handle = future.result()
    if not goal_handle.accepted:
        fail(f"{label}: goal rejected")
        return False

    result_future = goal_handle.get_result_async()
    deadline = time.time() + 15.0
    while not result_future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() > deadline:
            fail(f"{label}: execution timed out")
            return False

    result = result_future.result().result
    if result.reached_goal:
        ok(f"{label}: reached target position {position:.2f} rad")
        return True
    else:
        # stalled is acceptable — means gripper hit an object
        if result.stalled:
            ok(f"{label}: stalled at {result.position:.3f} rad (gripped object)")
        else:
            warn(f"{label}: did not fully reach {position:.2f} rad — pos={result.position:.3f}")
        return True


def run_grippers_test():
    header("STAGE 4 · Gripper Test")
    info("Prerequisite: ros2 launch workcell_moveit_config my_workcell.launch.py use_sim_time:=false")

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from control_msgs.action import GripperCommand
    except ImportError as e:
        fail(f"ROS2/control_msgs import failed: {e}  — source your workspace first")
        return

    rclpy.init()
    node = Node("hw_check_gripper_tester")

    try:
        for ctrl_name, joint_name, label in GRIPPER_CONTROLLERS:
            section(label)
            action_topic = f"/{ctrl_name}/gripper_cmd"
            ac = ActionClient(node, GripperCommand, action_topic)

            info(f"Waiting for {action_topic} (5 s) …")
            if not ac.wait_for_server(timeout_sec=5.0):
                fail(f"{label}: action server {action_topic} not available")
                ac.destroy()
                continue

            ok(f"{label}: action server available")

            # Open
            info(f"{label}: opening …")
            _send_gripper_goal(ac, node, GRIPPER_OPEN, f"{label} open")
            time.sleep(1.5)

            # Close
            info(f"{label}: closing …")
            _send_gripper_goal(ac, node, GRIPPER_CLOSE, f"{label} close")
            time.sleep(1.5)

            # Re-open (leave gripper open at end)
            info(f"{label}: re-opening to safe state …")
            _send_gripper_goal(ac, node, GRIPPER_OPEN, f"{label} re-open")
            time.sleep(1.0)

            ac.destroy()

    finally:
        node.destroy_node()
        rclpy.shutdown()


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Moonshot Workcell hardware check",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--preflight",  action="store_true", help="Stage 1: ports and network check (no ROS)")
    p.add_argument("--tables",     action="store_true", help="Stage 2: table motor test")
    p.add_argument("--arms",       action="store_true", help="Stage 3: arm movement test via MoveIt")
    p.add_argument("--grippers",   action="store_true", help="Stage 4: gripper open/close test")
    p.add_argument("--all",        action="store_true", help="Run all stages")
    p.add_argument(
        "--arm-ips", nargs=4, metavar=("IP1", "IP2", "IP3", "IP4"),
        help="IP addresses for Arm1 Arm2 Arm3 Arm4 (used in --preflight ping check)",
    )
    return p.parse_args()


def main():
    args = parse_args()

    run_all = args.all or not any([args.preflight, args.tables, args.arms, args.grippers])

    if run_all or args.preflight:
        run_preflight(args.arm_ips or [])

    if run_all or args.tables:
        run_tables_test()

    if run_all or args.arms:
        run_arms_test()

    if run_all or args.grippers:
        run_grippers_test()

    summary()


if __name__ == "__main__":
    main()
