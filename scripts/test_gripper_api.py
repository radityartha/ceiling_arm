#!/usr/bin/env python3
"""Direct Kinova Kortex API gripper test — bypasses ROS entirely.

Connects to one arm, reads the gripper position, commands close then open, and
reports the measured position at each step. This tells us whether the gripper
hardware actuates at all (independent of ros2_control / MoveIt).

The ROS driver must NOT be running (stop Terminal A first) so this can hold the
arm's session.

Usage (from repo root):
  /tmp/kortex_venv/bin/python scripts/test_gripper_api.py 192.168.2.10   # arm 4
"""
import sys
import time

from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2, Session_pb2


def read_gripper(base):
    req = Base_pb2.GripperRequest()
    req.mode = Base_pb2.GRIPPER_POSITION
    m = base.GetMeasuredGripperMovement(req)
    return m.finger[0].value if len(m.finger) else None


def command_gripper(base, value):
    """value: 0.0 = fully open, 1.0 = fully closed (normalized)."""
    cmd = Base_pb2.GripperCommand()
    cmd.mode = Base_pb2.GRIPPER_POSITION
    finger = cmd.gripper.finger.add()
    finger.finger_identifier = 1
    finger.value = value
    base.SendGripperCommand(cmd)


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.2.10"
    print(f"Connecting to arm at {ip}:10000 ...")

    transport = TCPTransport()
    router = RouterClient(transport, lambda ex: print(f"[router] {ex}"))
    transport.connect(ip, 10000)

    session_info = Session_pb2.CreateSessionInfo()
    session_info.username = "admin"
    session_info.password = "admin"
    session_info.session_inactivity_timeout = 60000
    session_info.connection_inactivity_timeout = 2000

    session_manager = SessionManager(router)
    session_manager.CreateSession(session_info)
    base = BaseClient(router)

    try:
        print(f"  gripper position (start): {read_gripper(base)}")

        print("  commanding CLOSE (0.7) ...")
        command_gripper(base, 0.7)
        time.sleep(2.5)
        print(f"  gripper position (after close): {read_gripper(base)}")

        print("  commanding OPEN (0.0) ...")
        command_gripper(base, 0.0)
        time.sleep(2.5)
        print(f"  gripper position (after open): {read_gripper(base)}")

        print("\nIf the values changed and the gripper physically moved, the "
              "hardware is fine and the problem is in ros2_control.\n"
              "If the values stayed ~0 and nothing moved, the gripper is not "
              "actuating at the arm/firmware level.")
    finally:
        session_manager.CloseSession()
        transport.disconnect()


if __name__ == "__main__":
    main()
