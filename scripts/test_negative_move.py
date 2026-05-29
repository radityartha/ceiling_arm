#!/usr/bin/env python3
"""Gentle test: Table 1 only — move -50mm linear then rotate +10 degrees."""
import rclpy
from rclpy.node import Node
from moving_table_interfaces.srv import MovingTable
import time

def send_command(node, client, table_id, distance_mm, angle_deg, label):
    req = MovingTable.Request()
    req.table_id       = table_id
    req.distance_mm    = float(distance_mm)
    req.angle_deg      = float(angle_deg)
    req.linear_speed   = 3000    # slow speed
    req.rotate_speed   = 1000    # slow rotation speed
    req.operation_type = 1

    print(f"\n→ [{label}]  dist={distance_mm:+.0f}mm  rot={angle_deg:+.0f}°")
    future = client.call_async(req)
    deadline = time.time() + 10.0
    while not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() > deadline:
            print(f"  ❌ service call timed out")
            return False

    res = future.result()
    status = "✅" if res.success else "❌"
    print(f"  {status} {res.message.strip()}")
    return res.success

def main():
    rclpy.init()
    node = Node("test_negative_move")

    client = node.create_client(MovingTable, "move_dual_table")
    print("Waiting for move_dual_table service...")
    if not client.wait_for_service(timeout_sec=8.0):
        print("❌ Service not available — is dual_table_controller running?")
        rclpy.shutdown()
        return
    print("✅ Service found")

    # Step 1: move table1 linear -50mm (no rotation)
    send_command(node, client, "table1", -50, 0, "Table1 linear -50mm")
    print("  Waiting 35s for motor to finish (watch Terminal 1 for position logs)...")
    time.sleep(35)

    # Step 2: rotate table1 +10 degrees (no linear)
    send_command(node, client, "table1", 0, 10, "Table1 rotate +10°")
    print("  Waiting 35s for motor to finish...")
    time.sleep(35)

    print("\n✅ Test sequence done.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
