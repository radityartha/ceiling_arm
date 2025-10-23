#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moving_table_control.srv import MovingTable
from ros2_ws.src.moving_table_control.moving_table.moving_table import (
    MovingTableController,
)
from moving_table.oml_mrtu import ModbusAZ, commPC
import threading


class DualTableController(Node):
    def __init__(self):
        super().__init__("dual_table_controller")

        # ------------------- Initialize Table 1 -------------------
        comm_table1 = commPC(argPort="/dev/ttyUSB0", argBaudrate=115200)
        motor1_t1 = ModbusAZ(comm=comm_table1, serverAddress=1)
        motor2_t1 = ModbusAZ(comm=comm_table1, serverAddress=2)
        motor3_t1 = ModbusAZ(comm=comm_table1, serverAddress=3)
        self.table1 = MovingTableController(
            motor1_t1, motor2_t1, motor3_t1, logger=self.get_logger()
        )

        # ------------------- Initialize Table 2 -------------------
        comm_table2 = commPC(argPort="/dev/ttyUSB1", argBaudrate=115200)
        motor1_t2 = ModbusAZ(comm=comm_table2, serverAddress=1)
        motor2_t2 = ModbusAZ(comm=comm_table2, serverAddress=2)
        motor3_t2 = ModbusAZ(comm=comm_table2, serverAddress=3)
        self.table2 = MovingTableController(
            motor1_t2, motor2_t2, motor3_t2, logger=self.get_logger()
        )

        # ------------------- Configure all motors -------------------
        for table in [self.table1, self.table2]:
            for m in [table.motor1, table.motor2, table.motor3]:
                table.configure_motor(m)

        # ------------------- Create ROS 2 Service -------------------
        self.srv = self.create_service(
            MoveTable, "move_dual_table", self.move_dual_table_callback
        )
        self.get_logger().info("Dual table service ready.")

    def move_dual_table_callback(self, request, response):
        self.get_logger().info(
            f"Received move request: "
            f"Table1 -> {request.distance_mm} mm, {request.angle_deg} deg | "
            f"Table2 -> {request.distance_mm} mm, {request.angle_deg} deg"
        )

        # --- Define threads for simultaneous movement ---
        def move_table1():
            self.table1.go_to_table(
                distance_mm=request.distance_mm,
                angle_degrees=request.angle_deg,
                linear_speed=request.linear_speed,
                rotate_speed=request.rotate_speed,
                operation_type=request.operation_type,
            )

        def move_table2():
            self.table2.go_to_table(
                distance_mm=request.distance_mm,
                angle_degrees=request.angle_deg,
                linear_speed=request.linear_speed,
                rotate_speed=request.rotate_speed,
                operation_type=request.operation_type,
            )

        # --- Start both threads ---
        thread1 = threading.Thread(target=move_table1)
        thread2 = threading.Thread(target=move_table2)
        thread1.start()
        thread2.start()

        # Wait until both tables finish
        thread1.join()
        thread2.join()

        response.success = True
        response.message = "✅ Both tables have completed their movements."
        return response


def main(args=None):
    print("✅ DualTableController started")
    rclpy.init(args=args)
    node = DualTableController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down dual_table_controller...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
