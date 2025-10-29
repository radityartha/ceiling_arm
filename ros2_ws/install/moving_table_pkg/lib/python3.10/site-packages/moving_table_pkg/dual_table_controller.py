import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from moving_table_interfaces.srv import MovingTable
from moving_table.moving_table import MovingTableController
from moving_table.oml_mrtu import ModbusAZ, commPC
import serial  # Added for exception handling
import threading  # Import threading for background tasks
import time  # Import time for joining threads during shutdown


class DualTableController(Node):
    def __init__(self):
        super().__init__("dual_table_controller")

        # --- Declare ROS 2 Parameters ---
        # (Parameter declarations remain the same)
        self.declare_parameter("table1.port", "/dev/ttyUSB0")
        self.declare_parameter("table1.baudrate", 115200)
        self.declare_parameter("table1.motor1_addr", 1)
        self.declare_parameter("table1.motor2_addr", 2)
        self.declare_parameter("table1.motor3_addr", 3)

        self.declare_parameter("table2.port", "/dev/ttyUSB1")
        self.declare_parameter("table2.baudrate", 115200)
        self.declare_parameter("table2.motor1_addr", 1)
        self.declare_parameter("table2.motor2_addr", 2)
        self.declare_parameter("table2.motor3_addr", 3)

        self.declare_parameter("motor_config.acceleration", 1000)
        self.declare_parameter("motor_config.speed", 100000)
        self.declare_parameter("motor_config.current", 1000)

        self.add_on_set_parameters_callback(self.parameters_callback)

        # Initialize attributes
        self.table1 = None
        self.table2 = None
        self.comm_table1 = None
        self.comm_table2 = None

        # --- Thread management ---
        self.table1_thread = None
        self.table2_thread = None
        self._shutdown_event = threading.Event()  # For signaling threads to stop

        # --- Initialize Tables using Parameters ---
        self._initialize_tables()

        # ------------------- Create ROS 2 Service -------------------
        self.srv = self.create_service(
            MovingTable, "move_dual_table", self.move_dual_table_callback
        )
        self.get_logger().info("Dual table service ready.")

    def parameters_callback(self, params):
        # (parameters_callback remains the same)
        for param in params:
            self.get_logger().info(f"Parameter '{param.name}' changed to {param.value}")
        # Consider re-initializing carefully, manage threads and ports
        return SetParametersResult(successful=True)

    def _initialize_tables(self):
        # (_initialize_tables remains largely the same, ensure ports closed)
        self.table1 = None
        self.table2 = None
        self._close_ports()  # Ensure ports are closed before potentially reopening

        # --- Initialize Table 1 ---
        try:
            port1 = self.get_parameter("table1.port").get_parameter_value().string_value
            baud1 = (
                self.get_parameter("table1.baudrate")
                .get_parameter_value()
                .integer_value
            )
            m1_addr1 = (
                self.get_parameter("table1.motor1_addr")
                .get_parameter_value()
                .integer_value
            )
            m2_addr1 = (
                self.get_parameter("table1.motor2_addr")
                .get_parameter_value()
                .integer_value
            )
            m3_addr1 = (
                self.get_parameter("table1.motor3_addr")
                .get_parameter_value()
                .integer_value
            )

            self.get_logger().info(
                f"Attempting to connect Table 1: Port={port1}, Baud={baud1}"
            )
            self.comm_table1 = commPC(argPort=port1, argBaudrate=baud1)

            motor1_t1 = ModbusAZ(comm=self.comm_table1, serverAddress=m1_addr1)
            motor2_t1 = ModbusAZ(comm=self.comm_table1, serverAddress=m2_addr1)
            motor3_t1 = ModbusAZ(comm=self.comm_table1, serverAddress=m3_addr1)
            self.table1 = MovingTableController(
                motor1_t1, motor2_t1, motor3_t1, logger=self.get_logger()
            )
            self.get_logger().info("Table 1 initialized.")
            self._configure_table_motors(self.table1, "Table 1")

        except (serial.SerialException, FileNotFoundError, Exception) as e:
            self.table1 = None
            self.get_logger().error(f"Failed to initialize Table 1: {e}", exc_info=True)
            if self.comm_table1:
                self.comm_table1.closePort()
                self.comm_table1 = None

        # --- Initialize Table 2 ---
        try:
            port2 = self.get_parameter("table2.port").get_parameter_value().string_value
            baud2 = (
                self.get_parameter("table2.baudrate")
                .get_parameter_value()
                .integer_value
            )
            m1_addr2 = (
                self.get_parameter("table2.motor1_addr")
                .get_parameter_value()
                .integer_value
            )
            m2_addr2 = (
                self.get_parameter("table2.motor2_addr")
                .get_parameter_value()
                .integer_value
            )
            m3_addr2 = (
                self.get_parameter("table2.motor3_addr")
                .get_parameter_value()
                .integer_value
            )

            self.get_logger().info(
                f"Attempting to connect Table 2: Port={port2}, Baud={baud2}"
            )
            self.comm_table2 = commPC(argPort=port2, argBaudrate=baud2)

            motor1_t2 = ModbusAZ(comm=self.comm_table2, serverAddress=m1_addr2)
            motor2_t2 = ModbusAZ(comm=self.comm_table2, serverAddress=m2_addr2)
            motor3_t2 = ModbusAZ(comm=self.comm_table2, serverAddress=m3_addr2)
            self.table2 = MovingTableController(
                motor1_t2, motor2_t2, motor3_t2, logger=self.get_logger()
            )
            self.get_logger().info("Table 2 initialized.")
            self._configure_table_motors(self.table2, "Table 2")

        except (serial.SerialException, FileNotFoundError, Exception) as e:
            self.table2 = None
            self.get_logger().error(f"Failed to initialize Table 2: {e}", exc_info=True)
            if self.comm_table2:
                self.comm_table2.closePort()
                self.comm_table2 = None

    def _configure_table_motors(self, table, table_name):
        # (_configure_table_motors remains the same)
        if not table:
            return

        acc = (
            self.get_parameter("motor_config.acceleration")
            .get_parameter_value()
            .integer_value
        )
        speed = (
            self.get_parameter("motor_config.speed").get_parameter_value().integer_value
        )
        current = (
            self.get_parameter("motor_config.current")
            .get_parameter_value()
            .integer_value
        )

        for m in [table.motor1, table.motor2, table.motor3]:
            try:
                self.get_logger().info(
                    f"Configuring {table_name} motor {m.serverAddress} (Acc:{acc}, Speed:{speed}, Current:{current})"
                )
                if not table.configure_motor(m, acc=acc, speed=speed, current=current):
                    self.get_logger().warning(
                        f"Configuration returned false for {table_name} motor {m.serverAddress}"
                    )
            except Exception as e:
                self.get_logger().error(
                    f"Failed to configure {table_name} motor {m.serverAddress}: {e}",
                    exc_info=True,
                )

    def _close_ports(self):
        # (_close_ports remains the same)
        if self.comm_table1:
            try:
                self.comm_table1.closePort()
                self.get_logger().info("Closed port for Table 1.")
            except Exception as e:
                self.get_logger().error(f"Error closing Table 1 port: {e}")
            finally:
                self.comm_table1 = None
        if self.comm_table2:
            try:
                self.comm_table2.closePort()
                self.get_logger().info("Closed port for Table 2.")
            except Exception as e:
                self.get_logger().error(f"Error closing Table 2 port: {e}")
            finally:
                self.comm_table2 = None

    def _background_move_task(self, table_id, table_controller, request):
        """Target function for the background movement thread."""
        self.get_logger().info(f"Background thread started for {table_id}")
        success = False
        try:
            # The actual blocking call happens here, in the background thread
            success = table_controller.go_to_table(
                distance_mm=request.distance_mm,
                angle_degrees=request.angle_deg,
                linear_speed=request.linear_speed,
                rotate_speed=request.rotate_speed,
                operation_type=request.operation_type,
            )
            if success:
                self.get_logger().info(
                    f"Background thread: {table_id} movement finished successfully."
                )
            else:
                self.get_logger().error(
                    f"Background thread: {table_id} movement failed (go_to_table returned False)."
                )

        except Exception as e:
            self.get_logger().error(
                f"Background thread: Exception during {table_id} movement: {e}",
                exc_info=True,
            )
        finally:
            # Clean up the thread reference once done
            if table_id == "table1":
                self.table1_thread = None
            elif table_id == "table2":
                self.table2_thread = None
            self.get_logger().info(f"Background thread finished for {table_id}")

    def move_dual_table_callback(self, request, response):
        target_table_id = request.table_id
        self.get_logger().info(
            f"Received move request for '{target_table_id}': "
            # ... (rest of log message) ...
        )

        target_table = None
        active_thread = None
        if target_table_id == "table1":
            target_table = self.table1
            active_thread = self.table1_thread
        elif target_table_id == "table2":
            target_table = self.table2
            active_thread = self.table2_thread
        else:
            response.success = False
            response.message = f"Error: Invalid table_id '{target_table_id}'. Use 'table1' or 'table2'."
            self.get_logger().error(response.message)
            return response

        if target_table is None:
            response.success = False
            response.message = (
                f"Error: Table '{target_table_id}' is not initialized or available."
            )
            self.get_logger().error(response.message)
            return response

        # Check if the table is already moving
        if active_thread is not None and active_thread.is_alive():
            response.success = False
            response.message = (
                f"Error: Table '{target_table_id}' is already moving. Please wait."
            )
            self.get_logger().warning(response.message)
            return response

        # --- Start the movement in a background thread ---
        try:
            # Create and start the thread
            new_thread = threading.Thread(
                target=self._background_move_task,
                args=(target_table_id, target_table, request),
                daemon=True,  # Allows program to exit even if thread is running (optional)
            )

            # Store reference to the active thread
            if target_table_id == "table1":
                self.table1_thread = new_thread
            elif target_table_id == "table2":
                self.table2_thread = new_thread

            new_thread.start()

            # --- Return success immediately ---
            response.success = True
            # Update message to indicate command accepted, not completed
            response.message = f"✅ Movement command accepted for Table '{target_table_id}'. Executing in background."
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = (
                f"Error starting background thread for '{target_table_id}': {e}"
            )
            self.get_logger().error(response.message, exc_info=True)

        return response  # Return immediately, don't wait for thread

    def on_shutdown(self):
        """Called when the node is shutting down."""
        self.get_logger().info("Shutting down dual_table_controller...")
        self._shutdown_event.set()  # Signal background threads to stop if possible

        # Wait briefly for threads to potentially finish current step
        if self.table1_thread and self.table1_thread.is_alive():
            self.get_logger().info("Waiting for Table 1 thread to join...")
            self.table1_thread.join(timeout=1.0)  # Short timeout
        if self.table2_thread and self.table2_thread.is_alive():
            self.get_logger().info("Waiting for Table 2 thread to join...")
            self.table2_thread.join(timeout=1.0)  # Short timeout

        self._close_ports()  # Close ports after attempting to stop threads


def main(args=None):
    # (main function remains the same, using MultiThreadedExecutor)
    print("✅ DualTableController started")
    rclpy.init(args=args)
    node = DualTableController()
    # Use MultiThreadedExecutor to handle service calls without blocking main thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().fatal(
            f"Unhandled exception in executor spin: {e}", exc_info=True
        )
    finally:
        node.on_shutdown()
        executor.shutdown()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
