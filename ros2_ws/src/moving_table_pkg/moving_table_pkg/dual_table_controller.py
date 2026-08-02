#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from moving_table_interfaces.srv import MovingTable
from moving_table.moving_table import MovingTableController
from moving_table.oml_mrtu import ModbusAZ, commPC
import serial
import threading
import time
import traceback  # Import traceback for logging
from sensor_msgs.msg import JointState
from rclpy.clock import Clock
from rclpy.qos import QoSProfile

# Constants
PULSES_PER_MM = 12000 / (40.0 * 3.14159265359)  # Pulses per mm (for fake hardware)
PULSES_PER_RAD = (9000 / 90) * (180 / 3.14159265359)  # Pulses per degree * deg per rad

# operation_type values for jog (continuous drive)
JOG_DIRECTIONS = {
    10: 'forward',
    11: 'backward',
    12: 'rotate_cw',
    13: 'rotate_ccw',
}

# How often to retry a table whose _initialize_table*() failed at startup
# (e.g. motor driver / RS-485 wasn't powered yet). Retries run in a
# background thread so a slow Modbus timeout (seen up to ~12s per table)
# never blocks joint_state publishing or the move service.
RETRY_INTERVAL_S = 15.0

# operation_type for home/preset
OP_PRESET_HOME = 99
OP_GOTO_HOME   = 98   # move to absolute encoder 0 (server-side, no client position needed)
OP_GOTO_ABS    = 97   # move to absolute position (distance_mm, angle_deg) read server-side


class DualTableController(Node):
    def __init__(self):
        super().__init__("dual_table_controller")

        # --- Declare ROS 2 Parameters ---
        self.declare_parameter(
            "use_fake_hardware", False
        )  # <-- NEW: Fake hardware flag
        self.declare_parameter("table1.port", "/dev/ttyUSB0")
        self.declare_parameter("table1.baudrate", 115200)
        self.declare_parameter("table1.motor1_addr", 1)
        self.declare_parameter("table1.motor2_addr", 2)
        self.declare_parameter("table1.motor3_addr", 3)
        self.declare_parameter("table1.linear_joint_name", "t1_linear_joint")
        self.declare_parameter("table1.rotation_joint_name", "t1_rotation_joint")

        self.declare_parameter("table2.port", "/dev/ttyUSB1")
        self.declare_parameter("table2.baudrate", 115200)
        self.declare_parameter("table2.motor1_addr", 1)
        self.declare_parameter("table2.motor2_addr", 2)
        self.declare_parameter("table2.motor3_addr", 3)
        self.declare_parameter("table2.linear_joint_name", "t2_linear_joint")
        self.declare_parameter("table2.rotation_joint_name", "t2_rotation_joint")

        self.declare_parameter("motor_config.acceleration", 1000)
        self.declare_parameter("motor_config.speed", 100000)
        self.declare_parameter("motor_config.current", 1000)
        self.declare_parameter("publish_rate", 10.0)

        # --- ros2_control bridge (topic_based_ros2_control real-hardware path) ---
        # Off by default: this lets MoveIt/reach_fusion drive the real table via
        # JointTrajectory goals instead of only the move_dual_table service. A
        # FollowJointTrajectory goal streams the target at controller_manager's
        # update rate (~100Hz); Modbus AZ motors take one absolute-move command
        # and ramp to it on their own, so we debounce to "target moved more than
        # tol" and only issue a new go_to_absolute when the previous one (if any)
        # has finished -- never stream raw setpoints onto the RS-485 bus.
        self.declare_parameter("bridge.enable", False)
        self.declare_parameter("bridge.linear_speed", 3000)
        self.declare_parameter("bridge.rotate_speed", 1000)
        self.declare_parameter("bridge.pos_tol_mm", 1.0)
        self.declare_parameter("bridge.rot_tol_deg", 0.3)

        self.add_on_set_parameters_callback(self.parameters_callback)

        # Initialize attributes
        self.table1 = None
        self.table2 = None
        self.comm_table1 = None
        self.comm_table2 = None
        self.use_fake_hardware = self.get_parameter("use_fake_hardware").value

        # Store joint names
        self.t1_linear_joint = self.get_parameter("table1.linear_joint_name").value
        self.t1_rotation_joint = self.get_parameter("table1.rotation_joint_name").value
        self.t2_linear_joint = self.get_parameter("table2.linear_joint_name").value
        self.t2_rotation_joint = self.get_parameter("table2.rotation_joint_name").value

        # Store current joint positions (initialize to 0 or URDF default)
        self.joint_positions = {
            self.t1_linear_joint: 0.0,
            self.t1_rotation_joint: 0.0,  # FIX: Set to 0.0 to match GUI
            self.t2_linear_joint: 0.0,
            self.t2_rotation_joint: 0.0,  # FIX: Set to 0.0 to match GUI
        }
        self.joint_state_lock = threading.Lock()

        # --- Thread management ---
        self.table1_thread = None
        self.table2_thread = None
        self._shutdown_event  = threading.Event()
        self.table1_stop_event = threading.Event()
        self.table2_stop_event = threading.Event()

        # --- Initialize Tables using Parameters ---
        self._retrying = False
        self._initialize_tables()
        # Seed joint positions from hardware immediately so the first published
        # joint_states reflect the real encoder position, not the 0.0 default.
        self._update_joint_positions()

        # Retry whichever table(s) failed to init above (e.g. motor driver
        # power / RS-485 wasn't ready yet) instead of staying dead forever.
        if not self.use_fake_hardware and (self.table1 is None or self.table2 is None):
            self.retry_timer = self.create_timer(
                RETRY_INTERVAL_S, self._retry_failed_tables_timer_cb)
        else:
            self.retry_timer = None

        # ------------------- Create ROS 2 Service -------------------
        self.srv = self.create_service(
            MovingTable, "move_dual_table", self.move_dual_table_callback
        )
        self.get_logger().info("Dual table service ready.")

        # ------------------- Create Joint State Publisher ---------------
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", qos_profile)
        # Dedicated feed for topic_based_ros2_control -- NOT "joint_states":
        # joint_state_broadcaster also publishes /joint_states once these
        # joints are wired into ros2_control, and two publishers on the same
        # topic race (~5:1, whichever fires last wins a given HUD read).
        self.hw_state_pub = self.create_publisher(JointState, "table_hw_states", qos_profile)
        self.publish_timer = self.create_timer(
            1.0 / self.get_parameter("publish_rate").value, self.publish_joint_states
        )
        self.get_logger().info(
            f"Publishing joint states at {self.get_parameter('publish_rate').value} Hz."
        )

        # ------------------- ros2_control bridge subscriber ---------------
        self._bridge_target_mm = {"table1": None, "table2": None}   # last dispatched, mm/deg
        self._bridge_target_deg = {"table1": None, "table2": None}
        self.create_subscription(JointState, "table_hw_commands", self._on_hw_command, qos_profile)

    def parameters_callback(self, params):
        result = SetParametersResult(successful=True)
        for param in params:
            self.get_logger().info(f"Parameter '{param.name}' changed to {param.value}")
            if param.name == "use_fake_hardware":
                self.use_fake_hardware = param.value
                self.get_logger().info(
                    f"Setting fake hardware to: {self.use_fake_hardware}"
                )
        return result

    def _initialize_tables(self):
        self._close_ports()
        self.table1 = None
        self.table2 = None

        if self.use_fake_hardware:
            self.get_logger().warn("Using FAKE HARDWARE for tables.")
            self.table1 = "fake"  # Set fake flag
            self.table2 = "fake"  # Set fake flag
            return  # Skip all real hardware initialization

        self._initialize_table1()
        self._initialize_table2()

    def _retry_failed_tables_timer_cb(self):
        """Timer callback: only KICKS OFF a retry in a background thread (must
        stay non-blocking -- publish_joint_states shares this node's default
        callback group, and a Modbus timeout here has taken ~12s per table)."""
        if self.table1 is not None and self.table2 is not None:
            self.retry_timer.cancel()
            return
        if self._retrying:
            return  # previous retry attempt still in flight
        self._retrying = True
        threading.Thread(target=self._retry_failed_tables, daemon=True).start()

    def _retry_failed_tables(self):
        # Only touches a table that is currently None -- never re-inits a
        # table that's already up, so this can't yank the port out from
        # under an in-flight move.
        try:
            if self.table1 is None:
                self.get_logger().info("Retrying Table 1 initialization...")
                self._initialize_table1()
            if self.table2 is None:
                self.get_logger().info("Retrying Table 2 initialization...")
                self._initialize_table2()
        finally:
            self._retrying = False

    def _initialize_table1(self):
        # --- Initialize Table 1 (Real Hardware) ---
        try:
            port1 = self.get_parameter("table1.port").value
            baud1 = self.get_parameter("table1.baudrate").value
            m1_addr1 = self.get_parameter("table1.motor1_addr").value
            m2_addr1 = self.get_parameter("table1.motor2_addr").value
            m3_addr1 = self.get_parameter("table1.motor3_addr").value
            self.get_logger().info(
                f"Attempting to connect Table 1: Port={port1}, Baud={baud1}"
            )
            self.comm_table1 = commPC(argPort=port1, argBaudrate=baud1)
            motor1_lin1 = ModbusAZ(comm=self.comm_table1, serverAddress=m1_addr1)
            motor2_lin1 = ModbusAZ(comm=self.comm_table1, serverAddress=m2_addr1)
            motor3_rot1 = ModbusAZ(comm=self.comm_table1, serverAddress=m3_addr1)
            self.table1 = MovingTableController(
                motor1_lin1, motor2_lin1, motor3_rot1, logger=self.get_logger(), timeout=120.0
            )
            self.get_logger().info("Table 1 initialized.")
            self._configure_table_motors(self.table1, "Table 1")
        except (serial.SerialException, FileNotFoundError, Exception) as e:
            self.table1 = None
            # --- FIX: Removed the buggy 'exc_info=True' ---
            self.get_logger().error(
                f"Failed to initialize Table 1: {e}\n{traceback.format_exc()}"
            )
            if self.comm_table1:
                self.comm_table1.client.close()
                self.comm_table1 = None

    def _initialize_table2(self):
        # --- Initialize Table 2 (Real Hardware) ---
        try:
            port2 = self.get_parameter("table2.port").value
            baud2 = self.get_parameter("table2.baudrate").value
            m1_addr2 = self.get_parameter("table2.motor1_addr").value
            m2_addr2 = self.get_parameter("table2.motor2_addr").value
            m3_addr2 = self.get_parameter("table2.motor3_addr").value
            self.get_logger().info(
                f"Attempting to connect Table 2: Port={port2}, Baud={baud2}"
            )
            self.comm_table2 = commPC(argPort=port2, argBaudrate=baud2)
            motor1_lin2 = ModbusAZ(comm=self.comm_table2, serverAddress=m1_addr2)
            motor2_lin2 = ModbusAZ(comm=self.comm_table2, serverAddress=m2_addr2)
            motor3_rot2 = ModbusAZ(comm=self.comm_table2, serverAddress=m3_addr2)
            self.table2 = MovingTableController(
                motor1_lin2, motor2_lin2, motor3_rot2, logger=self.get_logger(), timeout=120.0
            )
            self.get_logger().info("Table 2 initialized.")
            self._configure_table_motors(self.table2, "Table 2")
        except (serial.SerialException, FileNotFoundError, Exception) as e:
            self.table2 = None
            # --- FIX: Removed the buggy 'exc_info=True' ---
            self.get_logger().error(
                f"Failed to initialize Table 2: {e}\n{traceback.format_exc()}"
            )
            if self.comm_table2:
                self.comm_table2.client.close()
                self.comm_table2 = None

    def _configure_table_motors(self, table, table_name):
        if not table:
            return
        acc = self.get_parameter("motor_config.acceleration").value
        speed = self.get_parameter("motor_config.speed").value
        current = self.get_parameter("motor_config.current").value
        for m in [table.motor1, table.motor2, table.motor3]:
            if not m:
                continue
            try:
                self.get_logger().info(
                    f"Configuring {table_name} motor {m.serverAddress}..."
                )
                if not table.configure_motor(m, acc=acc, speed=speed, current=current):
                    self.get_logger().warning(
                        f"Configuration returned false for {table_name} motor {m.serverAddress}"
                    )
            except Exception as e:
                # --- FIX: Removed the buggy 'exc_info=True' ---
                self.get_logger().error(
                    f"Failed to configure {table_name} motor {m.serverAddress}: {e}\n{traceback.format_exc()}"
                )

    def _close_ports(self):
        if self.comm_table1:
            try:
                self.comm_table1.client.close()
                self.get_logger().info("Closed port for Table 1.")
            except Exception as e:
                self.get_logger().error(f"Error closing Table 1 port: {e}")
            finally:
                self.comm_table1 = None
        if self.comm_table2:
            try:
                self.comm_table2.client.close()
                self.get_logger().info("Closed port for Table 2.")
            except Exception as e:
                self.get_logger().error(f"Error closing Table 2 port: {e}")
            finally:
                self.comm_table2 = None

    def _update_joint_positions(self):
        if self.use_fake_hardware:
            return
        with self.joint_state_lock:
            if self.table1 and self.table1.motor1 and self.table1.motor3:
                try:
                    lin_pos_pulses = self.table1.motor1.readPosition()
                    if (
                        lin_pos_pulses
                        and isinstance(lin_pos_pulses, (list, tuple))
                        and len(lin_pos_pulses) >= 2
                    ):
                        self.joint_positions[self.t1_linear_joint] = (
                            float(lin_pos_pulses[1]) / PULSES_PER_MM / 1000.0
                        )
                    else:
                        self.get_logger().warning(
                            f"Could not read valid linear position for Table 1."
                        )
                    rot_pos_pulses = self.table1.motor3.readPosition()
                    if (
                        rot_pos_pulses
                        and isinstance(rot_pos_pulses, (list, tuple))
                        and len(rot_pos_pulses) >= 2
                    ):
                        self.joint_positions[self.t1_rotation_joint] = (
                            float(rot_pos_pulses[1]) / PULSES_PER_RAD
                        )
                    else:
                        self.get_logger().warning(
                            f"Could not read valid rotation position for Table 1."
                        )
                except Exception as e:
                    self.get_logger().error(f"Error reading positions for Table 1: {e}")
            if self.table2 and self.table2.motor1 and self.table2.motor3:
                try:
                    lin_pos_pulses = self.table2.motor1.readPosition()
                    if (
                        lin_pos_pulses
                        and isinstance(lin_pos_pulses, (list, tuple))
                        and len(lin_pos_pulses) >= 2
                    ):
                        self.joint_positions[self.t2_linear_joint] = (
                            float(lin_pos_pulses[1]) / PULSES_PER_MM / 1000.0
                        )
                    else:
                        self.get_logger().warning(
                            f"Could not read valid linear position for Table 2."
                        )
                    rot_pos_pulses = self.table2.motor3.readPosition()
                    if (
                        rot_pos_pulses
                        and isinstance(rot_pos_pulses, (list, tuple))
                        and len(rot_pos_pulses) >= 2
                    ):
                        self.joint_positions[self.t2_rotation_joint] = (
                            float(rot_pos_pulses[1]) / PULSES_PER_RAD
                        )
                    else:
                        self.get_logger().warning(
                            f"Could not read valid rotation position for Table 2."
                        )
                except Exception as e:
                    self.get_logger().error(f"Error reading positions for Table 2: {e}")

    def publish_joint_states(self):
        self._update_joint_positions()
        msg = JointState()
        msg.header.stamp = Clock().now().to_msg()
        with self.joint_state_lock:
            msg.name = list(self.joint_positions.keys())
            msg.position = list(self.joint_positions.values())
        self.joint_pub.publish(msg)
        hw_msg = JointState()
        hw_msg.header.stamp = msg.header.stamp
        hw_msg.name, hw_msg.position = msg.name, msg.position
        self.hw_state_pub.publish(hw_msg)

    def _on_hw_command(self, msg):
        """topic_based_ros2_control command callback (real-hardware bridge).

        msg carries the FollowJointTrajectory controller's per-cycle setpoint
        for ALL claimed joints (arms included, since gantry_N_with_arm_controller
        is one 14-joint controller) -- only look at the 2 table joints per
        table, and only act if bridge.enable is on."""
        if not self.get_parameter("bridge.enable").value:
            return
        pos = dict(zip(msg.name, msg.position))
        lin_speed = int(self.get_parameter("bridge.linear_speed").value)
        rot_speed = int(self.get_parameter("bridge.rotate_speed").value)
        pos_tol = float(self.get_parameter("bridge.pos_tol_mm").value)
        rot_tol = float(self.get_parameter("bridge.rot_tol_deg").value)
        for table_id, lin_j, rot_j, table, thread_attr in (
            ("table1", self.t1_linear_joint, self.t1_rotation_joint, self.table1, "table1_thread"),
            ("table2", self.t2_linear_joint, self.t2_rotation_joint, self.table2, "table2_thread"),
        ):
            if lin_j not in pos or rot_j not in pos or table is None:
                continue
            target_mm = pos[lin_j] * 1000.0
            target_deg = pos[rot_j] * 180.0 / 3.14159265359
            last_mm, last_deg = self._bridge_target_mm[table_id], self._bridge_target_deg[table_id]
            if (last_mm is not None
                    and abs(target_mm - last_mm) < pos_tol
                    and abs(target_deg - last_deg) < rot_tol):
                continue   # same target already dispatched (or within noise) -- skip
            active_thread = getattr(self, thread_attr)
            if active_thread is not None and active_thread.is_alive():
                continue   # a move is already in flight -- pick this target up once it finishes
            self._bridge_target_mm[table_id] = target_mm
            self._bridge_target_deg[table_id] = target_deg
            req = MovingTable.Request()
            req.table_id = table_id
            req.distance_mm = target_mm
            req.angle_deg = target_deg
            req.linear_speed = lin_speed
            req.rotate_speed = rot_speed
            req.operation_type = OP_GOTO_ABS
            stop_event = self.table1_stop_event if table_id == "table1" else self.table2_stop_event
            stop_event.clear()
            self.get_logger().info(
                f"bridge: {table_id} -> distance_mm={target_mm:.1f} angle_deg={target_deg:.1f}")
            new_thread = threading.Thread(
                target=self._background_move_task,
                args=(table_id, table, req, stop_event), daemon=True)
            setattr(self, thread_attr, new_thread)
            new_thread.start()

    def _background_move_task(self, table_id, table_controller, request, stop_event):
        self.get_logger().info(f"Background thread started for {table_id}")
        success = False
        try:
            if self.use_fake_hardware and table_controller == "fake":
                self.get_logger().info(f"SIMULATING move for {table_id}...")
                # For absolute moves distance_mm/angle_deg are the target; for
                # relative moves they are a delta added to the current fake position.
                with self.joint_state_lock:
                    lin_j = self.t1_linear_joint if table_id == "table1" else self.t2_linear_joint
                    rot_j = self.t1_rotation_joint if table_id == "table1" else self.t2_rotation_joint
                    if request.operation_type == OP_GOTO_ABS:
                        target_lin = request.distance_mm / 1000.0
                        target_rot = request.angle_deg * 3.14159 / 180.0
                    else:
                        target_lin = self.joint_positions[lin_j] + request.distance_mm / 1000.0
                        target_rot = self.joint_positions[rot_j] + request.angle_deg * 3.14159 / 180.0
                move_time = (abs(request.distance_mm) + abs(request.angle_deg)) / 100.0
                time.sleep(max(move_time, 1.0))
                with self.joint_state_lock:
                    self.joint_positions[lin_j] = target_lin
                    self.joint_positions[rot_j] = target_rot
                success = True
            elif request.operation_type == OP_GOTO_ABS:
                success = table_controller.go_to_absolute(
                    distance_mm=request.distance_mm,
                    angle_degrees=request.angle_deg,
                    linear_speed=request.linear_speed,
                    rotate_speed=request.rotate_speed,
                    stop_event=stop_event,
                )
            elif request.operation_type == OP_GOTO_HOME:
                success = table_controller.go_to_absolute_zero(
                    linear_speed=request.linear_speed,
                    rotate_speed=request.rotate_speed,
                    stop_event=stop_event,
                )
            elif request.operation_type in JOG_DIRECTIONS:
                direction = JOG_DIRECTIONS[request.operation_type]
                success = table_controller.jog(
                    direction=direction,
                    linear_speed=request.linear_speed,
                    rotate_speed=request.rotate_speed,
                    stop_event=stop_event,
                )
            else:
                success = table_controller.go_to_table(
                    distance_mm=request.distance_mm,
                    angle_degrees=request.angle_deg,
                    linear_speed=request.linear_speed,
                    rotate_speed=request.rotate_speed,
                    operation_type=request.operation_type,
                    stop_event=stop_event,
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
                f"Background thread: Exception during {table_id} movement: {e}\n{traceback.format_exc()}"
            )
        finally:
            if table_id == "table1":
                self.table1_thread = None
            elif table_id == "table2":
                self.table2_thread = None
            self.get_logger().info(f"Background thread finished for {table_id}")

    def move_dual_table_callback(self, request, response):
        target_table_id = request.table_id
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

        # ── JOG: fire-and-forget continuous drive, preempts any stale thread ──
        if request.operation_type in JOG_DIRECTIONS:
            direction = JOG_DIRECTIONS[request.operation_type]
            # Preempt any stale position-move thread
            stop_event = self.table1_stop_event if target_table_id == "table1" else self.table2_stop_event
            stop_event.set()
            try:
                target_table.start_continuous(
                    direction=direction,
                    linear_speed=request.linear_speed,
                    rotate_speed=request.rotate_speed,
                )
                response.success = True
                response.message = f"🕹 Jog {direction} on {target_table_id}"
            except Exception as e:
                response.success = False
                response.message = f"Jog error: {e}"
            self.get_logger().info(response.message)
            return response

        # ── HOME: preset current physical position as encoder zero ──
        if request.operation_type == OP_PRESET_HOME:
            # Make sure motors are stopped first
            stop_event = self.table1_stop_event if target_table_id == "table1" else self.table2_stop_event
            stop_event.set()
            try:
                target_table.stop_all()
                time.sleep(0.1)
                ok = target_table.preset_home()
                response.success = ok
                response.message = (
                    f"🏠 {target_table_id} home set at current position"
                    if ok else
                    f"⚠ {target_table_id} home preset had errors — check log"
                )
            except Exception as e:
                response.success = False
                response.message = f"Home error: {e}"
            self.get_logger().info(response.message)
            return response

        # ── STOP: halt motors immediately, also cancel any running thread ──
        if request.operation_type == 0:
            stop_event = self.table1_stop_event if target_table_id == "table1" else self.table2_stop_event
            stop_event.set()
            try:
                target_table.stop_all()
            except Exception as e:
                self.get_logger().error(f"Stop error: {e}")
            response.success = True
            response.message = f"🛑 Stopped {target_table_id}."
            self.get_logger().info(response.message)
            return response

        # ── Position move (existing go_to_table path) ──
        if active_thread is not None and active_thread.is_alive():
            response.success = False
            response.message = (
                f"Error: Table '{target_table_id}' is already moving. Please wait."
            )
            self.get_logger().warning(response.message)
            return response

        stop_event = self.table1_stop_event if target_table_id == "table1" else self.table2_stop_event
        stop_event.clear()

        try:
            new_thread = threading.Thread(
                target=self._background_move_task,
                args=(target_table_id, target_table, request, stop_event),
                daemon=True,
            )
            if target_table_id == "table1":
                self.table1_thread = new_thread
            elif target_table_id == "table2":
                self.table2_thread = new_thread
            new_thread.start()
            response.success = True
            response.message = f"✅ Movement command accepted for Table '{target_table_id}'. Executing in background."
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = (
                f"Error starting background thread for '{target_table_id}': {e}"
            )
            # --- FIX: Removed the buggy 'exc_info=True' ---
            self.get_logger().error(response.message + f"\n{traceback.format_exc()}")
        return response

    def on_shutdown(self):
        self.get_logger().info("Shutting down dual_table_controller...")
        self._shutdown_event.set()
        if self.table1_thread and self.table1_thread.is_alive():
            self.get_logger().info("Waiting for Table 1 thread to join...")
            self.table1_thread.join(timeout=1.0)
        if self.table2_thread and self.table2_thread.is_alive():
            self.get_logger().info("Waiting for Table 2 thread to join...")
            self.table2_thread.join(timeout=1.0)
        self._close_ports()


def main(args=None):
    print("✅ DualTableController started")
    rclpy.init(args=args)
    node = DualTableController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # --- FIX: Removed the buggy 'exc_info=True' ---
        node.get_logger().fatal(
            f"Unhandled exception in executor spin: {e}\n{traceback.format_exc()}"
        )
    finally:
        node.on_shutdown()
        executor.shutdown()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
