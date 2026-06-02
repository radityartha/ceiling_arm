#!/usr/bin/env python3
"""
Table Remote Controller — hold key to move, release to stop.
Works in any terminal (no X11 / display required).

  Hold W  →  linear forward
  Hold S  →  linear backward
  Hold D  →  rotate clockwise
  Hold A  →  rotate counter-clockwise
  ] / [   →  speed up / slow down
  1 / 2   →  switch table
  Q       →  quit
"""

import sys
import select
import termios
import tty
import threading
import time
import math
import rclpy
from rclpy.node import Node
from moving_table_interfaces.srv import MovingTable
from sensor_msgs.msg import JointState

# ── speeds (pulses/s) ────────────────────────────────────
DEFAULT_LIN_SPEED = 1600
DEFAULT_ROT_SPEED = 400
SPEED_STEP        = 200

# ── operation_type codes ─────────────────────────────────
OP_STOP        = 0
OP_JOG_FORWARD = 10
OP_JOG_BACK    = 11
OP_JOG_ROT_CW  = 12
OP_JOG_ROT_CCW = 13
OP_HOME        = 99
OP_GOTO_HOME   = 98

MOVE_KEYS = {
    'w': (OP_JOG_FORWARD,  "▶ linear forward"),
    's': (OP_JOG_BACK,     "◀ linear backward"),
    'd': (OP_JOG_ROT_CW,   "↻ rotate CW"),
    'a': (OP_JOG_ROT_CCW,  "↺ rotate CCW"),
}

# ── key-release detection ─────────────────────────────────
# OS sends key-repeat every ~30ms while held.
# If nothing arrives within KEY_TIMEOUT, key was released.
KEY_TIMEOUT = 0.15   # seconds


class TableController(Node):
    def __init__(self):
        super().__init__("table_keyboard")
        self.client = self.create_client(MovingTable, "move_dual_table")

        self.active_table = "table1"
        self.lin_speed    = DEFAULT_LIN_SPEED
        self.rot_speed    = DEFAULT_ROT_SPEED
        self._status      = "ready"
        self._lock        = threading.Lock()

        # Live joint positions — updated by /joint_states subscription
        self._positions = {
            "t1_linear_joint":   0.0,
            "t1_rotation_joint": 0.0,
            "t2_linear_joint":   0.0,
            "t2_rotation_joint": 0.0,
        }
        # Display origin offsets — subtracted from raw positions for HUD display
        self._origin = {k: 0.0 for k in self._positions}

        self.create_subscription(
            JointState, "/joint_states", self._joint_cb, 10
        )

    def _joint_cb(self, msg: JointState):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self._positions:
                    self._positions[name] = pos

    def get_active_position(self):
        """Return (linear_mm, rotation_deg) for the active table, with origin offset."""
        with self._lock:
            if self.active_table == "table1":
                lin_m = self._positions["t1_linear_joint"] - self._origin["t1_linear_joint"]
                rot_r = self._positions["t1_rotation_joint"] - self._origin["t1_rotation_joint"]
            else:
                lin_m = self._positions["t2_linear_joint"] - self._origin["t2_linear_joint"]
                rot_r = self._positions["t2_rotation_joint"] - self._origin["t2_rotation_joint"]
        return lin_m * 1000.0, math.degrees(rot_r)

    def zero_current(self, table_id: str = None):
        """Capture current position of given table (or active) as new display origin."""
        with self._lock:
            tid = table_id or self.active_table
            if tid == "table1":
                self._origin["t1_linear_joint"]   = self._positions["t1_linear_joint"]
                self._origin["t1_rotation_joint"] = self._positions["t1_rotation_joint"]
            else:
                self._origin["t2_linear_joint"]   = self._positions["t2_linear_joint"]
                self._origin["t2_rotation_joint"] = self._positions["t2_rotation_joint"]

    def zero_all(self):
        with self._lock:
            for k in self._positions:
                self._origin[k] = self._positions[k]

    def _call(self, op_type):
        req = MovingTable.Request()
        req.table_id       = self.active_table
        req.distance_mm    = 0.0
        req.angle_deg      = 0.0
        req.linear_speed   = self.lin_speed
        req.rotate_speed   = self.rot_speed
        req.operation_type = op_type
        future = self.client.call_async(req)
        future.add_done_callback(self._cb)

    def _cb(self, future):
        try:
            res = future.result()
            with self._lock:
                self._status = res.message.strip()[:60]
        except Exception as e:
            with self._lock:
                self._status = f"error: {e}"

    def start_jog(self, op_type):
        self._call(op_type)

    def send_stop(self):
        self._call(OP_STOP)

    def send_home(self):
        """Hardware preset: set current physical position as encoder zero."""
        self._call(OP_HOME)
        # Also reset display offset to 0 (encoder will be 0 after preset)
        with self._lock:
            for k in self._origin:
                self._origin[k] = 0.0
                self._positions[k] = 0.0

    def send_goto_home(self):
        """Move table to absolute encoder zero (0 mm, 0 deg).
        Uses OP_GOTO_HOME so the server reads motor position directly — avoids
        the stale /joint_states round-trip that caused the wrong-position bug."""
        req = MovingTable.Request()
        req.table_id       = self.active_table
        req.distance_mm    = 0.0
        req.angle_deg      = 0.0
        req.linear_speed   = self.lin_speed
        req.rotate_speed   = self.rot_speed
        req.operation_type = OP_GOTO_HOME
        future = self.client.call_async(req)
        future.add_done_callback(self._cb)

    def set_status(self, msg: str):
        with self._lock:
            self._status = msg

    @property
    def status(self):
        with self._lock:
            return self._status


def read_key(timeout: float) -> str | None:
    """
    Return the next key character within `timeout` seconds,
    or None if nothing arrives (key released / idle).
    Handles arrow-key escape sequences gracefully.
    """
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if not ready:
            return None
        ch = sys.stdin.read(1)
        # Swallow multi-byte escape sequences (arrow keys etc.) without acting on them
        if ch == '\x1b':
            select.select([sys.stdin], [], [], 0.05)
            try:
                sys.stdin.read(2)
            except Exception:
                pass
            return None
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def print_hud(node: TableController, moving: bool, direction: str):
    sys.stdout.write("\033[H")   # move cursor to top (no full clear = no flicker)
    lin_mm, rot_deg = node.get_active_position()
    move_str = direction if moving else "stopped"
    status   = node.status[:38]

    print(f"╔══════════════════════════════════════════════╗")
    print(f"║          TABLE REMOTE CONTROLLER             ║")
    print(f"╠══════════════════════════════════════════════╣")
    print(f"║  Table    : {node.active_table.upper():<33}║")
    print(f"║  Linear   : {lin_mm:>+8.1f} mm                     ║")
    print(f"║  Rotation : {rot_deg:>+8.1f} °                      ║")
    print(f"║  Speed    : lin={node.lin_speed:<6} rot={node.rot_speed:<6} pulse/s║")
    print(f"║  Motion   : {move_str:<33}║")
    print(f"║  Status   : {status:<33}║")
    print(f"╠══════════════════════════════════════════════╣")
    print(f"║  Hold W/S   →  linear forward/back            ║")
    print(f"║  Hold A/D   →  rotate CCW / CW                ║")
    print(f"║  ] / [      →  speed up / slow down           ║")
    print(f"║  1 / 2      →  switch table                   ║")
    print(f"║  Z / X      →  zero display (active / both)   ║")
    print(f"║  H          →  HOME: set hardware origin here ║")
    print(f"║  G          →  go to home position            ║")
    print(f"║  Q          →  quit                           ║")
    print(f"╚══════════════════════════════════════════════╝")
    sys.stdout.flush()


def main():
    rclpy.init()
    node = TableController()

    print("Connecting to move_dual_table service...")
    if not node.client.wait_for_service(timeout_sec=8.0):
        print("❌ Service not available — start dual_table_controller first.")
        rclpy.shutdown()
        return
    print("✅ Connected!\n")

    spin_t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_t.start()

    current_dir  = None   # direction currently jogging
    is_moving    = False
    last_label   = "—"

    # Clear screen once, then use cursor-home for updates
    sys.stdout.write("\033[2J")
    print_hud(node, is_moving, "—")
    last_redraw = time.time()

    try:
        while True:
            key = read_key(KEY_TIMEOUT)

            # Refresh HUD every 0.2s so position updates live
            if time.time() - last_redraw > 0.2:
                print_hud(node, is_moving, last_label if is_moving else "—")
                last_redraw = time.time()

            if key is None:
                # ── no key within timeout → key released → stop ──
                if is_moving:
                    is_moving   = False
                    current_dir = None
                    node.send_stop()
                    print_hud(node, is_moving, "—")
                continue

            key = key.lower()

            # ── quit ──
            if key in ('q', '\x03'):
                if is_moving:
                    node.send_stop()
                break

            # ── switch table ──
            if key in ('1', '2'):
                if is_moving:
                    node.send_stop()
                    is_moving   = False
                    current_dir = None
                node.active_table = f"table{key}"
                print_hud(node, is_moving, "—")
                continue

            # ── zero current position (display only) ──
            if key == 'z':
                node.zero_current()
                node.set_status(f"zeroed {node.active_table} (display)")
                print_hud(node, is_moving, last_label if is_moving else "—")
                last_redraw = time.time()
                continue
            if key == 'x':
                node.zero_all()
                node.set_status("zeroed both tables (display)")
                print_hud(node, is_moving, last_label if is_moving else "—")
                last_redraw = time.time()
                continue

            # ── HOME: set current physical position as hardware encoder zero ──
            if key == 'h':
                if is_moving:
                    node.send_stop()
                    is_moving   = False
                    current_dir = None
                node.send_home()
                node.set_status(f"🏠 hardware home set for {node.active_table}")
                print_hud(node, is_moving, "—")
                last_redraw = time.time()
                continue

            # ── GO TO HOME: move table back to encoder zero position ──
            if key == 'g':
                if is_moving:
                    node.send_stop()
                    is_moving   = False
                    current_dir = None
                node.send_goto_home()
                node.set_status(f"↩ going to home: {node.active_table}")
                print_hud(node, is_moving, "—")
                last_redraw = time.time()
                continue

            # ── speed adjust ──
            if key == ']':
                node.lin_speed = min(node.lin_speed + SPEED_STEP, 20000)
                node.rot_speed = min(node.rot_speed + SPEED_STEP // 2, 10000)
                print_hud(node, is_moving, current_dir or "—")
                continue
            if key == '[':
                node.lin_speed = max(node.lin_speed - SPEED_STEP, 200)
                node.rot_speed = max(node.rot_speed - SPEED_STEP // 2, 100)
                print_hud(node, is_moving, current_dir or "—")
                continue

            # ── movement keys ──
            if key in MOVE_KEYS:
                op_type, label = MOVE_KEYS[key]

                if key != current_dir:
                    # Direction changed — stop current jog first
                    if is_moving:
                        node.send_stop()
                        time.sleep(0.05)

                    current_dir = key
                    is_moving   = True
                    last_label  = label
                    node.start_jog(op_type)
                    print_hud(node, is_moving, label)
                # else: same key repeat while held — motor already jogging, ignore

    finally:
        # Restore terminal and make sure motor is stopped
        termios.tcsetattr(sys.stdin.fileno(),
                          termios.TCSADRAIN,
                          termios.tcgetattr(sys.stdin.fileno()))
        node.send_stop()
        time.sleep(0.2)
        print("\nBye.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
