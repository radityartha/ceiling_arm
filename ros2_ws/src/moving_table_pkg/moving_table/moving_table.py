import time
from typing import Optional
from moving_table.oml_mrtu import *

# Wheel and encoder parameters
WHEEL_CIRCUMFERENCE = 40.0 * 3.14159265359  # mm
PULSES_PER_REVOLUTION = 12000  # Encoder pulses per wheel revolution
PULSES_PER_DEGREE = 9000 / 90  # 9000 pulses = 90 degrees


class MovingTableController:
    def __init__(
        self, motor1, motor2, motor3, poll_interval=0.1, timeout=30.0, logger=None
    ):
        self.motor1 = motor1
        self.motor2 = motor2
        self.motor3 = motor3
        self.poll_interval = poll_interval
        self.timeout = timeout
        self.logger = logger

    def _log(self, msg):
        if self.logger:
            self.logger.info(msg)
        else:
            print(msg)

    def configure_motor(
        self, motor: ModbusAZ, acc: int = 1000, speed: int = 100000, current: int = 1000
    ) -> bool:
        self._log(f"Configuring motor {motor}")
        if not motor.writeParamAcc(time=acc, speed=speed):
            self._log(f"Failed to set acceleration for motor {motor}")
            return False
        if not motor.writeParamDec(time=acc, speed=speed):
            self._log(f"Failed to set deceleration for motor {motor}")
            return False
        if not motor.writeParamCurrent(current=current):
            self._log(f"Failed to set current for motor {motor}")
            return False
        motor.resetAlarm()
        return True

    def move_table(
        self, distance_mm: float, speed_pulses: int, operation_type: int
    ) -> bool:
        pulses = int((distance_mm / WHEEL_CIRCUMFERENCE) * PULSES_PER_REVOLUTION)
        self._log(f"Moving table {distance_mm} mm → {pulses} pulses")

        if not self.motor1.startPosition(
            position=pulses, speed=speed_pulses, OpeType=operation_type
        ):
            self._log("Failed to move Motor 1")
            return False
        if not self.motor2.startPosition(
            position=pulses, speed=speed_pulses, OpeType=operation_type
        ):
            self._log("Failed to move Motor 2")
            return False
        return True

    def rotate_table(
        self, angle_degrees: float, speed_pulses: int, operation_type: int
    ) -> bool:
        pulses = int(angle_degrees * PULSES_PER_DEGREE)
        self._log(f"Rotating table {angle_degrees}° → {pulses} pulses")

        if not self.motor3.startPosition(
            position=pulses, speed=speed_pulses, OpeType=operation_type
        ):
            self._log("Failed to rotate Motor 3")
            return False
        return True

    def go_to_table(
        self, distance_mm, angle_degrees, linear_speed, rotate_speed, operation_type=1, stop_event=None
    ):
        linear_pulses = int((distance_mm / WHEEL_CIRCUMFERENCE) * PULSES_PER_REVOLUTION)
        rotate_pulses = int(angle_degrees * PULSES_PER_DEGREE)
        motors = [self.motor1, self.motor2, self.motor3]
        increments = [linear_pulses, linear_pulses, rotate_pulses]
        speeds = [linear_speed, linear_speed, rotate_speed]

        for motor in motors:
            if not motor:
                self._log(f"❌ Motor {motor} is not available.")
                return False

        # Read starting absolute positions
        start_positions = []
        for motor in motors:
            motor.resetAlarm()
            pos = motor.readPosition()
            if not pos or not isinstance(pos, (list, tuple)) or len(pos) < 2:
                self._log(f"❌ Cannot read initial position from motor {motor.serverAddress}")
                return False
            start_positions.append(pos[1])

        # Always use absolute positioning (OpeType=1) — more reliable for both +/-
        abs_targets = [s + d for s, d in zip(start_positions, increments)]
        self._log(f"🎯 Start:{start_positions} → AbsTarget:{abs_targets} (Δ Lin:{linear_pulses}, Rot:{rotate_pulses})")

        for motor, abs_target, speed in zip(motors, abs_targets, speeds):
            result = motor.startPosition(position=abs_target, speed=speed, OpeType=1)
            if result[0] != 0:
                self._log(f"❌ Failed to start motor {motor.serverAddress}: {result}")
                return False

        start_time = time.time()
        while True:
            try:
                # Check for external stop signal
                if stop_event and stop_event.is_set():
                    self._log("🛑 Stop signal received — halting motors.")
                    for motor in motors:
                        try:
                            motor.stop()
                        except Exception:
                            pass
                    return False

                all_reached = True
                for motor, target in zip(motors, abs_targets):
                    pos = motor.readPosition()
                    if not pos or not isinstance(pos, (list, tuple)) or len(pos) < 2:
                        self._log(f"❌ Failed to read position from motor {motor.serverAddress}")
                        all_reached = False
                        break
                    current_pos = pos[1]
                    if abs(current_pos - target) > 50:
                        all_reached = False
                        self._log(f"Motor {motor.serverAddress}: pos={current_pos}, target={target}, diff={current_pos - target}")

                if all_reached:
                    self._log("✅ All motors reached target position.")
                    return True

                if (time.time() - start_time) > self.timeout:
                    self._log("⚠️ Timeout — stopping motors. Last positions: " +
                              str([motor.readPosition()[1] if motor.readPosition() else '?' for motor in motors]))
                    for motor in motors:
                        try:
                            motor.stop()
                        except Exception:
                            pass
                    return False

                time.sleep(self.poll_interval)

            except Exception as e:
                self._log(f"❌ Exception during position check: {e}")
                return False

    def go_to_absolute_zero(self, linear_speed, rotate_speed, stop_event=None):
        """Move each motor to encoder position 0 (set by H/ppreset).

        Uses go_to_table logic: reads each motor's current position directly,
        computes the delta needed to reach 0, then commands and polls until done.
        This avoids relying on the ROS /joint_states topic (which can be stale).
        """
        motors = [self.motor1, self.motor2, self.motor3]
        speeds = [linear_speed, linear_speed, rotate_speed]

        for motor in motors:
            if not motor:
                self._log("❌ Motor is not available.")
                return False

        # Read current encoder positions (same as go_to_table start-position read)
        start_positions = []
        for motor in motors:
            motor.resetAlarm()
            pos = motor.readPosition()
            if not pos or not isinstance(pos, (list, tuple)) or len(pos) < 2 or pos[0] != 0:
                self._log(f"❌ Cannot read position from motor {motor.serverAddress}: {pos}")
                return False
            start_positions.append(pos[1])

        abs_targets = [0, 0, 0]
        self._log(f"🏠 Going home: start={start_positions} → target={abs_targets}")

        # If already at home (within tolerance), skip movement
        if all(abs(p) <= 100 for p in start_positions):
            self._log("✅ Already at home position.")
            return True

        for motor, abs_target, speed in zip(motors, abs_targets, speeds):
            result = motor.startPosition(position=abs_target, speed=speed, OpeType=1)
            if result[0] != 0:
                self._log(f"❌ Failed to start motor {motor.serverAddress}: {result}")
                return False

        start_time = time.time()
        while True:
            if stop_event and stop_event.is_set():
                self._log("🛑 Stop signal — halting.")
                for m in motors:
                    try:
                        m.stop()
                    except Exception:
                        pass
                return False

            all_reached = True
            current = []
            for motor, target in zip(motors, abs_targets):
                pos = motor.readPosition()
                # pos[0] != 0 means Modbus error — must not be treated as position 1
                if not pos or not isinstance(pos, (list, tuple)) or len(pos) < 2 or pos[0] != 0:
                    self._log(f"⚠ Read error motor {motor.serverAddress}: {pos}")
                    all_reached = False
                    break
                current.append(pos[1])
                if abs(pos[1] - target) > 50:
                    all_reached = False

            if all_reached:
                self._log(f"✅ All motors at home. Positions: {current}")
                return True

            if time.time() - start_time > self.timeout:
                self._log(f"⚠️ Timeout going home. Last positions: {current}")
                for m in motors:
                    try:
                        m.stop()
                    except Exception:
                        pass
                return False

            time.sleep(self.poll_interval)

    def go_to_absolute(self, distance_mm, angle_degrees, linear_speed, rotate_speed, stop_event=None):
        """Move to an ABSOLUTE table position (distance_mm, angle_degrees).

        Unlike go_to_table (which adds a delta to the current position), this reads
        each motor's current encoder position directly and drives to the absolute
        target derived from the home origin (encoder 0 = home, set by H/ppreset).
        Because the target is computed server-side from the real encoder, it does
        NOT depend on the ROS /joint_states topic — so a sequence can be started
        from any table position without first homing.
        """
        motors = [self.motor1, self.motor2, self.motor3]
        speeds = [linear_speed, linear_speed, rotate_speed]

        linear_pulses = int((distance_mm / WHEEL_CIRCUMFERENCE) * PULSES_PER_REVOLUTION)
        rotate_pulses = int(angle_degrees * PULSES_PER_DEGREE)
        abs_targets = [linear_pulses, linear_pulses, rotate_pulses]

        for motor in motors:
            if not motor:
                self._log("❌ Motor is not available.")
                return False

        # Read current encoder positions (for already-there skip + logging)
        start_positions = []
        for motor in motors:
            motor.resetAlarm()
            pos = motor.readPosition()
            if not pos or not isinstance(pos, (list, tuple)) or len(pos) < 2 or pos[0] != 0:
                self._log(f"❌ Cannot read position from motor {motor.serverAddress}: {pos}")
                return False
            start_positions.append(pos[1])

        self._log(f"🎯 Absolute move: start={start_positions} → target={abs_targets}")

        if all(abs(s - t) <= 50 for s, t in zip(start_positions, abs_targets)):
            self._log("✅ Already at absolute target.")
            return True

        for motor, abs_target, speed in zip(motors, abs_targets, speeds):
            result = motor.startPosition(position=abs_target, speed=speed, OpeType=1)
            if result[0] != 0:
                self._log(f"❌ Failed to start motor {motor.serverAddress}: {result}")
                return False

        start_time = time.time()
        while True:
            if stop_event and stop_event.is_set():
                self._log("🛑 Stop signal — halting.")
                for m in motors:
                    try:
                        m.stop()
                    except Exception:
                        pass
                return False

            all_reached = True
            current = []
            for motor, target in zip(motors, abs_targets):
                pos = motor.readPosition()
                if not pos or not isinstance(pos, (list, tuple)) or len(pos) < 2 or pos[0] != 0:
                    self._log(f"⚠ Read error motor {motor.serverAddress}: {pos}")
                    all_reached = False
                    break
                current.append(pos[1])
                if abs(pos[1] - target) > 50:
                    all_reached = False

            if all_reached:
                self._log(f"✅ All motors at absolute target. Positions: {current}")
                return True

            if time.time() - start_time > self.timeout:
                self._log(f"⚠️ Timeout on absolute move. Last positions: {current}")
                for m in motors:
                    try:
                        m.stop()
                    except Exception:
                        pass
                return False

            time.sleep(self.poll_interval)

    def start_continuous(self, direction: str, linear_speed: int, rotate_speed: int):
        """Fire-and-forget: start continuous drive immediately (no blocking)."""
        def _run(motor, speed, dir_code, label):
            res = motor.startContinuous(speed, dir_code)
            if res and res[0] != 0:
                self._log(f"⚠ startContinuous {label} motor{motor.serverAddress} returned {res}")
            else:
                self._log(f"✅ startContinuous {label} motor{motor.serverAddress} speed={speed} OK")

        if direction == 'forward':
            self._log(f"▶ Jog forward speed={linear_speed}")
            _run(self.motor1, linear_speed, 0, 'fwd')
            _run(self.motor2, linear_speed, 0, 'fwd')
        elif direction == 'backward':
            self._log(f"◀ Jog backward speed={linear_speed}")
            _run(self.motor1, linear_speed, 1, 'bwd')
            _run(self.motor2, linear_speed, 1, 'bwd')
        elif direction == 'rotate_cw':
            self._log(f"↻ Jog rotate CW speed={rotate_speed}")
            _run(self.motor3, rotate_speed, 0, 'cw')
        elif direction == 'rotate_ccw':
            self._log(f"↺ Jog rotate CCW speed={rotate_speed}")
            _run(self.motor3, rotate_speed, 1, 'ccw')

    def jog(self, direction: str, linear_speed: int, rotate_speed: int, stop_event) -> bool:
        """Continuous drive until stop_event is set.
        direction: 'forward' | 'backward' | 'rotate_cw' | 'rotate_ccw'
        """
        try:
            if direction == 'forward':
                self._log(f"▶ Jog linear forward  speed={linear_speed}")
                self.motor1.startContinuous(linear_speed, 0)
                self.motor2.startContinuous(linear_speed, 0)
            elif direction == 'backward':
                self._log(f"◀ Jog linear backward speed={linear_speed}")
                self.motor1.startContinuous(linear_speed, 1)
                self.motor2.startContinuous(linear_speed, 1)
            elif direction == 'rotate_cw':
                self._log(f"↻ Jog rotate CW       speed={rotate_speed}")
                self.motor3.startContinuous(rotate_speed, 0)
            elif direction == 'rotate_ccw':
                self._log(f"↺ Jog rotate CCW      speed={rotate_speed}")
                self.motor3.startContinuous(rotate_speed, 1)
            else:
                self._log(f"❌ Unknown jog direction: {direction}")
                return False

            # Block until stop signal arrives (max 120 s safety)
            stop_event.wait(timeout=120.0)
            stop_event.clear()

        finally:
            # Always stop all motors on exit
            self._log("🛑 Jog stopping motors.")
            for motor in [self.motor1, self.motor2, self.motor3]:
                try:
                    motor.stop()
                except Exception:
                    pass

        return True

    def stop_all(self):
        for motor in [self.motor1, self.motor2, self.motor3]:
            try:
                motor.stop()
            except Exception:
                pass

    def preset_home(self) -> bool:
        """Preset the command position on all 3 motors to 0 at the current physical
        location. The AZ-series absolute encoder retains this origin across power cycles."""
        ok = True
        for motor in [self.motor1, self.motor2, self.motor3]:
            try:
                res = motor.ppreset()
                if res and res[0] == 0:
                    self._log(f"✅ Motor {motor.serverAddress} home preset")
                else:
                    self._log(f"⚠ Motor {motor.serverAddress} ppreset returned {res}")
                    ok = False
            except Exception as e:
                self._log(f"❌ Motor {motor.serverAddress} ppreset failed: {e}")
                ok = False
        return ok
