import time
from typing import Optional
from moving_table.oml_mrtu import *

# Wheel and encoder parameters
WHEEL_CIRCUMFERENCE = 40.0 * 3.14159265359  # mm
PULSES_PER_REVOLUTION = 12000  # Encoder pulses per wheel revolution
PULSES_PER_DEGREE = 9000 / 90  # 9000 pulses = 90 degrees


class MovingTableController:
    def __init__(
        self, motor1, motor2, motor3, poll_interval=0.1, timeout=10.0, logger=None
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
        self, distance_mm, angle_degrees, linear_speed, rotate_speed, operation_type
    ):
        linear_pulses = int((distance_mm / WHEEL_CIRCUMFERENCE) * PULSES_PER_REVOLUTION)
        rotate_pulses = int(angle_degrees * PULSES_PER_DEGREE)
        motors = [self.motor1, self.motor2, self.motor3]
        targets = [linear_pulses, linear_pulses, rotate_pulses]

        # Check if motors are configured/available before starting
        for motor in motors:
            if not motor:  # Or some other check if motor init failed
                self._log(f"❌ Motor {motor} is not available.")
                return False  # Indicate failure early

        # Start motion
        for motor, target, speed in zip(
            motors, targets, [linear_speed, linear_speed, rotate_speed]
        ):
            # Add error checking for startPosition if it returns bool
            if not motor.startPosition(
                position=target, speed=speed, OpeType=operation_type
            ):
                self._log(f"❌ Failed to start motor {motor.serverAddress}")
                # Optionally try to stop other motors here
                return False

        self._log(f"🎯 Moving to targets: Lin:{linear_pulses}, Rot:{rotate_pulses}")
        start_time = time.time()
        while True:
            all_reached = True
            try:  # Add error handling for communication
                for motor, target in zip(motors, targets):
                    pos = motor.readPosition()
                    if not pos or not isinstance(pos, (list, tuple)) or len(pos) < 2:
                        self._log(
                            f"❌ Failed to read valid position from motor {motor.serverAddress}"
                        )
                        all_reached = False
                        # Decide if you want to abort immediately or keep trying others
                        # return False # Uncomment to fail immediately on read error
                        break  # Break inner loop, will retry on next cycle or timeout

                    current_pos = pos[1]
                    # Check if position is within a small tolerance instead of exact match
                    # Define tolerance based on your encoder resolution/needs
                    tolerance = 10  # Example tolerance in pulses
                    if abs(current_pos - target) > tolerance:
                        all_reached = False
                        # self._log(f"Motor {motor.serverAddress}: Current={current_pos}, Target={target}") # Debug logging

                if not all_reached and (time.time() - start_time) <= self.timeout:
                    # If not all reached and not timed out, continue polling
                    time.sleep(self.poll_interval)
                    continue

                # If loop finishes, check conditions
                if all_reached:
                    self._log("✅ All motors reached target position.")
                    return True  # <<< RETURN TRUE ON SUCCESS >>>
                elif (time.time() - start_time) > self.timeout:
                    self._log("⚠️ Timeout waiting for motors to reach target")
                    # Optionally try to stop motors on timeout
                    # for m in motors: m.stopMotor() # Check if stopMotor exists
                    return False  # <<< RETURN FALSE ON TIMEOUT >>>
                else:
                    # Should not happen with the logic above, but handle defensively
                    self._log("❌ Unknown state in go_to_table loop.")
                    return False

            except Exception as e:
                self._log(f"❌ Exception during position check: {e}")
                # Optionally try to stop motors
                return False  # <<< RETURN FALSE ON EXCEPTION >>>
