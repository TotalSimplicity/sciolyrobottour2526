import math
from micropython import const
from motor_driver import MotorDriver
import time

class Motor:
    LEFT = const(1)
    RIGHT = const(2)

class Drivetrain:
    # Physical Adjustments
    rightMotorReversed = False
    leftMotorReversed = True
    rightEncoderReversed = True
    leftEncoderReversed = False

    MAX_POWER = 1000 # Capped for safety during testing
    MIN_POWER = 300
    STOP_TOLERANCE = 30
    MAX_INTEGRAL = 20000 # Anti-windup cap

    # Cross-coupled sync gain — penalizes one wheel getting ahead of the other
    # during straight moves. Set to 0 to disable. Tune up if the bot veers,
    # down if it fights itself.
    SYNC_KP = 4.0

    def __init__(self, k_constants: dict):
        self.driver = MotorDriver()

        self.wheel_diameter_cm = 4.3
        self.track_width_cm = 16
        self.ticks_per_rev = 1067

        self.kp = k_constants.get('kp', 0)
        self.ki = k_constants.get('ki', 0)
        self.kd = k_constants.get('kd', 0)

        self.target_ticks_left = 0
        self.target_ticks_right = 0

        self.last_error_left = 0
        self.last_error_right = 0
        self.integral_left = 0
        self.integral_right = 0

    def _clamp_power(self, power: int) -> int:
        return max(min(int(power), self.MAX_POWER), -self.MAX_POWER)

    def set_motor_power(self, motor, power: int):
        power = self._clamp_power(power)
        if motor == Motor.LEFT:
            if self.leftMotorReversed: power = -power
            self.driver.set_motor_power(Motor.LEFT, power)
        elif motor == Motor.RIGHT:
            if self.rightMotorReversed: power = -power
            self.driver.set_motor_power(Motor.RIGHT, power)

    def get_encoder(self, motor):
        if motor == Motor.LEFT:
            pos = self.driver.get_encoder(Motor.LEFT)
            return -pos if self.leftEncoderReversed else pos
        elif motor == Motor.RIGHT:
            pos = self.driver.get_encoder(Motor.RIGHT)
            return -pos if self.rightEncoderReversed else pos
        return 0

    def stop(self):
        self.driver.set_motor_power(0, 0)

    def _cm_to_rotations(self, cm):
        circumference = self.wheel_diameter_cm * math.pi
        return cm / circumference

    def _reset_pid_state(self):
        # Stale integral/derivative from a finished move would yank the bot
        # at the start of the next one.
        self.last_error_left = 0
        self.last_error_right = 0
        self.integral_left = 0
        self.integral_right = 0

    def move_cm(self, distance_cm):
        rotations = self._cm_to_rotations(distance_cm)
        self.change_target_rotation(Motor.LEFT, rotations)
        self.change_target_rotation(Motor.RIGHT, rotations)
        self._reset_pid_state()

    def turn_degrees(self, degrees):
        # Tuning hint: If it turns too far/short, adjust track_width_cm
        turn_circumference = self.track_width_cm * math.pi
        distance_cm = (degrees / 360.0) * turn_circumference
        rotations = self._cm_to_rotations(distance_cm)

        self.change_target_rotation(Motor.LEFT, rotations)
        self.change_target_rotation(Motor.RIGHT, -rotations)
        self._reset_pid_state()

    def is_at_target(self, tolerance_ticks=50):
        left_pos = self.get_encoder(Motor.LEFT)
        right_pos = self.get_encoder(Motor.RIGHT)

        left_error = abs(self.target_ticks_left - left_pos)
        right_error = abs(self.target_ticks_right - right_pos)

        return left_error <= tolerance_ticks and right_error <= tolerance_ticks

    def change_target_rotation(self, motor, change):
        delta_ticks = change * self.ticks_per_rev
        if motor == Motor.LEFT:
            self.target_ticks_left += delta_ticks
        elif motor == Motor.RIGHT:
            self.target_ticks_right += delta_ticks

    def update_pid(self):
        try:
            current_left = self.get_encoder(Motor.LEFT)
            current_right = self.get_encoder(Motor.RIGHT)
        except:
            print("pid - error reading encoders")
            return

        error_left = self.target_ticks_left - current_left
        error_right = self.target_ticks_right - current_right

        # --- 1. Deadband / Tolerance Check ---
        if abs(error_left) < self.STOP_TOLERANCE:
            error_left = 0
            self.integral_left = 0
        if abs(error_right) < self.STOP_TOLERANCE:
            error_right = 0
            self.integral_right = 0

        self.integral_left += error_left
        self.integral_right += error_right

        self.integral_left = max(min(self.integral_left, self.MAX_INTEGRAL), -self.MAX_INTEGRAL)
        self.integral_right = max(min(self.integral_right, self.MAX_INTEGRAL), -self.MAX_INTEGRAL)

        derivative_left = error_left - self.last_error_left
        derivative_right = error_right - self.last_error_right

        raw_power_left = (error_left * self.kp) + (self.integral_left * self.ki) + (derivative_left * self.kd)
        raw_power_right = (error_right * self.kp) + (self.integral_right * self.ki) + (derivative_right * self.kd)

        # --- 2. Cross-coupled sync (only on straight moves) ---
        # Without this, each motor PID is independent and one wheel can
        # outrun the other on uneven floor / battery sag, causing veer.
        if self.target_ticks_left == self.target_ticks_right:
            sync_err = current_right - current_left  # +ve = left is behind
            sync_correction = self.SYNC_KP * sync_err
            raw_power_left += sync_correction
            raw_power_right -= sync_correction

        # --- 3. Apply Minimum Power (Stiction Compensation) ---
        power_left = 0
        if raw_power_left > 0:
            power_left = raw_power_left + self.MIN_POWER
        elif raw_power_left < 0:
            power_left = raw_power_left - self.MIN_POWER

        power_right = 0
        if raw_power_right > 0:
            power_right = raw_power_right + self.MIN_POWER
        elif raw_power_right < 0:
            power_right = raw_power_right - self.MIN_POWER

        self.last_error_left = error_left
        self.last_error_right = error_right

        try:
            self.set_motor_power(Motor.LEFT, int(power_left))
            self.set_motor_power(Motor.RIGHT, int(power_right))
        except:
            return
        print(f"Left Pos: {current_left}/{self.target_ticks_left}, Right Pos: {current_right}/{self.target_ticks_right}, Power L: {int(power_left)}, Power R: {int(power_right)}")
