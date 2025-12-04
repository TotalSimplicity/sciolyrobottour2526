import math
from micropython import const
from motor_driver import MotorDriver
import time

class Motor:
    LEFT = const(2)
    RIGHT = const(1)

class Drivetrain:
    rightMotorReversed = False
    leftMotorReversed = True

    rightEncoderReversed = True
    leftEncoderReversed = False

    def __init__(self, k_constants: dict):
        self.driver = MotorDriver()
        
        self.wheel_diameter_cm = 4.3
        self.track_width_cm = 16    # CHANGE!!
        
        #self.ticks_per_rev = 14 * 20.4 * 4
        self.ticks_per_rev = 1067
        
        # PID Constants
        self.kp = k_constants.get('kp', 0)
        self.ki = k_constants.get('ki', 0)
        self.kd = k_constants.get('kd', 0)
        
        self.target_ticks_left = 0
        self.target_ticks_right = 0
        
        # PID Memory
        self.last_error_left = 0
        self.last_error_right = 0
        self.integral_left = 0
        self.integral_right = 0

    def set_motor_power(self, motor, power: int):
        if motor == Motor.LEFT:
            if self.leftMotorReversed:
                power = -power
            self.driver.set_motor_power(1, power)
        elif motor == Motor.RIGHT:
            if self.rightMotorReversed:
                power = -power
            self.driver.set_motor_power(2, power)

    def set_motor_powers(self, power):
        self.set_motor_power(Motor.LEFT, power)
        self.set_motor_power(Motor.RIGHT, power)

    def get_encoder(self, motor):
        if motor == Motor.LEFT:
            pos = self.driver.get_encoder(1)
            if self.leftEncoderReversed:
                pos = -pos
            return pos
        elif motor == Motor.RIGHT:
            pos = self.driver.get_encoder(2)
            if self.rightEncoderReversed:
                pos = -pos
            return pos
        else:
            raise ValueError("Invalid motor specified.")
        

    def stop(self):
        self.driver.set_motor_power(0, 0)

    # --- MATH HELPERS ---
    def _cm_to_rotations(self, cm):
        circumference = self.wheel_diameter_cm * math.pi
        return cm / circumference

    # --- MOVEMENT COMMANDS ---
    
    def move_cm(self, distance_cm):
        rotations = self._cm_to_rotations(distance_cm)
        self.change_target_rotation(Motor.LEFT, rotations)
        self.change_target_rotation(Motor.RIGHT, rotations)

    def turn_degrees(self, degrees):
        """
        Rotates the robot in place.
        Positive degrees = Turn Right (Clockwise)
        Negative degrees = Turn Left (Counter-Clockwise)
        """
        turn_circumference = self.track_width_cm * math.pi
        
        distance_cm = (degrees / 360.0) * turn_circumference
        
        rotations = self._cm_to_rotations(distance_cm)
        
        self.change_target_rotation(Motor.LEFT, rotations)
        self.change_target_rotation(Motor.RIGHT, -rotations)

    def is_at_target(self, tolerance_ticks=15):
        left_pos = self.get_encoder(Motor.LEFT)
        right_pos = self.get_encoder(Motor.RIGHT)
        
        left_error = abs(self.target_ticks_left - left_pos)
        right_error = abs(self.target_ticks_right - right_pos)
        
        if left_error <= tolerance_ticks and right_error <= tolerance_ticks:       
            time.sleep(1)
            return True
        return False

    def set_target_rotations(self, left_rotations, right_rotations):
        current_left = self.get_encoder(Motor.LEFT)
        current_right = self.get_encoder(Motor.RIGHT)
        
        self.target_ticks_left = current_left + (left_rotations * self.ticks_per_rev)
        self.target_ticks_right = current_right + (right_rotations * self.ticks_per_rev)
        
        self._reset_pid_terms()

    def change_target_rotation(self, motor, change):
        delta_ticks = change * self.ticks_per_rev
        
        if motor == Motor.LEFT:
            self.target_ticks_left += delta_ticks
        elif motor == Motor.RIGHT:
            self.target_ticks_right += delta_ticks

    def set_target_rotation(self, motor, target):
        current_pos = self.get_encoder(motor)
        target_ticks = current_pos + (target * self.ticks_per_rev)

        if motor == Motor.LEFT:
            self.target_ticks_left = target_ticks
            self.last_error_left = 0
            self.integral_left = 0
        elif motor == Motor.RIGHT:
            self.target_ticks_right = target_ticks
            self.last_error_right = 0
            self.integral_right = 0

    def _reset_pid_terms(self):
        self.last_error_left = 0
        self.last_error_right = 0
        self.integral_left = 0
        self.integral_right = 0

    def update_pid(self):
        current_left = self.get_encoder(Motor.LEFT)
        current_right = self.get_encoder(Motor.RIGHT)

        error_left = self.target_ticks_left - current_left
        error_right = self.target_ticks_right - current_right

        self.integral_left += error_left
        self.integral_right += error_right

        derivative_left = error_left - self.last_error_left
        derivative_right = error_right - self.last_error_right

        power_left = (error_left * self.kp) + (self.integral_left * self.ki) + (derivative_left * self.kd)
        power_right = (error_right * self.kp) + (self.integral_right * self.ki) + (derivative_right * self.kd)

        self.set_motor_power(Motor.LEFT, int(power_left))
        self.set_motor_power(Motor.RIGHT, int(power_right))

        self.last_error_left = error_left
        self.last_error_right = error_right