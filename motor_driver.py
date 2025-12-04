import struct
from machine import Pin, I2C
import time

class MotorDriver:
    # Register Definitions
    ADDR = 0x26
    PWM_CONTROL_REG = 0x07
    ENCODER_TOTAL_BASE_REG = 0x20 

    # Hardcoded I2C as requested (Note: SoftI2C is often more stable for batteries)
    i2c = I2C(0, scl=Pin(13), sda=Pin(12), freq=100000)

    # Internal state to remember motor speeds
    m1power = 0
    m2power = 0
    m3power = 0
    m4power = 0

    # Store offsets for the "Reset" function
    offsets = [0, 0, 0, 0]

    def __init__(self):
        try:
            self.set_motor_power(0, 0)
            # Optional: Reset encoders to 0 on startup
            self.reset_encoder(0)
        except:
            print("ERROR IN MOTOR_DRIVER.py - Check Wiring or Battery")
            # Blink LED error loop
            while True:
                led = Pin(25, Pin.OUT)
                led.value(1)
                time.sleep(0.5)
                led.value(0)
                time.sleep(0.5)

    def _write(self, reg, data):
        self.i2c.writeto_mem(self.ADDR, reg, bytearray(data))

    def _read(self, reg, length):
        return self.i2c.readfrom_mem(self.ADDR, reg, length)

    def set_motor_power(self, motor_number, power):
        """
        Sets the PWM power for a specific motor or all motors.
        motor_number: 1-4 for specific motor, 0 (or others) for ALL motors.
        power: -1000 to 1000
        """
        # Update internal state
        if motor_number == 1:
            self.m1power = power
        elif motor_number == 2:
            self.m2power = power
        elif motor_number == 3:
            self.m3power = power
        elif motor_number == 4:
            self.m4power = power
        else:
            # If number is not 1-4, set ALL motors
            self.m1power = power
            self.m2power = power
            self.m3power = power
            self.m4power = power

        # Clamp values to -1000 to 1000
        m1 = max(min(self.m1power, 1000), -1000)
        m2 = max(min(self.m2power, 1000), -1000)
        m3 = max(min(self.m3power, 1000), -1000)
        m4 = max(min(self.m4power, 1000), -1000)

        pwms = [
            (m1 >> 8) & 0xFF, m1 & 0xFF,
            (m2 >> 8) & 0xFF, m2 & 0xFF,
            (m3 >> 8) & 0xFF, m3 & 0xFF,
            (m4 >> 8) & 0xFF, m4 & 0xFF
        ]
        self._write(self.PWM_CONTROL_REG, pwms)

    def _read_raw_encoder(self, motor_id):
        """
        Internal helper: Reads the actual hardware register value.
        """
        if motor_id < 1 or motor_id > 4:
            return 0

        offset = (motor_id - 1) * 2
        high_reg = self.ENCODER_TOTAL_BASE_REG + offset
        low_reg = high_reg + 1

        high_buf = self._read(high_reg, 2)
        low_buf = self._read(low_reg, 2)

        high_val = high_buf[0] << 8 | high_buf[1]
        low_val = low_buf[0] << 8 | low_buf[1]

        encoder_val = (high_val << 16) | low_val

        if encoder_val >= 0x80000000:
            encoder_val -= 0x100000000

        return encoder_val

    def reset_encoder(self, motor_id=0):
        """
        Sets the current encoder position to 0.
        motor_id: 1-4 for specific motor, 0 for ALL motors.
        """
        if motor_id == 0:
            # Reset all
            for i in range(1, 5):
                self.offsets[i-1] = self._read_raw_encoder(i)
        elif 1 <= motor_id <= 4:
            # Reset specific
            self.offsets[motor_id-1] = self._read_raw_encoder(motor_id)

    def get_encoder(self, motor_id):
        """
        Reads the encoder value relative to the last reset.
        """
        if motor_id < 1 or motor_id > 4:
            return 0
        
        # Get raw hardware value
        raw_val = self._read_raw_encoder(motor_id)
        
        # Return value adjusted by the offset
        return raw_val - self.offsets[motor_id-1]

    def stop(self):
        """Stops all motors"""
        self.set_motor_power(0, 0)