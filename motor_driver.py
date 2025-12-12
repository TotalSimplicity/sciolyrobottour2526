import struct
from machine import Pin, I2C
import time
import _thread  # <--- IMPORT THIS

class MotorDriver:
    ADDR = 0x26
    PWM_CONTROL_REG = 0x07
    ENCODER_TOTAL_BASE_REG = 0x20 

    # It is safer to use 100,000 Hz if you are getting timeouts, 
    # but 400,000 is fine if the wires are short and locking is fixed.
    i2c = I2C(0, scl=Pin(13), sda=Pin(12), freq=100000)


    m1power = 0
    m2power = 0
    m3power = 0
    m4power = 0

    offsets = [0, 0, 0, 0]

    def __init__(self):
        # Create a Lock object. Only one thread can hold this at a time.
        self.i2c_lock = _thread.allocate_lock() 

        try:
            self.set_motor_power(0, 0)
            self.reset_encoder(0)
        except:
            print("ERROR IN MOTOR_DRIVER.py - Check Wiring or Battery")
            # ... (Rest of your error handling logic) ...

    # --- THE CRITICAL FIX IS IN THESE TWO FUNCTIONS ---

    def _write(self, reg, data):
        # Acquire the lock before talking. If the other thread has it, we wait here.
        with self.i2c_lock:
            self.i2c.writeto_mem(self.ADDR, reg, bytearray(data))

    def _read(self, reg, length):
        with self.i2c_lock:
            return self.i2c.readfrom_mem(self.ADDR, reg, length)

    # --------------------------------------------------

    def set_motor_power(self, motor_number, power):
        if motor_number == 1:
            self.m1power = power
        elif motor_number == 2:
            self.m2power = power
        elif motor_number == 3:
            self.m3power = power
        elif motor_number == 4:
            self.m4power = power
        else:
            self.m1power = power
            self.m2power = power
            self.m3power = power
            self.m4power = power

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
        if motor_id < 1 or motor_id > 4:
            return 0

        offset = (motor_id - 1) * 2
        high_reg = self.ENCODER_TOTAL_BASE_REG + offset
        low_reg = high_reg + 1

        # We must read high and low bytes together atomically if possible,
        # but the lock inside _read protects the I2C transaction itself.
        high_buf = self._read(high_reg, 2)
        low_buf = self._read(low_reg, 2)

        high_val = high_buf[0] << 8 | high_buf[1]
        low_val = low_buf[0] << 8 | low_buf[1]

        encoder_val = (high_val << 16) | low_val

        if encoder_val >= 0x80000000:
            encoder_val -= 0x100000000

        return encoder_val

    def reset_encoder(self, motor_id=0):
        if motor_id == 0:
            for i in range(1, 5):
                self.offsets[i-1] = self._read_raw_encoder(i)
        elif 1 <= motor_id <= 4:
            self.offsets[motor_id-1] = self._read_raw_encoder(motor_id)

    def get_encoder(self, motor_id):
        if motor_id < 1 or motor_id > 4:
            return 0
        
        raw_val = self._read_raw_encoder(motor_id)
        return raw_val - self.offsets[motor_id-1]

    def stop(self):
        self.set_motor_power(0, 0)