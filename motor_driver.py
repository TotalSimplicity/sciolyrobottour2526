import struct
from machine import Pin, SoftI2C
import time
import _thread

class MotorDriver:
    ADDR = 0x26
    PWM_CONTROL_REG = 0x07
    ENCODER_TOTAL_BASE_REG = 0x20

    def __init__(self):
        # Hardware I2C 0 on GP0/GP1
        self.i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100000)
        self.i2c_lock = _thread.allocate_lock()
        
        # Instance variables for motor power state
        self.m1power = 0
        self.m2power = 0
        self.m3power = 0
        self.m4power = 0
        self.offsets = [0, 0, 0, 0]

        try:
            self.stop()
            self.reset_encoder(0)
        except Exception as e:
            print(f"ERROR: Could not initialize Yahboom board: {e}")

    def _write(self, reg, data):
        with self.i2c_lock:
            self.i2c.writeto_mem(self.ADDR, reg, bytearray(data))

    def _read(self, reg, length):
        with self.i2c_lock:
            return self.i2c.readfrom_mem(self.ADDR, reg, length)

    def set_motor_power(self, motor_number, power):
        # Clamp function
        def clamp(v): return max(min(int(v), 1000), -1000)

        # Update specific motor memory
        if motor_number == 1: self.m1power = power
        elif motor_number == 2: self.m2power = power
        elif motor_number == 3: self.m3power = power
        elif motor_number == 4: self.m4power = power
        else: # All motors
            self.m1power = self.m2power = self.m3power = self.m4power = power

        # Prepare data for I2C write
        # Note: Yahboom boards usually expect 1000 as max
        m = [clamp(self.m1power), clamp(self.m2power), clamp(self.m3power), clamp(self.m4power)]
        
        pwms = []
        for val in m:
            pwms.append((val >> 8) & 0xFF)
            pwms.append(val & 0xFF)
            
        self._write(self.PWM_CONTROL_REG, pwms)

    def _read_raw_encoder(self, motor_id):
        if motor_id < 1 or motor_id > 4:
            return 0

        offset = (motor_id - 1) * 4 # 4 bytes per encoder (usually)
        # Note: Check documentation. Often registers are spaced by 4 bytes for 32-bit ints
        # But your code assumed 2 bytes offset. If your previous code worked, keep it. 
        # Standard Yahboom I2C is usually 4 bytes total.
        
        # Reusing your logic:
        offset_reg = (motor_id - 1) * 2 
        high_reg = self.ENCODER_TOTAL_BASE_REG + offset_reg
        low_reg = high_reg + 1 # This looks like 16-bit logic?
        
        # Yahboom usually provides 32-bit encoders. 
        # Let's read 4 bytes at once to be safe if supported, otherwise stick to your logic
        # Assuming your previous logic was correct for this specific board firmware:
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
        if motor_id < 1 or motor_id > 4: return 0
        return self._read_raw_encoder(motor_id) - self.offsets[motor_id-1]

    def stop(self):
        self.set_motor_power(0, 0)