from machine import I2C, Pin
import time

# Scan for I2C devices
i2c = I2C(1, scl=Pin(19), sda=Pin(18), freq=100000)
print("Scanning I2C bus...")
devices = i2c.scan()

if devices:
    print(f"Found devices at addresses: {[hex(d) for d in devices]}")
else:
    print("NO I2C DEVICES FOUND - Driver is not responding!")

print("\nChecklist:")
print("1. Is the motor driver powered? (Check power LED)")
print("2. Is the battery/power supply voltage adequate? (7-12V)")
print("3. Try power cycling the motor driver completely")
print("4. Check if Type-C cable is firmly connected")