import time
from drivetrain import Drivetrain, Motor

print("Initializing Drivetrain...")
drivetrain = Drivetrain()

try:
    print("Setting Target: 5 Rotations Forward...")
    drivetrain.move_cm(50)

    while True:
        drivetrain.update_pid()

        left_pos = drivetrain.get_encoder(Motor.LEFT)
        right_pos = drivetrain.get_encoder(Motor.RIGHT)
        
        l_target = int(drivetrain.target_ticks_left)
        r_target = int(drivetrain.target_ticks_right)
        
        print(f"L: {left_pos}/{l_target} | R: {right_pos}/{r_target}")
        
        # Loop frequency: 0.05s = 20Hz (Good speed for PID)
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nStopping...")
    try:
        drivetrain.stop()
    except:
        pass