import time
from drivetrain import Drivetrain, Motor

print("Initializing Drivetrain...")

kc = 14
period_oscillation = 1

kp = kc*0.5
# ki = 2*(kp/period_oscillation)
# kd = kp * (period_oscillation/8)
ki = 0
kd = 0

k_constants = {
    'kp': kp,
    'ki': ki,
    'kd': kd
}

drivetrain = Drivetrain(k_constants)

try:
    drivetrain.set_target_rotation(Motor.LEFT, 1)   # 5 rotations forward
    drivetrain.set_target_rotation(Motor.RIGHT, 1)  # 5 rotations forward

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