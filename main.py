import time
from drivetrain import Drivetrain, Motor
import _thread

print("Initializing Drivetrain...")

kc = 14
period_oscillation = 1 #def wrong


# Ziegler-Nichols stuff
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

def run_until_target():
    while not drivetrain.is_at_target():
        time.sleep(0.05)

def pid_loop():
    while True:
        drivetrain.update_pid()
        time.sleep(0.05)

try:

    thread = _thread.start_new_thread(pid_loop, ())
    print("thread started")
    drivetrain.move_cm(30.48)
    run_until_target()
    drivetrain.turn_degrees(90)
    run_until_target()
    drivetrain.move_cm(30.48)

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