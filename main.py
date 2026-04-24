import time
from drivetrain import Drivetrain, Motor
import _thread

print("Initializing Drivetrain...")

# --- Tuning Constants ---
kc = 14
period_oscillation = 1 

# Ziegler-Nichols (Commented out logic handled cleanly)
# kp = kc * 0.6  # Standard ZN
# ki = 2 * kp / period_oscillation
# kd = kp * period_oscillation / 8

# Manual Overrides for testing
kp = 3  # Start lower than 14, maybe 1.0 or 2.0
ki = 0.0
kd = 0.75  # A little damping usually helps

k_constants = {
    'kp': kp,
    'ki': ki,
    'kd': kd
}

run_threads = True
drivetrain = Drivetrain(k_constants)

def run_until_target():
    print("Moving...")
    start_time = time.time()
    while True:
        try:
            if drivetrain.is_at_target():
                print("Target reached.")
                break
            
            # Timeout safety
            if time.time() - start_time > 10: 
                print("Timed out!")
                break
        except Exception as e:
            print("Encoder read error:", e)
            continue
        time.sleep(0.05) # Check every 50ms

def pid_loop():
    global run_threads
    print("PID Thread Started")
    while run_threads:
        drivetrain.update_pid()
        time.sleep(0.02) # 50Hz Loop
    print("PID Thread exiting...")

# Start the PID thread
thread = _thread.start_new_thread(pid_loop, ())

try:
    # --- Main Test Routine ---
    print("Starting move in 2 seconds...")
    time.sleep(2)
    
    # 1. Move Forward
    drivetrain.move_cm(15)
    run_until_target()
    time.sleep(1)
    drivetrain.move_cm(-15)
    run_until_target()
    time.sleep(1)
    drivetrain.turn_degrees(90)
    run_until_target()
    time.sleep(1)

    print("Routine Complete.")

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    # Ensure motors stop even if code crashes
    run_threads = False
    time.sleep(0.2) # Wait for thread to exit
    drivetrain.stop()
    print("Clean exit.")