import RPi.GPIO as GPIO
import time
import serial
############################################################
"""
MAIN LOOP 
    1 drive forward
    2 deploy an ultrasonic scan
    3 pivot to best angle based on sweep
    4 driveforward
    5 loop 
"""
############################################################
"""
servo_sweep_scan:

    - sweep by moving the sensors incrementally left and right
    
    - record both distance measurements at each step
    
    - center and evaluate which angle (measured in steps from center)
    
    - calculates the best reading (scoring function)
    
    returns:
        best_angle (int)          #negative for left, positive for right, 0 for center.
"""
############################################################
"""

"""
############################################################


# GPIO PINS for ULTRASONIC
TRIG_LEFT = 17
ECHO_LEFT = 27
TRIG_RIGHT = 22
ECHO_RIGHT = 23

# SERIAL INFO
SERIAL_PORT = '/dev/ttyUSB0'  # may change with lidar, etc.
BAUD_RATE = 9600

# DISTANCE THRESHOLD (for deciding if path is “clear”)
CLEAR_DISTANCE = 25.0

# Sweep increments: more steps = finer search, but slower
PIVOT_STEPS = 12 

# CONSTANTS FOR SCORING
MAX_OFFSET = 10.0       # maximum acceptable difference (cm) between left & right readings
PENALTY_FACTOR = 1.0    # penalty for imbalance (per cm difference)

# INITIALIZE GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(TRIG_LEFT, GPIO.OUT)
GPIO.setup(ECHO_LEFT, GPIO.IN)
GPIO.setup(TRIG_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_RIGHT, GPIO.IN)

# INITIALIZE SERIAL
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) 
    print(f"Serial connection established on {SERIAL_PORT}")
except Exception as e:
    print("Failed to open serial port:", e)
    exit()

# FUNCTIONS
def send_command(command):
    """
    Send a command string to the Arduino.
    For example: 'driveF', 'pivotL', 'pivotR', etc.
    """
    cmd = command.strip() + "\n"
    ser.write(cmd.encode('utf-8'))
    print("Sent:", command)

def read_response():
    """
    Non-blocking read of any available lines from the Arduino.
    """
    while ser.in_waiting:
        line = ser.readline().decode('utf-8').strip()
        if line:
            print("Arduino:", line)

def measure_distance(trig_pin, echo_pin):
    """
    Measure distance (in cm) using an ultrasonic sensor.
    Returns the distance in cm or None on error.
    """
    # Ensure trigger is low, then send a 10µs pulse.
    GPIO.output(trig_pin, False)
    time.sleep(0.0002)
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)

    # Wait for echo to start (rising edge)
    start_time = time.time()
    timeout = start_time + 0.04 
    while GPIO.input(echo_pin) == 0:
        start_time = time.time()
        if start_time > timeout:
            return None

    # Wait for echo to end (falling edge)
    end_time = time.time()
    timeout = end_time + 0.04
    while GPIO.input(echo_pin) == 1:
        end_time = time.time()
        if end_time > timeout:
            return None

    # Calculate distance (speed of sound: 34300 cm/s)
    pulse_duration = end_time - start_time
    distance = pulse_duration * 34300 / 2.0
    return distance

def get_ultrasonic_distances():
    """
    Returns a tuple (left_dist, right_dist) in cm.
    If a sensor read fails, returns None for that sensor.
    """
    left = measure_distance(TRIG_LEFT, ECHO_LEFT)
    right = measure_distance(TRIG_RIGHT, ECHO_RIGHT)
    return (left, right)

def path_is_clear(left_dist, right_dist, threshold=CLEAR_DISTANCE):
    """
    Returns True if both distances are above the threshold.
    """
    if left_dist is None or right_dist is None:
        return False
    return (left_dist > threshold and right_dist > threshold)

def servo_sweep_scan():
    """
    - sweep by moving the sensors incrementally left and right
    
    - record both distance measurements at each step
    
    - center and evaluate which angle (measured in steps from center)
    
    - calculates the best reading (high combined distance and balanced left/right values)
    
    Returns:
        best_angle (int): negative for left, positive for right, 0 for center.
    """
    measurements = []  # (angle, left, right)
    
    # 2-1 zero and get measurements
    send_command("steerZ")
    left, right = get_ultrasonic_distances()
    measurements.append((0, left, right))
    print("Center measurement:", (0, left, right))
    time.sleep(0.2)
    
    # 2-2 sweep and record turning Left
    for i in range(1, PIVOT_STEPS + 1):
        send_command("pivotL")  # turn L
        time.sleep(0.3)
        send_command("S")       # stop movement
        time.sleep(0.2)
        left, right = get_ultrasonic_distances() # calc distances
        measurements.append((-i, left, right))
        print(f"Left sweep step {-i} measurement:", (left, right)) 
    
    # 2-3 go to center/zero (from L)
        send_command("steerZ")
        time.sleep(0.3)
        send_command("S")
        time.sleep(0.2)
    
    # 2-4 sweep and record turning Right
    for i in range(1, PIVOT_STEPS + 1):
        send_command("pivotR") #turn R
        time.sleep(0.3)
        send_command("S") 
        time.sleep(0.2)
        left, right = get_ultrasonic_distances()
        measurements.append((i, left, right))
        print(f"Right sweep step {i} measurement:", (left, right))
    
    # 2-5 go to center/zero (from R)
    for i in range(PIVOT_STEPS):
        send_command("pivotL")
        time.sleep(0.3)
        send_command("S")
        time.sleep(0.2)
    

    
    # 3 - SCORING FUNCTION
    #     mr gpt wrote this i dont get it
    #    Score = (left + right) - (penalty * abs(left - right))
    best_angle = 0
    best_score = -1
    for angle, left, right in measurements:
        if left is None or right is None:
            continue  # skip invalid readings
        diff = abs(left - right)
        score = left + right - PENALTY_FACTOR * diff
        print(f"Angle {angle}: left {left:.1f} cm, right {right:.1f} cm, diff {diff:.1f}, score {score:.1f}")
        if score > best_score:
            best_score = score
            best_angle = angle
    
    print("All sweep measurements:", measurements)
    print("Best angle chosen:", best_angle, "with score:", best_score)
    return best_angle
#MAIN LOOP
def main():
    try:
        while True:
            #drive forward
            send_command("driveF")
            time.sleep(1.0)  # drive forward for 1 second
            send_command("S")
            read_response()
            time.sleep(0.5)
            
            # sweep scan
            print("Performing servo sweep scan...")
            best_angle = servo_sweep_scan()
            
            # go to best angle
            if best_angle < 0:
                print(f"Pivoting left {abs(best_angle)} step(s) to align.")
                for i in range(abs(best_angle)):
                    send_command("pivotL")
                    time.sleep(0.3)
                    send_command("S")
                    time.sleep(0.2)
            elif best_angle > 0:
                print(f"Pivoting right {best_angle} step(s) to align.")
                for i in range(best_angle):
                    send_command("pivotR")
                    time.sleep(0.3)
                    send_command("S")
                    time.sleep(0.2)
            else:
                print("Best angle is center; no pivot needed.")
            
            # drive forward again after pivot sequence
            send_command("driveF")
            time.sleep(1.0)
            send_command("S")
            read_response()
            
            time.sleep(1.0)  # pause before the next cycle
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        GPIO.cleanup()
        send_command("S")
        ser.close()

if __name__ == "__main__":
    main()
