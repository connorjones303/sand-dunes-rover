#THIS PROGRAM [final_nav_roverv9-1] IS A COMBINATION OF dspv8-3 AND nav_roverv7-6

import RPi.GPIO as GPIO
import time
import serial
import smbus
import math
############################################################
"""
MAIN LOOP 
    1 drive forward
    2 deploy an ultrasonic scan
    3 pivot to best angle based on sweep
    4 set target heading from compass and drive straight using heading correction
    5 loop 
"""
############################################################
"""
servo_sweep_scan:

    - sweep by moving the sensors incrementally left and right
    
    - record both distance measurements at each step
    
    - center and evaluate which angle (measured in steps from center)
    
    - calculates the best reading (high combined distance and balanced left/right values)
    
    Returns:
        best_angle (int): negative for left, positive for right, 0 for center.
"""
############################################################
"""
COMPASS FUNCTIONS:

    - init_compass: initializes the HMC5883L magnetometer
    - read_compass_data: reads a 16-bit value from the compass
    - get_compass_heading: returns the current heading in degrees (0-360)
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

# CUSTOM VARIABLES for drive-straight
STEERING_TIME = 0.4     # base steering time (to be scaled)
HEADING_THRESHOLD = 5.0 # threshold (degrees) for heading correction

# COMPASS SETTINGS
COMPASS_ADDRESS = 0x1E

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

# INITIALIZE I2C (for both compass and any other I2C sensors)
bus = smbus.SMBus(1)

############################################################
#                     COMPASS FUNCTIONS                  #
############################################################

def init_compass():
    # Configuration Register A: 8-average, 15 Hz, normal measurement
    bus.write_byte_data(COMPASS_ADDRESS, 0x00, 0x70)
    # Configuration Register B: Gain = 5
    bus.write_byte_data(COMPASS_ADDRESS, 0x01, 0xA0)
    # Mode Register: Continuous measurement mode
    bus.write_byte_data(COMPASS_ADDRESS, 0x02, 0x00)
    time.sleep(0.1)
    print("Compass initialized.")

def read_compass_data(addr):
    high = bus.read_byte_data(COMPASS_ADDRESS, addr)
    low = bus.read_byte_data(COMPASS_ADDRESS, addr+1)
    value = (high << 8) | low
    if value > 32767:
        value = value - 65536
    return value

def get_compass_heading():
    x = read_compass_data(0x03)
    z = read_compass_data(0x05)
    y = read_compass_data(0x07)
    heading_rad = math.atan2(y, x)
    if heading_rad < 0:
        heading_rad += 2 * math.pi
    heading_deg = math.degrees(heading_rad)
    return heading_deg

def heading_difference(current, target):
    diff = current - target
    while diff < -180:
        diff += 360
    while diff > 180:
        diff -= 360
    return diff

############################################################
#                   ULTRASONIC FUNCTIONS                   #
############################################################

def send_command(command):
    cmd = command.strip() + "\n"
    ser.write(cmd.encode('utf-8'))
    print("Sent:", command)

def read_response():
    while ser.in_waiting:
        line = ser.readline().decode('utf-8').strip()
        if line:
            print("Arduino:", line)

def measure_distance(trig_pin, echo_pin):
    GPIO.output(trig_pin, False)
    time.sleep(0.0002)
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)

    start_time = time.time()
    timeout = start_time + 0.04 
    while GPIO.input(echo_pin) == 0:
        start_time = time.time()
        if start_time > timeout:
            return None

    end_time = time.time()
    timeout = end_time + 0.04
    while GPIO.input(echo_pin) == 1:
        end_time = time.time()
        if end_time > timeout:
            return None

    pulse_duration = end_time - start_time
    distance = pulse_duration * 34300 / 2.0
    return distance

def get_ultrasonic_distances():
    left = measure_distance(TRIG_LEFT, ECHO_LEFT)
    right = measure_distance(TRIG_RIGHT, ECHO_RIGHT)
    return (left, right)

def path_is_clear(left_dist, right_dist, threshold=CLEAR_DISTANCE):
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
        send_command("pivotR") # turn R
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
    #     Score = (left + right) - (penalty * abs(left - right))
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

############################################################
#                       MAIN LOOP                          #
############################################################
def main():
    # Initialize compass (added)
    init_compass()
    
    try:
        while True:
            # drive forward initial burst
            send_command("driveF")
            time.sleep(2.0)
            send_command("S")
            read_response()
            time.sleep(0.5)
            
            # perform ultrasonic sweep scan
            print("Performing servo sweep scan...")
            best_angle = servo_sweep_scan()
            
            # pivot to best angle based on ultrasonic scan
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
            
            # SET TARGET HEADING BEFORE MOVING
            target_heading = get_compass_heading()
            print("New target heading set to:", target_heading)
            
            # dsp - drive straight with heading stabilization
            drive_duration = 3.0  # adjust as needed
            start_drive = time.time()
            while time.time() - start_drive < drive_duration:
                current_heading = get_compass_heading()
                diff = heading_difference(current_heading, target_heading)
                print(f"Driving straight: Current Heading: {current_heading:.1f}°, Target: {target_heading:.1f}°, Diff: {diff:.1f}°")
                if abs(diff) > HEADING_THRESHOLD:
                    adjust_time = STEERING_TIME * (abs(diff) / HEADING_THRESHOLD)
                    if diff > 0:
                        send_command("steerL")
                        time.sleep(adjust_time)
                        send_command("steerZ")
                        print(f"Adjusting: drifting right, steering left for {adjust_time:.2f} sec.")
                    elif diff < 0:
                        send_command("steerR")
                        time.sleep(adjust_time)
                        send_command("steerZ")
                        print(f"Adjusting: drifting left, steering right for {adjust_time:.2f} sec.")
                send_command("driveF")
                read_response()
                time.sleep(0.5)
            
            # after dsp, loop to rescan
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        GPIO.cleanup()
        send_command("S")
        ser.close()

if __name__ == "__main__":
    main()
