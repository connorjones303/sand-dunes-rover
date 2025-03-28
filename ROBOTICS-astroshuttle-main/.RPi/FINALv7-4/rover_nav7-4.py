import RPi.GPIO as GPIO
import time
import serial

# GPIO PINS
TRIG_LEFT = 17
ECHO_LEFT = 27
TRIG_RIGHT = 22
ECHO_RIGHT = 23

# SERIAL
SERIAL_PORT = '/dev/ttyUSB0'  # e.g., '/dev/ttyACM0', '/dev/ttyUSB0', etc.
BAUD_RATE = 9600

# DISTANCE THRESHOLD
CLEAR_DISTANCE = 25.0

# Pivot increments: # more steps = finer search, but slower
PIVOT_STEPS = 10 

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
    Send a simple command string to the Arduino (e.g. 'driveF', 'pivotL', etc.).
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
    Measure distance (in cm) using an HC-SR04 or similar ultrasonic sensor.
    Returns the distance in cm or None on error.
    """
    # send a 10us pulse to TRIG
    GPIO.output(trig_pin, False)
    time.sleep(0.0002)
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)

    # ECHO go high (start time)
    start_time = time.time()
    timeout = start_time + 0.04 
    while GPIO.input(echo_pin) == 0:
        start_time = time.time()
        if start_time > timeout:
            return None

    # ECHO go low (end time)
    end_time = time.time()
    timeout = end_time + 0.04
    while GPIO.input(echo_pin) == 1:
        end_time = time.time()
        if end_time > timeout:
            return None

    # calc pulse length
    pulse_duration = end_time - start_time
    distance = pulse_duration * 34300 / 2.0
    return distance

def get_ultrasonic_distances():
    """
    Returns a tuple (left_dist, right_dist) in cm.
    If a sensor read fails, returns (None, None) or one None if only one fails.
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

def pivot_search():
    """
    Pivot in small increments left or right to find the angle that gives
    the largest combined distance (left + right).
    We'll do PIVOT_STEPS increments to the left, measuring each time,
    and similarly to the right, then pick the best angle.
    """
    best_sum = 0.0
    best_direction = 0  # negative = left, positive = right, 0 = none
    steps_taken = 0

    # 1) Pivot left in small increments
    for i in range(PIVOT_STEPS):
        send_command("pivotL")  # pivot a small step left
        time.sleep(0.3)         # give it time to move a bit
        left, right = get_ultrasonic_distances()
        read_response()
        if left is not None and right is not None:
            current_sum = left + right
            if current_sum > best_sum:
                best_sum = current_sum
                best_direction = -(i+1)  # store how many left pivots we did
        # Stop after each small pivot
        send_command("S")
        time.sleep(0.2)

    # 2) Reset to original orientation
    # pivot right the same number of steps we pivoted left
    for i in range(PIVOT_STEPS):
        send_command("pivotR")
        time.sleep(0.3)
        send_command("S")
        time.sleep(0.2)

    # 3) Pivot right in small increments
    for i in range(PIVOT_STEPS):
        send_command("pivotR")
        time.sleep(0.3)
        left, right = get_ultrasonic_distances()
        read_response()
        if left is not None and right is not None:
            current_sum = left + right
            if current_sum > best_sum:
                best_sum = current_sum
                best_direction = (i+1)  # store how many right pivots we did
        send_command("S")
        time.sleep(0.2)

    # 4) Move back to best angle
    if best_direction < 0:
        # pivot left best_direction times
        steps_needed = abs(best_direction)
        for _ in range(steps_needed):
            send_command("pivotL")
            time.sleep(0.3)
            send_command("S")
            time.sleep(0.2)
    elif best_direction > 0:
        steps_needed = best_direction
        for _ in range(steps_needed):
            send_command("pivotR")
            time.sleep(0.3)
            send_command("S")
            time.sleep(0.2)
    # else best_direction == 0 => no pivot

def main():
    """
    Main loop:
    1. Measure left and right distances
    2. If path is clear, drive forward a bit, then stop
    3. If not clear, pivot search to find best angle, then drive
    """
    try:
        while True:
            left_dist, right_dist = get_ultrasonic_distances()
            read_response()  # read any Arduino messages
            print(f"Left: {left_dist:.1f} cm, Right: {right_dist:.1f} cm")

            if path_is_clear(left_dist, right_dist):
                print("Path is clear, driving forward...")
                send_command("driveF")
                time.sleep(1.0)  # drive forward for 1 second
                send_command("S")
            else:
                print("Path blocked, searching for clear angle...")
                pivot_search()
                # after pivot_search, presumably we are at best angle
                # try driving again
                send_command("driveF")
                time.sleep(1.0)
                send_command("S")

            time.sleep(1.0)  # small pause before next check
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        GPIO.cleanup()
        send_command("S")
        ser.close()

if __name__ == "__main__":
    main()
