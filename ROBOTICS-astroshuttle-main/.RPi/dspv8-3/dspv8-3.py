
#8-3 is written for MPU6050

import RPi.GPIO as GPIO
import time
import serial
import smbus
import math

# custom variables (to be tested and adjusted)
steering_time = 0.4
BUTTON_PIN = 4

# MPU6050 I2C address (typically 0x68)
MPU6050_ADDRESS = 0x68

# Arduino serial config
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

# STEERING THRESHOLD
HEADING_THRESHOLD = 5.0

# Global variables for gyro integration
heading_value = 0.0         # integrated heading (in degrees)
last_update_time = time.time()  # used for integration time delta

# initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# initialize serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print("Serial connection established on", SERIAL_PORT)
except Exception as e:
    print("Failed to open serial port:", e)
    exit()

# initialize I2C
bus = smbus.SMBus(1)

########################################################
#                  MPU6050 FUNCTIONS                   #
########################################################

def init_mpu6050():
    """
    Initialize the MPU6050.
    Wake up the MPU6050 (it starts in sleep mode).
    """
    # Write to power management register 0x6B to wake up MPU6050.
    bus.write_byte_data(MPU6050_ADDRESS, 0x6B, 0)
    time.sleep(0.1)
    # (Optionally, you can configure gyro settings here if needed.)
    print("MPU6050 initialized.")

def get_gyro_z():
    """
    Reads the raw gyro Z-axis data from the MPU6050.
    The gyro registers for Z-axis are 0x47 (high byte) and 0x48 (low byte).
    """
    high = bus.read_byte_data(MPU6050_ADDRESS, 0x47)
    low = bus.read_byte_data(MPU6050_ADDRESS, 0x48)
    value = (high << 8) | low
    if value > 32767:
        value = value - 65536
    return value

def get_heading():
    """
    Integrates the gyro Z-axis reading to calculate a relative heading (yaw).
    Returns the heading in degrees (0-360).
    """
    global heading_value, last_update_time
    current_time = time.time()
    dt = current_time - last_update_time
    last_update_time = current_time

    # Get raw gyro Z value and convert it to degrees per second.
    # For MPU6050 set to ±250°/s, the sensitivity is ~131 LSB/(°/s).
    gyro_z_raw = get_gyro_z()
    gyro_z = gyro_z_raw / 131.0  # convert raw reading to °/s

    # Integrate the gyro reading over time to update heading.
    heading_value += gyro_z * dt
    heading_value %= 360  # normalize to 0-360 degrees
    return heading_value

########################################################
#                    HELPER FUNCTIONS                  #
########################################################

def heading_difference(current, target):
    """
    Calculates the smallest difference between current and target heading,
    normalized to be within -180° to +180°.
    """
    diff = current - target
    while diff < -180:
        diff += 360
    while diff > 180:
        diff -= 360
    return diff

def send_command(command):
    """
    Send a command string to the Arduino.
    Example commands: 'driveF', 'steerL', 'steerR', 'steerZ', 'S'.
    """
    cmd = command.strip() + "\n"
    ser.write(cmd.encode('utf-8'))
    print("Sent:", command)

def read_response():
    """
    Read any available output from the Arduino.
    """
    while ser.in_waiting:
        line = ser.readline().decode('utf-8').strip()
        if line:
            print("Arduino:", line)

########################################################
#                    MAIN PROGRAM                      #
########################################################

def main():
    global heading_value, last_update_time  # ensure global variables are used

    # Initialize MPU6050 (instead of HMC5883L)
    init_mpu6050()

    # wait for button press to start rover movement
    print("Waiting for button press to start.")
    while GPIO.input(BUTTON_PIN):  # waiting for the button (active LOW)
        time.sleep(0.1)
    print("Button pressed. Starting the rover.")

    # Save starting heading and use it as target_heading.
    # (The initial integrated heading may be arbitrary; you could also force it to zero if desired.)
    start_heading = get_heading()  # first reading becomes the target.
    target_heading = start_heading
    print("Target heading set to:", target_heading)
    
    try:
        while True:
            # Update and get current heading via MPU6050 integration.
            current_heading = get_heading()
            diff = heading_difference(current_heading, target_heading)
            print(f"Current Heading: {current_heading:.1f}°, Target: {target_heading:.1f}°, Diff: {diff:.1f}°")
            
            # Compare headings to threshold
            if abs(diff) > HEADING_THRESHOLD:
                # Scale steering duration based on how far off course we are.
                adjust_time = steering_time * (abs(diff) / HEADING_THRESHOLD)
                if diff > 0:
                    # Drifting right: steer left.
                    send_command("steerL")
                    time.sleep(adjust_time)
                    send_command("steerZ")
                    print(f"Drifting right, steering left for {adjust_time:.2f} seconds.")
                elif diff < 0:
                    # Drifting left: steer right.
                    send_command("steerR")
                    time.sleep(adjust_time)
                    send_command("steerZ")
                    print(f"Drifting left, steering right for {adjust_time:.2f} seconds.")
            else:
                # Within threshold: center steering.
                send_command("steerZ")
                print("Heading is on target; centering steering.")

            # Drive forward command (rover never stops)
            send_command("driveF")
            read_response()
            time.sleep(0.5)
    
    except KeyboardInterrupt:
        send_command("S")
        print("Exiting...")
    
    finally:
        GPIO.cleanup()
        send_command("S")
        ser.close()

if __name__ == "__main__":
    main()
