
#dsp8-4 uses BOTH the HMC5883L + MPU6050

import RPi.GPIO as GPIO
import time
import serial
import smbus
import math

# custom variables (to be tested and adjusted)
steering_time = 0.4
BUTTON_PIN = 4

# Sensor I2C addresses
HMC5883L_ADDRESS = 0x1E   # Magnetometer
MPU6050_ADDRESS = 0x68    # MPU6050 Gyro/Accel

# Arduino serial config
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

# STEERING THRESHOLD
HEADING_THRESHOLD = 5.0

# Global variables for MPU6050 gyro integration
heading_value = 0.0         # integrated gyro heading (in degrees)
last_update_time = time.time()  # time of last gyro update

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
#                Sensor Initialization               #
########################################################

def init_compass():
    """ Initialize the HMC5883L (magnetometer) """
    # Configuration Register A: 8-average, 15 Hz, normal measurement
    bus.write_byte_data(HMC5883L_ADDRESS, 0x00, 0x70)
    # Configuration Register B: Gain = 5
    bus.write_byte_data(HMC5883L_ADDRESS, 0x01, 0xA0)
    # Mode Register: Continuous measurement mode
    bus.write_byte_data(HMC5883L_ADDRESS, 0x02, 0x00)
    time.sleep(0.1)
    print("HMC5883L (magnetometer) initialized.")

def init_mpu6050():
    """ Initialize the MPU6050 (gyro) """
    # Wake up MPU6050 (it starts in sleep mode)
    bus.write_byte_data(MPU6050_ADDRESS, 0x6B, 0)
    time.sleep(0.1)
    print("MPU6050 initialized.")

########################################################
#                 Magnetometer Functions             #
########################################################

def read_raw_data(addr):
    high = bus.read_byte_data(HMC5883L_ADDRESS, addr)
    low = bus.read_byte_data(HMC5883L_ADDRESS, addr+1)
    value = (high << 8) | low
    if value > 32767:
        value = value - 65536
    return value

def get_mag_heading():
    x = read_raw_data(0x03)
    z = read_raw_data(0x05)
    y = read_raw_data(0x07)
    heading_rad = math.atan2(y, x)
    if heading_rad < 0:
        heading_rad += 2 * math.pi
    heading_deg = math.degrees(heading_rad)
    return heading_deg

########################################################
#                 Gyroscope Functions                #
########################################################

def get_gyro_z():
    """
    Read raw gyro Z-axis data from the MPU6050.
    Gyro registers for Z-axis are 0x47 (high byte) and 0x48 (low byte).
    """
    high = bus.read_byte_data(MPU6050_ADDRESS, 0x47)
    low = bus.read_byte_data(MPU6050_ADDRESS, 0x48)
    value = (high << 8) | low
    if value > 32767:
        value = value - 65536
    return value

def get_gyro_heading():
    """
    Integrate the MPU6050 gyro Z-axis reading to compute a relative heading.
    Returns heading in degrees (0-360).
    """
    global heading_value, last_update_time
    current_time = time.time()
    dt = current_time - last_update_time
    last_update_time = current_time
    gyro_z_raw = get_gyro_z()
    # For MPU6050 at ±250°/s, sensitivity is ~131 LSB/(°/s)
    gyro_z = gyro_z_raw / 131.0
    heading_value += gyro_z * dt
    heading_value %= 360
    return heading_value

########################################################
#                 Fused Heading Function             #
########################################################

def get_fused_heading():
    """
    Combine the magnetometer and gyro headings using a complementary filter.
    The fused heading is computed as:
         fused = alpha * gyro_heading + (1 - alpha) * mag_heading
    """
    alpha = 0.98  # weight for gyro (high frequency), 1-alpha for magnetometer (low frequency)
    gyro_heading = get_gyro_heading()
    mag_heading = get_mag_heading()
    # Simple linear combination (note: this may need additional handling near wrap-around)
    fused = alpha * gyro_heading + (1 - alpha) * mag_heading
    fused %= 360
    return fused

########################################################
#                 Helper and Command Functions         #
########################################################

def heading_difference(current, target):
    """
    Calculate the smallest difference between current and target headings,
    normalized to -180° to +180°.
    """
    diff = current - target
    while diff < -180:
        diff += 360
    while diff > 180:
        diff -= 360
    return diff

def send_command(command):
    """ Send a command to the Arduino """
    cmd = command.strip() + "\n"
    ser.write(cmd.encode('utf-8'))
    print("Sent:", command)

def read_response():
    """ Read output from the Arduino """
    while ser.in_waiting:
        line = ser.readline().decode('utf-8').strip()
        if line:
            print("Arduino:", line)

########################################################
#                     MAIN PROGRAM                   #
########################################################

def main():
    global heading_value, last_update_time

    # Initialize both sensors
    init_compass()
    init_mpu6050()

    # Wait for button press to start rover movement
    print("Waiting for button press to start.")
    while GPIO.input(BUTTON_PIN):  # button is active LOW
        time.sleep(0.1)
    print("Button pressed. Starting the rover.")

    # Save starting fused heading as target heading
    start_heading = get_fused_heading()
    target_heading = start_heading
    print("Target heading set to:", target_heading)

    try:
        while True:
            current_heading = get_fused_heading()
            diff = heading_difference(current_heading, target_heading)
            print(f"Current Heading: {current_heading:.1f}°, Target: {target_heading:.1f}°, Diff: {diff:.1f}°")

            # Compare heading difference to threshold and adjust steering
            if abs(diff) > HEADING_THRESHOLD:
                adjust_time = steering_time * (abs(diff) / HEADING_THRESHOLD)  # variable steering time
                if diff > 0:
                    send_command("steerL")
                    time.sleep(adjust_time)
                    send_command("steerZ")
                    print(f"Drifting right, steering left for {adjust_time:.2f} seconds.")
                elif diff < 0:
                    send_command("steerR")
                    time.sleep(adjust_time)
                    send_command("steerZ")
                    print(f"Drifting left, steering right for {adjust_time:.2f} seconds.")
            else:
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
