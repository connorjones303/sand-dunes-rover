
#8-2 is written for HMC5883L

import RPi.GPIO as GPIO
import time
import serial
import smbus
import math

#custom variables (to be tested and adjusted)
steering_time = 0.4
BUTTON_PIN = 4

# HMC5883L I2C address
HMC5883L_ADDRESS = 0x1E

# Arduino serial config
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

# STEERING THRESHOLD
HEADING_THRESHOLD = 5.0

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

# initialize compass
def init_compass():
    # Configuration Register A: 8-average, 15 Hz, normal measurement
    bus.write_byte_data(HMC5883L_ADDRESS, 0x00, 0x70)
    # Configuration Register B: Gain = 5
    bus.write_byte_data(HMC5883L_ADDRESS, 0x01, 0xA0)
    bus.write_byte_data(HMC5883L_ADDRESS, 0x02, 0x00)
    time.sleep(0.1)

########################################################
#                       FUNCTIONS                      #
########################################################

# interprets data from compass
def read_raw_data(addr):
    high = bus.read_byte_data(HMC5883L_ADDRESS, addr)
    low = bus.read_byte_data(HMC5883L_ADDRESS, addr+1)
    value = (high << 8) | low
    if value > 32767:
        value = value - 65536
    return value

# reads data and calculates heading
def get_heading():
    x = read_raw_data(0x03)
    z = read_raw_data(0x05)
    y = read_raw_data(0x07)
    # gpt assisted calcs
    heading_rad = math.atan2(y, x)
    if heading_rad < 0:
        heading_rad += 2 * math.pi
    # converting to degrees
    heading_deg = math.degrees(heading_rad)
    return heading_deg

# compares heading values
def heading_difference(current, target):
    diff = current - target
    while diff < -180:
        diff += 360
    while diff > 180:
        diff -= 360
    return diff

# for sending outputs to Arduino
def send_command(command):
    cmd = command.strip() + "\n"
    ser.write(cmd.encode('utf-8'))
    print("Sent:", command)

# read output from Arduino
def read_response():
    while ser.in_waiting:
        line = ser.readline().decode('utf-8').strip()
        if line:
            print("Arduino:", line)
            
            
            

########################################################
#                     MAIN PROGRAM                     #
########################################################

# main loop/program
def main():
    # initialize compass
    init_compass()
    print("Compass initialized.")

     # wait for button press to start rover movement
    print("Waiting for button press to start.")

    # on button press, start rover/program
    while GPIO.input(BUTTON_PIN):
        time.sleep(0.1)
    print("Button pressed. Starting the rover.")

    # save starting heading and save to = target_heading = start_heading
    start_heading = get_heading()
    target_heading = get_heading()
    print("Target heading set to:", target_heading)
    
    try:
        while True:
            # get and save current_heading
            current_heading = get_heading()

            # calculate difference
            diff = heading_difference(current_heading, target_heading)
            print(f"Current Heading: {current_heading:.1f}°, Target: {target_heading:.1f}°, Diff: {diff:.1f}°")
            
            #compare headings to threshold
            if abs(diff) > HEADING_THRESHOLD:
                # scale the steering_time based off of how off course we are
                scaled_time = steering_time * (abs(diff) / HEADING_THRESHOLD)   

                if diff > 0:
                    # if current_heading is greater than target_heading, rover is drifting right, send commands to steerL
                    send_command("steerL")
                    time.sleep(scaled_time)
                    send_command("steerZ")
                    print("Drifting right, steering left.")
                elif diff < 0:
                    # if current_heading is less than target_heading, rover is drifting left, send commands to steerR
                    send_command("steerR")
                    time.sleep(scaled_time)
                    send_command("steerZ")
                    print("Drifting left, steering right.")

            else:
                # if within threshold, center the steering
                send_command("steerZ")
                print("Heading is on target; centering steering.")

            # drive forward command (repeats as rover never stops)            
            send_command("driveF")
            read_response()
            
            # delay before checking again
            time.sleep(0.5)
    

    # on keyboard input, stop rover and shutdown script
    except KeyboardInterrupt:
        send_command("S")
        print("Exiting...")

    finally:
        GPIO.cleanup()
        send_command("S") 
        ser.close()

if __name__ == "__main__":
    main()
