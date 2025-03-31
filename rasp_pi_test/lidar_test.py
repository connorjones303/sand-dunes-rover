import serial
import struct
import math

from robot2.logger import logger # Nathan W: Implement logging (logs and prints to console in one statement)

# Nathan W: Set up variables as it was done in other files
PORT = '/dev/tty.usbserial-0001'  # or 'COM4' on Windows (replace with your port)
BAUD_RATE = 230400
TIMEOUT = 1

# Open serial connection
def initialize_serial(port=PORT, baudrate=BAUD_RATE, timeout=TIMEOUT): # Nathan W: Put this block of code in a function like other files
    try: # Nathan W: Added a try/catch for serial connection to exit gracefully
        ser = serial.Serial(
            port= port,
            baudrate=baudrate,
            timeout=timeout
        )
        return ser
    except Exception as e:
        logger.critical(f"Error opening serial port: {e}")
        exit(1)

def read_lidar_data(ser):
    # Loop and Wait until we read a packet header (0x54)
    while True:
        if ser.read(1) == b'\x54':
            break

    try: # Nathan W: Added a try/except for reading from the server
        # Read packet type and point count, tells us how many points will be in the packet99
        point_count = ord(ser.read(1)) & 0x1F # Lower 5 bits indicate point count (removed ver_len since it was only used for point_count)

        # Read remaining packet data
        data = ser.read(6 + point_count*3 + 4)  # Speed(2) + StartAngle(2) + Data(n*3) + EndAngle(2) + Timestamp(2) + CRC(1)
    except Exception as e:
        logger.critical(f"Error reading from serial port: {e}")
        exit(1)
    
    # Extract speed (degrees/sec)
    speed = struct.unpack('<H', data[0:2])[0]
    
    # Extract start and end angles (in 0.01 degree increments)
    start_angle = struct.unpack('<H', data[2:4])[0] / 100.0
    end_angle = struct.unpack('<H', data[4+point_count*3:6+point_count*3])[0] / 100.0

    # Process each measurement point
    points = []
    for i in range(point_count):
        offset = 4 + i*3
        distance = struct.unpack('<H', data[offset:offset+2])[0]  # in mm
        intensity = data[offset+2]  # signal strength
        
        # Calculate angle for this point using linear interpolation
        angle = start_angle + (end_angle - start_angle) * (i / (point_count-1))
        
        # Convert polar coordinates to Cartesian (x,y)
        if distance > 0:
            angle_rad = math.radians(angle)
            x = distance * math.cos(angle_rad)
            y = distance * math.sin(angle_rad)
            points.append((angle, distance, intensity, x, y))
    
    return points

def main(): # Nathan W: Added main method like other test files
    ser = initialize_serial()

    # Read and print 10 sets of measurements
    try:
        for _ in range(10):
            points = read_lidar_data(ser)
            logger.info(f"Got {len(points)} points")
            for angle, distance, intensity, x, y in points:
                logger.info(f"  Angle: {angle:.2f}°, Distance: {distance}mm, Intensity: {intensity}, x:{x}, y: {y}")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
