import serial
import struct
import math
import time
import os
from datetime import datetime

# Open serial connection (replace with your port)
ser = serial.Serial(
    port='/dev/ttyUSB0', # or 'COM4' on Windows
    baudrate=230400,
    timeout=1
)

# FieldSize (Bytes)Description
# Header        1 Byte     Always 0x54 (start of packet)
# VerLen        1 Byte     Upper 3 bits: Packet type Lower 5 bits: number of Points in packet
# Speed         2 Bytes    Rotation speed (degrees/sec)
# Start Angle   2 Bytes    Start angle (0.01° increments)
# Data Points   3 * N Bytes Each point: Distance (2 Bytes) + Intensity (1 Byte)
# End Angle     2 Bytes    End angle (0.01° increments)
# Timestamp     2 Bytes    Timestamp (ms)
# CRC Check     1 Byte     Packet validation




# Create a timestamped filename for the output

# example:
# LiDAR Data Collection Started: 2025-03-23 15:42:31
# Format: timestamp,angle,distance,intensity,x,y
# 1711212151.324,352.45,342,127,328.4,-95.2
# 1711212151.324,354.78,356,135,352.1,-32.6
# 1711212151.324,357.12,321,142,320.3,-15.1
# 1711212151.324,359.76,298,118,298.0,-1.2
# 1711212151.324,2.34,315,125,314.7,12.9
# 1711212151.324,5.67,378,131,376.3,37.4
# 1711212151.324,8.93,402,103,397.3,62.5
# 1711212151.324,12.48,287,98,280.4,62.0
# 1711212151.324,15.76,352,112,338.7,95.6
# 1711212151.324,18.25,423,129,401.5,132.6
# 1711212151.324,22.54,289,87,267.3,110.5
# 1711212151.324,25.18,335,95,303.2,142.3
# 1711212151.324,28.63,358,107,313.8,172.0
# 1711212151.324,32.45,275,85,232.3,147.5
# 1711212151.324,35.92,312,93,252.7,183.1
# 1711212151.324,39.18,289,88,224.0,182.3
# 1711212151.324,43.25,347,112,252.6,237.5
# 1711212151.324,47.52,302,94,203.8,222.9
# 1711212151.324,52.16,324,88,198.9,255.5
# 1711212151.324,56.87,275,76,150.0,231.0
# 1711212151.755,312.45,365,116,243.3,-271.9
# 1711212151.755,315.78,342,124,240.4,-243.2
# 1711212151.755,319.12,298,135,222.8,-198.2
# 1711212151.755,322.76,387,128,303.7,-239.7
# 1711212151.755,325.34,412,112,336.7,-236.6
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
cartesian_output_file = f"cartesian_lidar_data_{timestamp}.txt"
raw_output_file = f"raw_lidar_data_{timestamp}.txt"


# The read function will read lidar_read packets into an array of cartesian points
def read_to_cartesian(lidar_read):

    # Extract start and end angles (in 0.01 degree increments)
    start_angle = struct.unpack('<H', lidar_read.data[2:4])[0] / 100.0
    end_angle = struct.unpack('<H', lidar_read.data[4+lidar_read.point_count*3:6+lidar_read.point_count*3])[0] / 100.0
    
    # Process each measurement point
    points = []
    if lidar_read.point_count == 1: # avoid divide by zero error
        return points
        
    for i in range(lidar_read.point_count):
        offset = 4 + i*3
        distance = struct.unpack('<H', lidar_read.data[offset:offset+2])[0] # in mm
        intensity = lidar_read.data[offset+2] # signal strength
        
        # Calculate angle for this point using linear interpolation
        angle = start_angle + (end_angle - start_angle) * (i / (lidar_read.point_count-1))
        
        # Convert polar coordinates to Cartesian (x,y) in mm
     
        angle_rad = math.radians(angle)
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        points.append((angle, distance, intensity, x, y))
    
    return points # [ (angle, distance, intensity, x, y), (angle, distance, intensity, x, y), ...]

# reads and returns lidar_read sensor packet bytes into object
def read():
    # Loop and Wait until we read a packet header (0x54)
    while True:
        if ser.read(1) == b'\x54':
            break
    
    # Read packet type and point count, tells us how many points will be in the packet
    ver_len = ord(ser.read(1))
    point_count = ver_len & 0x1F # Lower 5 bits indicate point count
    
    # Read remaining packet data
    data = ser.read(6 + point_count*3 + 4) # Speed(2) + StartAngle(2) + Data(n*3) + EndAngle(2) + Timestamp(2) + CRC(1)
    
    # Extract speed (degrees/sec)
    speed = struct.unpack('<H', data[0:2])[0]
    
    lidar_data = {
        point_count: point_count,
        ver_len: ver_len,
        data: data,
        speed: speed
    }

    return lidar_data

def record():
    print(f"Writing LiDAR data to {raw_output_file}")
    print(f"Writing LiDAR data to {cartesian_output_file}")
    print("Press Ctrl+C to stop...")
    
    date_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    # Write header to file
    with open(raw_output_file, 'w') as f:
        f.write("# Raw LiDAR Data Collection Started: " + date_time + "\n")
        f.write("# Format: timestamp,angle,distance,intensity,x,y\n")
    # Write header to file
    with open(cartesian_output_file, 'w') as f:
        f.write("# Cartesain LiDAR Data Collection Started: " + date_time + "\n")
        f.write("# Format: timestamp,angle,distance,intensity,x,y\n")
    
    scan_count = 0
    while True:
        scan_count += 1
        current_time = time.time()
        lidar_read = read()
        cartesian_points = read_to_cartesian(lidar_read)
        
        if len(cartesian_points) > 0:
        # Only print summary to console to avoid flooding
            print(f"Scan #{scan_count}: Got {len(cartesian_points)} points, saving to {cartesian_output_file}")
        
        # Append data to the file
        with open(cartesian_output_file, 'a') as f:
            for angle, distance, intensity, x, y in cartesian_points:
                # Write a formatted line for each point
                f.write(f"{current_time:.3f},{angle:.2f},{distance},{intensity},{x:.1f},{y:.1f}\n")
        # Optional: Add a delay between scans if needed
        # time.sleep(0.1)
def stop_record():
    print(f"Data collection stopped. Data saved to {raw_output_file}")
    ser.close()