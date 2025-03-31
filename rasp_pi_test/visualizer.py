import serial
import struct
import math
import matplotlib.pyplot as plt
import numpy as np
import time
from matplotlib.animation import FuncAnimation
from collections import deque

from robot2.logger import logger # Nathan W: Implement logging (logs and prints to console in one statement)

# Nathan W: Set up variables as it was done in other files
PORT = '/dev/tty.usbserial-0001' # (replace with your port)
BAUD_RATE = 230400
TIMEOUT = 1

# Open serial connection
def initialize_serial(port=PORT, baudrate=BAUD_RATE, timeout=TIMEOUT): # Nathan W: Added default value for timeout
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout
        )
        return ser
    except serial.SerialException as e:
        logger.critical(f"Error opening serial port: {e}")
        return None # Nathan W: Is there a reason this would not exit?

def read_lidar_data(ser):
    # Loop and Wait until we read a packet header (0x54)
    while True:
        if ser.read(1) == b'\x54':
            break
    
    # Read packet type and point count
    ver_len = ord(ser.read(1))
    point_count = ver_len & 0x1F  # Lower 5 bits indicate point count
    
    # Read remaining packet data
    data = ser.read(6 + point_count*3 + 4)  # Speed(2) + StartAngle(2) + Data(n*3) + EndAngle(2) + Timestamp(2) + CRC(1)
    
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
        if point_count > 1:
            angle = start_angle + (end_angle - start_angle) * (i / (point_count-1))
        else:
            angle = start_angle
        
        # Convert polar coordinates to Cartesian (x,y)
        if distance > 0:
            adjusted_angle = angle - 90 # adjust angle so that y axis is the 0 degree point, to help with visualize orientation easier
            angle_rad = math.radians(adjusted_angle)
            x = distance * math.cos(angle_rad)
            y = distance * math.sin(angle_rad)
            points.append((angle, distance, intensity, x, y))
    
    return points

def take_snapshot(num_frames=100): # Nathan W: removed port parameter (not needed)
    """Collect several frames of LiDAR data and display them as a static snapshot"""
    ser = initialize_serial() # Nathan W: Removed port argument (same as default)
    if not ser:
        return
    
    all_points = []
    
    try:
        # Collect data from multiple frames
        for _ in range(num_frames):
            points = read_lidar_data(ser)
            all_points.extend(points)
            logger.info(f"Collected {len(points)} points in this frame")
    
    finally:
        ser.close()
    
    # Extract x and y coordinates
    x_coords = [p[3] for p in all_points]
    y_coords = [p[4] for p in all_points]
    intensities = [p[2] for p in all_points]
    
    # Create a scatter plot
    plt.figure(figsize=(10, 10))
    scatter = plt.scatter(x_coords, y_coords, c=intensities, cmap='viridis', s=10, alpha=0.7)
    plt.colorbar(scatter, label='Intensity')
    
    # Set equal aspect ratio to prevent distortion
    plt.axis('equal')
    plt.grid(True)
    plt.title(f'LiDAR Snapshot ({len(all_points)} points from {num_frames} frames)')
    plt.xlabel('X Position (mm)')
    plt.ylabel('Y Position (mm)')
    
    # Add range circles
    max_range = max([p[1] for p in all_points]) * 1.1  # 10% margin
    ranges = [max_range/4, max_range/2, 3*max_range/4, max_range]
    for r in ranges:
        circle = plt.Circle((0, 0), r, fill=False, color='gray', linestyle='--', alpha=0.5)
        plt.gca().add_patch(circle)
    
    plt.show()

def continuous_view(max_frames=100): # Nathan W: Removed port parameter (not needed)
    """Display LiDAR data in a continuously updating view"""
    ser = initialize_serial() # Nathan W: Removed port argument (same as default)
    if not ser:
        return
    
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Use deques to store a fixed number of points
    x_history = deque(maxlen=max_frames * 2)  # Assuming max 32 points per frame
    y_history = deque(maxlen=max_frames * 2)
    
    # Initial scatter plot
    scatter = ax.scatter([], [], s=10, alpha=0.5)
    
    # Set axis properties
    ax.set_xlim(-5000, 5000)  # 5 meters in each direction
    ax.set_ylim(-5000, 5000)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_title('LiDAR Real-time View')
    ax.set_xlabel('X Position (mm)')
    ax.set_ylabel('Y Position (mm)')
    
    # Add a circle at the center to represent the sensor
    sensor = plt.Circle((0, 0), 50, fill=True, color='red')
    ax.add_patch(sensor)
    
    # Add range circles
    ranges = [1000, 2000, 3000, 4000]
    for r in ranges:
        circle = plt.Circle((0, 0), r, fill=False, color='gray', linestyle='--', alpha=0.5)
        ax.add_patch(circle)
    
    def update(frame):
        try:
            points = read_lidar_data(ser)
            if points:
                # Extract coordinates
                x_coords = [p[3] for p in points]
                y_coords = [p[4] for p in points]
                
                # Add to history
                x_history.extend(x_coords)
                y_history.extend(y_coords)
                
                # Update scatter plot
                scatter.set_offsets(np.column_stack([list(x_history), list(y_history)]))
                
                # Update title with point count
                ax.set_title(f'LiDAR Real-time View ({len(x_history)} points)')
                
                return scatter,
        except Exception as e:
            logger.error(f"Error in update: {e}")
        
        return scatter,
    
    try:
        # Create animation
        ani = FuncAnimation(fig, update, frames=range(max_frames), interval=50, blit=True)
        plt.show()
    finally:
        ser.close()

def main():
    import argparse # Nathan W: Is there a reason this import is done at the function level
    
    parser = argparse.ArgumentParser(description='LiDAR Visualization Tool')
    parser.add_argument('--port', type=str, default=PORT, help='Serial port for LiDAR') # Nathan W: Could be removed I believe
    parser.add_argument('--mode', type=str, choices=['snapshot', 'continuous'], default='continuous',
                        help='Visualization mode: snapshot or continuous')
    parser.add_argument('--frames', type=int, default=10, help='Number of frames to capture for snapshot mode')
    
    args = parser.parse_args()
    
    if args.mode == 'snapshot':
        take_snapshot(num_frames=args.frames) # Nathan W: Removed port argument (same as default)
    else:
        continuous_view() # Nathan W: Removed port argument (same as default)

if __name__ == "__main__":
    main()
