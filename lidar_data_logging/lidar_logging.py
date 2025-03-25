import serial
import struct
import math
import time
from datetime import datetime
from binary_sensor_classes import BinarySensorLogger

class LidarReader:
    """
    A class to read and process LiDAR data from a serial connection.
    Supports binary and Cartesian output formats.
    """
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=230400, timeout=1):
        """
        Initialize the LiDAR reader with serial connection parameters.
        
        Args:
            port (str): Serial port (e.g., '/dev/ttyUSB0' or 'COM4')
            baudrate (int): Baud rate for serial connection
            timeout (int): Serial read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.scan_count = 0
        
        # File outputs
        self.cartesian_file = None
        self.binary_logger = None
    
    def connect(self):
        """Establish serial connection to the LiDAR."""
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout
        )
        return self
    
    def disconnect(self):
        """Close the serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed.")
    
    def setup_logging(self, enable_cartesian=True, enable_binary=True):
        """
        Set up file logging for LiDAR data.
        
        Args:
            enable_cartesian (bool): Whether to log Cartesian coordinates as text
            enable_binary (bool): Whether to log raw binary data
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        date_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        if enable_cartesian:
            cartesian_output_file = f"cartesian_lidar_data_{timestamp}.txt"
            self.cartesian_file = open(cartesian_output_file, 'w')
            self.cartesian_file.write(f"# Cartesian LiDAR Data Collection Started: {date_time}\n")
            self.cartesian_file.write("# Format: timestamp,angle,distance,intensity,x,y\n")
            print(f"Writing Cartesian LiDAR data to {cartesian_output_file}")
        
        if enable_binary:
            binary_output_file = f"binary_lidar_data_{timestamp}.bin"
            self.binary_logger = BinarySensorLogger(binary_output_file)
            self.binary_logger.open(version=1)
            print(f"Writing binary LiDAR data to {binary_output_file}")
        
        return self
    
    def close_logs(self):
        """Close any open log files."""
        if self.cartesian_file:
            self.cartesian_file.close()
            self.cartesian_file = None
            
        if self.binary_logger:
            self.binary_logger.close()
            self.binary_logger = None
    
    def read_packet(self):
        """
        Read a single LiDAR packet from the serial connection.
        
        Returns:
            dict: A dictionary containing packet data
        """
        if not self.ser or not self.ser.is_open:
            raise ConnectionError("Serial connection not established. Call connect() first.")
        
        # Loop until we read a packet header (0x54)
        while True:
            header_byte = self.ser.read(1)
            if header_byte == b'\x54':
                break
        
        timestamp = time.time()
        # Read packet type and point count
        ver_len_byte = self.ser.read(1)
        ver_len = ord(ver_len_byte)
        point_count = ver_len & 0x1F  # Lower 5 bits indicate point count
        
        # Read remaining packet data
        data = self.ser.read(6 + point_count*3 + 4)  # Speed(2) + StartAngle(2) + Data(n*3) + EndAngle(2) + Timestamp(2) + CRC(1)
        
        # Extract speed (degrees/sec)
        speed = struct.unpack('<H', data[0:2])[0]
        
        # Create packet object
        packet = {
            'point_count': point_count,
            'ver_len': ver_len,
            'data': data,
            'speed': speed,
            'timestamp': timestamp,
            'raw_data': header_byte + ver_len_byte + data  # Store complete raw packet
        }
        
        return packet
    
    def packet_to_cartesian(self, packet):
        """
        Convert a LiDAR packet to Cartesian coordinates.
        
        Args:
            packet (dict): LiDAR packet data
            
        Returns:
            list: Points as (angle, distance, intensity, x, y)
        """
        # Extract start and end angles (in 0.01 degree increments)
        start_angle = struct.unpack('<H', packet['data'][2:4])[0] / 100.0
        end_angle = struct.unpack('<H', packet['data'][4+packet['point_count']*3:6+packet['point_count']*3])[0] / 100.0
        
        # Process each measurement point
        points = []
        if packet['point_count'] <= 1:  # avoid divide by zero error
            return points
            
        for i in range(packet['point_count']):
            offset = 4 + i*3
            distance = struct.unpack('<H', packet['data'][offset:offset+2])[0]  # in mm
            intensity = packet['data'][offset+2]  # signal strength
            
            # Calculate angle for this point using linear interpolation
            angle = start_angle + (end_angle - start_angle) * (i / (packet['point_count']-1))
            
            # Convert polar coordinates to Cartesian (x,y) in mm
            angle_rad = math.radians(angle)
            x = distance * math.cos(angle_rad)
            y = distance * math.sin(angle_rad)
            points.append((angle, distance, intensity, x, y))
        
        return points
    
    def read_and_process(self, print_to_console=True):
        """
        Read a packet and process it, logging to configured outputs.
        
        Args:
            print_to_console (bool): Whether to print summary to console
            
        Returns:
            tuple: (raw_packet, cartesian_points)
        """
        self.scan_count += 1
        packet = self.read_packet()
        
        # Log binary data if enabled
        if self.binary_logger:
            self.binary_logger.write_packet(packet['raw_data'], packet['timestamp'])
        
        # Convert to Cartesian if needed for logging or return
        cartesian_points = self.packet_to_cartesian(packet) if (self.cartesian_file or print_to_console) else None
        
        # Log to Cartesian file if enabled
        if self.cartesian_file and cartesian_points:
            current_time = packet['timestamp']
            for angle, distance, intensity, x, y in cartesian_points:
                self.cartesian_file.write(f"{current_time:.3f},{angle:.2f},{distance},{intensity},{x:.1f},{y:.1f}\n")
            self.cartesian_file.flush()
        
        # Print to console if enabled
        if print_to_console:
            if cartesian_points:
                print(f"Scan #{self.scan_count}: Got {len(cartesian_points)} points at {packet['speed']} deg/sec")
            else:
                print(f"Scan #{self.scan_count}: Got packet with {packet['point_count']} points")
        
        return packet, cartesian_points
    
    def continuous_read(self, duration=None):
        """
        Continuously read LiDAR data until interrupted.
        
        Args:
            duration (float): Optional duration in seconds, None for indefinite
        """
        start_time = time.time()
        try:
            print("Reading LiDAR data, press Ctrl+C to stop...")
            while True:
                self.read_and_process()
                
                # Check if duration has elapsed
                if duration and (time.time() - start_time) > duration:
                    print(f"Duration of {duration} seconds reached.")
                    break
                
        except KeyboardInterrupt:
            print("Reading stopped by user.")
        finally:
            self.close_logs()
    
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close_logs()
        self.disconnect()


# Example usage
if __name__ == "__main__":
    # Basic usage with both logging types enabled
    with LidarReader(port='/dev/ttyUSB0').connect() as lidar:
        lidar.setup_logging(enable_cartesian=True, enable_binary=True)
        lidar.continuous_read(duration=10)  # Read for 10 seconds
    
    # Using only binary logging
    """
    with LidarReader(port='/dev/ttyUSB0').connect() as lidar:
        lidar.setup_logging(enable_cartesian=False, enable_binary=True)
        lidar.continuous_read(duration=10)
    """
    
    # Using only Cartesian text output
    """
    with LidarReader(port='/dev/ttyUSB0').connect() as lidar:
        lidar.setup_logging(enable_cartesian=True, enable_binary=False)
        lidar.continuous_read(duration=10)
    """