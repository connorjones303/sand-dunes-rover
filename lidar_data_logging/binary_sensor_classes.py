import struct
import time
import datetime


class BinarySensorLogger:
    """
    Logger for raw sensor data with binary timestamp headers.
    
    File Format:
    - 8 bytes: File header "SENSORLOG"
    - 2 bytes: Version (uint16)
    - 4 bytes: Reserved
    
    For each record:
    - 8 bytes: Timestamp (double, seconds since epoch)
    - 4 bytes: Data length (uint32)
    - N bytes: Raw sensor data
    """
    
    def __init__(self, filename):
        """Initialize the logger with a filename."""
        self.filename = filename
        self.file = None
        
    def open(self, version=1):
        """Open the file and write the header."""
        self.file = open(self.filename, 'wb')
        
        # Write file header
        self.file.write(b'SENSORLOG')
        
        # Write version as uint16
        self.file.write(struct.pack('<H', version))
        
        # Write reserved bytes
        self.file.write(b'\x00\x00\x00\x00')
        
        return self
        
    def write_packet(self, data, timestamp=None):
        """Write a sensor data packet with current timestamp."""
        if not self.file or self.file.closed:
            raise IOError("File not open")
            
        # Get current timestamp if not provided
        if timestamp is None:
            timestamp = time.time()
            
        # Write timestamp as double
        self.file.write(struct.pack('<d', timestamp))
        
        # Write data length as uint32
        self.file.write(struct.pack('<I', len(data)))
        
        # Write the raw data
        self.file.write(data)
        
        # Flush to ensure data is written
        self.file.flush()
        
        return timestamp
        
    def close(self):
        """Close the file."""
        if self.file and not self.file.closed:
            self.file.close()
            
    def __enter__(self):
        """Context manager entry."""
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()


class BinarySensorReader:
    """Reader for binary sensor data files."""
    
    def __init__(self, filename):
        """Initialize the reader with a filename."""
        self.filename = filename
        self.file = None
        
    def open(self):
        """Open the file and read the header."""
        self.file = open(self.filename, 'rb')
        
        # Read and verify file header
        header = self.file.read(8)
        if header != b'SENSORLOG':
            raise ValueError("Invalid file format")
            
        # Read version
        version_bytes = self.file.read(2)
        self.version = struct.unpack('<H', version_bytes)[0]
        
        # Skip reserved bytes
        self.file.read(4)
        
        return self
        
    def read_packet(self):
        """Read the next sensor data packet.
        
        Returns:
            tuple: (timestamp, data) or None if end of file
        """
        if not self.file or self.file.closed:
            raise IOError("File not open")
            
        # Try to read timestamp
        timestamp_bytes = self.file.read(8)
        if len(timestamp_bytes) < 8:
            return None  # End of file
            
        # Read timestamp
        timestamp = struct.unpack('<d', timestamp_bytes)[0]
        
        # Read data length
        length_bytes = self.file.read(4)
        data_length = struct.unpack('<I', length_bytes)[0]
        
        # Read data
        data = self.file.read(data_length)
        
        return timestamp, data
        
    def read_all_packets(self):
        """Read all packets from the file."""
        packets = []
        while True:
            packet = self.read_packet()
            if packet is None:
                break
            packets.append(packet)
        return packets
        
    def close(self):
        """Close the file."""
        if self.file and not self.file.closed:
            self.file.close()
            
    def __enter__(self):
        """Context manager entry."""
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()


# Example usage
def demo_write_sensor_data():
    """Demo function to write sample sensor data."""
    # Simulated sensor reading function
    def read_sensor():
        # This would be your actual sensor reading code
        # For demo, we'll just create some random bytes
        import random
        return bytes([random.randint(0, 255) for _ in range(16)])
    
    # Write 10 sensor readings
    with BinarySensorLogger('sensor_data.bin').open() as logger:
        for _ in range(10):
            data = read_sensor()
            timestamp = logger.write_packet(data)
            # Print for demo purposes
            print(f"Wrote {len(data)} bytes at {datetime.datetime.fromtimestamp(timestamp)}")
            # Simulate delay between readings
            time.sleep(0.1)
    
    print("Data written to sensor_data.bin")


def demo_read_sensor_data():
    """Demo function to read the sample sensor data."""
    with BinarySensorReader('sensor_data.bin').open() as reader:
        packets = reader.read_all_packets()
        
    print(f"Read {len(packets)} packets:")
    for i, (timestamp, data) in enumerate(packets):
        dt = datetime.datetime.fromtimestamp(timestamp)
        print(f"Packet {i+1}: {dt} - {len(data)} bytes: {data.hex()}")


if __name__ == "__main__":
    print("Writing sample sensor data...")
    demo_write_sensor_data()
    
    print("\nReading sample sensor data...")
    demo_read_sensor_data()