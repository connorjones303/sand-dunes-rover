#!/usr/bin/env python3
"""
MPU-6050 Accelerometer/Gyroscope Interface for Raspberry Pi
- Uses standard I2C pins (SDA, SCL) on Raspberry Pi (Bus 1)
- Includes I2C bus scanning and error handling
"""

import smbus
import time
import sys

# MPU-6050 Registers (from datasheet)
MPU_ADDRESS = 0x68      # Default I2C address of MPU-6050
PWR_MGMT_1 = 0x6B       # Power management register
CONFIG = 0x1A           # Configuration register
GYRO_CONFIG = 0x1B      # Gyroscope configuration register
ACCEL_CONFIG = 0x1C     # Accelerometer configuration register

# Data registers
ACCEL_XOUT_H = 0x3B
ACCEL_XOUT_L = 0x3C
ACCEL_YOUT_H = 0x3D
ACCEL_YOUT_L = 0x3E
ACCEL_ZOUT_H = 0x3F
ACCEL_ZOUT_L = 0x40

GYRO_XOUT_H = 0x43
GYRO_XOUT_L = 0x44
GYRO_YOUT_H = 0x45
GYRO_YOUT_L = 0x46
GYRO_ZOUT_H = 0x47
GYRO_ZOUT_L = 0x48

# Sensitivity scale factors (adjust based on your configuration)
ACCEL_SCALE = 16384.0  # +/- 2g range (default)
GYRO_SCALE = 131.0     # +/- 250 deg/s range (default)


class SensorData:
    """Class to hold and display sensor data from MPU-6050"""
    
    def __init__(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z):
        self.accel_x = accel_x  # Acceleration in g
        self.accel_y = accel_y
        self.accel_z = accel_z
        self.gyro_x = gyro_x    # Angular velocity in deg/s
        self.gyro_y = gyro_y
        self.gyro_z = gyro_z
    
    def __str__(self):
        return (f"Accel (g): X={self.accel_x:.2f}, Y={self.accel_y:.2f}, Z={self.accel_z:.2f} | "
                f"Gyro (deg/s): X={self.gyro_x:.2f}, Y={self.gyro_y:.2f}, Z={self.gyro_z:.2f}")


class MPU6050:
    """MPU-6050 sensor interface"""
    
    def __init__(self, bus_num=1, address=MPU_ADDRESS):
        """Initialize the sensor with the specified bus and address"""
        self.address = address
        try:
            self.bus = smbus.SMBus(bus_num)
            print(f"Connected to I2C bus {bus_num}")
        except Exception as e:
            print(f"Error connecting to I2C bus {bus_num}: {str(e)}")
            sys.exit(1)
    
    def scan_bus(self):
        """Scan the I2C bus for devices and return list of addresses"""
        print("Scanning I2C bus...")
        devices = []
        for addr in range(128):
            try:
                self.bus.read_byte(addr)
                hex_addr = f"0x{addr:02X}"
                devices.append(addr)
                print(f"Device found at address: {hex_addr}")
            except Exception:
                pass
        
        if not devices:
            print("No I2C devices found! Check connections.")
        return devices
    
    def initialize(self):
        """Initialize the MPU-6050 sensor"""
        try:
            # Check if sensor is present
            devices = self.scan_bus()
            if self.address not in devices:
                print(f"MPU-6050 not found at address 0x{self.address:02X}!")
                print("Check connections and address settings.")
                sys.exit(1)
            
            # Wake up MPU-6050 (Power management register)
            self.bus.write_byte_data(self.address, PWR_MGMT_1, 0)
            
            # Set gyroscope range (+/- 250 deg/s)
            self.bus.write_byte_data(self.address, GYRO_CONFIG, 0)
            
            # Set accelerometer range (+/- 2g)
            self.bus.write_byte_data(self.address, ACCEL_CONFIG, 0)
            
            # Wait for sensor to stabilize
            time.sleep(0.1)
            print("MPU-6050 initialized successfully")
            
        except Exception as e:
            print(f"Error initializing MPU-6050: {str(e)}")
            sys.exit(1)
    
    def read_word(self, high_reg, low_reg):
        """Read a 16-bit value from two consecutive registers and return as signed int"""
        try:
            high = self.bus.read_byte_data(self.address, high_reg)
            low = self.bus.read_byte_data(self.address, low_reg)
            
            # Combine high and low bytes
            value = (high << 8) + low
            
            # Convert to signed value if negative
            if value >= 0x8000:
                value = value - 0x10000
                
            return value
        except Exception as e:
            print(f"Error reading from sensor: {str(e)}")
            return 0
    
    def get_sensor_data(self):
        """Read raw data from MPU-6050 and return a SensorData object"""
        try:
            # Read accelerometer data
            accel_x_raw = self.read_word(ACCEL_XOUT_H, ACCEL_XOUT_L)
            accel_y_raw = self.read_word(ACCEL_YOUT_H, ACCEL_YOUT_L)
            accel_z_raw = self.read_word(ACCEL_ZOUT_H, ACCEL_ZOUT_L)
            
            # Read gyroscope data
            gyro_x_raw = self.read_word(GYRO_XOUT_H, GYRO_XOUT_L)
            gyro_y_raw = self.read_word(GYRO_YOUT_H, GYRO_YOUT_L)
            gyro_z_raw = self.read_word(GYRO_ZOUT_H, GYRO_ZOUT_L)
            
            # Convert to physical units
            accel_x = accel_x_raw / ACCEL_SCALE
            accel_y = accel_y_raw / ACCEL_SCALE
            accel_z = accel_z_raw / ACCEL_SCALE
            
            gyro_x = gyro_x_raw / GYRO_SCALE
            gyro_y = gyro_y_raw / GYRO_SCALE
            gyro_z = gyro_z_raw / GYRO_SCALE
            
            return SensorData(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
            
        except Exception as e:
            print(f"Error getting sensor data: {str(e)}")
            # Return zeros if there's an error
            return SensorData(0, 0, 0, 0, 0, 0)
    
    def close(self):
        """Close the I2C bus"""
        self.bus.close()


def main():
    """Main function to read and display sensor data"""
    try:
        # Create and initialize MPU-6050 object (bus 1 is standard on newer Pi models)
        sensor = MPU6050(bus_num=1)
        sensor.initialize()
        
        print("\nReading MPU-6050 data. Press Ctrl+C to stop...\n")
        
        # Main loop to read and display data
        while True:
            sensor_data = sensor.get_sensor_data()
            print(sensor_data)
            time.sleep(0.1)  # 10 readings per second
            
    except KeyboardInterrupt:
        print("\nProgram stopped by user")
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    finally:
        if 'sensor' in locals():
            sensor.close()
            print("I2C bus closed")


if __name__ == "__main__":
    main()