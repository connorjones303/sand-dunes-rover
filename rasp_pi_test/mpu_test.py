import smbus # smbus-cffi is the package install name
import time

from robot2.logger import logger # Nathan W: Implement logging (logs and prints to console in one statement)

# MPU-6050 Registers
PWR_MGMT_1 = 0x6B
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

# MPU-6050 I2C address (default is 0x68)
MPU_ADDRESS = 0x68

# Sensitivity scale factors (adjust based on your MPU-6050 settings)
ACCEL_SCALE = 16384.0  # +/- 2g range (default)
GYRO_SCALE = 131.0     # +/- 250 deg/s range (default)

class SensorData:
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

def read_word(bus, addr, high_reg, low_reg):
    """Read a 16-bit value from two consecutive registers and return as signed int."""
    high = bus.read_byte_data(addr, high_reg)
    low = bus.read_byte_data(addr, low_reg)
    val = (high << 8) + low
    if val >= 0x8000:  # Convert to signed if negative
        val = val - 0x10000
    return val

def get_sensor_data(bus, addr):
    """Read raw data from MPU-6050 and return a SensorData object."""
    # Read accelerometer data
    accel_x_raw = read_word(bus, addr, ACCEL_XOUT_H, ACCEL_XOUT_L)
    accel_y_raw = read_word(bus, addr, ACCEL_YOUT_H, ACCEL_YOUT_L)
    accel_z_raw = read_word(bus, addr, ACCEL_ZOUT_H, ACCEL_ZOUT_L)

    # Read gyroscope data
    gyro_x_raw = read_word(bus, addr, GYRO_XOUT_H, GYRO_XOUT_L)
    gyro_y_raw = read_word(bus, addr, GYRO_YOUT_H, GYRO_YOUT_L)
    gyro_z_raw = read_word(bus, addr, GYRO_ZOUT_H, GYRO_ZOUT_L)

    # Convert to physical units
    accel_x = accel_x_raw / ACCEL_SCALE
    accel_y = accel_y_raw / ACCEL_SCALE
    accel_z = accel_z_raw / ACCEL_SCALE
    gyro_x = gyro_x_raw / GYRO_SCALE
    gyro_y = gyro_y_raw / GYRO_SCALE
    gyro_z = gyro_z_raw / GYRO_SCALE

    return SensorData(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)

def main():
    # Initialize I2C bus (bus 1 on newer Raspberry Pi models)
    try: # Nathan W: Added a try/except for initializing I2C bus
        bus = smbus.SMBus(1)
    except Exception as e:
        logger.critical(f"Failed to connect to MPU-6050: {e}")
        exit(1)

    # Wake up MPU-6050 (set PWR_MGMT_1 to 0)
    try: # Nathan W: Added a try/except for writing to I2C Bus
        bus.write_byte_data(MPU_ADDRESS, PWR_MGMT_1, 0)
    except Exception as e:
        logger.error(f"Failed to write to MPU-6050: {e}")

    logger.info("Reading MPU-6050 data. Press Ctrl+C to stop...")
    try:
        while True:
            # Get sensor data as an object
            sensor_data = get_sensor_data(bus, MPU_ADDRESS)
            
            # Print to terminal
            logger.info(sensor_data)
            
            # Small delay to avoid overwhelming the terminal
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("\nStopped by user.")
    finally:
        bus.close()

if __name__ == "__main__":
    main()
