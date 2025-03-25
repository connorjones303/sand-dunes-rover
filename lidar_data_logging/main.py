from lidar_logging import LidarReader

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