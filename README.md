# Sand Dunes Rover Project

Spring 2025
Arapahoe Community College Robotics Club - rover project

## Overview

This project implements a rover system designed for sand dune environments. The system consists of:

1. An Arduino-controlled **Rover** with DC motors, servo steering, IMU sensors, and LIDAR
2. A Raspberry Pi **Controller** that interfaces with sensors and communicates with the Arduino

## System Architecture

### Rover Hardware

The rover consists of:

- Arduino controller with SoftPWM for motor control
- 2 DC motor channels for differential drive
- 2 Servo channels for wheel steering
- I2C bus for sensor communication

### Sensor System

The sensor system includes:

- MPU6050 IMU (Accelerometer/Gyroscope) connected via I2C
- LIDAR sensor connected via serial port for distance measurement
- UART connection between Raspberry Pi and Arduino for commands

## Features

- **Movement Control**: Forward, backward, left, right, stop, and active braking
- **Precision Steering**: Servo-controlled wheel direction with configurable angles
- **IMU Data**: Real-time acceleration and gyroscope data collection
- **Motion Detection**: Algorithms to detect significant movement changes
- **LIDAR Scanning**: Collects distance measurements across 360° rotation
- **Data Processing**: Converts LIDAR polar coordinates to Cartesian
- **Data Logging**: Records LIDAR measurements to timestamped files

## Control System

### Arduino Motor Control

- **DC Motor States**:
  - Forward: IN1=HIGH, IN2=LOW
  - Reverse: IN1=LOW, IN2=HIGH
  - Brake: IN1=HIGH, IN2=HIGH
  - Coast: IN1=LOW, IN2=LOW

- **Servo Control**:
  - Straight position: 45 degrees (configurable)
  - Turn margin: 45 degrees (configurable)

### Command Interface

The rover accepts the following UART commands:

- **f**: Move forward
- **b**: Move backward
- **l**: Turn left
- **r**: Turn right
- **s**: Stop motors (coast)
- **t**: Brake motors (active stop)
- **w**: Start writing LIDAR data
- **e**: End writing LIDAR data
- **q**: Quit controller program

## Sensor Details

### MPU6050 IMU Sensor

- Measures acceleration and angular velocity in 3 axes
- Detects movement with configurable thresholds
- Provides data in physical units (g for acceleration, degrees/s for rotation)

### LIDAR Sensor

- Rotational scanner providing 360° distance measurements
- Data provided in polar format (angle, distance, intensity)
- Converts to Cartesian coordinates (x, y)
- Outputs timestamped data for analysis and mapping

## Data Format

### IMU Data Format

```
Accel (g): X=0.00, Y=0.00, Z=0.00 | Gyro (deg/s): X=0.00, Y=0.00, Z=0.00
```
### LIDAR Data Format
```
timestamp,angle,distance,intensity,x,y
1711212151.324,352.45,342,127,328.4,-95.2
```
## Getting Started

### Prerequisites

- Arduino with SoftPWM, Servo, and SoftI2CMaster libraries
- Raspberry Pi with Python 3
- Required Python packages:
  - serial
  - smbus
  - time
  - struct
  - math
  - os
  - datetime

### Hardware Setup

1. Connect Arduino to Raspberry Pi via UART (/dev/ttyACM0)
2. Connect MPU6050 to I2C bus 1
3. Connect LIDAR to serial port (/dev/ttyUSB0)
4. Connect motors and servos as defined in the pin assignments

### Running the Controller

1. Power on the rover hardware
2. Run the Python controller script
3. Enter commands via the terminal interface

## Troubleshooting

### IMU Issues
- Check I2C bus connections
- Verify the IMU address (default: 0x68)
- Use scan_bus() function to detect connected I2C devices

### LIDAR Issues
- Verify serial port connection (/dev/ttyUSB0)
- Ensure baudrate is set to 230400
- Check data format with debug output

### Motor Control Issues
- Verify the Arduino is properly responding to commands
- Check motor driver connections
- Verify servo positions and calibration

