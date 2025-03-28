# ROBOTICS-astroshuttle

**COSGC ROBOTICS CHALLENGE 2024-2025**  
**TEAM AXIOM @ ARAPAHOE COMMUNITY COLLEGE**  
**ROVR: Oberon**  
**DANIEL KRONEBERGER**

---

## Table of Contents

- [Project Overview](#project-overview)
- [Hardware Components](#hardware-components)
- [Software Components](#software-components)
- [Ultrasonic Navigation System](#ultrasonic-navigation-system)
- [LiDAR System](#lidar-system)
- [Navigation and Control](#navigation-and-control)

---

## Project Overview

**ROVR: Oberon** is an autonomously navigating Martian rover developed for the COSGC Robotics Challenge 2024-2025. Developed with TEAM AXIOM @ ACC.

---

## Hardware Components

- **Motors:**  
  - 4× 25mm gear motors (without encoders)  
  - 2× 20mm gear motors (without encoders)

- **Motor Drivers:**  
  - L298N or BTS7960 motor controllers (depending on availability and performance requirements)

- **Arduino:**  
  - Receives serial commands from the Raspberry Pi and translates them into PWM signals for the motor drivers.  
  - Also manages servo control for steering.

- **Raspberry Pi:**  
  - Runs Ubuntu 22.04 with ROS 2 Humble.  
  - Hosts SLAM and navigation nodes as well as a custom Python command hub that communicates with the Arduino.

- **LD19 LiDAR Module:**  
  - Provides 2D scan data on the `/scan` topic for SLAM mapping and localization.

- **Ultrasonic Sensors:**  
  - Mounted near the wheels, these provide real-time distance measurements to assist in obstacle avoidance and fine navigation adjustments.

- **Additional Components:**  
  - Wiring, power supplies (including boost converters), mounting hardware, and other electronics needed to integrate the system into a mobile robotic platform.

---

## Software Components

- **ROS 2 Humble on Ubuntu 22.04**  
  - Serves as the core middleware for robotic communication and process management.

- **slam_toolbox**  
  - Performs real-time SLAM for mapping and localization in either mapping or localization mode.

- **Nav2 (Navigation 2)**  
  - Plans paths and generates velocity commands on the `/cmd_vel` topic based on user-defined navigation goals.

- **Custom Serial Bridge Node**  
  - Converts ROS velocity commands into serial commands for the Arduino.

- **Arduino Firmware**  
  - Processes a universal serial command protocol to execute commands like `driveF`, `pivotL`, `skidL`, etc.  
  - Controls motors and servos based on these commands.

- **Python Navigation Scripts**  
  - Run on the Raspberry Pi to collect ultrasonic sensor data and autonomously decide optimal navigation commands.  
  - Examples include routines for forward motion followed by pivot scanning to find the best clearance.

---

## Ultrasonic Navigation System

The ultrasonic system enhances local navigation by providing real-time distance measurements:

- **Sensor Setup:**  
  - Two HC-SR04 (or similar) ultrasonic sensors are mounted on the left and right sides of the rover.  
  - Each sensor uses a TRIG and an ECHO pin connected to the Raspberry Pi’s GPIO.

- **Operation:**  
  - The Python script periodically triggers each sensor, calculates the distance based on the echo pulse duration, and evaluates the clearance on both sides.  
  - When the rover is in motion, if either sensor detects an obstacle (or if the combined distance is below a defined threshold), the system initiates a pivot routine.

- **Pivot Routine:**  
  - The rover first drives forward for a preset duration.  
  - It then sequentially pivots left and right (straightening the wheels between movements) to measure the distances at several small angles.  
  - The angle with the greatest combined distance from both sensors is chosen as the optimal heading.  
  - If the distances remain below a threshold, the rover will pivot further to try to clear the obstacle before resuming forward motion.

---

## LiDAR System

The LiDAR system provides a higher-level view of the environment for mapping and navigation:

1. **Mapping Phase:**  
   - The LD19 LiDAR publishes scan data to the `/scan` topic.  
   - `slam_toolbox` processes these scans to generate a 2D map in real time.  
   - The map is visualized in RViz, allowing for immediate feedback during the mapping process.  
   - Once a satisfactory map is generated, it can be saved using the `/slam_toolbox/save_map` service.

2. **Navigation Phase:**  
   - The system is reconfigured to run `slam_toolbox` in localization mode using the saved map.  
   - A navigation goal is set in RViz via the “2D Nav Goal” tool.  
   - Nav2 computes a global path and publishes local velocity commands to `/cmd_vel`.

3. **Motion Execution:**  
   - A custom Serial Bridge Node converts `/cmd_vel` commands into serial commands (such as `driveF` or `pivotL`) that are sent to the Arduino.  
   - The Arduino executes these commands, controlling the motors and servos to move the rover along the planned path.

4. **Optional Odometry Bridge:**  
   - In the absence of wheel encoders, a dummy odometry publisher creates an `/odom` topic from the LiDAR-based pose estimate, ensuring compatibility with Nav2’s requirements.

---

## Navigation and Control

The rover’s navigation system is a combination of high-level planning and low-level sensor-based adjustments:

- **Universal Serial Command Protocol:**  
  A standard set of commands has been defined to control the rover’s movements:

  - `skidL` – skid steer left (in-place turn) [DC only]  
  - `skidR` – skid steer right (in-place turn) [DC only]  
  - `driveF` – drive forward  
  - `driveB` – drive backward  
  - `driveL` – servo-assisted turn left (no differential drive)  
  - `driveR` – servo-assisted turn right (no differential drive)  
  - `DdriveL` – servo-assisted turn left (with slight differential drive)  
  - `DriveR` – servo-assisted turn right (with slight differential drive)  
  - `pivotL` – turns servos into a circle and skids left  
  - `pivotR` – turns servos into a circle and skids right  
  - `steerL` – servo ONLY turn left (no drive)  
  - `steerR` – servo ONLY turn right (no drive)  
  - `steerZ` – zero the servos  
  - `S` – stop motors  
  - `C` – show commands

- **Raspberry Pi Command Hub:**  
  - A Python application running on the Pi communicates with the Arduino over serial.  
  - It reads ultrasonic sensor data via the GPIO pins.  
  - Based on sensor input, it determines whether to drive forward or perform a pivot scan.  
  - The system tests multiple pivot angles (with wheel straightening between each pivot) and selects the angle that yields the maximum clearance.  
  - The chosen command is then sent to the Arduino to execute the movement.

---

*For any questions or contributions, please contact Daniel Kroneberger or open an issue on this repository.*

