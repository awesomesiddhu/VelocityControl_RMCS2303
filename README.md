# MinimalModbus ROS Node for RMCS-5024 Motor and RMCS2303 Motor Driver

## Overview
This repository contains a ROS (Robot Operating System) node for controlling the RMCS-5024 motor equipped with a Rhino RMCS2303 Motor Driver. The code interfaces with the motor driver using the MinimalModbus library and is designed to control the motor based on Twist messages, providing both linear and angular velocities.

## Hardware Requirements
- Rhino 200RPM 15Kgcm DC Planetary Geared High Precision Encoder Servo Motor (RMCS-5024)
- RMCS2303 Motor Driver

## Dependencies
- ROS (Robot Operating System)
- MinimalModbus library
- Python 3

## Installation
1. Install ROS on your system if not already installed.
2. Install the MinimalModbus library by running:
   ```bash
   pip install minimalmodbus
   ```
3. Clone this repository into your ROS workspace (e.g., catkin_ws/src).

## Usage
1. Connect the RMCS-5024 motor to the RMCS2303 Motor Driver.
2. Connect the RMCS2303 Motor Driver to your computer via USB.
3. Build your ROS workspace:
   ```bash
   cd catkin_ws
   catkin_make
   source devel/setup.bash
   ```
4. Run the ROS node:
   ```bash
   rosrun your_package_name minimal_modbus_node.py
   ```
   Replace `your_package_name` with the name of your ROS package.

5. Publish Twist messages to the `/cmd_vel` topic to control the motor.

## Important Note
This code is designed for use with the Rhino 200RPM 15Kgcm DC Planetary Geared High Precision Encoder Servo Motor (RMCS-5024) and the RMCS2303 Motor Driver. As of the last update, similar code tailored specifically for this setup is not readily available online.

## Disclaimer
Please be cautious when working with hardware, especially when dealing with motors and motor drivers. Ensure proper connections and safety precautions to prevent damage to equipment or injury.

Feel free to reach out for any issues or questions related to this code. Contributions and improvements are welcome!

## Author
Name: Siddharth D Srinivas
Email ID: siddharth.srinivas03@gmail.com
LinkedIn: www.linkedin.com/in/siddharth-d-srinivas-855917247
