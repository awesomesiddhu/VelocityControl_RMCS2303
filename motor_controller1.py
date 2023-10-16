#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import minimalmodbus
import serial
import math

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        self.right_motor = minimalmodbus.Instrument('/dev/serial1', 1, minimalmodbus.MODE_ASCII)  # port name, slave address (in decimal), mode

        self.right_motor.serial.baudrate = 9600

        self.wheel_radius = 0.1 #meters
        self.gear_ratio = 90

        velocity_mps = 1.0 #m/s

        rpm = (velocity_mps / (2 * math.pi * self.wheel_radius)) * 60 * self.gear_ratio

        # Ensure base_motor_rpm is within the valid range (0-65535)
        rpm = min(max(int(rpm), 0), 65535)

        if velocity_mps >= 0:
            # Write base motor RPM to appropriate register
            self.right_motor.write_register(2048, rpm, functioncode=6)

            self.right_motor.write_register(257, 1, functioncode=6)
        else:
            # Write base motor RPM to appropriate register
            self.right_motor.write_register(2048, -rpm, functioncode=6)

            self.right_motor.write_register(265, 1, functioncode=6)
def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)

if __name__ == '__main__':
    main()
