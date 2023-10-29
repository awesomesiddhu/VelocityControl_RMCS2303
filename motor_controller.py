#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import minimalmodbus
import math

class MinimalModbusNode(Node):
    def __init__(self):
        super().__init__('minimal_modbus_node')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.rmcs2303_right = minimalmodbus.Instrument('/dev/ttyUSB0', 7, minimalmodbus.MODE_ASCII)
        self.rmcs2303_left = minimalmodbus.Instrument('/dev/ttyUSB0', 6, minimalmodbus.MODE_ASCII)

        self.rmcs2303_right.serial.baudrate = 9600
        self.rmcs2303_left.serial.baudrate = 9600

    def listener_callback(self, msg):
        gear_ratio = 90
        wheel_radius = 0.1 #meters
        lin_velocity = msg.linear.x # m/s
        ang_velocity = msg.angular.z
        velocity = lin_velocity + 5*ang_velocity
        rpm= abs((velocity / (2*math.pi*wheel_radius))*60*gear_ratio)
        
        try:
            self.rmcs2303_right.write_register(2, 2048, number_of_decimals=0, functioncode=6, signed=False) # set encoder count to 0
            self.rmcs2303_right.write_register(14, rpm, number_of_decimals=0, functioncode=6, signed=False)  # Speed command for base motor in rpm
            
            if (velocity>=0):
                self.rmcs2303_right.write_register(2, 257, number_of_decimals=0, functioncode=6, signed=False) # Enable motor in CW
            else:
                self.rmcs2303_right.write_register(2, 265, number_of_decimals=0, functioncode=6, signed=False) # Enable motor in CCW
            
            while True:
                right_speed_feedback = self.rmcs2303_right.read_register(24) #current speed feedback
                print("Speed feedback : ", right_speed_feedback)

        except KeyboardInterrupt:
            if (velocity>=0):
                self.rmcs2303_right.write_register(2, 256, number_of_decimals=0, functioncode=6, signed=False) # disable motor in CW
            else:
                self.rmcs2303_right.write_register(2, 264, number_of_decimals=0, functioncode=6, signed=False) # disable motor in CCW
            
            self.rmcs2303_right.write_register(2, 2304, number_of_decimals=0, functioncode=6, signed=False) #restarts the drive
            self.rmcs2303_right.write_register(2, 2304, number_of_decimals=0, functioncode=6, signed=False) #restarts the drive

def main(args=None):
    rclpy.init(args=args)

    minimal_modbus_node = MinimalModbusNode()

    rclpy.spin(minimal_modbus_node)

    minimal_modbus_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
