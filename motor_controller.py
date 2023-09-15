import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading

class CommandReader(Node):
    def _init_(self):
        super().__init__('command_reader')
        self.publisher = self.create_publisher(String, 'motor_commands', 10)
        thread = threading.Thread(target=self.read_commands)
        thread.start()

    def read_commands(self):
        while rclpy.ok():
            command = input('Enter command: ')
            msg = String()
            msg.data = command
            self.publisher.publish(msg)

class MotorController(Node):
    def _init_(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(String, 'motor_commands', self.command_callback, 10)

    def command_callback(self, msg):
        command = msg.data.split()
        if command[0] == 'SPEED':
            send_uart_command(f'SPEED {command[1]}\n')
        elif command[0] == 'DIRECTION':
            send_uart_command(f'DIRECTION {command[1]}\n')

def send_uart_command(command):
    # Replace '/dev/ttyS0' with your UART port and '9600' with your baud rate
    with serial.Serial('/dev/ttyS0', 9600, timeout=1) as ser:
        ser.write(command.encode())

def main(args=None):
    rclpy.init(args=args)
    command_reader = CommandReader()
    motor_controller = MotorController()
    rclpy.spin(command_reader)
    rclpy.spin(motor_controller)

if _name_ == '__main__':
    main()