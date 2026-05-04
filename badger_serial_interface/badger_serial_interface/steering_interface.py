#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Int64
import re


# Found in ls "/dev/serial/by-id/"
stm_id = '/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066FFF373146363143224542-if02'

class MyNode(Node):
    def __init__(self):
        super().__init__('serial_interface_publisher')
        self.stm = serial.Serial()
        self.stm.port = stm_id
        self.stm.baudrate = 115200
        self.stm.timeout = 2
        self.stm.open()

        self.status_publisher = self.create_publisher(Int64, '/badger/badger_status', 10)



    def state_publisher(self):
        # Reads the serial bus and publishes the status on ros custom topic
        if self.stm.in_waiting > 0:
            try:
                # Read line and decode
                line = self.stm.readline().decode('utf-8').strip()
                
                # Regex to find "Steering amount: [number]"
                match = re.search(r"Steering amount:\s*(\d+)", line)
                
                if match:
                    raw_val = int(match.group(1))
                    
                    # Create and publish message
                    msg = UInt64()
                    msg.data = raw_val
                    self.publisher_.publish(msg)
                    
                    # Optional: log the value
                    # self.get_logger().info(f"Published: {raw_val}")
                    
            except Exception as e:
                self.get_logger().warn(f"Error parsing serial: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()