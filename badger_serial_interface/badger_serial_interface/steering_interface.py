#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import numpy as np
from collections import deque
from std_msgs.msg import Float64 

# STM32 serial device
stm_id = '/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF373146363143225155-if02'

class MyNode(Node):

    def __init__(self):
        super().__init__('serial_interface_publisher')

        # bits Mapping
        self.bit_coords = [
            82, 214, 313, 430, 510, 600, 723, 831, 929, 1028, 1128, 1201, 1306, 1410, 1529, 1628,
            1737, 1826, 1906, 2016, 2119, 2226, 2337, 2443, 2520, 2617, 2716, 2815, 2908, 3052, 3106, 3258, 3323, 3408
        ]

        # Corresponding angles
        # I have normalized these so 0° (center) , and Left = Negative
        self.angle_coords = [
            48, 43, 40,36,35,32,29,25,23,20,18,15,12,9,5,2,0,-2,-4,-7,-8,-10,-14,-18,-20,-21,-28,-26,-30,-35,-40,-43,-45,-49
        ]

        self.stm = serial.Serial(
            port=stm_id,
            baudrate=921600,
            timeout=0.1
        )

        # Publishing
        self.angle_publisher = self.create_publisher(
            Float64,
            '/badger/angle_degrees',
            100
        )

        self.filter_window = 50
        # init of the queue to enable moving average
        self._filter_buf: deque[int] = deque(maxlen=self.filter_window)

        self.timer = self.create_timer(0.001, self.state_publisher)

    def get_angle_from_bits(self, bits):

        return np.interp(bits, self.bit_coords, self.angle_coords)

    def state_publisher(self):
        try:
            byte = self.stm.read(1)


            # Implementing message validation
            if len(byte) == 0 or byte[0] != 0xAA:
                return

            rest = self.stm.read(3)
            if len(rest) != 3:
                return

            low, high_nibble, checksum = rest[0], rest[1], rest[2]

            if (low ^ high_nibble) != checksum:
                return
            # If message is valid decode it
            val = low | ((high_nibble & 0x0F) << 8)

            # Apply moving-average to raw bits first for smoothness
            self._filter_buf.append(val)
            filtered_bits = sum(self._filter_buf) / len(self._filter_buf)

            # Transform bits to Angle
            angle_val = self.get_angle_from_bits(filtered_bits)

            msg = Float64()
            msg.data = angle_val
            self.angle_publisher.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.stm.is_open:
            node.stm.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
