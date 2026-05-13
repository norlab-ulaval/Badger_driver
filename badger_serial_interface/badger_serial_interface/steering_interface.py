#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import numpy as np
from collections import deque
from std_msgs.msg import Float64

# STM32 serial device
stm_id = '/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066FFF373146363143224542-if02'

class MyNode(Node):

    def __init__(self):
        super().__init__('serial_interface_publisher')

        # bits Mapping
        self.bit_coords = [
            82, 214, 313, 430, 510, 600, 723, 831, 929, 1028, 1128, 1201, 1306, 1410, 1529, 1628,
            1737, 1826, 1906, 2016, 2119, 2226, 2337, 2443, 2520, 2617, 2716, 2815, 2908, 3052, 3106, 3258, 3323, 3408
        ]

        # Corresponding angles — 0° = center, Left = Negative
        self.angle_coords = [
            48, 43, 40,36,35,32,29,25,23,20,18,15,12,9,5,2,0,-2,-4,-7,-8,-10,-14,-18,-20,-21,-28,-26,-30,-35,-40,-43,-45,-49
        ]

        self.stm = serial.Serial(
            port=stm_id,
            baudrate=921600,
            timeout=0.1
        )

        # Yaw: steering angle in degrees (existing)
        self.yaw_publisher = self.create_publisher(Float64, '/badger/yaw', 100)
        # Pitch: new angle published as raw bits
        self.pitch_publisher = self.create_publisher(Float64, '/badger/pitch_bits', 100)

        self.filter_window = 50
        self._filter_buf: deque[int] = deque(maxlen=self.filter_window)

        self.timer = self.create_timer(0.001, self.state_publisher)

    def get_angle_from_bits(self, bits):
        return np.interp(bits, self.bit_coords, self.angle_coords)

    def state_publisher(self):
        try:
            byte = self.stm.read(1)

            if len(byte) == 0 or byte[0] != 0xAA:
                return

            rest = self.stm.read(5)

            if len(rest) != 5:
                return

            adc1_low, adc1_high, adc2_low, adc2_high, checksum = rest

            # Validate checksum over all 4 data bytes
            if (adc1_low ^ adc1_high ^ adc2_low ^ adc2_high) != checksum:
                return

            # Decode both ADC values
            pitch_bits = adc1_low | ((adc1_high & 0x0F) << 8)
            yaw_bits = adc2_low | ((adc2_high & 0x0F) << 8)

            # Moving average on yaw bits
            self._filter_buf.append(yaw_bits)
            filtered_yaw_bits = sum(self._filter_buf) / len(self._filter_buf)

            # Publish yaw as degrees
            yaw_msg = Float64()
            yaw_msg.data = self.get_angle_from_bits(filtered_yaw_bits)
            self.yaw_publisher.publish(yaw_msg)

            # Publish pitch as raw bits
            pitch_msg = Float64()
            pitch_msg.data = float(pitch_bits)
            self.pitch_publisher.publish(pitch_msg)

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