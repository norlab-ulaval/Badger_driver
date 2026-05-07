#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Int64


# STM32 serial device
stm_id = '/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF373146363143225155-if02'


class MyNode(Node):

    def __init__(self):
        super().__init__('serial_interface_publisher')

        self.stm = serial.Serial(
            port=stm_id,
            baudrate=921600,      # match STM32 code
            timeout=0.1          # wait max 0.1s for data
        )

        self.status_publisher = self.create_publisher(
            Int64,
            '/badger/badger_status',
            100
        )

        # Fast polling timer
        self.timer = self.create_timer(
            0.001,   # 1 kHz polling
            self.state_publisher
        )

    def state_publisher(self):
        # This function will check the usb bus until a packet is received upond reception
        #the packet is processed and published as an angle matched to the real polynomial behavior of the steering axis

        try:
            
            self.stm.reset_input_buffer()

            # read exactly 2 bytes or timeout
            data = self.stm.read(2)

            if len(data) == 2:
                val = data[0] | ((data[1] & 0x0F) << 8)

                msg = Int64()
                msg.data = val
                self.status_publisher.publish(msg)

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