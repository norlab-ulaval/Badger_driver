#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from collections import deque
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

        # Moving-average filter (window size configurable)
        self.filter_window = 10
        self._filter_buf: deque[int] = deque(maxlen=self.filter_window)

        # Fast polling timer
        self.timer = self.create_timer(
            0.001,   # 1 kHz polling
            self.state_publisher
        )

    def state_publisher(self):
        # Reads 4-byte framed packets: [0xAA | low_byte | high_nibble | XOR_checksum]
        # Resynchronises automatically if byte alignment is lost.

        try:
            # Scan for the start marker 0xAA
            byte = self.stm.read(1)
            if len(byte) == 0 or byte[0] != 0xAA:
                return   # nothing ready or out of sync – try again next tick

            # Read the remaining 3 bytes of the packet
            rest = self.stm.read(3)
            if len(rest) != 3:
                return   # incomplete packet

            low, high_nibble, checksum = rest[0], rest[1], rest[2]

            # Validate XOR checksum
            if (low ^ high_nibble) != checksum:
                self.get_logger().warn(
                    f"Checksum error: low=0x{low:02X} high=0x{high_nibble:02X} "
                    f"expected=0x{low ^ high_nibble:02X} got=0x{checksum:02X}"
                )
                return

            val = low | ((high_nibble & 0x0F) << 8)

            # Apply moving-average filter
            self._filter_buf.append(val)
            filtered_val = int(sum(self._filter_buf) / len(self._filter_buf))

            msg = Int64()
            msg.data = filtered_val
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