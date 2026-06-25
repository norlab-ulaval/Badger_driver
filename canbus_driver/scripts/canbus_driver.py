#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
import can
import cantools
import os
from ament_index_python.packages import get_package_share_directory

class EncoderCanNode(Node):
    def __init__(self):
        super().__init__('encoder_can_node')

        default_dbc = os.path.join(
        get_package_share_directory('canbus_driver'),
        'config',
        'badger.dbc'
        )

        self.declare_parameter('dbc_path', default_dbc)
        self.declare_parameter('can_interface', 'can32')
        self.declare_parameter('bitrate', 250000)

        dbc_path = self.get_parameter('dbc_path').get_parameter_value().string_value
        can_interface = self.get_parameter('can_interface').get_parameter_value().string_value

        self.db = cantools.database.load_file(dbc_path)
        self.encoder_msg = self.db.get_message_by_name('ENCODER_DATA')

        self.bus = can.interface.Bus(
            channel=can_interface,
            bustype='socketcan'
        )

        self.pub_speed = self.create_publisher(Int32, 'encoder/speed', 10)
        self.pub_direction = self.create_publisher(Bool, 'encoder/direction', 10)

        self.timer = self.create_timer(0.001, self.read_can)

        self.get_logger().info(f'EncoderCanNode started on {can_interface}, DBC: {dbc_path}')

    def read_can(self):
        try:
            raw = self.bus.recv(timeout=0.0)
            if raw is None:
                return

            if raw.arbitration_id != self.encoder_msg.frame_id:
                return

            decoded = self.db.decode_message(raw.arbitration_id, raw.data)

            speed_msg = Int32()
            speed_msg.data = int(decoded['SPEED'])
            self.pub_speed.publish(speed_msg)

            dir_msg = Bool()
            dir_msg.data = bool(decoded['DIRECTION'])
            self.pub_direction.publish(dir_msg)

            self.get_logger().debug(
                f'Speed: {decoded["SPEED"]} ticks/s | Direction: {"FWD" if decoded["DIRECTION"] else "REV"}'
            )

        except can.CanError as e:
            self.get_logger().error(f'CAN read error: {e}')
        except cantools.database.errors.DecodeError as e:
            self.get_logger().error(f'Decode error: {e}')

    def destroy_node(self):
        self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
