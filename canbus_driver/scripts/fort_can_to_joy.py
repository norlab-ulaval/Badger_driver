#!/usr/bin/env python3
import os

import can
import cantools
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

MAGNITUDE_MAX = 1023.0  # 10-bit magnitude field


def axis_value(decoded, prefix):
    """Reconstruct a signed -1.0..+1.0 axis value from the VSC's split
    Neutral/Negative/Positive status + Magnitude signal layout.

    Only one of the three status signals should read 'Set' at a time.
    Error/Unavailable/NotSet all fall back to 0.0 (safe default)."""
    magnitude = decoded.get(f'{prefix}_Magnitude', 0)
    positive = decoded.get(f'{prefix}_PositiveStatus')
    negative = decoded.get(f'{prefix}_NegativeStatus')

    # cantools returns NamedSignalValue objects when a VAL_ table is present;
    # str() them so comparisons work whether or not choices resolved.
    if str(positive) == 'Set':
        return float(magnitude) / MAGNITUDE_MAX
    if str(negative) == 'Set':
        return -float(magnitude) / MAGNITUDE_MAX
    return 0.0


def status_pressed(decoded, key):
    return 1 if str(decoded.get(key)) == 'Set' else 0


class FortCanToJoy(Node):
    def __init__(self):
        super().__init__('fort_can_to_joy')

        # Parameters
        self.declare_parameter('dbc_file', 'fort_vsc.dbc')
        self.declare_parameter('can_interface', 'can32')
        self.declare_parameter('can_bustype', 'socketcan')
        self.declare_parameter('can_bitrate', 250000)
        self.declare_parameter('joy_topic', 'joy')

        dbc_path = self.get_parameter('dbc_file').value
        can_interface = self.get_parameter('can_interface').value
        can_bustype = self.get_parameter('can_bustype').value
        can_bitrate = self.get_parameter('can_bitrate').value
        joy_topic = self.get_parameter('joy_topic').value

        # Load DBC File
        if not os.path.exists(dbc_path):
            self.get_logger().error(f"DBC file not found at: {dbc_path}")
            raise FileNotFoundError(f"DBC file not found at: {dbc_path}")

        try:
            self.db = cantools.database.load_file(dbc_path)
            self.get_logger().info(f"Loaded DBC database successfully: {dbc_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse DBC file: {e}")
            raise e

        # Publisher
        self.publisher = self.create_publisher(Joy, joy_topic, 10)

        # Cached state — updated incrementally as each of the VSC's separate
        # CAN messages arrives, since axes/buttons are split across frames.
        self.state = {
            'LeftX': 0.0, 'LeftY': 0.0, 'LeftZ': 0.0,
            'RightX': 0.0, 'RightY': 0.0, 'RightZ': 0.0,
            'DPadUp': 0, 'DPadDown': 0, 'DPadLeft': 0, 'DPadRight': 0,
            'Button1': 0, 'Button2': 0, 'Button3': 0, 'Button4': 0,
            'EStopActive': 0,
        }

        # Set up python-can bus + notifier
        try:
            self.bus = can.interface.Bus(
                channel=can_interface,
                bustype=can_bustype,
                bitrate=can_bitrate,
            )
        except Exception as e:
            self.get_logger().error(f"Failed to open CAN bus '{can_interface}': {e}")
            raise e

        self.notifier = can.Notifier(self.bus, [self.can_callback])
        self.get_logger().info(
            f"FORT CAN-to-Joy bridge (cantools + python-can) initialized on '{can_interface}'."
        )

    def can_callback(self, msg: can.Message):
        frame_id = msg.arbitration_id
        can_data = bytes(msg.data)

        try:
            message_def = self.db.get_message_by_frame_id(frame_id)
            decoded = message_def.decode(can_data)
        except KeyError:
            return
        except cantools.database.DecodeError:
            self.get_logger().warn(
                f"Failed to decode CAN frame with ID 0x{frame_id:X}",
                throttle_duration_sec=2.0,
            )
            return

        name = message_def.name

        if name == 'VSC_LeftJoystickBasic':
            self.state['LeftX'] = axis_value(decoded, 'LeftX')
            self.state['LeftY'] = axis_value(decoded, 'LeftY')
            self.state['DPadUp'] = status_pressed(decoded, 'LeftDPad_UpStatus')
            self.state['DPadDown'] = status_pressed(decoded, 'LeftDPad_DownStatus')
            self.state['DPadLeft'] = status_pressed(decoded, 'LeftDPad_LeftStatus')
            self.state['DPadRight'] = status_pressed(decoded, 'LeftDPad_RightStatus')

        elif name == 'VSC_LeftJoystickExtended':
            self.state['LeftZ'] = axis_value(decoded, 'LeftZ')

        elif name == 'VSC_RightJoystickBasic':
            self.state['RightX'] = axis_value(decoded, 'RightX')
            self.state['RightY'] = axis_value(decoded, 'RightY')
            self.state['Button1'] = status_pressed(decoded, 'Button1Status')
            self.state['Button2'] = status_pressed(decoded, 'Button2Status')
            self.state['Button3'] = status_pressed(decoded, 'Button3Status')
            self.state['Button4'] = status_pressed(decoded, 'Button4Status')

        elif name == 'VSC_RightJoystickExtended':
            self.state['RightZ'] = axis_value(decoded, 'RightZ')

        elif name == 'VSC_Heartbeat':
            self.state['EStopActive'] = 1 if decoded.get('EStopIndication', 0) > 0 else 0

        else:
            # VSC_RemoteStatus and others aren't part of the Joy message
            return

        self.publish_joy()

    def publish_joy(self):
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.header.frame_id = 'fort_vsc'

        joy_msg.axes = [
            self.state['LeftX'],
            self.state['LeftY'],
            self.state['LeftZ'],
            self.state['RightX'],
            self.state['RightY'],
            self.state['RightZ'],
        ]

        joy_msg.buttons = [
            self.state['DPadUp'],
            self.state['DPadDown'],
            self.state['DPadLeft'],
            self.state['DPadRight'],
            self.state['Button1'],
            self.state['Button2'],
            self.state['Button3'],
            self.state['Button4'],
            self.state['EStopActive'],
        ]

        self.publisher.publish(joy_msg)

    def destroy_node(self):
        try:
            self.notifier.stop()
        except Exception:
            pass
        try:
            self.bus.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FortCanToJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()