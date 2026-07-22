#!/usr/bin/env python3

import struct
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32
import can
import cantools
import struct as _s
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import math

# CANopen SDO expedited download command specifier for 4-byte payload
_SDO_CS_4B = 0x23


class CanbusDriver(Node):

    def __init__(self):
        super().__init__('Badger_canbus_motor_driver')

        self.init_params()
        self.init_can()
        self.init_subs()
        self.init_pubs()

    def init_params(self):
        default_dbc = os.path.join(
            get_package_share_directory('canbus_driver'),
            'config',
            'badger.dbc'
        )

        self.declare_parameter('dbc_path', default_dbc)
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('node_id', 1)
        # Seconds without a topic message before the motor is zeroed (watchdog)
        self.declare_parameter('cmd_timeout_s', 0.1)
        self.declare_parameter('send_rate_hz', 10.0)
        self.declare_parameter('pwm_max_duty', 1023)
        self.declare_parameter('max_speed', 500)
        self.declare_parameter('min_speed', 450)
        self.declare_parameter('max_steering', 1000)

        self.dbc_path         = self.get_parameter('dbc_path').get_parameter_value().string_value
        self.can_interface    = self.get_parameter('can_interface').get_parameter_value().string_value
        self.node_id          = self.get_parameter('node_id').get_parameter_value().integer_value
        self._cmd_timeout     = self.get_parameter('cmd_timeout_s').get_parameter_value().double_value
        self.send_rate_hz     = self.get_parameter('send_rate_hz').get_parameter_value().double_value
        self.pwm_max_duty     = self.get_parameter('pwm_max_duty').get_parameter_value().integer_value
        self.max_speed        = self.get_parameter('max_speed').get_parameter_value().integer_value
        self.min_speed        = self.get_parameter('min_speed').get_parameter_value().integer_value
        self.max_steering     = self.get_parameter('max_steering').get_parameter_value().integer_value

        self.art_cmd        = 0    
        self._last_cmd_time = None  # time.monotonic() of last received message

        # PWM channel watchdog state
        self._pwm_cmd       = 0
        self._last_pwm_time = None

    def init_can(self):
        # ── Load DBC and look up the command messages ───────
        self.db          = cantools.database.load_file(self.dbc_path)

        self.cmd_msg     = self.db.get_message_by_name('ROBOTEQ_CANGO')
        self.encoder_msg = self.db.get_message_by_name('ENCODER_DATA')
        self.pwm_msg     = self.db.get_message_by_name('PWM_CMD')
        self.obj_index   = self.cmd_msg.frame_id   # = CANopen object index
        self.obj_sub     = 0x01

        # SDO CAN IDs (computed from node_id at runtime)
        self.sdo_tx_id   = 0x600 + self.node_id   # request  (master → slave)
    

        self.sdo_rx_id   = 0x580 + self.node_id   # response (slave  → master)
        self._send_count = 0

        # Fixed SDO header: cs | index_lo | index_hi | sub-index
        self._sdo_header = bytes([
            _SDO_CS_4B,
            self.obj_index & 0xFF,
            (self.obj_index >> 8) & 0xFF,
            self.obj_sub,
        ])

        # ── CAN bus 
        self.get_logger().info(f"Opening CAN interface: {self.can_interface}")
        self.bus = can.interface.Bus(channel=self.can_interface, bustype='socketcan')

        # TPDO1 receive: 0x180 + node_id  (position feedback)
        self.pos_cob_id = 0x180 + self.node_id   # 0x181 for node_id=1
        self.pos_msg = self.db.get_message_by_name('ARTICULATION_FEEDBACK')

        # ── NMT Start Node 
        # Roboteq boots in Pre-Operational state. Send NMT Start to transition
        # to Operational, which enables motion commands via SDO / PDO.
        self._send_can(bytes([0x01, self.node_id]), 0x000)

        time.sleep(0.1)   # give the controller time to transition to Operational
        self.get_logger().info(
            f"Ready — node_id={self.node_id}, CAN ID=0x{self.sdo_tx_id:03X}, "
        )

    def init_subs(self):
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self._twist_callback, 10
        )

    def init_pubs(self):        
        # ── Timer ────────────────────────────────────────────────────────────────
        self.timer     = self.create_timer(1.0 / self.send_rate_hz, self._send_steering)
        self.rx_timer  = self.create_timer(0.02, self._read_can)  # 50 Hz RX drain
        self.pwm_timer = self.create_timer(1.0 / self.send_rate_hz, self._send_pwm)

        # ── Publisher ────────────────────────────────────────────────────────────
        self.speed_pub   = self.create_publisher(Float32, 'sensor/speed', 10)
        self.yaw_pub     = self.create_publisher(Int32, 'sensor/yaw', 10)

    def _twist_callback(self, msg: Twist) -> None:
        # Articulation is controlled in position from -1000 to 1000
        self.art_cmd        = msg.angular.z * self.max_steering

        # Signed: negative linear.x → reverse (ESP32 drives GPIO 25 HIGH and
        # takes fabs() of this value for the PWM duty)
        
        joystick_deadzone = 0.3

        self._pwm_cmd = (msg.linear.x - joystick_deadzone ) * (self.max_speed - self.min_speed) + self.min_speed

        
        self._last_cmd_time = time.monotonic()
        self._last_pwm_time = time.monotonic()

    def _send_steering(self) -> None:
        """
        Send position commands to the CAN open interface of the Roboteq articulation motor controller 
        """
        stale = (
            self._last_cmd_time is None
            or (time.monotonic() - self._last_cmd_time) > self._cmd_timeout
        )
        value = 0 if stale else self.art_cmd

        val_bytes = self.cmd_msg.encode({'Ch1_Value': value})
        payload   = self._sdo_header + bytes(val_bytes)

        self._send_can(payload, self.sdo_tx_id)

    def _send_pwm(self) -> None:
        """
        Formats the velocity commands from /cmd_vel to fit the duty cycle format fit for the esp32
        """
        stale = (
            self._last_pwm_time is None
            or (time.monotonic() - self._last_pwm_time) > self._cmd_timeout
        )
        value = 0 if stale else self._pwm_cmd

        payload = self.pwm_msg.encode({'Duty_Value': value})

        self._send_can(payload, self.pwm_msg.frame_id)

    def _send_can(self, data, id):
        """
        Sends a frame on the bus and handles network layer buffer overflow errors gracefully.
        """
        msg = can.Message(
            arbitration_id=id,
            data=data,
            is_extended_id=False,
        )

        try:
            self.bus.send(msg)
        except can.CanError as e:
            self.get_logger().error(f"General CAN send error on ID 0x{id:03X}: {e}")

    def _read_can(self) -> None:
        """
        Drain the RX buffer at 50 Hz.
        """
        for _ in range(32):   # cap iterations so the callback can't block the executor
            raw = self.bus.recv(timeout=0.0)
            if raw is None:
                break
            if raw.arbitration_id == self.encoder_msg.frame_id:
                if len(raw.data) == 4:
                    rpm = self.encoder_msg.decode(raw.data)['RPM_RAW']

                
                speed_msg = Float32()
                speed_msg.data = rpm
                self.speed_pub.publish(speed_msg)

            if raw.arbitration_id == self.pos_cob_id and self.pos_msg is not None:
                # TPDO1: position feedback configured on the controller as yaw
                try:
                    decoded = self.pos_msg.decode(raw.data)
                    # Use the first signal (by start-bit order) as the position value
                    first_sig = sorted(self.pos_msg.signals, key=lambda s: s.start)[0]
                    value = int(decoded[first_sig.name])
                    msg = Int32()
                    msg.data = value
                    self.yaw_pub.publish(msg)
                except Exception as e:
                    self.get_logger().warn(f"Decode 0x{self.pos_cob_id:03X} error: {e}")

            elif raw.arbitration_id == self.sdo_rx_id:
                cs = raw.data[0]                    
                if cs == 0x80:
                    code = _s.unpack('<I', bytes(raw.data[4:8]))[0]
                    hint = {
                        0x06010002: "object is read-only",
                        0x06020000: "object does not exist",
                        0x06040041: "object cannot be mapped to PDO",
                        0x08000000: "general device error",
                    }.get(code, "unknown")
                    self.get_logger().error(
                        f"SDO ABORT ← 0x{self.sdo_rx_id:03X} "
                        f"0x{code:08X} ({hint}) raw=[{raw.data.hex(' ')}]"
                    )
                

    def destroy_node(self):
        self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CanbusDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()