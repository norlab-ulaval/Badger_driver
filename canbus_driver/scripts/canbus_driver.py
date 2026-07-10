#!/usr/bin/env python3


import struct
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import can
import cantools
import struct as _s
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist

_CMD_DBC_NAME = {
    'cango':   'ROBOTEQ_CANGO',   # Object 0x2000:01 — !G,  all modes
    'motpos':  'ROBOTEQ_MOTPOS',  # Object 0x2001:01 — !P,  closed-loop position (absolute)
    'mposrel': 'ROBOTEQ_MPOSREL', # Object 0x200F:01 — !PR, closed-loop position (relative)
    'motvel':  'ROBOTEQ_MOTVEL',  # Object 0x2002:01 — !S,  closed-loop speed only
}
# CANopen SDO expedited download command specifier for 4-byte payload
_SDO_CS_4B = 0x23

# PWM channel (CAN ID 0x100, → ESP32) limits. Signed: negative = reverse
# direction on the ESP32 (GPIO 25 driven HIGH there), magnitude drives the
# PWM duty cycle.
_PWM_MAX_DUTY = 1023


class RoboteqVelocitySender(Node):

    def __init__(self):
        super().__init__('Badger_canbus_motor_driver')

        # ── Parameters ─────────────────────────────────────────────────────────────

        self.init_params()

        # ── CAN bus ─────────────────────────────────────────────────────────────
        self.init_can()

        # ── Subscriber ───────────────────────────────────────────────────────────
        self.init_subs()

        # ── Publisher / Timers ───────────────────────────────────────────────────

        self.init_pubs()

        self.get_logger().info(
            f"Ready — node_id={self.node_id}, cmd_mode='{self.cmd_mode}', "
            f"timeout={self._cmd_timeout}s, {self.send_rate_hz} Hz\n"
        )

    def init_params(self):

        default_dbc = os.path.join(
            get_package_share_directory('canbus_driver'),
            'config',
            'badger.dbc'
        )

        self.declare_parameter('dbc_path', default_dbc)
        self.declare_parameter('can_interface', 'can32')
        self.declare_parameter('node_id', 1)
        self.declare_parameter('cmd_mode', 'cango')
        # Seconds without a topic message before the motor is zeroed (watchdog)
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('send_rate_hz', 10.0)

        self.dbc_path         = self.get_parameter('dbc_path').get_parameter_value().string_value
        self.can_interface    = self.get_parameter('can_interface').get_parameter_value().string_value
        self.node_id          = self.get_parameter('node_id').get_parameter_value().integer_value
        self.cmd_mode         = self.get_parameter('cmd_mode').get_parameter_value().string_value
        self._cmd_timeout     = self.get_parameter('cmd_timeout_s').get_parameter_value().double_value
        self.send_rate_hz     = self.get_parameter('send_rate_hz').get_parameter_value().double_value

        self.vel_cmd        = 0    # updated by /cmd_ch1 topic
        self._last_cmd_time = None  # time.monotonic() of last received message

        # PWM channel watchdog state
        self._pwm_cmd       = 0
        self._last_pwm_time = None

        if self.cmd_mode not in _CMD_DBC_NAME:
            raise ValueError(
                f"cmd_mode must be one of {list(_CMD_DBC_NAME)}, got '{self.cmd_mode}'"
            )



    def init_can(self):

        # ── Load DBC and look up the command message for the selected mode ───────
        # frame_id of each ROBOTEQ_* message IS the CANopen object index.
        # Sub-index is always 0x01 (channel 1).  SDO CS byte is 0x23 (4-byte exp.).
        self.db          = cantools.database.load_file(self.dbc_path)
        dbc_msg_name     = _CMD_DBC_NAME[self.cmd_mode]
        self.cmd_msg     = self.db.get_message_by_name(dbc_msg_name)
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

        # ── CAN bus ─────────────────────────────────────────────────────────────
        self.get_logger().info(f"Opening CAN interface: {self.can_interface}")
        self.bus = can.interface.Bus(channel=self.can_interface, bustype='socketcan')

        # TPDO1 receive: 0x180 + node_id  (position feedback)
        self.pos_cob_id = 0x180 + self.node_id   # 0x181 for node_id=1
        self.pos_msg = self.db.get_message_by_name('ARTICULATION_FEEDBACK')

        # ── NMT Start Node ────────────────────────────────────────────────────
        # Roboteq boots in Pre-Operational state.  Send NMT Start to transition
        # to Operational, which enables motion commands via SDO / PDO.
        # CAN ID 0x000, data [0x01, node_id]  (or 0x00 = all nodes)
        self.get_logger().info(f"Sending NMT Start Node → node_id={self.node_id}")
        nmt_start = can.Message(
            arbitration_id=0x000,
            data=bytes([0x01, self.node_id]),
            is_extended_id=False,
        )
        self.bus.send(nmt_start)
        time.sleep(0.1)   # give the controller time to transition to Operational


    def init_subs(self):
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_callback, 10
        )


    def init_pubs(self):        
        
        self.get_logger().info(
            f"Ready — node_id={self.node_id}, CAN ID=0x{self.sdo_tx_id:03X}, "
            f"cmd_mode='{self.cmd_mode}' (SDO→0x{self.obj_index:04X}:{self.obj_sub:02X}), "
            f"timeout={self._cmd_timeout}s, {self.send_rate_hz} Hz\n"
        )
        # ── Timer ────────────────────────────────────────────────────────────────
        self.timer     = self.create_timer(1.0 / self.send_rate_hz, self._send_velocity)
        self.rx_timer  = self.create_timer(0.02, self._read_can)  # 50 Hz RX drain
        self.pwm_timer = self.create_timer(1.0 / self.send_rate_hz, self._send_pwm)

        # ── Publisher ────────────────────────────────────────────────────────────
        self.speed_pub   = self.create_publisher(Int32, 'sensor/speed', 10)
        self.yaw_pub     = self.create_publisher(Int32, 'sensor/yaw', 10)
        self.pub_pwm_100 = self.create_publisher(Int32, 'can/fb_100', 10)


    def _cmd_callback(self, msg: Twist) -> None:
        self.vel_cmd        = msg.angular.z * 1000
        # Signed: negative linear.x → reverse (ESP32 drives GPIO 25 HIGH and
        # takes fabs() of this value for the PWM duty). Previously clamped
        # to [0, 1023], which silently discarded direction.
        self._pwm_cmd       = max(-_PWM_MAX_DUTY, min(_PWM_MAX_DUTY, msg.linear.x * 1000))
        self._last_cmd_time = time.monotonic()
        self._last_pwm_time = time.monotonic()


        self.get_logger().info(f"cmd_ch1 → {self.vel_cmd}")

    def _send_velocity(self) -> None:

        stale = (
            self._last_cmd_time is None
            or (time.monotonic() - self._last_cmd_time) > self._cmd_timeout
        )
        value = 0 if stale else self.vel_cmd

        val_bytes = self.cmd_msg.encode({'Ch1_Value': value})
        payload   = self._sdo_header + bytes(val_bytes)

        try:
            self.bus.send(can.Message(
                arbitration_id=self.sdo_tx_id,
                data=payload,
                is_extended_id=False,
            ))
        except can.CanError as e:
            self.get_logger().error(f"CAN send error: {e}")
            return

    def _send_pwm(self) -> None:
        """
        Send PWM duty cycle on CAN ID 0x100.
        Format: 2 bytes, little-endian *signed* int16, range -1023..1023.
        Negative values command reverse direction on the ESP32 (GPIO 25
        HIGH there); the ESP32 takes fabs() of the value for the actual
        duty cycle written to the PWM pin.
        Sends 0 if /cmd_vel has gone silent for longer than cmd_timeout_s (watchdog).

        NOTE: packed manually with struct rather than via cantools'
        pwm_msg.encode(), because the DBC's Duty_Value signal is very
        likely defined as unsigned — encoding a negative number through it
        could raise or silently wrap. If you update the DBC to mark
        Duty_Value as signed, you can switch back to
        self.pwm_msg.encode({'Duty_Value': value}).
        """
        stale = (
            self._last_pwm_time is None
            or (time.monotonic() - self._last_pwm_time) > self._cmd_timeout
        )
        value = 0 if stale else self._pwm_cmd

        value = max(-_PWM_MAX_DUTY, min(_PWM_MAX_DUTY, int(value)))
        payload = _s.pack('<h', value)

        self._send_can(payload, self.pwm_msg.frame_id)

    def _send_can(self, data, id):

        """
        Sends a frame on the bus and logs that might come up from the hardware 
        """

        try:
            self.bus.send(can.Message(
                arbitration_id=id,
                data=data,
                is_extended_id=False,
            ))
        except can.CanError as e:
            self.get_logger().error(f"CAN PWM send error: {e}")


    def _read_can(self) -> None:
        """
        Drain the RX buffer at 50 Hz.
        Handles both TPDO1 position frames (0x181) and SDO abort responses (0x581).
        """
        for _ in range(32):   # cap iterations so the callback can't block the executor
            raw = self.bus.recv(timeout=0.0)
            if raw is None:
                break
            if raw.arbitration_id == self.encoder_msg.frame_id:
                if len(raw.data) == 4:
                    # ESP32 sends IEEE-754 float32 RPM, little-endian
                    rpm = _s.unpack('<f', bytes(raw.data))[0]
                else:
                    self.get_logger().warn(
                        f"ENCODER_DATA: unexpected length {len(raw.data)}, skipping"
                    )
                    continue
                speed_msg = Int32()
                speed_msg.data = round(rpm)
                self.speed_pub.publish(speed_msg)
                self.get_logger().debug(f"encoder RPM={rpm:.2f}")

            if raw.arbitration_id == self.pos_cob_id and self.pos_msg is not None:
                #  TPDO1: position feedback configured on the controller as yaw
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
                if cs == 0x60:
                    self.get_logger().info(
                        f"SDO ACK  ← 0x{self.sdo_rx_id:03X}  [{raw.data.hex(' ')}]"
                    )
                elif cs == 0x80:
                    code = _s.unpack('<I', bytes(raw.data[4:8]))[0]
                    hint = {
                        0x06010002: "object is read-only",
                        0x06020000: "object does not exist",
                        0x06040041: "object cannot be mapped to PDO",
                        0x08000000: "general device error",
                    }.get(code, "unknown")
                    self.get_logger().error(
                        f"SDO ABORT ← 0x{self.sdo_rx_id:03X}  "
                        f"0x{code:08X} ({hint})  raw=[{raw.data.hex(' ')}]"
                    )
                else:
                    self.get_logger().warn(
                        f"Unexpected SDO response cs=0x{cs:02X}  [{raw.data.hex(' ')}]"
                    )

    def destroy_node(self):
        self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoboteqVelocitySender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()