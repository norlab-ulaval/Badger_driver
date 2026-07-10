#!/usr/bin/env python3
"""
Roboteq motor command node — sends velocity / position / open-loop commands
via CANopen SDO and publishes sensor feedback from the CAN bus.

cmd_mode parameter:
  'cango'  → Object 0x2000:01 (Cmd_CANGO  ≡ !G 1 <v>) — ALL modes, -1000..+1000
  'motvel' → Object 0x2002:01 (Cmd_MOTVEL ≡ !S 1 <v>) — Closed Loop SPEED only
  'motpos' → Object 0x2001:01 (Cmd_MOTPOS ≡ !P 1 <v>) — Closed Loop POSITION only

Topics in:
  /cmd_ch1  (std_msgs/Int32) — motor command (velocity, position or power %)
  /cmd/vel  (std_msgs/Int32) — raw PWM duty (0-1023) on CAN ID 0x100

Topics out:
  /encoder/speed          (std_msgs/Int32)
  /articulation/sensor/yaw (std_msgs/Int32)  — 0x181 TPDO1 first signal
  /can/fb_100             (std_msgs/Int32)  — echo of 0x100 frames
"""

import struct as _s
import time
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import can
import cantools
from ament_index_python.packages import get_package_share_directory


# ── DBC message name for each command mode ──────────────────────────────────
# frame_id of each ROBOTEQ_* message = CANopen object index (stripped of the
# extended-frame bit 0x80000000 that is set in the DBC for >11-bit IDs).
_CMD_DBC_NAME = {
    'cango':  'ROBOTEQ_CANGO',   # 0x2000:01  !G — open-loop / any mode
    'motvel': 'ROBOTEQ_MOTVEL',  # 0x2002:01  !S — closed-loop speed
    'motpos': 'ROBOTEQ_MOTPOS',  # 0x2001:01  !P — closed-loop position
}

# CANopen SDO expedited download (write) CS byte for 4-byte payload
_SDO_CS_4B = 0x23


class RoboteqVelocitySender(Node):

    def __init__(self):
        super().__init__('Badger_canbus_motor_driver')
        self.init_params()
        self.init_can()
        self.init_subs()
        self.init_pubs()

    # ── Init helpers ─────────────────────────────────────────────────────────

    def init_params(self):
        default_dbc = os.path.join(
            get_package_share_directory('canbus_driver'),
            'config', 'badger.dbc'
        )
        self.declare_parameter('dbc_path',     default_dbc)
        self.declare_parameter('can_interface', 'can32')
        self.declare_parameter('node_id',       1)
        self.declare_parameter('cmd_mode',      'cango')   # 'cango' | 'motvel' | 'motpos'
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('send_rate_hz',  10.0)

        self.dbc_path      = self.get_parameter('dbc_path').get_parameter_value().string_value
        self.can_interface = self.get_parameter('can_interface').get_parameter_value().string_value
        self.node_id       = self.get_parameter('node_id').get_parameter_value().integer_value
        self.cmd_mode      = self.get_parameter('cmd_mode').get_parameter_value().string_value
        self._cmd_timeout  = self.get_parameter('cmd_timeout_s').get_parameter_value().double_value
        self.send_rate_hz  = self.get_parameter('send_rate_hz').get_parameter_value().double_value

        if self.cmd_mode not in _CMD_DBC_NAME:
            raise ValueError(
                f"cmd_mode must be one of {list(_CMD_DBC_NAME)}, got '{self.cmd_mode}'"
            )

        self.vel_cmd        = 0     # current setpoint, updated by /cmd_ch1
        self._last_cmd_time = None  # watchdog: time.monotonic() of last msg
        self._pwm_cmd       = 0
        self._last_pwm_time = None

    def init_can(self):
        self.db = cantools.database.load_file(self.dbc_path)

        # ── Motor command message (selected by cmd_mode) ─────────────────────
        dbc_msg_name   = _CMD_DBC_NAME[self.cmd_mode]
        self.cmd_msg   = self.db.get_message_by_name(dbc_msg_name)
        self.obj_index = self.cmd_msg.frame_id   # = CANopen object index
        self.obj_sub   = 0x01

        self.sdo_tx_id   = 0x600 + self.node_id
        self.sdo_rx_id   = 0x580 + self.node_id
        self._send_count = 0

        self._sdo_header = bytes([
            _SDO_CS_4B,
            self.obj_index & 0xFF,
            (self.obj_index >> 8) & 0xFF,
            self.obj_sub,
        ])

        # ── PWM message (CAN ID 0x100) ────────────────────────────────────────
        self.pwm_msg = self.db.get_message_by_name('PWM_CMD')

        # ── Feedback messages ─────────────────────────────────────────────────
        self.encoder_msg = self.db.get_message_by_name('ENCODER_DATA')
        self.pos_cob_id  = 0x180 + self.node_id   # TPDO1
        try:
            self.pos_msg = self.db.get_message_by_name('ARTICULATION_FEEDBACK')
        except KeyError:
            self.pos_msg = None

        self.get_logger().info(
            f"cmd_mode='{self.cmd_mode}' → DBC '{dbc_msg_name}' "
            f"(Object 0x{self.obj_index:04X}:{self.obj_sub:02X}), "
            f"SDO TX=0x{self.sdo_tx_id:03X}"
        )

        # ── Open CAN bus ──────────────────────────────────────────────────────
        self.bus = can.interface.Bus(channel=self.can_interface, bustype='socketcan')

    def init_subs(self):
        self.sub     = self.create_subscription(Int32, 'cmd_ch1',  self._cmd_callback, 10)
        self.pwm_sub = self.create_subscription(Int32, '/cmd/vel', self._pwm_callback, 10)

    def init_pubs(self):
        self.timer     = self.create_timer(1.0 / self.send_rate_hz, self._send_velocity)
        self.pwm_timer = self.create_timer(1.0 / self.send_rate_hz, self._send_pwm)
        self.rx_timer  = self.create_timer(0.02, self._read_can)   # 50 Hz RX drain

        self.speed_pub   = self.create_publisher(Int32, 'encoder/speed',           10)
        self.yaw_pub     = self.create_publisher(Int32, 'articulation/sensor/yaw', 10)
        self.pub_pwm_100 = self.create_publisher(Int32, 'can/fb_100',              10)

        self.get_logger().info(
            f"Ready — interface={self.can_interface}, node_id={self.node_id}, "
            f"cmd_mode='{self.cmd_mode}', timeout={self._cmd_timeout}s, "
            f"{self.send_rate_hz} Hz\n"
            f"  Motor cmd : ros2 topic pub /cmd_ch1 std_msgs/msg/Int32 '{{data: 500}}'\n"
            f"  PWM cmd   : ros2 topic pub /cmd/vel std_msgs/msg/Int32 '{{data: 512}}'"
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _cmd_callback(self, msg: Int32) -> None:
        self.vel_cmd        = msg.data
        self._last_cmd_time = time.monotonic()
        self.get_logger().info(f"cmd_ch1 → {self.vel_cmd}")

    def _pwm_callback(self, msg: Int32) -> None:
        self._pwm_cmd       = max(0, min(1023, msg.data))
        self._last_pwm_time = time.monotonic()
        self.get_logger().info(f"/cmd/vel → PWM {self._pwm_cmd}")

    # ── Periodic send ────────────────────────────────────────────────────────

    def _send_velocity(self) -> None:
        """SDO write to Cmd_CANGO / Cmd_MOTVEL / Cmd_MOTPOS depending on cmd_mode."""
        stale = (
            self._last_cmd_time is None
            or (time.monotonic() - self._last_cmd_time) > self._cmd_timeout
        )
        value   = 0 if stale else self.vel_cmd
        payload = self._sdo_header + bytes(self.cmd_msg.encode({'Ch1_Value': value}))

        try:
            self.bus.send(can.Message(
                arbitration_id=self.sdo_tx_id,
                data=payload,
                is_extended_id=False,
            ))
        except can.CanError as e:
            self.get_logger().error(f"CAN send error: {e}")
            return

        self._send_count += 1
        if self._send_count <= 3 or self._send_count % 50 == 0:
            self.get_logger().info(
                f"[#{self._send_count}] SDO 0x{self.sdo_tx_id:03X} "
                f"[{payload.hex(' ')}]  value={value}"
                + (" (stale→0)" if stale else "")
            )

    def _send_pwm(self) -> None:
        """Send raw PWM duty on CAN ID 0x100."""
        stale = (
            self._last_pwm_time is None
            or (time.monotonic() - self._last_pwm_time) > self._cmd_timeout
        )
        value   = 0 if stale else self._pwm_cmd
        payload = self.pwm_msg.encode({'Duty_Value': value})

        try:
            self.bus.send(can.Message(
                arbitration_id=self.pwm_msg.frame_id,
                data=payload,
                is_extended_id=False,
            ))
        except can.CanError as e:
            self.get_logger().error(f"CAN PWM send error: {e}")

    # ── RX drain ─────────────────────────────────────────────────────────────

    def _read_can(self) -> None:
        for _ in range(32):
            raw = self.bus.recv(timeout=0.0)
            if raw is None:
                break

            aid = raw.arbitration_id

            if aid == self.encoder_msg.frame_id:
                decoded   = self.db.decode_message(aid, raw.data)
                direction = bool(decoded['DIRECTION'])
                speed_msg = Int32()
                speed_msg.data = int(decoded['SPEED']) * (1 if direction else -1)
                self.speed_pub.publish(speed_msg)

            elif aid == 0x100:
                try:
                    decoded = self.db.decode_message(0x100, raw.data)
                    out      = Int32()
                    out.data = int(decoded['Duty_Value'])
                    self.pub_pwm_100.publish(out)
                except Exception as e:
                    self.get_logger().warn(f"Decode 0x100 error: {e}")

            elif aid == self.pos_cob_id and self.pos_msg is not None:
                try:
                    decoded  = self.pos_msg.decode(raw.data)
                    first    = sorted(self.pos_msg.signals, key=lambda s: s.start)[0]
                    yaw_msg  = Int32()
                    yaw_msg.data = int(decoded[first.name])
                    self.yaw_pub.publish(yaw_msg)
                except Exception as e:
                    self.get_logger().warn(f"Decode 0x{self.pos_cob_id:03X} error: {e}")

            elif aid == self.sdo_rx_id and raw.data[0] == 0x80:
                code = _s.unpack('<I', bytes(raw.data[4:8]))[0]
                hint = {
                    0x06010002: "object is read-only",
                    0x06020000: "object does not exist",
                    0x08000000: "general device error",
                }.get(code, "")
                self.get_logger().error(
                    f"SDO abort 0x{code:08X} {hint}"
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
