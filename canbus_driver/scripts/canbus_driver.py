#!/usr/bin/env python3
"""
Roboteq velocity command sender — sends a constant velocity via CANopen SDO.

Why SDO
  The EDS default maps RPDO1 to Cmd_VAR (Object 0x2005, VAR[9]/VAR[10]).
  VAR[n] are just user integer variables; the motor ignores them without a
  MicroBasic script reading and forwarding them.

  Solution: write directly to Object 0x2002:01 (Cmd_MOTVEL "Set Velocity")
  via SDO.  This is the exact CANopen equivalent of the serial !S command that can e executed from the roborun+ console.
  No MicroBasic script, no PDO remapping required.

SDO frame  (CAN ID = 0x600 + node_id):
  byte 0   : 0x23       ← expedited download, 4 data bytes
  bytes 1-2: 0x02 0x20  ← object index 0x2002 (little-endian)
  byte 3   : 0x01       ← sub-index 1 = channel 1
  bytes 4-7: velocity   ← INT32, little-endian
"""

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

_CMD_DBC_NAME = {
    'cango':  'ROBOTEQ_CANGO',   # Object 0x2000:01 — !G, all modes
    'motvel': 'ROBOTEQ_MOTVEL',  # Object 0x2002:01 — !S, closed-loop speed only
}
# CANopen SDO expedited download command specifier for 4-byte payload
_SDO_CS_4B = 0x23


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
        # 'cango'  = !G, works in ALL modes  (open-loop: -1000..+1000 % power)
        # 'motvel' = !S, ONLY works in Closed Loop SPEED mode
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
            raise ValueError(f"cmd_mode must be one of {list(_CMD_DBC_NAME)}, got '{self.cmd_mode}'")




    def init_can(self):

        # ── Load DBC and look up the command message for the selected mode ───────
        # frame_id of each ROBOTEQ_* message IS the CANopen object index.
        # Sub-index is always 0x01 (channel 1).  SDO CS byte is 0x23 (4-byte exp.).
        self.db       = cantools.database.load_file(self.dbc_path)
        self.cmd_msg  = self.db.get_message_by_name(f"{_CMD_DBC_NAME[self.cmd_mode]}")
        self.encoder_msg = self.db.get_message_by_name('ENCODER_DATA')
        self.pwm_msg  = self.db.get_message_by_name('PWM_CMD')
        self.obj_index = self.cmd_msg.frame_id   # e.g. 0x2000 for cango
        self.obj_sub   = 0x01                    # channel 1, always sub-index 1

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
        self.get_logger().info(
            f"TPDO1 listener: CAN ID=0x{self.pos_cob_id:03X} "
            f"→ topic /position_181"
        )


    def init_subs(self):
        self.sub = self.create_subscription(
            Int32, 'cmd_ch1', self._cmd_callback, 10
        )
        self.pwm_sub = self.create_subscription(
            Int32, '/cmd/vel', self._pwm_callback, 10
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


    def _cmd_callback(self, msg: Int32) -> None:
        self.vel_cmd        = msg.data
        self._last_cmd_time = time.monotonic()
        self.get_logger().info(f"cmd_ch1 → {self.vel_cmd}")

    def _send_velocity(self) -> None:
        """
        Send a command to channel 1 via SDO expedited write.
        Sends 0 if no message has been received on /cmd_ch1 within cmd_timeout_s.
        """
        stale = (
            self._last_cmd_time is None
            or (time.monotonic() - self._last_cmd_time) > self._cmd_timeout
        )
        value = 0 if stale else self.vel_cmd

        val_bytes = self.cmd_msg.encode({'Ch1_Value': value})
        payload   = self._sdo_header + bytes(val_bytes)

        self._send_can(payload, self.cmd_msg.frame_id)

    def _pwm_callback(self, msg: Int32) -> None:
        self._pwm_cmd       = max(0, min(1023, msg.data))  # clamp to valid range
        self._last_pwm_time = time.monotonic()

    def _send_pwm(self) -> None:
        """
        Send PWM duty cycle on CAN ID 0x100.
        Format: 2 bytes, little-endian uint16_t, range 0-1023.
        Sends 0 if /cmd/vel has gone silent for longer than cmd_timeout_s (watchdog).
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
        Sends a frame on the bus while and logs that might come up from the hardware 
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
                decoded = self.db.decode_message(raw.arbitration_id, raw.data)
                direction = bool(decoded['DIRECTION'])
                sign = 1 if direction else -1
                speed_msg = Int32()
                speed_msg.data = int(decoded['SPEED']) * sign
                self.speed_pub.publish(speed_msg)

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

            elif raw.arbitration_id == self.sdo_rx_id and raw.data[0] == 0x80:
                #  SDO abort response canOpen trouble shooting
                code = _s.unpack('<I', bytes(raw.data[4:8]))[0]
                hint = {
                    0x06010002: "object is read-only",
                    0x06020000: "object does not exist",
                    0x08000000: "general device error",
                }.get(code, "")
                self.get_logger().error(
                    f"SDO abort 0x{code:08X} {hint} — "
                    f"try cmd_mode:='motvel' if using closed-loop speed mode"
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