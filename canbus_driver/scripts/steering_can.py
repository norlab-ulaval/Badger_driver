#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int16

import can
import cantools
import threading
import time

def create_roboteq_db(node_id=1):
    """
    Constructs the exact runtime structural message database definitions 
    mirroring the PDO configurations in the roboteq_motor_controllers_v80.eds file.
    """
    # ------------------ RPDO1 (Channel 1 Command) ------------------
    # Mapping layout: Object 0x6040 (Controlword, 16-bit) + 16-bit Padding + Object 0x60FF (Target Velocity, 32-bit)
    rpdo1_msg = cantools.database.can.Message(
        frame_id=0x200 + node_id,
        name='RPDO1_CH1',
        length=8,
        signals=[
            cantools.database.can.Signal(name='Controlword', start=0, length=16, byte_order='little_endian', is_signed=False),
            cantools.database.can.Signal(name='TargetVelocity', start=32, length=32, byte_order='little_endian', is_signed=True)
        ]
    )
    
    # ------------------ RPDO2 (Channel 2 Command) ------------------
    # Mapping layout: Object 0x6840 (Controlword, 16-bit) + 16-bit Padding + Object 0x68FF (Target Velocity, 32-bit)
    rpdo2_msg = cantools.database.can.Message(
        frame_id=0x300 + node_id,
        name='RPDO2_CH2',
        length=8,
        signals=[
            cantools.database.can.Signal(name='Controlword', start=0, length=16, byte_order='little_endian', is_signed=False),
            cantools.database.can.Signal(name='TargetVelocity', start=32, length=32, byte_order='little_endian', is_signed=True)
        ]
    )
    
    # ------------------ TPDO1 (Channel 1 Feedback) ------------------
    # Mapping layout: Object 0x6041 (Statusword, 16-bit) + 16-bit Padding + Object 0x606C (Velocity Actual Value, 32-bit)
    tpdo1_msg = cantools.database.can.Message(
        frame_id=0x180 + node_id,
        name='TPDO1_CH1',
        length=8,
        signals=[
            cantools.database.can.Signal(name='Statusword', start=0, length=16, byte_order='little_endian', is_signed=False),
            cantools.database.can.Signal(name='ActualVelocity', start=32, length=32, byte_order='little_endian', is_signed=True)
        ]
    )
    
    # ------------------ TPDO2 (Channel 2 Feedback) ------------------
    # Mapping layout: Object 0x6841 (Statusword, 16-bit) + 16-bit Padding + Object 0x686C (Velocity Actual Value, 32-bit)
    tpdo2_msg = cantools.database.can.Message(
        frame_id=0x280 + node_id,
        name='TPDO2_CH2',
        length=8,
        signals=[
            cantools.database.can.Signal(name='Statusword', start=0, length=16, byte_order='little_endian', is_signed=False),
            cantools.database.can.Signal(name='ActualVelocity', start=32, length=32, byte_order='little_endian', is_signed=True)
        ]
    )
    
    # FIX: Passing messages explicitly through constructor forces cantools to 
    # build the internal index lookups (_name_to_message), avoiding KeyErrors.
    db = cantools.database.can.Database(messages=[rpdo1_msg, rpdo2_msg, tpdo1_msg, tpdo2_msg])
    return db


class RoboteqCantoolsNode(Node):
    def __init__(self):
        super().__init__('roboteq_cantools_driver')
        
        # --- Parameters ---
        self.declare_parameter('interface', 'socketcan')
        self.declare_parameter('channel', 'can32') # change to your specific physical channel
        self.declare_parameter('bitrate', 250000)
        self.declare_parameter('target_node_id', 1)

        interface = self.get_parameter('interface').get_parameter_value().string_value
        channel = self.get_parameter('channel').get_parameter_value().string_value
        bitrate = self.get_parameter('bitrate').get_parameter_value().integer_value
        self.target_node_id = self.get_parameter('target_node_id').get_parameter_value().integer_value
        
        # Initialize cantools structural configuration database
        self.db = create_roboteq_db(self.target_node_id)
        
        # Pull definitions using get_message_by_name (Safe from KeyError now)
        self.msg_rpdo1 = self.db.get_message_by_name('RPDO1_CH1')
        self.msg_rpdo2 = self.db.get_message_by_name('RPDO2_CH2')

        # Run-time Global States
        self.COB_NMT = 0x000
        self.controlword_operational = 0x000F  # Standard DS402 Operation Enable Command
        self.ch1_target_velocity = 0
        self.ch2_target_velocity = 0

        # --- Initialize underlying native SocketCAN driver ---
        try:
            self.bus = can.interface.Bus(interface=interface, channel=channel, bitrate=bitrate)
        except Exception as e:
            self.get_logger().error(f"Could not connect to CAN interface {channel}: {str(e)}")
            raise e

        # --- ROS 2 Publishers & Subscribers Pipeline ---
        self.sub_ch1_cmd = self.create_subscription(Int32, 'motor_ch1/cmd_vel', self.ch1_cmd_callback, 10)
        self.sub_ch2_cmd = self.create_subscription(Int32, 'motor_ch2/cmd_vel', self.ch2_cmd_callback, 10)

        self.pub_ch1_actual = self.create_publisher(Int32, 'motor_ch1/actual_vel', 10)
        self.pub_ch2_actual = self.create_publisher(Int32, 'motor_ch2/actual_vel', 10)
        self.pub_ch1_status = self.create_publisher(Int16, 'motor_ch1/statusword', 10)
        self.pub_ch2_status = self.create_publisher(Int16, 'motor_ch2/statusword', 10)

        # Bootup initialization 
        self.init_motor_controller()

        # Timers & Receiving Loop Thread
        self.control_timer = self.create_timer(0.02, self.control_loop_callback) # 50Hz Control Loop
        self.running = True
        self.rx_thread = threading.Thread(target=self.can_receive_thread, daemon=True)
        self.rx_thread.start()

    def init_motor_controller(self):
        """Puts node to operational state and sequences DS402 steps."""
        self.get_logger().info("Resetting Node Communications...")
        self.send_nmt_command(0x81)
        time.sleep(1.0) # Wait for MDC2460 reboot cycle

        self.get_logger().info("Moving Target Node to OPERATIONAL State...")
        self.send_nmt_command(0x01)
        time.sleep(0.1)

        # Transition standard DS402 sequence: Shutdown -> Switch On -> Operation Enabled
        self.get_logger().info("Sequencing DS402 State Engine to Operation Enabled...")
        self.send_rpdo_frames(0x0006, 0, 0x0006, 0) # Shutdown
        time.sleep(0.05)
        self.send_rpdo_frames(0x0007, 0, 0x0007, 0) # Switch On
        time.sleep(0.05)
        self.send_rpdo_frames(0x000F, 0, 0x000F, 0) # Enable Operation
        time.sleep(0.05)

    def send_nmt_command(self, cs):
        msg = can.Message(arbitration_id=self.COB_NMT, data=[cs, self.target_node_id], is_extended_id=False)
        self.bus.send(msg)

    def send_rpdo_frames(self, ch1_cw, ch1_vel, ch2_cw, ch2_vel):
        """Utilizes cantools database to encode signals into structured binary payloads."""
        # Pack Channel 1 Payload
        ch1_payload = self.msg_rpdo1.encode({'Controlword': ch1_cw, 'TargetVelocity': ch1_vel})
        msg1 = can.Message(arbitration_id=self.msg_rpdo1.frame_id, data=ch1_payload, is_extended_id=False)
        
        # Pack Channel 2 Payload
        ch2_payload = self.msg_rpdo2.encode({'Controlword': ch2_cw, 'TargetVelocity': ch2_vel})
        msg2 = can.Message(arbitration_id=self.msg_rpdo2.frame_id, data=ch2_payload, is_extended_id=False)
        
        try:
            self.bus.send(msg1)
            self.bus.send(msg2)
        except Exception as e:
            self.get_logger().warn(f"CAN Frame transmission failure: {e}")

    def ch1_cmd_callback(self, msg):
        self.ch1_target_velocity = msg.data

    def ch2_cmd_callback(self, msg):
        self.ch2_target_velocity = msg.data

    def control_loop_callback(self):
        """Cyclically called at 50Hz to dispatch driving demands downstream."""
        self.send_rpdo_frames(
            self.controlword_operational, self.ch1_target_velocity,
            self.controlword_operational, self.ch2_target_velocity
        )

    def can_receive_thread(self):
        """Background loop catching asynchronously returned telemetry frames via cantools."""
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg is None:
                    continue

                # Safely intercept frames configured within our active Database definition
                if msg.arbitration_id in self.db._frame_id_to_message:
                    decoded = self.db.decode_message(msg.arbitration_id, msg.data)
                    
                    if msg.arbitration_id == (0x180 + self.target_node_id):   # TPDO1_CH1
                        self.pub_ch1_actual.publish(Int32(data=int(decoded['ActualVelocity'])))
                        self.pub_ch1_status.publish(Int16(data=int(decoded['Statusword'])))
                        
                    elif msg.arbitration_id == (0x280 + self.target_node_id): # TPDO2_CH2
                        self.pub_ch2_actual.publish(Int32(data=int(decoded['ActualVelocity'])))
                        self.pub_ch2_status.publish(Int16(data=int(decoded['Statusword'])))
                        
            except Exception as e:
                self.get_logger().error(f"RX Parsing Failure inside Thread Worker: {str(e)}")

    def destroy_node(self):
        self.running = False
        self.control_timer.cancel()
        self.get_logger().info("Bringing driver down safely. Sending safe stop commands...")
        # Dispatch DS402 Quick Stop state machine modification
        self.send_rpdo_frames(0x0005, 0, 0x0005, 0)
        time.sleep(0.1)
        self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoboteqCantoolsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()