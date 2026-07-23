import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogError,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    share_path = get_package_share_directory("canbus_driver")

    # Load config files
    config = os.path.join(share_path, "config", "canbus_driver.yaml")
    db_file = os.path.join(share_path, "config", "badger.dbc")

    # Declare the CAN interface argument (default: from YAML)
    can_iface = LaunchConfiguration("can_interface", default="can32")
    declare_can_iface = DeclareLaunchArgument(
        "can_interface",
        default_value="can32",
        description="CAN interface name (e.g. can0, can1, can2, can32)"
    )

    # 1. First task: Shut down the existing CAN interface if it was left open
    destroy_canbus = ExecuteProcess(
        name="destroy_can",
        cmd=["sudo", "ip", "link", "set", can_iface, "down"],
        shell=False
    )

    # 2. Second task: Bring the CAN interface up
    init_canbus = ExecuteProcess(
        name="init_can",
        cmd=["sudo", "ip", "link", "set", can_iface, "up", "type", "can", "bitrate", "250000"],
        shell=False
    )

    # 3. Third task: Run the node
    smartec_motor_driver = Node(
        package="canbus_driver",
        executable="canbus_driver.py",
        name="canbus_driver",
        parameters=[config, {"dbc_path": db_file, "can_interface": can_iface}],
    )

    def start_driver_after_can_init(event, _context):
        if event.returncode == 0:
            return [smartec_motor_driver]
        return [
            LogError(msg=[
                "Failed to initialize ", can_iface, "; Badger driver will not start"
            ]),
            EmitEvent(event=Shutdown(reason="Badger CAN interface initialization failed")),
        ]

    
    # When 'destroy_canbus' finishes, start 'init_canbus'
    step_2_trigger = RegisterEventHandler(
        OnProcessExit(
            target_action=destroy_canbus,
            on_exit=[init_canbus]
        )
    )

    # When 'init_canbus' finishes, start the 'smartec_motor_driver' node
    step_3_trigger = RegisterEventHandler(
        OnProcessExit(
            target_action=init_canbus,
            on_exit=start_driver_after_can_init
        )
    )

    # Create launch description and add the initial trigger action
    ld = LaunchDescription()
    
    ld.add_action(declare_can_iface)
    ld.add_action(destroy_canbus)
    ld.add_action(step_2_trigger)
    ld.add_action(step_3_trigger)

    return ld
