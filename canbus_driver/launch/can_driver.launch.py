import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    share_path = get_package_share_directory("canbus_driver")

    # Load config files
    config = os.path.join(share_path, "config", "canbus_driver.yaml")
    db_file = os.path.join(share_path, "config", "badger.dbc")

    # 1. First task: Shut down the existing CAN interface if it was left open
    destroy_canbus = ExecuteProcess(
        name="destroy_can",
        cmd=["sudo", "ip", "link", "set", "can32", "down"],
        shell=False
    )

    # 2. Second task: Bring the CAN interface up
    init_canbus = ExecuteProcess(
        name="init_can",
        cmd=["sudo", "ip", "link", "set", "can32", "up", "type", "can", "bitrate", "250000"],
        shell=False
    )

    # 3. Third task: Run the node using your python virtual environment directly
    smartec_motor_driver = Node(
        package="canbus_driver",
        executable="canbus_driver.py",
        name="canbus_driver",
        parameters=[config, {"database_file": db_file}],
    )

    
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
            on_exit=[smartec_motor_driver]
        )
    )

    # Create launch description and add the initial trigger action
    ld = LaunchDescription()
    
    # We only add the FIRST action and the event chains to the description
    ld.add_action(destroy_canbus)
    ld.add_action(step_2_trigger)
    ld.add_action(step_3_trigger)

    return ld