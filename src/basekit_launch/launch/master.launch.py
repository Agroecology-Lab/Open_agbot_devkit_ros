import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Load configuration file for the driver
    config = os.path.join(
        get_package_share_directory('basekit_launch'),
        'config',
        'thinkpad_override.yaml'
    )

    # 1. Grab hardware ports from the environment (with safety defaults)
    # This is where the magic happens: it reads the -e flags from Docker!
    gps_port = os.environ.get('GPS_PORT', '/dev/ttyACM1')
    mcu_port = os.environ.get('MCU_PORT', '/dev/ttyACM0')

    print(f"🚀 [LAUNCH] Starting with GPS: {gps_port} and MCU: {mcu_port}")

    return LaunchDescription([
        # U-Blox GPS Node
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps_node',
            # We now use the variable instead of a hardcoded string
            parameters=[{
                'device': gps_port, 
                'uart1.baudRate': 115200,
                'tmode3': 0
            }]
        ),

        # Basekit Driver (The ESP32/MCU)
        Node(
            package='basekit_driver',
            executable='basekit_driver_node',
            name='basekit_driver_node',
            # We pass the port via parameter here too if the driver supports it
            parameters=[config, {'port': mcu_port}]
        ),

        # UI Node
        Node(
            package='basekit_ui', 
            executable='basekit_ui_node', 
            name='basekit_ui_node'
        )
    ])
