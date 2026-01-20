import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. GPS Node (Rover Mode)
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps_node',
            output='screen',
            parameters=[{
                'device': '/dev/ttyACM0',
                'baudrate': 115200,
                'frame_id': 'gps',
                # Ensure TMODE is disabled (Rover mode)
                'tmode3': 0, 
                'nav_rate': 1,
                'publish': {'all': True}
            }]
        ),
        # 2. Driver Node (ThinkPad Serial Port)
        Node(
            package='basekit_driver',
            executable='basekit_driver_node',
            name='basekit_driver_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM1',
                'baudrate': 115200,
                'read_data': {'list': ['v_batt', 'motor_status']}
            }]
        ),
        # 3. UI Node
        Node(
            package='basekit_ui',
            executable='basekit_ui_node',
            name='basekit_ui_node',
            output='screen'
        )
    ])
