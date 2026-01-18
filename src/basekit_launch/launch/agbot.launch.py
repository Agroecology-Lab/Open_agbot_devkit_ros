from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # 1. GPS Node
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps_node',
            parameters=[{
                'device': '/dev/ttyACM0',
                'uart1/baudrate': 9600,
                'frame_id': 'gps',
                'nav_rate': 1,
                'tmode3': 0
            }],
            respawn=True
        ),
        
        # 2. Basekit Driver
        TimerAction(period=3.0, actions=[
            Node(
                package='basekit_driver',
                executable='basekit_driver_node',
                name='basekit_driver_node',
                parameters=[{
                    'serial_port': '/dev/ttyACM1',
                    'read_data.list': ['odom', 'bms', 'bumper'],
                    'read_data.odom.type': 'json',
                    'read_data.odom.default': '',
                    'read_data.bms.type': 'json',
                    'read_data.bms.default': '',
                    'read_data.bumper.type': 'json',
                    'read_data.bumper.default': ''
                }],
                output='screen',
                respawn=True
            )
        ]),
        
        # 3. Web UI Node
        Node(
            executable='python3',
            arguments=['/workspace/src/basekit_ui/basekit_ui/ui_node.py'],
            name='web_ui',
            output='screen'
        )
    ])
