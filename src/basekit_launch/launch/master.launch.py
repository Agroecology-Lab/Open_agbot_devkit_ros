import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Setup paths to other launch files
    basekit_driver_dir = get_package_share_directory('basekit_driver')
    basekit_ui_dir = get_package_share_directory('basekit_ui')
    
    return LaunchDescription([
        # 2. Basekit Driver Node (Serial with Microcontroller)
        Node(
            package='basekit_driver',
            executable='basekit_driver_node',
            name='basekit_driver_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM1',
                'baudrate': 115200
            }]
        ),

        # 3. U-Blox GPS Node (Hard-coded for 115200)
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps_node',
            namespace='',
            output='screen',
            parameters=[{
                'device': '/dev/ttyACM0',
                'baudrate': 115200,
                'uart1.baudrate': 115200,
                'frame_id': 'gps',
                'publish': {
                    'all': True,
                    'nav': {'pvt': True}
                },
                'inf': {'all': True},
                'gnss': {
                    'gps': True,
                    'glonass': True,
                    'galileo': True,
                    'beidou': True
                }
            }]
        ),

        # 4. Basekit UI Node (Web Interface)
        Node(
            package='basekit_ui',
            executable='basekit_ui_node',
            name='basekit_ui_node',
            output='screen'
        ),

        # 5. Static Transforms (IMU, GNSS, Base Link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_link_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_imu_gnss',
            arguments=['0', '0', '0', '0', '0', '0', 'imu', 'gnss']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_imu_vsm',
            arguments=['0', '0', '0', '0', '0', '0', 'imu', 'vsm']
        ),
    ])
