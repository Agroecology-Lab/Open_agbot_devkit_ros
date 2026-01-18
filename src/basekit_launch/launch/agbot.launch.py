import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Path to the Driver Configuration file
    # This file contains the 'read_data' definitions that were causing the crash
    driver_config = os.path.join(
        get_package_share_directory('basekit_driver'),
        'config',
        'basekit_driver.yaml'
    )

    # 2. Path to the U-Blox Configuration file
    ublox_config = os.path.join(
        get_package_share_directory('basekit_launch'),
        'config',
        'ublox.yaml'
    )

    # 3. U-Blox GPS Node
    ublox_gps_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        namespace='',
        parameters=[ublox_config],
        output='screen'
    )

    # 4. Basekit Driver Node (The Motor Controller)
    # We pass the driver_config here to fix the ParameterNotDeclaredException
    basekit_driver_node = Node(
        package='basekit_driver',
        executable='basekit_driver_node',
        name='basekit_driver_node',
        output='screen',
        parameters=[driver_config]
    )

    return LaunchDescription([
        ublox_gps_node,
        basekit_driver_node
    ])
