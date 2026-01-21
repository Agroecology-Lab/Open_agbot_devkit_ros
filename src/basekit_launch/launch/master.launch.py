import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Basekit Driver Node (MCU)
    driver_node = Node(
        package='basekit_driver',
        executable='basekit_driver_node',
        name='basekit_driver_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 2. U-blox GPS Node
    # Path to your config file we fixed earlier
    config_file = os.path.join(
        get_package_share_directory('basekit_launch'),
        'config',
        'ublox.yaml'
    )
    
    gps_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='screen',
        parameters=[config_file]
    )

    # 3. Basekit UI Node
    ui_node = Node(
        package='basekit_ui',
        executable='basekit_ui_node',
        name='basekit_ui_node',
        output='screen',
        respawn=True
    )

    return LaunchDescription([
        driver_node,
        gps_node,
        ui_node
    ])
