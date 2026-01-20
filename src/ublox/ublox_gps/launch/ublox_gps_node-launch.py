import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Detect the GPS port from the environment, defaulting to ACM1
    env_gps_port = "/dev/gps_hardware"

    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(get_package_share_directory('ublox_gps'), 'config', 'zed_f9p.yaml'),
                              description='Path to the config file'),
        
        # We explicitly force the device parameter to use our environment variable
        Node(
            package='ublox_gps',
            executable='ublox_gps_node', prefix=['taskset -c 0,1'],
            name='ublox_gps_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'device': env_gps_port}  # This override wins!
            ]
        )
    ])
