import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare Arguments to receive values from the Docker 'command'
    declare_gps_port = DeclareLaunchArgument('gps_port', default_value='/dev/ttyACM1')
    declare_mcu_port = DeclareLaunchArgument('mcu_port', default_value='/dev/ttyACM0')

    # Paths to configs
    basekit_launch_dir = get_package_share_directory('basekit_launch')
    config_file = os.path.join(basekit_launch_dir, 'config', 'basekit.yaml')
    ublox_config = os.path.join(get_package_share_directory('ublox_gps'), 'config', 'zed_f9p.yaml')

    # Driver Node
    basekit_driver_node = Node(
        package='basekit_driver',
        executable='basekit_driver_node',
        name='controller',
        parameters=[config_file, {'port': os.environ.get('MCU_PORT', '/dev/ttyACM0')}],
        remappings=[('/cmd_vel', '/cmd_vel_nav')]
    )

    # GPS Node
    ublox_gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ublox_gps'), 'launch', 'ublox_gps_node-launch.py')
        ),
        launch_arguments={'params_file': ublox_config, 'device': LaunchConfiguration('gps_port')}.items()
    )

    ui_node = Node(package='basekit_ui', executable='basekit_ui_node', name='web_ui')

    return LaunchDescription([
        declare_gps_port,
        declare_mcu_port,
        basekit_driver_node,
        ublox_gps_launch,
        ui_node
    ])
