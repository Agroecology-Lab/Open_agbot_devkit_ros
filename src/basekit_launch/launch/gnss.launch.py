import glob

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def find_gnss_port():
    """Find the first available serial port that might be a GNSS receiver."""
    patterns = [
        '/dev/ttyACM*',  # USB ACM devices
        '/dev/ttyUSB*',  # USB-Serial adapters
        '/dev/ttyS*'     # Standard serial ports
    ]

    for pattern in patterns:
        ports = glob.glob(pattern)
        for port in ports:
            # You might want to add more sophisticated detection here
            # For now, we'll take the first available port
            return port

    # Fallback to a default port if none found
    return '/dev/ttyACM0'


def generate_launch_description():
    """Generate launch description for U-blox F9P GNSS receiver using automatepro_gnss_driver."""
    # TF publishers for different frames
    tf_publishers = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=f'0 0 0 0 0 0 {source} {target}'.split(' '),
            name=f'tf_{source}_{target}'
        )
        for source, target in [
            ('base_link', 'imu'),
            ('imu', 'gnss'),
            ('imu', 'vsm'),
            ('imu', 'aux1')
        ]
    ]

    # Configuration file setup
    # Note: We keep the argument names similar but point to a default ublox config if available
    default_file_name = 'ublox_f9p.yaml'
    name_arg_file_name = 'file_name'
    arg_file_name = DeclareLaunchArgument(
        name_arg_file_name,
        default_value=TextSubstitution(text=str(default_file_name))
    )

    name_arg_file_path = 'path_to_config'
    arg_file_path = DeclareLaunchArgument(
        name_arg_file_path,
        default_value=[
            get_package_share_directory('devkit_launch'),
            '/config/',
            LaunchConfiguration(name_arg_file_name)
        ]
    )

    # Find the serial port
    serial_port = find_gnss_port()

    # Create a modified configuration for automatepro_gnss_driver (ublox_gps)
    config = {
        'device': serial_port,
        'uart1.baudrate': 921600,
        'frame_id': 'gnss',
        'config_on_startup': False  # Recommended by Lemvos for pre-configured devices
    }

    # Create the composable node for automatepro_gnss_driver
    # Package: ublox_gps, Plugin: ublox_node::UbloxNode
    composable_node = ComposableNode(
        name='ublox_gps_node',
        package='ublox_gps',
        plugin='ublox_node::UbloxNode',
        parameters=[
            LaunchConfiguration(name_arg_file_path),
            config
        ]
    )

    # Create the container
    container = ComposableNodeContainer(
        name='ublox_gps_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        emulate_tty=True,
        sigterm_timeout='20',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    return launch.LaunchDescription([
        arg_file_name,
        arg_file_path,
        container,
        *tf_publishers
    ])


