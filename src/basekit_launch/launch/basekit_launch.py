import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. PATH DEFINITIONS
    nav2_params = os.path.join(get_package_share_directory('basekit_launch'), 'config', 'nav2_params.yaml')
    
    # Corrected to match your specific file on disk to prevent map_server failure
    nav2_map_path = '/workspace/maps/nav2_test_map.yaml'
    topo_map_path = '/workspace/maps/test_map.yaml'
    
    # 2. HARDWARE MAPPING (Defaults set to your physical host ports)
    mcu_port = os.environ.get('MCU_PORT', '/dev/ttyACM1')
    gps_port = os.environ.get('GPS_PORT', '/dev/ttyACM0')
    
    return LaunchDescription([
        # --- SECTION 1: INFRASTRUCTURE ---
        Node(
            package='rosbridge_server', 
            executable='rosbridge_websocket', 
            name='rosbridge_websocket',
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),

        # --- SECTION 2: NAV2 BACK-END (Lifecycle Managed) ---
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': nav2_map_path}, nav2_params],
            output='screen'
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[nav2_params],
            output='screen'
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[nav2_params],
            output='screen'
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[nav2_params],
            output='screen'
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[nav2_params],
            output='screen'
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'autostart': True, 
                'node_names': [
                    'map_server', 
                    'planner_server', 
                    'controller_server', 
                    'behavior_server', 
                    'bt_navigator'
                ]
            }],
            output='screen'
        ),

        # --- SECTION 3: TOPOLOGICAL STACK ---
        Node(
            package="topological_navigation", 
            executable="map_manager2.py",
            name="map_manager", 
            arguments=[topo_map_path],
            parameters=[{'no_db': True}],
            output='screen'
        ),
        Node(
            package="topological_navigation",
            executable="localisation2.py",
            name="topological_localisation",
            output='screen'
        ),
        Node(
            package="topological_navigation",
            executable="navigation2.py",
            name="navigation_executor",
            output='screen'
        ),

        # --- SECTION 4: HARDWARE DRIVERS & UI ---
        Node(
            package='ublox_gps', 
            executable='ublox_gps_node', 
            name='ublox_gps_node',
            parameters=[{
                'device': gps_port, 
                'nav_rate': 1,
                'tmode3': 0,  # CRITICAL: Prevents driver crash on ZED-F9P
                'uart1.baudrate': 9600
            }],
            output='screen'
        ),
        Node(
            package='basekit_driver', 
            executable='basekit_driver_node', 
            name='basekit_driver_node',
            parameters=[{'port': mcu_port}],
            output='screen'
        ),
        Node(
            package='basekit_ui', 
            executable='basekit_ui_node', 
            name='web_ui',
            parameters=[{'webroot': '/workspace/src/basekit_ui/web'}],
            output='screen'
        )
    ])
