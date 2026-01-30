import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

# --- VERSIONING ---
VERSION = "5.6-STABLE-FLATTENED"

def generate_launch_description():
    ld = LaunchDescription()

    # --- ENVIRONMENT FIXES ---
    os.environ['LD_LIBRARY_PATH'] = os.environ.get('LD_LIBRARY_PATH', '') + ':/opt/ros/humble/lib'

    # 1. Hardware Inference Logic
    gps_port = os.environ.get('GPS_PORT', '/dev/ttyACM1')
    mcu_port = os.environ.get('MCU_PORT', '/dev/ttyACM0')

    is_hardware_present = os.path.exists(gps_port) and os.path.exists(mcu_port)
    is_sim = not is_hardware_present
    mode_label = "HARDWARE" if is_hardware_present else "SIMULATION"

    # Version Banner
    ld.add_action(LogInfo(msg="======================================================"))
    ld.add_action(LogInfo(msg=f"🚀 AGBOT MASTER LAUNCH V{VERSION}"))
    ld.add_action(LogInfo(msg=f"🎮 MODE: {mode_label} | GPS: {gps_port}"))
    ld.add_action(LogInfo(msg="======================================================"))

    # 2. UI & Communication Bridge
    ld.add_action(Node(
        package='rosbridge_server', executable='rosbridge_websocket',
        name='rosbridge_websocket', output='screen',
        parameters=[{'port': 9090, 'address': '0.0.0.0'}]
    ))

    ld.add_action(Node(
        package='basekit_ui', executable='basekit_ui_node',
        name='web_ui', output='screen',
        parameters=[{'use_sim_time': is_sim}]
    ))

    # 3. Navigation Stack & Lifecycle Management
    # Standardised to avoid 'local_costmap/local_costmap' nesting
    ld.add_action(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': is_sim,
            'autostart': True,
            'node_names': ['controller_server', 'local_costmap_node']
        }]
    ))

    ld.add_action(Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[{'use_sim_time': is_sim}]
    ))

    ld.add_action(Node(
        package='nav2_costmap_2d', executable='nav2_costmap_2d',
        name='local_costmap_node', 
        output='screen', 
        parameters=[{'use_sim_time': is_sim}]
    ))

    # 4. Topological Navigation
    ld.add_action(Node(
        package="topological_navigation", executable="map_manager2.py",
        name="map_manager", arguments=['/workspace/maps/test_map.yaml'],
        parameters=[{"tmap_file": "/workspace/maps/test_map.yaml"}]
    ))

    # 5. Conditional Driver Loading
    if is_hardware_present:
        config_path = '/workspace/src/ublox/ublox_gps/config/zed_f9p.yaml'
        
        ld.add_action(Node(
            package='ublox_gps', 
            executable='ublox_gps_node', 
            name='ublox_gps_node', 
            output='screen',
            parameters=[
                config_path, 
                {
                    'device': gps_port,
                    'baudrate': 460800,
                    'uart1.baudrate': 460800,
                    'frame_id': 'gps',
                    'publish': {'nav': {'pvt': True}},
                    'ubx': {'enabled': True},
                    'meas_rate': 200,
                    'nav_rate': 1,
                    'tmode3': 0,
                    'config_on_startup': True
                }
            ],
            respawn=True
        ))
        
        ld.add_action(Node(
            package='basekit_driver', executable='basekit_driver_node', 
            name='basekit_driver_node', 
            parameters=[{'port': mcu_port}], 
            respawn=True
        ))
    
    return ld
