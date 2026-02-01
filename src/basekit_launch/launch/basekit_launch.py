import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

# --- VERSIONING ---
VERSION = "6.0-STABLE-CONDITIONAL"

def generate_launch_description():
    ld = LaunchDescription()

    # --- ENVIRONMENT FIXES ---
    os.environ['LD_LIBRARY_PATH'] = os.environ.get('LD_LIBRARY_PATH', '') + ':/opt/ros/humble/lib'

    # 1. Hardware Inference Logic
    # Ports are passed from manage.py via .env
    gps_port = os.environ.get('GPS_PORT', 'virtual')
    mcu_port = os.environ.get('MCU_PORT', 'virtual')

    is_gps_present = os.path.exists(gps_port) and gps_port != 'virtual'
    is_mcu_present = os.path.exists(mcu_port) and mcu_port != 'virtual'
    
    # Simulation mode logic
    is_sim = not (is_gps_present and is_mcu_present)
    mode_label = "REAL HARDWARE" if not is_sim else "GHOST SIMULATION"

    # Version Banner
    ld.add_action(LogInfo(msg="======================================================"))
    ld.add_action(LogInfo(msg=f"🚀 AGBOT MASTER LAUNCH V{VERSION}"))
    ld.add_action(LogInfo(msg=f"🎮 MODE: {mode_label}"))
    ld.add_action(LogInfo(msg=f"📡 GPS: {gps_port} ({'FOUND' if is_gps_present else 'OFFLINE/VIRTUAL'})"))
    ld.add_action(LogInfo(msg=f"📡 MCU: {mcu_port} ({'FOUND' if is_mcu_present else 'OFFLINE/VIRTUAL'})"))
    ld.add_action(LogInfo(msg="======================================================"))

    # 2. UI & Communication Bridge (Always Run)
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

    # 3. Conditional Navigation Stack
    # If GPS is missing, we disable Nav2 to prevent Lifecycle Manager service hangs
    if is_gps_present:
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
        
        # GPS Driver
        config_path = '/workspace/src/ublox/ublox_gps/config/zed_f9p.yaml'
        ld.add_action(Node(
            package='ublox_gps', executable='ublox_gps_node', name='ublox_gps_node', 
            output='screen',
            parameters=[config_path, {'device': gps_port, 'config_on_startup': True}],
            respawn=True
        ))
    else:
        ld.add_action(LogInfo(msg="⚠️ NAV STACK DISABLED: Hardware GPS not detected."))

    # 4. Topological Navigation (Always Run for UI/Mapping)
    tmap_path = '/workspace/maps/test_map.yaml'
    ld.add_action(Node(
        package="topological_navigation", 
        executable="map_manager2.py",
        name="map_manager", 
        arguments=[tmap_path],
        parameters=[{
            "tmap_file": tmap_path,
            "use_sim_time": is_sim,
            "pointset": "agbot_fields" 
        }],
        output='screen'
    ))

    # 5. Basekit Driver (Handles Sim internal to node)
    ld.add_action(Node(
        package='basekit_driver', executable='basekit_driver_node', 
        name='basekit_driver_node', 
        parameters=[{'port': mcu_port, 'sim': is_sim}], 
        respawn=True
    ))

    return ld
