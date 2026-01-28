from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    map_manager = Node(package="topological_navigation", executable="map_manager2.py", name="map_manager", parameters=[{"tmap_file": "/workspace/maps/test_map.yaml"}], arguments=['/workspace/maps/test_map.yaml'])
    map_manager = Node(package="topological_navigation", executable="map_manager2.py", name="map_manager", parameters=[{"tmap_file": "/workspace/maps/test_map.yaml"}], arguments=['/workspace/maps/test_map.yaml'])
    sim = LaunchConfiguration('sim')
    gps_port = LaunchConfiguration('gps_port')
    mcu_port = LaunchConfiguration('mcu_port')
    map_name = LaunchConfiguration('map_name', default='agbot_field')

    return LaunchDescription([nav2_controller, map_manager, 
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('gps_port', default_value='virtual'),
        DeclareLaunchArgument('mcu_port', default_value='virtual'),
        DeclareLaunchArgument('map_name', default_value='agbot_field'),

        # Driver Node
        Node(
            package='basekit_driver',
            executable='basekit_driver_node',
            parameters=[{'use_sim_time': sim, 'gps_port': gps_port, 'mcu_port': mcu_port}]
        ),

        # Web UI
        Node(
            package='basekit_ui',
            executable='basekit_ui_node',
            parameters=[{'use_sim_time': sim}]
        ),

        # Topological Navigation (AOC Branch)
        Node(
            package='topological_navigation',
            executable='navigation2.py',
            name='topological_navigation',
            parameters=[{'map_name': map_name, 'use_sim_time': sim}],
            output='screen'
        )
    ])

# Minimal Nav2 Controller addition
from launch_ros.actions import Node
nav2_controller = Node(
    package='nav2_controller',
    executable='controller_server',
    name='controller_server',
    output='screen',
    parameters=[{'use_sim_time': True}]
)
