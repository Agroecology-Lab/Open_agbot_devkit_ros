import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. Hardware Launch
    # This starts ublox_gps and the basekit_driver
    agbot_launch_dir = os.path.join(get_package_share_directory('basekit_launch'), 'launch')
    agbot_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(agbot_launch_dir, 'agbot.launch.py'))
    )

    # 2. Start the UI / Web Server
    # We only call this ONCE to avoid the port 8080 "Address already in use" error.
    # We use the absolute path to bypass the 'libexec' registration issues.
    gui_server = ExecuteProcess(
        cmd=['python3', '/workspace/src/basekit_ui/basekit_ui/ui_node.py'],
        output='screen'
    )

    return LaunchDescription([
        agbot_hardware,
        gui_server
    ])
