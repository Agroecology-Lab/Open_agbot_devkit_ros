#!/usr/bin/env python3
import argparse
import subprocess
import os
import sys

# Standardised workspace path used inside the container
CONTAINER_WS = "/open_agbot_ws"
IMAGE_TAG = "openagbot:dev"

# Terminal Colors
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
RESET = "\033[0m"

def run_unwedge():
    """Clear hardware locks and zombie processes on the host safely."""
    print(f"{GREEN}--- Cleaning Host Environment & Resetting Hardware ---{RESET}")
    
    # 1. Kill the Web UI port 8080 (Forceful release)
    subprocess.run("sudo fuser -k 8080/tcp || true", shell=True, capture_output=True)
    
    # 2. Release the Serial Ports (Forceful release)
    subprocess.run("sudo fuser -k /dev/ttyACM0 || true", shell=True, capture_output=True)
    subprocess.run("sudo fuser -k /dev/ttyACM1 || true", shell=True, capture_output=True)
    
    # 3. Targeted kill of ROS processes ONLY 
    # (Avoids killing this manage.py script)
    subprocess.run("sudo pkill -9 -f _node || true", shell=True)
    subprocess.run("sudo pkill -9 -f ros2 || true", shell=True)
    
    # 4. Sync Hardware Baudrates
    print(f"{GREEN}--- Syncing Hardware to 115200 ---{RESET}")
    subprocess.run("sudo stty -F /dev/ttyACM0 115200 raw -echo", shell=True)
    subprocess.run("sudo stty -F /dev/ttyACM1 115200 raw -echo", shell=True)

def run_maintenance(nuclear=False):
    """Clean build artifacts and Docker storage."""
    print(f"{YELLOW}--- Running Maintenance ---{RESET}")
    folders = ['build', 'install', 'log']
    for folder in folders:
        if os.path.exists(folder):
            subprocess.run(["sudo", "rm", "-rf", folder])
    if nuclear:
        subprocess.run(["docker", "system", "prune", "-a", "--volumes", "-f"])

def launch_container(mode="fast"):
    """Brings up the container using docker run."""
    # Step 1: Clean the host so the container can start fresh
    run_unwedge()
    
    ws_path = os.getcwd()
    
    # Auto-switch to rebuild if install folder is missing
    if mode == "fast" and not os.path.exists("install"):
        print(f"{YELLOW}[!] No 'install' folder found. Switching to REBUILD mode...{RESET}")
        mode = "rebuild"

    # Set up PYTHONPATH for ROS 2 inside the container
    python_path = (
        "/opt/ros/humble/lib/python3.10/site-packages:"
        "/usr/lib/python3/dist-packages:"
        f"{CONTAINER_WS}/install/basekit_ui/lib/python3.10/site-packages:"
        f"{CONTAINER_WS}/install/basekit_driver/lib/python3.10/site-packages"
    )

    # Launch command points to the master launch file we verified
    launch_cmd = "ros2 launch basekit_launch master.launch.py"
    base_setup = "source /opt/ros/humble/setup.bash"

    if mode == "fast":
        print(f"{GREEN}--- Fast Launch: Bringing up Container ---{RESET}")
        setup_cmds = f"{base_setup} && source install/setup.bash && {launch_cmd}"
    else:
        print(f"{YELLOW}--- Rebuild Launch: Compiling then Launching ---{RESET}")
        setup_cmds = f"{base_setup} && colcon build --symlink-install && source install/setup.bash && {launch_cmd}"

    # The actual Docker activation command
    docker_cmd = [
        "docker", "run", "-it", "--rm",
        "--privileged", "--network", "host",
        "-v", "/dev:/dev",
        "-v", f"{ws_path}:{CONTAINER_WS}",
        "-e", f"PYTHONPATH={python_path}",
        "-e", "PYTHONUNBUFFERED=1",
        "-e", "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
        IMAGE_TAG, "bash", "-c", setup_cmds
    ]

    try:
        subprocess.run(docker_cmd)
    except KeyboardInterrupt:
        print(f"\n{YELLOW}Stopping Robot...{RESET}")
        run_unwedge()

def main():
    parser = argparse.ArgumentParser(description="Open Agbot Devkit Management Tool")
    parser.add_argument('command', nargs='?', choices=['run', 'build', 'rebuild', 'clean'], default='run')
    parser.add_argument('--nuclear', action='store_true')
    args = parser.parse_args()

    if args.command == 'clean': 
        run_maintenance(nuclear=args.nuclear)
    elif args.command == 'rebuild': 
        launch_container(mode="rebuild")
    else: 
        launch_container(mode="fast")

if __name__ == "__main__":
    main()
