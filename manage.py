#!/usr/bin/env python3
import argparse
import subprocess
import os
import sys
import time

# --- Configuration ---
CONTAINER_WS = "/open_agbot_ws"
IMAGE_TAG = "openagbot:dev"

# Terminal Colors for Verbose Feedback
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BLUE = "\033[94m"
BOLD = "\033[1m"
RESET = "\033[0m"

def log(msg, color=BLUE):
    print(f"{color}{BOLD}[manage.py]{RESET} {msg}")

def run_hardware_sync():
    """Detects hardware via fixusb.py and loads variables into the process."""
    log("Running hardware detection (fixusb.py)...", YELLOW)
    try:
        # Run the external script to refresh .env
        subprocess.run(["python3", "fixusb.py"], check=True)
        
        if os.path.exists(".env"):
            with open(".env", "r") as f:
                for line in f:
                    if "=" in line and not line.startswith("#"):
                        key, value = line.strip().split("=", 1)
                        os.environ[key] = value
            log(f"✅ Hardware Mapped: GPS={os.environ.get('GPS_PORT')} | MCU={os.environ.get('MCU_PORT')}", GREEN)
        else:
            log("❌ .env file missing after fixusb.py!", RED)
            sys.exit(1)
    except Exception as e:
        log(f"❌ Error during hardware sync: {e}", RED)
        sys.exit(1)

def run_unwedge():
    """Forcefully clears locks and resets baud rates at the OS level."""
    log("Clearing hardware locks and syncing baud rates to 115200...", YELLOW)
    
    gps = os.environ.get('GPS_PORT', '/dev/ttyACM1')
    mcu = os.environ.get('MCU_PORT', '/dev/ttyACM0')

    # 1. Kill any process holding the ports (like previous failed runs)
    subprocess.run(f"sudo fuser -k {gps} 8080/tcp {mcu} || true", shell=True, capture_output=True)
    
    # 2. Fix permissions
    subprocess.run(f"sudo chmod 666 {gps} {mcu} || true", shell=True)
    
    # 3. Force OS-level baud rate (The 'Nuclear' fix for 'Could not configure baud' errors)
    subprocess.run(f"sudo stty -F {gps} 115200 raw -echo", shell=True)
    subprocess.run(f"sudo stty -F {mcu} 115200 raw -echo", shell=True)
    
    # 4. Clean up zombie ROS processes
    subprocess.run("sudo pkill -9 -f _node || true", shell=True)
    subprocess.run("sudo pkill -9 -f ros2 || true", shell=True)
    log("✨ Host environment is now clean.", GREEN)

def run_maintenance(nuclear=False):
    """Deep clean of build artifacts using sudo to bypass permission locks."""
    log("Starting maintenance clean...", YELLOW)
    folders = ['build', 'install', 'log']
    for folder in folders:
        if os.path.exists(folder):
            log(f"🗑️  Removing {folder} as root...", RED)
            subprocess.run(["sudo", "rm", "-rf", folder])
    
    if nuclear:
        log("☢️  Pruning all Docker data (Nuclear Option)...", RED)
        subprocess.run(["docker", "system", "prune", "-a", "--volumes", "-f"])
    log("✅ Cleanup complete.", GREEN)

def launch_container(mode="fast"):
    """Orchestrates the build and run within the Docker environment."""
    run_hardware_sync()
    run_unwedge()
    
    ws_path = os.getcwd()
    
    # Check if we can actually do a 'fast' launch
    if mode == "fast" and not os.path.exists("install"):
        log("⚠️  No 'install' folder found. Forcing REBUILD mode...", YELLOW)
        mode = "rebuild"

    # Capture ports for environment passing
    gps = os.environ.get('GPS_PORT', '/dev/ttyACM1')
    mcu = os.environ.get('MCU_PORT', '/dev/ttyACM0')











# Build the PYTHONPATH to include our custom packages
    python_path = (
        "/opt/ros/humble/lib/python3.10/site-packages:"
        "/usr/lib/python3/dist-packages:"
        f"{CONTAINER_WS}/install/basekit_ui/lib/python3.10/site-packages:"
        f"{CONTAINER_WS}/install/basekit_driver/lib/python3.10/site-packages"
    )

    
    
# Prepare ROS commands
    # These variables MUST match the 'DeclareLaunchArgument' names in master.launch.py
    launch_cmd = (
        f"ros2 launch basekit_launch master.launch.py "
        f"gps_port:={gps} "
        f"mcu_port:={mcu} "
        f"ublox_params_file:={CONTAINER_WS}/src/ublox/ublox_gps/config/zed_f9p.yaml "
        f"device:={gps} "
        f"baudrate:=115200"
    )




    base_setup = "source /opt/ros/humble/setup.bash"


    if mode == "fast":
        log("🚀 Fast Launch: Skipping build...", GREEN)
        inner_cmd = f"{base_setup} && source install/setup.bash && {launch_cmd}"
    else:
        log("🛠️  Rebuild Launch: Running colcon build...", YELLOW)
        # Use --symlink-install so edits to Python/YAML in src/ take effect immediately
        inner_cmd = f"{base_setup} && colcon build --symlink-install && source install/setup.bash && {launch_cmd}"

    # The Docker Command - Passing Host Environment to Container
    docker_cmd = [
        "docker", "run", "-it", "--rm",
        "--privileged", "--network", "host",
        "-v", "/dev:/dev",
        "-v", f"{ws_path}:{CONTAINER_WS}",
        "-e", f"PYTHONPATH={python_path}",
        "-e", f"GPS_PORT={gps}",
        "-e", f"MCU_PORT={mcu}",
        "-e", "PYTHONUNBUFFERED=1",
        "-e", "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
        IMAGE_TAG, "bash", "-c", inner_cmd
    ]

    log(f"📦 Starting Docker Container ({IMAGE_TAG})...", BLUE)
    try:
        subprocess.run(docker_cmd)
    except KeyboardInterrupt:
        print(f"\n{YELLOW}🛑 Shutdown signal received. Cleaning up...{RESET}")
        run_unwedge()

def main():
    parser = argparse.ArgumentParser(description="Open Agbot Devkit Manager")
    parser.add_argument('command', nargs='?', choices=['run', 'build', 'rebuild', 'clean'], default='run')
    parser.add_argument('--nuclear', action='store_true', help="Prune Docker system during clean")
    args = parser.parse_args()

    if args.command == 'clean': 
        run_maintenance(nuclear=args.nuclear)
    elif args.command in ['rebuild', 'build']: 
        launch_container(mode="rebuild")
    else: 
        launch_container(mode="fast")

if __name__ == "__main__":
    main()
