#!/usr/bin/env python3
import argparse
import subprocess
import os
import shutil
import sys

def ensure_src_deps():
    if not os.path.exists("src/nmea_msgs"):
        print("Required message packages missing in src. Running install_deps.sh...")
        subprocess.run(["./install_deps.sh"], check=True)

# Call this inside your 'rebuild' or 'build' command logic

# Standardised workspace path used inside the container
CONTAINER_WS = "/open_agbot_ws"
IMAGE_TAG = "openagbot:dev"

# Terminal Colors
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
RESET = "\033[0m"

def run_maintenance(nuclear=False):
    """Clean build artifacts and Docker storage."""
    print(f"{YELLOW}--- Running {'Nuclear' if nuclear else 'Standard'} Maintenance ---{RESET}")
    folders = ['build', 'install', 'log']
    for folder in folders:
        if os.path.exists(folder):
            subprocess.run(["sudo", "rm", "-rf", folder])
    if nuclear:
        subprocess.run(["docker", "system", "prune", "-a", "--volumes", "-f"])

def build_docker_image(no_cache=False):
    """Build the base Docker image."""
    print(f"{GREEN}--- Building Docker Image [{IMAGE_TAG}] ---{RESET}")
    cmd = ["docker", "build", "-t", IMAGE_TAG, "-f", "docker/Dockerfile", "."]
    if no_cache: cmd.insert(3, "--no-cache")
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print(f"{RED}Error: Docker build failed.{RESET}")
        sys.exit(1)

def launch_container(mode="fast"):
    """Run the container. Fast mode skips all apt and build steps."""
    ws_path = os.getcwd()
    
    if mode == "fast" and not os.path.exists("install"):
        print(f"{YELLOW}[!] No 'install' folder found. Switching to REBUILD mode...{RESET}")
        mode = "rebuild"

    python_path = (
        "/opt/ros/humble/lib/python3.10/site-packages:"
        "/usr/lib/python3/dist-packages:"
        f"{CONTAINER_WS}/install/basekit_ui/lib/python3.10/site-packages:"
        f"{CONTAINER_WS}/install/basekit_driver/lib/python3.10/site-packages"
    )

    # NO MORE APT-GET UPDATE HERE
    base_setup = "source /opt/ros/humble/setup.bash"

    if mode == "fast":
        print(f"{GREEN}--- Fast Launch: No Internet/Compile Needed ---{RESET}")
        setup_cmds = f"{base_setup} && source install/setup.bash && ros2 launch basekit_launch basekit_launch.py"
    else:
        print(f"{YELLOW}--- Rebuild Launch: Compiling Workspace ---{RESET}")
        # Note: We still assume the Docker image has libasio-dev now
        setup_cmds = f"{base_setup} && colcon build --symlink-install && source install/setup.bash && ros2 launch basekit_launch basekit_launch.py"

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
        print("\nStopping...")

def main():
    parser = argparse.ArgumentParser(description="Open Agbot Devkit Management Tool")
    parser.add_argument('command', nargs='?', choices=['run', 'build', 'rebuild', 'clean'], default='run')
    parser.add_argument('--nuclear', action='store_true')
    parser.add_argument('--no-cache', action='store_true')
    args = parser.parse_args()

    if args.command == 'clean': run_maintenance(nuclear=args.nuclear)
    elif args.command == 'build': build_docker_image(no_cache=args.no_cache)
    elif args.command == 'rebuild': launch_container(mode="rebuild")
    elif args.command == 'run': launch_container(mode="fast")

if __name__ == "__main__":
    main()
