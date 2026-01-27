#!/usr/bin/env python3
import subprocess
import os
import sys
import signal
import time
import argparse

# Configuration
IMAGE_NAME = "openagbot:dev"
CONTAINER_NAME = "open_agbot"

def run_clean():
    """Historical cleanup to manage ThinkPad storage limits."""
    print("Purging docker build cache and unused containers...")
    subprocess.run(["docker", "system", "prune", "-f"], check=False)
    subprocess.run(["docker", "builder", "prune", "-af"], check=False)

def run_build():
    print("Cleaning old build artifacts...")
    subprocess.run(['sudo', 'rm', '-rf', 'build/', 'install/', 'log/'], check=False)
    
    print("Building Docker Image...")
    # Using the standard Agroecology Lab Dockerfile location
    subprocess.run(["docker", "build", "-t", IMAGE_NAME, "-f", "docker/Dockerfile", "."], check=True)

    print("Syncing build artifacts to host (Sequential Mode)...")
    # Mapping current user/group IDs prevents permission issues on the host
    user_info = f"{os.getuid()}:{os.getgid()}"
    
    cmd_sync = [
        "docker", "run", "--rm", 
        "--user", user_info,
        "-v", f"{os.getcwd()}:/workspace", 
        "-w", "/workspace",
        IMAGE_NAME, "bash", "-c", 
        "source /opt/ros/humble/setup.bash && colcon build --symlink-install --executor sequential --parallel-workers 1"
    ]
    subprocess.run(cmd_sync, check=True)
    
    # Post-build prune to save disk space on ThinkPad
    subprocess.run(["docker", "builder", "prune", "-f"], check=False)
    print("Build and Sync Complete.")

def run_runtime(extra_args):
    # 1. Hardware Discovery
    if os.path.exists('fixusb.py'):
        subprocess.run(['python3', 'fixusb.py'], check=True)

    # 2. Load Detected Ports
    ports = {}
    if os.path.exists('.env'):
        with open('.env', 'r') as f:
            for line in f:
                if '=' in line and not line.startswith('#'):
                    k, v = line.strip().split('=', 1)
                    ports[k.strip()] = v.strip()
    
    gps_p = ports.get('GPS_PORT', 'virtual')
    mcu_p = ports.get('MCU_PORT', 'virtual')
    is_virtual = (gps_p == 'virtual' and mcu_p == 'virtual')
    
    # Cleanup previous instances
    subprocess.run(["docker", "rm", "-f", CONTAINER_NAME], capture_output=True)

    # 3. Handle Simulation if Hardware is missing (Ghost Mode)
    sim_proc = None
    if is_virtual:
        print("Mode: Simulation (Ghost)")
        if os.path.exists('sim_lizard.sh'):
            sim_proc = subprocess.Popen(['bash', 'sim_lizard.sh'], stdout=subprocess.DEVNULL, preexec_fn=os.setsid)
            time.sleep(1)

    try:
        pass_through_args = " ".join(extra_args)
        sim_flag = "sim:=true" if is_virtual else "sim:=false"
        
        print(f"Launching {CONTAINER_NAME} | GPS: {gps_p} | MCU: {mcu_p}")
        
        cmd = [
            "docker", "run", "-it", "--rm", "--name", CONTAINER_NAME,
            "--net=host", "--privileged", "--env-file", ".env",
            "-v", "/dev:/dev", 
            "-v", f"{os.getcwd()}:/workspace", "-w", "/workspace",
            IMAGE_NAME, "bash", "-c",
            f"source /opt/ros/humble/setup.bash && source install/setup.bash && "
            f"ros2 launch basekit_launch basekit_launch.py {sim_flag} "
            f"gps_port:={gps_p} mcu_port:={mcu_p} {pass_through_args}"
        ]
        subprocess.run(cmd)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        if sim_proc:
            # Kill the simulation process group
            os.killpg(os.getpgid(sim_proc.pid), signal.SIGTERM)

if __name__ == "__main__":
    # Check for specific utility commands
    if len(sys.argv) > 1:
        if sys.argv[1] == "build":
            run_build()
            sys.exit(0)
        elif sys.argv[1] == "clean":
            run_clean()
            sys.exit(0)

    # Default logic: Auto-build if install is missing, then run
    if not os.path.exists('install/setup.bash'):
        run_build()
    
    # Pass any extra arguments (like --ros-args) to the runtime
    # If the first arg was 'up', we skip it to avoid passing 'up' to ros2 launch
    args_to_pass = sys.argv[1:]
    if args_to_pass and args_to_pass[0] == "up":
        args_to_pass = args_to_pass[1:]
        
    run_runtime(args_to_pass)
