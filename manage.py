#!/usr/bin/env python3
import subprocess
import os
import sys
import time

# Configuration
IMAGE_NAME = "openagbot:dev"
CONTAINER_NAME = "open_ag_runtime"

def run_build(full=False):
    """
    Handles both Light and Full builds using the multi-stage Dockerfile.
    - Light (build): Targets the 'runtime' stage. Fast, uses cached dependencies.
    - Full (full-build): Rebuilds everything from scratch without cache.
    """
    print("🧹 Cleaning up old container artifacts...")
    subprocess.run(["docker", "rm", "-f", CONTAINER_NAME], capture_output=True)

    if full:
        print("🔥 [FULL-BUILD] Rebuilding all dependencies and workspace (No Cache)...")
        # Forces a complete rebuild of Stage 1 (MongoDB, ROS 2, Pip)
        cmd = [
            "docker", "build", "--no-cache",
            "-t", IMAGE_NAME,
            "-f", "docker/Dockerfile", "."
        ]
    else:
        print("⚡ [BUILD-LIGHT] Rebuilding workspace code only (Using Cache)...")
        # Targets the final stage; skips Stage 1 if the environment hasn't changed
        cmd = [
            "docker", "build",
            "--target", "runtime",
            "-t", IMAGE_NAME,
            "-f", "docker/Dockerfile", "."
        ]

    result = subprocess.run(cmd)
    if result.returncode == 0:
        print(f"✅ {'Full' if full else 'Light'} build successful.")
    else:
        print(f"❌ {'Full' if full else 'Light'} build failed.")
        sys.exit(result.returncode)

def run_runtime(extra_args):
    """
    Hardware discovery and ROS 2 Launch.
    """
    # 1. Hardware Discovery (Run fixusb.py if it exists)
    if os.path.exists('fixusb.py'):
        print("🔍 Running hardware discovery (fixusb.py)...")
        subprocess.run(['python3', 'fixusb.py'], check=True)

    # 2. Parse Environment Variables
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

    # 3. Final Cleanup before Start
    subprocess.run(["docker", "rm", "-f", CONTAINER_NAME], capture_output=True)
    
    if not is_virtual:
        print(f"✅ Hardware Mode | GPS: {gps_p} | MCU: {mcu_p}")
        subprocess.run(['sudo', 'chmod', '666', gps_p, mcu_p], check=False)
        
        # Non-blocking MCU Wakeup
        print(f"🔋 Sending wake-up signal to MCU on {mcu_p}...")
        os.system(f"stty -F {mcu_p} 115200 && (echo 's' > {mcu_p} &)")
        time.sleep(0.2)
    else:
        print("🎮 Simulation Mode: No hardware required.")

    # 4. ROS 2 Launch with isolated Volume Mounts
    try:
        launch_args = [arg for arg in extra_args if ":=" in arg]
        sim_flag = "sim:=true" if is_virtual else "sim:=false"
        
        print(f"🚀 Starting {CONTAINER_NAME}...")
        cmd = [
            "docker", "run", "-it", "--rm", "--name", CONTAINER_NAME,
            "--net=host", "--privileged", "--env-file", ".env",
            "-v", "/dev:/dev", 
            # DANGER AVOIDED: We mount only /src. 
            # This prevents host binaries from polluting the container's /install.
            "-v", f"{os.getcwd()}/src:/workspace/src", 
            "-w", "/workspace",
            IMAGE_NAME, "bash", "-c",
            f"source /opt/ros/humble/setup.bash && source install/setup.bash && "
            f"ros2 launch basekit_launch basekit_launch.py {sim_flag} "
            f"device:={gps_p} mcu_port:={mcu_p} {' '.join(launch_args)}"
        ]
        subprocess.run(cmd)
    except KeyboardInterrupt:
        print("\n🛑 Shutdown sequence initiated.")

if __name__ == "__main__":
    raw_args = sys.argv[1:]
    
    # Handle Build Commands
    if "full-build" in raw_args:
        run_build(full=True)
        sys.exit(0)
    elif "build" in raw_args:
        run_build(full=False)
        sys.exit(0)
    
    # Handle Launch (Implicitly runs 'up')
    # Filter out 'up' if user typed 'manage.py up'
    processed_args = [a for a in raw_args if a != "up"]
    run_runtime(processed_args)
