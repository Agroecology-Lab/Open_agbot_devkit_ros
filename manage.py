#!/usr/bin/env python3
import subprocess, os, sys

IMAGE_NAME = "agbot_image"
CONTAINER_NAME = "open_agbot"
WORKSPACE = os.getcwd()

def run_cmd(cmd):
    return subprocess.run(cmd, shell=True)

def launch():
    print("🔍 Running fixusb.py...")
    run_cmd("python3 fixusb.py")
    
    mcu, gps = "/dev/ttyACM[0-9]", "/dev/ttyACM2"
    if os.path.exists(".env"):
        with open(".env", "r") as f:
            for line in f:
                if "MCU_PORT" in line: mcu = line.split("=")[1].strip()
                if "GPS_PORT" in line: gps = line.split("=")[1].strip()

    print(f"🛠️  PATCHING SOURCE...")
    # Ensure modules folder is a package
    run_cmd("touch src/basekit_driver/basekit_driver/modules/__init__.py")
    # Apply C++ GPS path fix
    run_cmd(f"sed -i 's|/dev/ttyACM[0-9]|{gps}|g' src/ublox/ublox_gps/src/node.cpp")

    # Update Driver Config
    with open("src/basekit_driver/config/basekit_driver.yaml", "w") as f:
        f.write(f"""
basekit_driver_node:
  ros__parameters:
    port: "{mcu}"
    read_data:
      list: ["battery", "encoder_left", "encoder_right"]
""")

    print(f"🚀 Building Full Workspace and Launching...")
    run_cmd("sudo chmod 666 /dev/ttyACM*")
    run_cmd(f"docker rm -f {CONTAINER_NAME} 2>/dev/null || true")

    # Removed --packages-select to ensure 'basekit_launch' is installed
    launch_cmd = (
        f"docker run -it --name {CONTAINER_NAME} --net=host --privileged "
        f"-v /dev:/dev -v {WORKSPACE}:/open_agbot_ws -w /open_agbot_ws "
        f"{IMAGE_NAME} bash -c '"
        f"source /opt/ros/humble/setup.bash && "
        f"colcon build --symlink-install && "
        f"source install/setup.bash && "
        f"ros2 launch basekit_launch master.launch.py'"
    )
    run_cmd(launch_cmd)

if __name__ == "__main__":
    launch()
