import os
import subprocess
import sys

def write_file(path, content):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(content.strip())
    print(f"✅ Written: {path}")

def fix_cmake(path):
    if not os.path.exists(path):
        print(f"❌ Could not find {path}")
        return
    with open(path, 'r') as f:
        content = f.read()
    
    # Check if 'install(DIRECTORY config' is missing
    if "install(DIRECTORY" in content and "config" not in content:
        # Simple replacement to add config to existing directory install
        content = content.replace("DIRECTORY launch", "DIRECTORY launch config")
        with open(path, 'w') as f:
            f.write(content)
        print(f"✅ Updated CMakeLists.txt: {path}")
    elif "install(DIRECTORY config" not in content:
        # Append a new install block
        with open(path, 'a') as f:
            f.write("\ninstall(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})\n")
        print(f"✅ Added install block to CMakeLists.txt: {path}")
    else:
        print(f"ℹ️ CMakeLists.txt already contains config installation.")

# 1. Configuration Contents
driver_yaml = """
basekit_driver_node:
  ros__parameters:
    read_data:
      list: ["odom", "bms", "bumper"]
      odom: {default: 0.0}
      bms: {default: 0.0}
      bumper: {default: 0.0}
    serial_port: "/dev/ttyACM1"
    baud_rate: 115200
"""

ublox_yaml = """
ublox_gps_node:
  ros__parameters:
    device: "/dev/ttyACM0"
    frame_id: "gps"
    baudrate: 9600
    tmode3: 0
    nav_rate: 1
    publish: {all: true}
"""

print("--- Step 1: Writing YAML Configs ---")
write_file('/workspace/src/basekit_driver/config/basekit_driver.yaml', driver_yaml)
write_file('/workspace/src/basekit_launch/config/ublox.yaml', ublox_yaml)

print("\n--- Step 2: Fixing Build Files ---")
fix_cmake('/workspace/src/basekit_launch/CMakeLists.txt')

print("\n--- Step 3: Cleaning and Building (Limited Resources) ---")
subprocess.run(["pkill", "-9", "python3"])
subprocess.run(["rm", "-rf", "/workspace/build", "/workspace/install"])

# Using --parallel-workers 1 to prevent 'Killed' error on ThinkPad
build_cmd = ["colcon", "build", "--symlink-install", "--parallel-workers", "1"]
result = subprocess.run(build_cmd, cwd="/workspace")

if result.returncode != 0:
    print("\n❌ Build failed. Check the errors above.")
    sys.exit(1)

print("\n--- Step 4: Verification Check ---")
paths_to_check = [
    "/workspace/install/basekit_driver/share/basekit_driver/config/basekit_driver.yaml",
    "/workspace/install/basekit_launch/share/basekit_launch/config/ublox.yaml"
]

all_good = True
for p in paths_to_check:
    if os.path.exists(p):
        print(f"✅ Verified in Install: {p}")
    else:
        print(f"❌ MISSING in Install: {p}")
        all_good = False

if all_good:
    print("\n" + "="*50)
    print("🚀 SUCCESS! SYSTEM READY.")
    print("Run: source install/setup.bash && ros2 launch basekit_launch master.launch.py")
    print("="*50)
