import os
import subprocess
import sys
import shutil
import socket

# --- CONFIGURATION ---
DOCKERFILE_URL = "https://raw.githubusercontent.com/robbrit/feldfreund_devkit_ros/refs/heads/main/docker/Dockerfile"
REPO_URL = "https://github.com/Agroecology-Lab/Open_agbot_devkit_ros.git"
GNSS_REPO_URL = "https://github.com/Lemvos/automatepro_gnss_driver"
TARGET_DIR = os.path.expanduser("~/open_agbot_ws")
DOCKER_DIR = os.path.join(TARGET_DIR, "docker")
DOCKERFILE_PATH = os.path.join(DOCKER_DIR, "Dockerfile")

def get_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0)
        s.connect(('10.254.254.254', 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"

def run_cmd(cmd, description):
    print(f"\n[🚀] {description}...")
    try:
        process = subprocess.Popen(cmd, shell=True)
        process.communicate()
        if process.returncode != 0:
            raise subprocess.CalledProcessError(process.returncode, cmd)
    except Exception as e:
        print(f"\n[❌] STOPPING: {description} failed.")
        sys.exit(1)

def main():
    local_ip = get_ip()
    print(f"=== Open-AgBot Setup (Standard COPY Mode) ===")

    # 1. Setup Folders
    if not os.path.exists(TARGET_DIR):
        run_cmd(f"git clone {REPO_URL} {TARGET_DIR}", "Cloning Base Repository")
    os.chdir(TARGET_DIR)

    # 2. Get GNSS Driver
    gnss_path = os.path.join(TARGET_DIR, "automatepro_gnss_driver")
    if not os.path.exists(gnss_path):
        run_cmd(f"git clone {GNSS_REPO_URL} {gnss_path}", "Cloning GNSS Driver")

    os.makedirs(DOCKER_DIR, exist_ok=True)

    # 3. Logic: Only add COPY lines for files that actually exist
    # This replaces the broken "2>/dev/null || true" hack
    optional_copies = ""
    if os.path.exists(os.path.join(TARGET_DIR, "requirements.txt")):
        optional_copies += "COPY requirements.txt /root/\nRUN pip install -r /root/requirements.txt || true\n"
    
    if os.path.exists(os.path.join(TARGET_DIR, "requirements-dev.txt")):
        optional_copies += "COPY requirements-dev.txt /root/\nRUN pip install -r /root/requirements-dev.txt || true\n"

    # 4. Write the Clean Dockerfile
    with open(DOCKERFILE_PATH, "w") as f:
        f.write(r"""FROM ros:humble

RUN apt-get update && apt-get install -y wget unzip curl

RUN mkdir -p /root/.lizard && \
    cd /root && \
    LATEST_VERSION=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    wget https://github.com/zauberzeug/lizard/releases/download/${LATEST_VERSION}/lizard_firmware_and_devtools_${LATEST_VERSION}_esp32.zip && \
    unzip lizard_firmware_and_devtools_${LATEST_VERSION}_esp32.zip -d /root/.lizard && \
    rm -f lizard_firmware_and_devtools_${LATEST_VERSION}_esp32.zip

RUN sed -i 's|http://archive.ubuntu.com/ubuntu/|https://archive.ubuntu.com/ubuntu/|g' /etc/apt/sources.list
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates gnupg2 lsb-release && update-ca-certificates
RUN rm -f /etc/apt/sources.list.d/ros2.sources
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] https://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

# ASIO and libvdpau
RUN apt-get update && apt-get install -y libvdpau1 libasio-dev || apt-get -f install -y

RUN apt-get install -y --no-install-recommends \
    -o Acquire::https::Verify-Peer=false \
    ros-humble-domain-bridge \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-foxglove-bridge \
    ros-humble-xacro \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-foxglove-msgs \
    ros-humble-tf-transformations \
    ros-humble-septentrio-gnss-driver \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-diagnostic-updater \
    ros-humble-usb-cam \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-image-tools \
    ros-humble-compressed-image-transport \
    ros-humble-axis-camera \
    python3-pip python3-serial python3-requests python3-yaml git nano

# Dynamically injected copies from Python logic
""" + optional_copies + r"""

COPY ./basekit_driver /workspace/src/basekit_driver
COPY ./basekit_launch /workspace/src/basekit_launch
COPY ./basekit_ui /workspace/src/basekit_ui
COPY ./automatepro_gnss_driver /workspace/src/automatepro_gnss_driver

WORKDIR /workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --parallel-workers 2
""")

    # 5. Build and Launch
    run_cmd("docker compose build", "Building ROS 2 Stack")
    run_cmd("docker compose up -d", "Starting AgBot")

    # 6. Start Portainer if not running
    if "portainer" not in subprocess.getoutput("docker ps -a"):
        subprocess.run("docker run -d -p 9443:9443 --name portainer --restart always -v /var/run/docker.sock:/var/run/docker.sock -v portainer_data:/data portainer/portainer-ce:latest", shell=True)

    print("\n" + "="*50)
    print("✨ SUCCESS: OPEN-AGBOT IS RUNNING")
    print(f"📍 NiceGUI Interface: http://{local_ip}:8080")
    print(f"📍 Portainer Admin:   https://{local_ip}:9443")
    print("="*50)

if __name__ == "__main__":
    main()
