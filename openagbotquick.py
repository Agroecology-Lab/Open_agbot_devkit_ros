import os, subprocess, sys, shutil, socket

# --- CONFIGURATION ---
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
    except Exception: return "127.0.0.1"

def run_cmd(cmd, description):
    print(f"\n[🚀] {description}...")
    process = subprocess.Popen(cmd, shell=True)
    process.communicate()
    if process.returncode != 0:
        print(f"\n[❌] STOPPING: {description} failed."); sys.exit(1)

def main():
    local_ip = get_ip()
    print(f"=== Open-AgBot Setup: Symlink Collision Fix ===")

    if not os.path.exists(TARGET_DIR):
        run_cmd(f"git clone {REPO_URL} {TARGET_DIR}", "Cloning Base Repository")
    os.chdir(TARGET_DIR)

    gnss_path = os.path.join(TARGET_DIR, "automatepro_gnss_driver")
    if not os.path.exists(gnss_path):
        run_cmd(f"git clone {GNSS_REPO_URL} {gnss_path}", "Cloning GNSS Driver")

    os.makedirs(DOCKER_DIR, exist_ok=True)

    with open(DOCKERFILE_PATH, "w") as f:
        f.write(r"""FROM ros:humble

# 1. System Essentials
RUN apt-get update && apt-get install -y \
    wget unzip curl git python3-pip libasio-dev python3-serial && \
    rm -rf /var/lib/apt/lists/*

# 2. Lizard Firmware
RUN mkdir -p /root/.lizard && cd /root && \
    LATEST_VERSION=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    wget https://github.com/zauberzeug/lizard/releases/download/${LATEST_VERSION}/lizard_firmware_and_devtools_${LATEST_VERSION}_esp32.zip && \
    unzip lizard_firmware_and_devtools_${LATEST_VERSION}_esp32.zip -d /root/.lizard && \
    rm -f *.zip

# 3. Python Stack
RUN pip install --upgrade pip
RUN pip install nicegui pyserial requests pyyaml

# 4. ROS Dependencies
RUN rosdep update --include-eol-distros || true

# 5. Copy Source
WORKDIR /workspace
COPY ./basekit_driver ./src/basekit_driver
COPY ./basekit_launch ./src/basekit_launch
COPY ./basekit_ui ./src/basekit_ui
COPY ./automatepro_gnss_driver ./src/automatepro_gnss_driver

# 6. Rosdep
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro humble \
    --skip-keys="pyserial nicegui"

# 7. CLEAN WATERFALL BUILD
# Added --symlink-install to EVERY step to prevent collision
# Added rm -rf build/ install/ to ensure clean workspace
RUN . /opt/ros/humble/setup.sh && \
    rm -rf build/ install/ && \
    colcon build --symlink-install --packages-select ublox_serialization --parallel-workers 1 && \
    colcon build --symlink-install --packages-select ublox_msgs --parallel-workers 1 && \
    colcon build --symlink-install --parallel-workers 2
""")

    with open("docker-compose.yml", "w") as f:
        f.write("""
services:
  open-agbot:
    build: { context: ., dockerfile: docker/Dockerfile }
    container_name: open-agbot-main
    privileged: true
    network_mode: host
    restart: always
""")

    run_cmd("docker compose build", "Executing Clean Waterfall Build")
    run_cmd("docker compose up -d", "Launching AgBot")

    print(f"\n" + "="*50)
    print(f"✨ SUCCESS! EVERYTHING BUILT AND RUNNING")
    print(f"📍 UI: http://{local_ip}:8080")
    print(f"📍 Management: https://{local_ip}:9443")
    print(f"="*50)

if __name__ == "__main__":
    main()
