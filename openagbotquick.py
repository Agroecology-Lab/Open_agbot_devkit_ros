import os, subprocess, sys, shutil

# --- CONFIGURATION ---
DOCKERFILE_URL = "https://raw.githubusercontent.com/robbrit/feldfreund_devkit_ros/refs/heads/main/docker/Dockerfile"
REPO_URL = "https://github.com/Agroecology-Lab/Open_agbot_devkit_ros.git"
TARGET_DIR = os.path.expanduser("~/open_agbot_ws")
DOCKER_DIR = os.path.join(TARGET_DIR, "docker")
DOCKERFILE_PATH = os.path.join(DOCKER_DIR, "Dockerfile")

def run_cmd(cmd, description):
    """Executes command and exits immediately on any error."""
    print(f"\n[🚀] {description}...")
    process = subprocess.Popen(cmd, shell=True)
    process.communicate()
    
    if process.returncode != 0:
        print(f"\n[❌] STOPPING: {description} failed.")
        sys.exit(1)

def apply_surgery(path):
    """Fixes the specific 'unknown instruction' syntax error on the fly."""
    print(f"[🩹] Patching Dockerfile syntax at line 45...")
    if not os.path.exists(path):
        return

    with open(path, 'r') as f:
        lines = f.readlines()

    fixed_lines = []
    for line in lines:
        # If the line contains the ros package but the previous line was missing a backslash
        # Docker sees it as a new (unknown) command. 
        # We look for the line that is likely missing the joiner:
        if "ros-humble-domain-bridge" in line and not fixed_lines[-1].strip().endswith("\\"):
            # Fix the PREVIOUS line by adding the backslash
            fixed_lines[-1] = fixed_lines[-1].rstrip() + " \\\n"
        fixed_lines.append(line)

    with open(path, 'w') as f:
        f.writelines(fixed_lines)

def main():
    print("=== Open-AgBot Production Setup (Self-Healing Version) ===")

    # 1. Workspace Setup
    if not os.path.exists(TARGET_DIR):
        run_cmd(f"git clone {REPO_URL} {TARGET_DIR}", "Cloning Base Repository")
    os.makedirs(DOCKER_DIR, exist_ok=True)
    os.chdir(TARGET_DIR)

    # 2. Fetch and Patch
    run_cmd(f"wget -q -O {DOCKERFILE_PATH} {DOCKERFILE_URL}", "Fetching Dockerfile")
    apply_surgery(DOCKERFILE_PATH)

    # 3. Generate Production docker-compose.yml
    with open("docker-compose.yml", "w") as f:
        f.write("""
services:
  open-agbot:
    build:
      context: .
      dockerfile: docker/Dockerfile
    container_name: open-agbot-main
    privileged: true
    network_mode: host
    ipc: host
    pid: host
    devices:
      - "/dev/ttyUSB0:/dev/ttyUSB0"
      - "/dev/ttyACM0:/dev/ttyACM0"
    deploy:
      resources:
        limits:
          cpus: '6.0'
          memory: 3000M
    restart: always
""")

    # 4. Build Stage (Fail-Fast)
    run_cmd("docker compose build", "Building ROS 2 Stack")

    # 5. Launch Stage
    run_cmd("docker compose up -d", "Launching Open-AgBot Containers")

    # 6. Success Output
    print("\n" + "="*50)
    print("✨  OPEN-AGBOT DEPLOYMENT SUCCESSFUL  ✨")
    print(f"📍 NiceGUI Dashboard: http://localhost:8080")
    print("="*50)

if __name__ == "__main__":
    main()
