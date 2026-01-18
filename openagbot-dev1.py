import os
import subprocess

def launch():
    # Ensure we are launching from the root of the workspace
    ws_path = os.getcwd()
    
    # 1. Define Python Paths
    python_path = (
        "/opt/ros/humble/lib/python3.10/site-packages:"
        "/usr/lib/python3/dist-packages:"
        "/workspace/install/basekit_ui/lib/python3.10/site-packages:"
        "/workspace/install/basekit_driver/lib/python3.10/site-packages:"
        "$PYTHONPATH" 
    )

    # 2. Define the sequence of commands
    # NOTE: I added --parallel-workers 1 to prevent your ThinkPad from crashing during build
    setup_cmds = (
        "cd /workspace && "
        "source /opt/ros/humble/setup.bash && "
        "colcon build --symlink-install --parallel-workers 1 && "
        "source install/setup.bash && "
        "ros2 launch basekit_launch master.launch.py"
    )

    # 3. Construct Docker command
    docker_cmd = [
        "docker", "run", "-it", 
        # "--rm",  <-- REMOVED to prevent container from disappearing
        "--name", "agbot_dev",           # Named for easy restart
        "--privileged",                  
        "--network", "host",             
        "-v", "/dev:/dev",               
        "-v", f"{ws_path}:/workspace",   
        "-w", "/workspace",              
        "-e", f"PYTHONPATH={python_path}",
        "-e", "PYTHONUNBUFFERED=1",
        "openagbot-basekit:latest",
        "bash", "-c", setup_cmds
    ]

    print(f"--- Starting AgBot Dev Environment in Container ---")
    print(f"--- Persistence: Container will be saved as 'agbot_dev' if it stops ---")

    try:
        subprocess.run(docker_cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"\nDocker failed with exit code {e.returncode}")
        print("You can try restarting it with: docker start -ai agbot_dev")
    except KeyboardInterrupt:
        print("\nStopping AgBot DevKit...")

if __name__ == "__main__":
    launch()
