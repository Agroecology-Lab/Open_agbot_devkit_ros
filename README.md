# ⚠️ ARCHIVED / REPLACED
**This repository has been succeeded by the Jazzy-native Feldfreund Devkit.**

For the latest features, ROS 2 Jazzy support, and the new hardware abstraction layer, please move to:
👉 **[Feldfreund Devkit - Jazzy Branch](https://github.com/Agroecology-Lab/feldfreund_devkit_ros/tree/jazzy)**

---

This repo holds a legacy humble robot. In /main is a minimal version, in /dev a version that brings up Nav2 and Topological nav

# Open AgBot DevKit Humble (ROS 2)

An open-source, containerised ROS 2 stack for autonomous agricultural robotics. This repository provides the drivers and orchestration for the Open AgBot platform, featuring RTK-GNSS localization and ESP32-based hardware control.

Development is led by the <a href="https://agroecologylab.org.uk" target="_blank">Agroecology Lab </a> building on the core developed by <a href="https://github.com/zauberzeug/" target="_blank">Zauberzeug</a> 

---

## Quick Start

### 1. Clone the Repository
Open a terminal on your host machine and download the workspace:
```
git clone https://github.com/Agroecology-Lab/Open_agbot_devkit_ros.git
cd Open_agbot_devkit_ros
```
### 2. Initialization
Run the first-time setup script to configure your environment, install host dependencies, and build the Docker images:
```
python3 firstrun.py
```
### 3. Build & Launch
Use the management script to build the ROS 2 workspace and launch the robot stack. This script automatically handles hardware discovery and port permissions:
```
python3 manage.py
```
*Note: manage.py requires no arguments for standard operation. It automatically detects hardware ports (ESP32 & u-blox), updates your .env configuration, builds the workspace via colcon, and launches the containers.*

---

## Management & Tools

### manage.py
The primary entry point for the system. While it runs the full stack by default, it supports several optional arguments for development:

| Argument | Behaviour / Implementation                                                                 |
| :------- | :----------------------------------------------------------------------------------------- |
| (None)   | Runs fixusb.py, then executes docker compose up -d to immediately start the stack.         |
| build    | Executes docker compose build to compile the devkit images.                                |
| down     | Executes docker compose down to stop and remove the containers and networks.               |


- Auto-Detects Hardware: Identifies the real USB paths for the GPS and MCU on the host, so you don't have to guess if they are ACM0 or ACM2.

- Hardcodes the "Un-configurable": Uses sed to patch hardware paths directly into C++ source files on the host before they are compiled.

- Fixes "Root" Permissions: Uses sudo to wipe old build/ and install/ folders that the Docker container (running as root) locked down.

- Forces a Live Build: Runs colcon build inside the container to ensure the software is compiled specifically for your host's current sensor setup.

- Overlays the Workspace: Mounts your host’s src and install folders over the container’s internal files, making your local code the "Source of Truth."


### Interactive Shell
To enter the running container for debugging or manual ROS 2 commands:
```
./login.sh
```
### Diagnostics
If hardware is connected but topics are not flowing, run the diagnostic tool from inside the container:

#### After running ./login.sh
```
python3 src/agbot-diagnostic.py
```

![TUI Status.](https://raw.githubusercontent.com/Agroecology-Lab/Open_agbot_devkit_ros/refs/heads/main/assets/Screenshot%20From%202026-01-21%2018-07-45.png)

---

## Web User Interface

![TUI Status.](https://raw.githubusercontent.com/Agroecology-Lab/Open_agbot_devkit_ros/refs/heads/main/assets/Screenshot%20From%202026-01-21%2020-37-03.png)


---
## Roadmap

Check new roadmap here https://github.com/Agroecology-Lab/feldfreund_devkit_ros/tree/sowbot?tab=readme-ov-file#sowbot-roadmap


---

## Project Structure

* src/basekit_driver: ROS 2 node interfacing with the ESP32 MCU for battery status, bumpers, and odometry.
* src/ublox: Driver suite for ZED-F9P RTK-GNSS modules.
* src/basekit_launch: Centralized launch files to coordinate sensor fusion and driver startup.
* src/basekit_ui: Web-based dashboard for real-time robot monitoring and control.
* manage.py / firstrun.py: DevOps tooling for container and environment lifecycle.

---

## Hardware Requirements

The stack is pre-configured for the <a href="https://agroecologylab.co.uk" target="_blank">Agroecology Lab reference designs </a>

You may also have success with alternate platforms such as

- Compute: Linux-based host (Avaota A1, Raspberry Pi, Jetson) running Docker.
- MCU: ESP32 Control Board 
- GPS: u-blox ZED-F9P 
- Communication: UART.
- <a href="https://lizard.dev/module_reference/" target="_blank">Hardware & Motor drivers supported by Lizard </a>M

---

##  Topic Reference

| Topic | Type | Description |
| :--- | :--- | :--- |
| /battery_state | sensor_msgs/BatteryState | Voltage and charge status |
| /ublox_gps_node/fix | sensor_msgs/NavSatFix | Centimeter-level RTK global position |
| /odom | nav_msgs/Odometry | Wheel encoder feedback and dead reckoning |
| /cmd_vel | geometry_msgs/Twist | Velocity commands sent to the motor controllers |
| /diagnostics | diagnostic_msgs/DiagnosticArray | Aggregated system health status |

---

## License

This project is licensed under the Apache License 2.0. See the LICENSE file for details.
