# Open AgBot DevKit (ROS 2)

An open-source, containerised ROS 2 stack for autonomous agricultural robotics. This repository provides the drivers and orchestration for the Open AgBot platform, featuring RTK-GNSS localization and ESP32-based hardware control.

Development is led by the <a href="https://agroecologylab.org.uk" target="_blank">Agroecology Lab </a> building on the core developed by <a href="https://github.com/zauberzeug/" target="_blank">Zauberzeug</a> 

(Latest work with Topological Nav is in the /dev branch, but under heavy development)
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

Ratings scale from 0.1 (Conceptual) to 1.0 (Production-Ready), with 0.0 indicating a planned or non-validated integration.


| Maturity | Feature | Description |
| :--- | :--- | :--- |
| **0.9** | **<a href="https://github.com/Agroecology-Lab/Open_agbot_devkit_ros" target="_blank">Containerized Deployment</a>** | Full ROS 2 Humble stack managed via Docker and the `manage.py` orchestration script. |
| **0.7** | **<a href="https://github.com/KumarRobotics/ublox" target="_blank">KumarRobotics Ublox Driver</a>** | Industry-standard driver providing high-bandwidth UBX binary data and 
| **0.6** | **<a href="https://github.com/Agroecology-Lab/Open_agbot_devkit_ros" target="_blank">Stable Device Addressing</a>** | Persistent symlinking via the `fixusb.py` utility to map hardware to `/dev/esp` and `/dev/gps`. |
| **0.5** | **<a href="https://github.com/Agroecology-Lab/Open_agbot_devkit_ros" target="_blank">Environment-Driven Configuration</a>** | Host-agnostic architecture using dynamic environment variables within the launch system. |
| **0.3** | **<a href="https://github.com/LCAS/sentor" target="_blank">Sentor Safety & Health Monitoring</a>** | Integrated hardware-software heartbeat and topic-based diagnostics to trigger automated recovery or emergency motor cut-off. |
| **0.3** | **<a href="https://github.com/Agroecology-Lab/Open_agbot_devkit_ros" target="_blank">Dynamic Hardware Abstraction</a>** | Lizard firmware engine integration for real-time ESP32 configuration via the basekit driver. |
| **0.4** | **<a href="https://github.com/Agroecology-Lab/Open_agbot_devkit_ros" target="_blank">Real-time Telemetry Dashboard</a>** | Web-based cockpit for monitoring battery and GPS health via the `basekit_ui` package. |
| **0.5** | **<a href="https://github.com/Agroecology-Lab/Open_agbot_devkit_ros" target="_blank">Stable Device Addressing</a>** | Persistent symlinking via the `fixusb.py` utility to map hardware to `/dev/esp` and `/dev/gps`. |
| **0.4** | **<a href="https://github.com/Agroecology-Lab/Open_agbot_devkit_ros" target="_blank">Environment-Driven Configuration</a>** | Host-agnostic architecture using dynamic environment variables within the launch system. |
| **0.6** | **<a href="https://github.com/LCAS/sentor" target="_blank">Sentor Safety & Health Monitoring</a>** | Integrated hardware-software heartbeat and topic-based diagnostics to trigger automated recovery or emergency motor cut-off. |
| **0.5** | **<a href="https://github.com/Agroecology-Lab/Open_agbot_devkit_ros" target="_blank">Dynamic Hardware Abstraction</a>** | Lizard firmware engine integration for real-time ESP32 configuration via the basekit driver. |
| **0.5** | **<a href="https://github.com/Agroecology-Lab/Open_agbot_devkit_ros" target="_blank">Real-time Telemetry Dashboard</a>** | Web-based cockpit for monitoring battery and GPS health via the `basekit_ui` package. |
| **0.1** | **<a href="https://github.com/LCAS/topological_navigation" target="_blank">Topological Navigation</a>** | Integration of the LCAS topological framework for graph-based semantic waypoint navigation. |
| **0.0** | **<a href="https://github.com/Agroecology-Lab/visual-multi-crop-row-navigation/tree/ROS2" target="_blank">Visual Crop-Row Navigation</a>** | Vision-based guidance system for following crop rows; currently in porting status for ROS 2. |
| **0.0** | **<a href="https://github.com/MoffKalast/vizanti/tree/ros2" target="_blank">Vizanti Web Visualization</a>** | Planned integration of a web-based mission planner and 3D visualizer for remote operations. |


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
