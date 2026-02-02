# Open AgBot DevKit (ROS 2)

An open-source, containerised ROS 2 stack for autonomous agricultural robotics. This repository provides the drivers and orchestration for the Open AgBot platform, featuring RTK-GNSS localization and ESP32-based hardware control.

Development is led by the <a href="https://agroecologylab.org.uk" target="_blank">Agroecology Lab </a> building on the core developed by <a href="https://github.com/zauberzeug/" target="_blank">Zauberzeug</a> 


This /dev branch, is under heavy development and may be broken at any given moment, for a somewhat stable but less feature complete version check out the /main branch.


## Quick Start

### 1. Clone the Repository
Open a terminal on your host machine and download the workspace:
```
git clone -b dev https://github.com/Agroecology-Lab/Open_agbot_devkit_ros.git
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
## Roadmap

Ratings scale from 0.1 (Conceptual) to 1.0 (Production-Ready), with 0.0 indicating a planned or non-validated integration.


### AgBot Development Status Matrix

| Maturity | Feature | Description |
| :--- | :--- | :--- |
| **0.9** | **Containerised Deployment** | Full ROS 2 Humble stack managed via Docker and the `manage.py` orchestration script. |
| **0.8** | **[KumarRobotics Ublox Driver](https://github.com/KumarRobotics/ublox)** | Modified driver providing high-bandwidth UBX binary data via a fail-safe boot sequence. |
| **0.7** | **Stable Device Addressing** | Persistent symlinking via the `fixusb.py` utility to map hardware to `/dev/esp` and `/dev/gps`. |
| **0.6** | **[Sentor Safety & Health Monitoring](https://github.com/LCAS/sentor)** | Integrated hardware-software heartbeat and topic-based diagnostics to trigger automated recovery or emergency motor cut-off. |
| **0.5** | **Dynamic Hardware Abstraction** | Lizard firmware integration for real-time ESP32 configuration via the basekit driver. |
| **0.5** | **Real-time Telemetry & Teleop Dashboard** | Web-based cockpit for joystick control, monitoring battery and GPS health via the `basekit_ui` package. |
| **0.1** | **[Topological Navigation](https://github.com/LCAS/topological_navigation)** | Integration of the LCAS topological framework for graph-based semantic waypoint navigation. |
| **0.0** | **[Visual Crop-Row Navigation](https://github.com/Agroecology-Lab/visual-multi-crop-row-navigation/tree/ROS2)** | Vision-based guidance system for following crop rows; currently in porting status for ROS 2. |
| **0.0** | **[Vizanti Web Visualisation](https://github.com/MoffKalast/vizanti/tree/ros2)** | Planned integration of a web-based mission planner and 3D visualiser for remote operations. |



---

## Management & Tools

### manage.py
The primary entry point for the system. While it runs the full stack by default, it supports several optional arguments for development:

| Command                  | Logic / Argument | Resulting Action                                                                 |
|--------------------------|------------------|----------------------------------------------------------------------------------|
| `./manage.py`            | (No arguments)   | Runs `run_runtime()` immediately using the existing image.                       |
| `./manage.py build`      | `build`          | Runs `run_build(full=False)`. Re-compiles your code in seconds.                  |
| `./manage.py full-build` | `full-build`     | Runs `run_build(full=True)`. Re-downloads ROS/MongoDB/Drivers (10+ mins).        |
| `./manage.py stop`       | `stop` / `down`  | Force-kills the running container and cleans up the ROS 2 network state.         |


### Interactive Shell
To enter the running container for debugging or manual ROS 2 commands:
```
./login.sh
```
### Diagnostics
If hardware is connected but topics are not flowing, run the diagnostic tool from inside the container:

#### After running ./login.sh
```
./agbot-diagnostic.py
```

![TUI Status.](https://raw.githubusercontent.com/Agroecology-Lab/Open_agbot_devkit_ros/refs/heads/main/assets/Screenshot%20From%202026-01-21%2018-07-45.png)

You can also make it verbose with

```
./agbot-diagnostic.py full
```


### Dev branch Topic reference


● /BASEKIT_DRIVER_NODE
  ├─ Publishers: /battery_state: sensor_msgs/msg/BatteryState, /bumper_back_state: std_msgs/msg/Bool, /bumper_front_bottom_state: std_msgs/msg/Bool, /bumper_front_top_state: std_msgs/msg/Bool

● /BEHAVIOR_SERVER
  ├─ Publishers: /behavior_server/transition_event: lifecycle_msgs/msg/TransitionEvent, /bond: bond/msg/Status, /cmd_vel: geometry_msgs/msg/Twist, /parameter_events: rcl_interfaces/msg/ParameterEvent

● /BT_NAVIGATOR
  ├─ Publishers: /bond: bond/msg/Status, /bt_navigator/transition_event: lifecycle_msgs/msg/TransitionEvent, /parameter_events: rcl_interfaces/msg/ParameterEvent, /rosout: rcl_interfaces/msg/Log

● /BT_NAVIGATOR_NAVIGATE_THROUGH_POSES_RCLCPP_NODE
  ├─ Publishers: /behavior_tree_log: nav2_msgs/msg/BehaviorTreeLog, /parameter_events: rcl_interfaces/msg/ParameterEvent, /rosout: rcl_interfaces/msg/Log

● /BT_NAVIGATOR_NAVIGATE_TO_POSE_RCLCPP_NODE
  ├─ Publishers: /behavior_tree_log: nav2_msgs/msg/BehaviorTreeLog, /parameter_events: rcl_interfaces/msg/ParameterEvent, /rosout: rcl_interfaces/msg/Log

● /CONTROLLER_SERVER
  ├─ Publishers: /bond: bond/msg/Status, /cmd_vel: geometry_msgs/msg/Twist, /controller_server/transition_event: lifecycle_msgs/msg/TransitionEvent, /cost_cloud: sensor_msgs/msg/PointCloud2

● /GLOBAL_COSTMAP/GLOBAL_COSTMAP
  ├─ Publishers: /global_costmap/costmap: nav_msgs/msg/OccupancyGrid, /global_costmap/costmap_raw: nav2_msgs/msg/Costmap, /global_costmap/costmap_updates: map_msgs/msg/OccupancyGridUpdate, /global_costmap/global_costmap/transition_event: lifecycle_msgs/msg/TransitionEvent

● /LIFECYCLE_MANAGER_NAVIGATION
  ├─ Publishers: /bond: bond/msg/Status, /diagnostics: diagnostic_msgs/msg/DiagnosticArray, /parameter_events: rcl_interfaces/msg/ParameterEvent, /rosout: rcl_interfaces/msg/Log

● /LOCAL_COSTMAP/LOCAL_COSTMAP
  ├─ Publishers: /local_costmap/costmap: nav_msgs/msg/OccupancyGrid, /local_costmap/costmap_raw: nav2_msgs/msg/Costmap, /local_costmap/costmap_updates: map_msgs/msg/OccupancyGridUpdate, /local_costmap/local_costmap/transition_event: lifecycle_msgs/msg/TransitionEvent

● /MAP_MANAGER
  ├─ Publishers: /parameter_events: rcl_interfaces/msg/ParameterEvent, /rosout: rcl_interfaces/msg/Log, /tf: tf2_msgs/msg/TFMessage, /topological_map: topological_navigation_msgs/msg/TopologicalMap

● /MAP_SERVER
  ├─ Publishers: /bond: bond/msg/Status, /map: nav_msgs/msg/OccupancyGrid, /map_server/transition_event: lifecycle_msgs/msg/TransitionEvent, /parameter_events: rcl_interfaces/msg/ParameterEvent

● /NAVIGATION_EXECUTOR
  ├─ Publishers: /boundary_checker: nav_msgs/msg/Path, /center_node/pose: geometry_msgs/msg/PoseStamped, /current_edge: std_msgs/msg/String, /parameter_events: rcl_interfaces/msg/ParameterEvent

● /NAVIGATION_EXECUTOR
  ├─ Publishers: /boundary_checker: nav_msgs/msg/Path, /center_node/pose: geometry_msgs/msg/PoseStamped, /current_edge: std_msgs/msg/String, /parameter_events: rcl_interfaces/msg/ParameterEvent

● /NAVIGATION_EXECUTOR
  ├─ Publishers: /boundary_checker: nav_msgs/msg/Path, /center_node/pose: geometry_msgs/msg/PoseStamped, /current_edge: std_msgs/msg/String, /parameter_events: rcl_interfaces/msg/ParameterEvent

● /NAVIGATION_EXECUTOR
  ├─ Publishers: /boundary_checker: nav_msgs/msg/Path, /center_node/pose: geometry_msgs/msg/PoseStamped, /current_edge: std_msgs/msg/String, /parameter_events: rcl_interfaces/msg/ParameterEvent

● /NAVIGATION_EXECUTOR
  ├─ Publishers: /boundary_checker: nav_msgs/msg/Path, /center_node/pose: geometry_msgs/msg/PoseStamped, /current_edge: std_msgs/msg/String, /parameter_events: rcl_interfaces/msg/ParameterEvent

● /PLANNER_SERVER
  ├─ Publishers: /bond: bond/msg/Status, /parameter_events: rcl_interfaces/msg/ParameterEvent, /plan: nav_msgs/msg/Path, /planner_server/transition_event: lifecycle_msgs/msg/TransitionEvent

● /ROSBRIDGE_WEBSOCKET
  ├─ Publishers: /client_count: std_msgs/msg/Int32, /connected_clients: rosbridge_msgs/msg/ConnectedClients, /parameter_events: rcl_interfaces/msg/ParameterEvent, /rosout: rcl_interfaces/msg/Log

● /STATIC_MAP_TO_ODOM
  ├─ Publishers: /parameter_events: rcl_interfaces/msg/ParameterEvent, /rosout: rcl_interfaces/msg/Log, /tf_static: tf2_msgs/msg/TFMessage

● /STATIC_ODOM_TO_BASE
  ├─ Publishers: /parameter_events: rcl_interfaces/msg/ParameterEvent, /rosout: rcl_interfaces/msg/Log, /tf_static: tf2_msgs/msg/TFMessage

● /TOPOLOGICAL_LOCALISATION
  ├─ Publishers: /closest_edges: topological_navigation_msgs/msg/ClosestEdges, /closest_node: std_msgs/msg/String, /closest_node_distance: std_msgs/msg/Float32, /current_node: std_msgs/msg/String

● /TRANSFORM_LISTENER_IMPL_5909EBFC1970
  ├─ Publishers: /rosout: rcl_interfaces/msg/Log

● /TRANSFORM_LISTENER_IMPL_5EA934DEBA90
  ├─ Publishers: /rosout: rcl_interfaces/msg/Log

● /TRANSFORM_LISTENER_IMPL_743C40006D20
  ├─ Publishers: /rosout: rcl_interfaces/msg/Log

● /UBLOX_GPS_NODE
  ├─ Publishers: /diagnostics: diagnostic_msgs/msg/DiagnosticArray, /nmea: nmea_msgs/msg/Sentence, /parameter_events: rcl_interfaces/msg/ParameterEvent, /rosout: rcl_interfaces/msg/Log

● /WEB_UI
  ├─ Publishers: /cmd_vel: geometry_msgs/msg/Twist, /emergency_stop: std_msgs/msg/Bool, /parameter_events: rcl_interfaces/msg/ParameterEvent, /rosout: rcl_interfaces/msg/Log


---

## Web User Interface

![TUI Status.](https://raw.githubusercontent.com/Agroecology-Lab/Open_agbot_devkit_ros/refs/heads/main/assets/Screenshot%20From%202026-01-21%2020-37-03.png)


---

## Hardware Requirements

The stack is pre-configured for the <a href="https://sowbot.co.uk" target="_blank">Sowbot reference designs </a>

You may also have success with alternate platforms such as

- Compute: Linux-based host (Avaota A1, Raspberry Pi, Jetson) running Docker.
- MCU: ESP32 Control Board 
- GPS: u-blox ZED-F9P 
- Communication: UART.
- <a href="https://lizard.dev/module_reference/" target="_blank">Hardware & Motor drivers supported by Lizard </a>


---

## License

This project is licensed under the Apache License 2.0. See the LICENSE file for details.
