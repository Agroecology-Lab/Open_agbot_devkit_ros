# Open AgBot Devkit ROS

This repository contains the ROS 2 workspace and Docker configuration for the Open AgBot development kit. This project is a fork of the original work by [Zauberzeug](https://github.com/zauberzeug) and has been optimized for agricultural robotics research using ROS 2 Humble.

## Project Structure

The workspace is organized to manage hardware drivers, user interfaces, and system-wide launch configurations within a containerized environment.

* src/basekit_driver: Manages serial communication with the motor controller.
* src/basekit_ui: A web-based dashboard for robot monitoring and control.
* src/basekit_launch: Contains system-wide launch files and hardware configurations.
* src/ublox: Integrated GNSS drivers for precise positioning.

## Prerequisites

1. Ubuntu 22.04 LTS.
2. Docker and Docker Compose.
3. Git.

## Installation and Setup

Clone the repository to your local machine:
   ```
   git clone https://github.com/Agroecology-Lab/Open_agbot_devkit_ros.git
   cd Open_agbot_devkit_ros
   ```
---

## Quick Start Sequence

Follow these steps to initialize the robot:

1. **Connect Hardware:** Plug in the u-blox GPS and ESP32 MCU via USB.
2. **Scan and Map:**
   ```
   python3 fixusb.py
   ```

3. **Initial Setup**
Run once on new systems. This is the automated installer designed to configure a fresh Linux installation for the Agbot environment.

* **Function:** Checks for and installs system-level dependencies (Docker, Git, Python3, Pip) and pulls the latest base images from Docker Hub.
* **Output:** Configures user groups (adding the user to 'docker' and 'dialout') and initializes the ROS 2 workspace structure.
* **Usage:** `
```
python3 openagbotquick.py
```

4. **Launch Stack:**
```
python3 openagbot-dev.py
```

5. **Access UI: Open a web browser on the host machine and navigate to http://localhost:8080.**
   

# Open Agbot Helper Scripts detail

Use these scripts to manage the hardware-to-software bridge on your host machine.

### 1. fixusb.py (Hardware Mapping)
**Run this first.** This script prevents the common issue where Linux swaps the port assignments (/dev/ttyACM0 vs /dev/ttyACM1) between the GPS and MCU.

* **Function:** Identifies hardware via unique Vendor (VID) and Product (PID) IDs.
* **Output:** Generates a .env file for Docker and fixes serial permissions (chmod 666).
* **Usage:** `python3 fixusb.py`


### 2. openagbot-dev.py (Developer Entry)
**Primary daily workflow.** This script bridges your local environment to the ROS 2 Humble container.

* **Function:** Mounts the local /src directory for live coding and forwards X11/GUI for tools like Rviz2 and Foxglove.
* **Output:** Compiles the workspace and launches the full ROS 2 stack (GPS, Controller, and UI nodes).
* **Usage:** `python3 openagbot-dev.py`

### 3. cleanstart.sh (Deep Reset)
**System Reset.** Use this if the environment becomes unstable or if build artifacts conflict after major code changes.

* **Function:** Performs a deep clean of the workspace and Docker daemon.
* **Output:** Deletes build, install, and log folders and prunes stale Docker layers before initiating a fresh build.
* **Usage:** `./cleanstart.sh`

### 4. .Git Automation Helper (`git.sh`)

The `git.sh` script is a utility designed to standardize the workflow for the Open Agbot DevKit, ensuring consistent branch management and conflict prevention

**Push to the main branch:**
```
./git.sh your commit message
```

**Push to the dev branch:**

```
./git.sh -dev your commit message
```


### Quick Start

**Push to the main branch:**
```bash
./git.sh "Fixed motor controller timeout"


## Components

### Basekit Driver
The BaseKit driver is based on the ATB Potsdam field_friend_driver. It manages communication with an ESP32 microcontroller running Lizard firmware, a domain-specific language for defining hardware behavior on embedded systems.

The package provides:
* config/basekit.liz: Basic Lizard configuration for BaseKit robot.
* config/basekit.yaml: Corresponding ROS 2 driver configuration.

Available ROS 2 topics:
* /cmd_vel (geometry_msgs/Twist): Control robot movement.
* /odom (nav_msgs/Odometry): Robot odometry data.
* /battery_state (sensor_msgs/BatteryState): Battery status information.
* /bumper_front_top_state (std_msgs/Bool): Front top bumper state.
* /bumper_front_bottom_state (std_msgs/Bool): Front bottom bumper state.
* /bumper_back_state (std_msgs/Bool): Back bumper state.
* /emergency_stop (std_msgs/Bool): Software emergency stop control.
* /estop1_state (std_msgs/Bool): Hardware emergency stop 1 state.
* /estop2_state (std_msgs/Bool): Hardware emergency stop 2 state.
* /configure (std_msgs/Empty): Trigger loading of the Lizard configuration file.

### Camera System
The camera system supports both USB cameras and AXIS cameras, managed through a unified launch system in camera_system.launch.py.

* USB Cameras: Provides video streaming via the usb_cam ROS 2 package; configured through config/camera.yaml.
* AXIS Cameras: Integrates with the ROS 2 AXIS camera driver; credentials managed through config/secrets.yaml.
* Visualization: Integrates with Foxglove Studio for remote viewing. The Foxglove Bridge is accessible via WebSocket on port 8765.

### GNSS System
The system supports high-precision positioning via Septentrio or u-blox F9P receivers. Driver selection is managed through config/gnss.yaml.

#### Septentrio Integration
Uses the Septentrio ROS 2 driver for industrial-grade positioning. Available topics:
* /pvtgeodetic: Position, velocity, and time in geodetic coordinates.
* /poscovgeodetic: Position covariance.
* /atteuler: Attitude in Euler angles.
* /gpsfix: Detailed GPS fix information.
* /aimplusstatus: AIM+ interference monitoring status.

#### u-blox F9P Integration
Supports the ZED-F9P module for cost-effective RTK positioning.
* Provides NMEA messages and UBX protocol support.
* Supports NTRIP clients for achieving RTK-Fixed status.
* Configuration is managed via ublox_gps node parameters.

### Basekit UI
The example UI provides a robot control interface built with NiceGUI. It features a joystick control interface for movement, real-time GNSS data visualization, and safety system monitoring (bumpers and e-stops).

The interface is accessible via web browser at http://localhost:8080.

<div align="center">
  <img src="assets/BasekitUI.png" alt="Example UI Screenshot" width="500"/>
  <div style="font-size: 0.95em; color: #555; margin-top: 0.5em;">
    Example UI: Control, data, safety, and GPS map in one interface.
  </div>
</div>

## Docker Setup

The system runs entirely within Docker to ensure environment consistency across different hardware platforms.

* The basekit service uses a Dockerfile that installs ROS 2 Humble and necessary Python dependencies (NiceGUI, pyserial).
* Volume mapping links the local src/ directory to /workspace/src inside the container, enabling real-time code development.

To restart the system after making changes to the code:
python3 openagbotquick.py --clean

## Hardware Configuration

Port mappings for serial devices are defined in docker/docker-compose.yml. Common mappings include:
* Controller Port: /dev/ttyACM0 or /dev/ttyTHS0.
* GPS Port: /dev/ttyACM1.

Ensure your user has dialout permissions on the host machine to allow the container access to these ports.

## License

Refer to the LICENSE file for details on usage and distribution rights.

























## Docker Setup

### Using Docker Compose

1. Build and run the container:

```bash
cd docker
docker-compose up --build
```

2. Run in detached mode:

```bash
docker-compose up -d
```

3. Attach to running container:

```bash
docker-compose exec basekit bash
```

4. Stop containers:

```bash
docker-compose down
```

The Docker setup includes:

- All necessary ROS2 packages
- Lizard communication tools
- Camera drivers
- GNSS drivers

## Connect to UI

To access the user interface (UI), follow these steps:

1. **Connect to the Robot's Wi-Fi:**
   Join the robot's WLAN network.

2. **Open the UI in your browser:**
   Navigate to:

   ```
   http://<ROBOT-IP>:80
   ```

   (Replace `<ROBOT-IP>` with the actual IP address once you have it.)

## Launch Files

The system can be started using different launch files:

- `basekit.launch.py`: Launches all components
- `basekit_nocams.launch.py`: Launches all components without the cameras
- `field_friend.launch.py`: Launches only Field Friend driver
- `camera_system.launch.py`: Launches complete camera system (USB + AXIS) and Foxglove Bridge
- `usb_camera.launch.py`: Launches USB camera only
- `axis_cameras.launch.py`: Launches AXIS cameras only
- `gnss.launch.py`: Launches GNSS system
- `ui.launch.py`: Launches the example UI node

To launch the complete system:

```bash
ros2 launch basekit_launch basekit.launch.py
```

## AXIS Camera Authentication

The AXIS cameras can be configured to use either digest or basic authentication. To check and configure the authentication mode:

1. Check current authentication settings:

```bash
curl --digest -u root:pw "http://192.168.42.3/axis-cgi/admin/param.cgi?action=list&group=Network.HTTP" | cat
```

2. Switch authentication mode (e.g., from digest to basic):

```bash
curl --digest -u root:pw "http://192.168.42.3/axis-cgi/admin/param.cgi?action=update&Network.HTTP.AuthenticationPolicy=basic" | cat
```

Replace `root:pw` with your camera's credentials and `192.168.42.3` with your camera's IP address. The authentication mode can be set to either `basic` or `digest`. Note that you should always use the `--digest` flag in these commands even when switching to basic auth, as the camera's current setting might be using digest authentication.

## Quickstart guide

### 1. Clone the Repository

```bash
git clone https://github.com/zauberzeug/basekit_ros.git
cd basekit_ros
```

### 2. Validate Configuration

Before building, check and adjust if needed:

1. **ROS2 Configuration** (`basekit_launch/config/basekit.yaml`):

   - Verify `serial_port` matches your setup (default: "/dev/ttyTHS0")
   - Check `flash_parameters` for your hardware (default: "-j orin --nand")

2. **Lizard Configuration** (`basekit_launch/config/basekit.liz`):
   - Verify motor configuration matches your hardware
   - Check pin assignments for bumpers and emergency stops
   - Adjust any other hardware-specific settings

### 3. Build with Docker

```bash
./docker.sh u
```

### 4. Send Lizard Configuration

Once the system is running:

- Use the "Send Lizard Config" button in the UI
- Or use the `/configure` topic in ROS2

### 5. Ready to Go

Check the UI at `http://<ROBOT-IP>:80` to control and monitor your robot.

## Future features

This repository is still work in progress. Please feel free to contribute or reach out to us, if you need any unimplemented feature.

- Complete tf2 frames
- Handle camera calibrations
- Robot visualization
