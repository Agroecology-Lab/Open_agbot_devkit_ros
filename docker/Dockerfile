# Use the official ROS 2 Humble base
FROM ros:humble-ros-base

# 1. Setup Environment & Mirrors
ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV PYTHONPATH="/root/.lizard"

# 2. Hardening Apt & Installing Base Tools
# We switch to the Azure mirror for better reliability during ROS buildfarm syncs
RUN sed -i 's/archive.ubuntu.com/azure.archive.ubuntu.com/g' /etc/apt/sources.list && \
    apt-get update && apt-get install -y --fix-missing --no-install-recommends \
    wget curl ca-certificates unzip git nano python3-pip software-properties-common && \
    add-apt-repository universe && \
    apt-get update

# 3. Robust Dependency Management
# We split the install to ensure that if a ROS mirror is syncing, 
# we can retry specific packages without failing the whole build.
RUN for i in {1..5}; do apt-get update && break || sleep 5; done && \
    apt-get install -y --fix-missing --no-install-recommends \
    # Hardware Communication
    python3-serial udev libasio-dev \
    # ROS Build Tools
    python3-colcon-common-extensions \
    # Required ROS 2 Humble Packages
    ros-humble-rtcm-msgs \
    ros-humble-usb-cam \
    ros-humble-xacro \
    ros-humble-nmea-msgs \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-controller-manager \
    ros-humble-diff-drive-controller \
    ros-humble-twist-mux \
    # Middleware (CycloneDDS & Iceoryx)
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-iceoryx-binding-c ros-humble-iceoryx-hoofs ros-humble-iceoryx-posh && \
    rm -rf /var/lib/apt/lists/*

# 4. Lizard Espresso Firmware & Devtools
# Instead of a static link, this fetches the latest version dynamically
RUN mkdir -p /root/.lizard && cd /root && \
    LATEST_TAG=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    wget -O lizard.zip "https://github.com/zauberzeug/lizard/releases/download/${LATEST_TAG}/lizard_firmware_and_devtools_${LATEST_TAG}_esp32.zip" && \
    unzip -o lizard.zip -d /root/.lizard && rm lizard.zip && \
    chmod +x /root/.lizard/espresso.py

# 5. Core Python Dependencies (NiceGUI / AgBot Stack)
# 6. Workspace Setup
WORKDIR /workspace
# Copy the source code from the local context
COPY ./src ./src

# Build the workspace using a staged approach to handle ublox dependencies
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select ublox_serialization && \
    . install/setup.sh && \
    colcon build --symlink-install --packages-select ublox_msgs && \
    . install/setup.sh && \
    colcon build --symlink-install --parallel-workers 1

# 7. Environment Sourcing
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/install/setup.bash" >> /root/.bashrc

# 8. Entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.sh && source install/setup.bash && ros2 launch basekit_launch basekit.launch.py"]
