# Use the official ROS 2 Humble base
FROM ros:humble-ros-base

# 1. Setup Environment
ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Ensure ROS and custom modules are discoverable
ENV PYTHONPATH="/root/.lizard:/workspace/install/basekit_ui/lib/python3.10/site-packages:/workspace/install/basekit_driver/lib/python3.10/site-packages:${PYTHONPATH}"
SHELL ["/bin/bash", "-c"]

# 2. Mirror & System Tooling
RUN sed -i 's/archive.ubuntu.com/azure.archive.ubuntu.com/g' /etc/apt/sources.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    wget curl ca-certificates unzip git nano python3-pip software-properties-common && \
    add-apt-repository universe && \
    apt-get update

# 3. Robust Dependency Install (With specific missing modules)
# Added: ros-humble-gps-msgs, ros-humble-tf-transformations, python3-transforms3d
RUN for i in {1..5}; do \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-serial udev libasio-dev python3-colcon-common-extensions \
    python3-setuptools python3-wheel python3-transforms3d \
    ros-humble-gps-msgs \
    ros-humble-tf-transformations \
    ros-humble-rtcm-msgs ros-humble-usb-cam ros-humble-xacro \
    ros-humble-nmea-msgs ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher ros-humble-controller-manager \
    ros-humble-diff-drive-controller ros-humble-twist-mux \
    ros-humble-rmw-cyclonedds-cpp ros-humble-iceoryx-binding-c && \
    break || (echo "Retry $i/5 failing, waiting..." && sleep 10); \
    done && rm -rf /var/lib/apt/lists/*

# 4. Web UI & Logic Dependencies
RUN pip3 install nicegui lizard

# 5. Lizard Espresso Firmware
RUN mkdir -p /root/.lizard && cd /root && \
    LATEST_TAG=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    wget -O lizard.zip "https://github.com/zauberzeug/lizard/releases/download/${LATEST_TAG}/lizard_firmware_and_devtools_${LATEST_TAG}_esp32.zip" && \
    unzip -o lizard.zip -d /root/.lizard && rm lizard.zip && \
    chmod +x /root/.lizard/espresso.py

# 6. Workspace Setup
WORKDIR /workspace
COPY ./src ./src

# 7. Staged Build Sequence (Prevents C++/Python Cross-build Failures)
# We build dependencies first, source them, then build the rest.
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select ublox_serialization && \
    . install/setup.sh && \
    colcon build --symlink-install --packages-select ublox_msgs && \
    . install/setup.sh && \
    colcon build --symlink-install --parallel-workers 1

# 8. Environment Configuration
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/install/setup.bash" >> /root/.bashrc

# 9. Generic Entrypoint
CMD ["/bin/bash"]
