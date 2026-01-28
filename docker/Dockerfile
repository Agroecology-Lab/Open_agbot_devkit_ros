# ==========================================
# STAGE 1: The Builder
# ==========================================
FROM ros:humble-ros-base AS builder

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV WORKSPACE=/workspace
WORKDIR $WORKSPACE

SHELL ["/bin/bash", "-c"]

# 1. Infrastructure & MongoDB Repositories
RUN for i in {1..5}; do \
    apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates curl wget gnupg2 unzip git lsb-release && \
    curl -fsSL https://www.mongodb.org/static/pgp/server-6.0.asc | gpg --dearmor --yes -o /usr/share/keyrings/mongodb-server-6.0.gpg && \
    echo "deb [ arch=amd64,arm64 signed-by=/usr/share/keyrings/mongodb-server-6.0.gpg ] https://repo.mongodb.org/apt/ubuntu jammy/mongodb-org/6.0 multiverse" | tee /etc/apt/sources.list.d/mongodb-org-6.0.list && \
    apt-get update && break || sleep 10; \
    done

# 2. Comprehensive Dependency Install
RUN for i in {1..5}; do \
    apt-get update && \
    apt-get install -y --no-install-recommends --fix-missing \
    python3-pip python3-setuptools python3-yaml python3-numpy python3-serial python3-colcon-common-extensions \
    libasio-dev udev qtbase5-dev mongodb-org-server mongodb-org-shell python3-pymongo \
    ros-humble-navigation2 ros-humble-nav2-msgs ros-humble-nav2-controller ros-humble-nav2-bt-navigator \
    ros-humble-ublox-gps ros-humble-nmea-msgs ros-humble-rtcm-msgs ros-humble-gps-msgs \
    ros-humble-tf2-ros ros-humble-tf-transformations ros-humble-visualization-msgs \
    ros-humble-rmw-cyclonedds-cpp ros-humble-iceoryx-binding-c \
    ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
    ros-humble-twist-mux ros-humble-xacro ros-humble-usb-cam && \
    break || (echo "Retry $i/5 failing..." && sleep 10); \
    done && apt-get clean && rm -rf /var/lib/apt/lists/*

# 3. Python Ecosystem & Lizard Firmware
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir nicegui lizard transforms3d

RUN mkdir -p /root/.lizard && cd /root && \
    LATEST_TAG=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    wget -q -O lizard.zip "https://github.com/zauberzeug/lizard/releases/download/${LATEST_TAG}/lizard_firmware_and_devtools_${LATEST_TAG}_esp32.zip" && \
    unzip -qo lizard.zip -d /root/.lizard && rm lizard.zip && \
    chmod +x /root/.lizard/espresso.py

# 4. Two-Phase Workspace Build
COPY ./src ./src
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select ublox_serialization ublox_msgs --executor sequential && \
    . install/setup.sh && \
    colcon build --symlink-install --executor sequential --parallel-workers 1

# ==========================================
# STAGE 2: The Runtime
# ==========================================
FROM ros:humble-ros-base
WORKDIR /workspace

# Copy build artifacts and firmware
COPY --from=builder /workspace/install /workspace/install
COPY --from=builder /root/.lizard /opt/lizard
COPY --from=builder /usr/share/keyrings/mongodb-server-6.0.gpg /usr/share/keyrings/mongodb-server-6.0.gpg
COPY --from=builder /etc/apt/sources.list.d/mongodb-org-6.0.list /etc/apt/sources.list.d/mongodb-org-6.0.list

# --- SYSTEM LINKER FIX ---
# This registers custom message libraries globally so C++ nodes (Nav2) can find them
RUN echo "/workspace/install/map_msgs/lib" > /etc/ld.so.conf.d/agbot_messages.conf && \
    echo "/workspace/install/topological_navigation_msgs/lib" >> /etc/ld.so.conf.d/agbot_messages.conf && \
    ldconfig

# Update Library and Python Paths

ENV LD_LIBRARY_PATH="/workspace/install/map_msgs/lib:/workspace/install/topological_navigation_msgs/lib:/opt/ros/humble/lib"
ENV PYTHONPATH="/workspace/install/topological_navigation/lib/python3.10/site-packages:/workspace/install/basekit_driver/lib/python3.10/site-packages"


# Make sure the scripts are executable
RUN chmod +x /workspace/install/topological_navigation/lib/topological_navigation/*.py || true

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 5. Resilient Runtime Install
RUN for i in {1..5}; do \
    apt-get update && \
    apt-get install -y --no-install-recommends --fix-missing \
    python3-pip python3-serial udev libasio-dev \
    ros-humble-navigation2 \
    ros-humble-ublox-gps \
    ros-humble-tf-transformations \
    ros-humble-rmw-cyclonedds-cpp && \
    break || sleep 10; \
    done && \
    python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir nicegui lizard transforms3d && \
    apt-get clean && rm -rf /var/lib/apt/lists/*
