# ==========================================
# STAGE 1: env-base (The Dependency Layer)
# ==========================================
# This stage is frozen. It only rebuilds if you change the apt/pip lists.
FROM ros:humble-ros-base AS env-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# 1. System Tools + ROS 2 Build-time & Runtime Deps
RUN for i in {1..5}; do \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    ca-certificates curl wget gnupg2 unzip git build-essential cmake \
    python3-pip python3-setuptools python3-colcon-common-extensions \
    libasio-dev libgeographic-dev net-tools iputils-ping htop nano \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-ublox-gps ros-humble-ublox-serialization ros-humble-ublox-msgs \
    ros-humble-nmea-msgs ros-humble-diagnostic-msgs ros-humble-diagnostic-updater \
    ros-humble-tf2-geometry-msgs ros-humble-tf-transformations \
    ros-humble-laser-geometry ros-humble-pcl-conversions ros-humble-nav2-common \
    ros-humble-twist-mux ros-humble-xacro ros-humble-ros2cli \
    ros-humble-teleop-twist-keyboard && \
    break || (sleep 10); \
    done

# 2. MongoDB 6.0 Setup
RUN curl -fsSL https://www.mongodb.org/static/pgp/server-6.0.asc | gpg --dearmor --yes -o /usr/share/keyrings/mongodb-server-6.0.gpg && \
    echo "deb [ arch=amd64,arm64 signed-by=/usr/share/keyrings/mongodb-server-6.0.gpg ] https://repo.mongodb.org/apt/ubuntu jammy/mongodb-org/6.0 multiverse" | tee /etc/apt/sources.list.d/mongodb-org-6.0.list && \
    apt-get update && apt-get install -y mongodb-org-server mongodb-org-shell

# 3. Python Dependencies
RUN pip3 install --no-cache-dir nicegui lizard transforms3d pymongo

# 4. Lizard Firmware Prep
RUN mkdir -p /opt/lizard && cd /tmp && \
    LATEST_TAG=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")' || echo "v0.1.0") && \
    wget -q -O lizard.zip "https://github.com/zauberzeug/lizard/releases/download/${LATEST_TAG}/lizard_firmware_and_devtools_${LATEST_TAG}_esp32.zip" && \
    unzip -qo lizard.zip -d /opt/lizard && rm lizard.zip

# ==========================================
# STAGE 2: builder (Compilation Environment)
# ==========================================
# Inherits everything from env-base. Only re-compiles if ./src changes.
FROM env-base AS builder
WORKDIR /workspace

COPY ./src ./src
RUN . /opt/ros/humble/setup.sh && \
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
    # We add CXXFLAGS to ignore the unused-result warning globally for this build
    export CXXFLAGS="-Wno-error=unused-result" && \
    colcon build --symlink-install --executor sequential --event-handlers console_direct+    
    
    
    

# ==========================================
# STAGE 3: runtime (The Light Build Target)
# ==========================================
# This inherits from env-base, skipping the heavy installs from Stage 1.
FROM env-base AS runtime
WORKDIR /workspace

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV PYTHONPATH="/workspace/install/lib/python3.10/site-packages:/opt/ros/humble/lib/python3.10/site-packages"
ENV LD_LIBRARY_PATH="/workspace/install/lib:/opt/ros/humble/lib:/usr/local/lib"

# Transfer Build Artifacts from Builder
COPY --from=builder /workspace/install /workspace/install
COPY --from=builder /opt/lizard /opt/lizard

# System Linker Update
RUN echo "/workspace/install/lib" > /etc/ld.so.conf.d/agbot_final.conf && \
    echo "/opt/ros/humble/lib" >> /etc/ld.so.conf.d/agbot_final.conf && \
    ldconfig

# Quality of Life + Auto-Sourcing
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/install/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw
