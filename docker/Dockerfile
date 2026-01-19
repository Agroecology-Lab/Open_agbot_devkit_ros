# ==========================================
# STAGE 1: The Builder
# ==========================================
FROM ros:humble-ros-base AS builder
WORKDIR /workspace

# 1. Setup Environment
ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV AG_PATHS="/root/.lizard:/workspace/install/basekit_ui/lib/python3.10/site-packages:/workspace/install/basekit_driver/lib/python3.10/site-packages"
ENV PYTHONPATH="${AG_PATHS}"

SHELL ["/bin/bash", "-c"]

# 2. Mirror & System Tooling (With Production/Dev Toggle)
# HACK: Bypass signature checks for local network/time issues. COMMENT FOR PRODUCTION.
ARG APT_OPTS="-o Acquire::AllowInsecureRepositories=true"
ARG APT_AUTH="--allow-unauthenticated"

RUN for i in {1..5}; do \
    apt-get update -y ${APT_OPTS} && \
    apt-get install -y ${APT_AUTH} --no-install-recommends \
    ca-certificates curl wget gnupg2 unzip git software-properties-common && \
    # HACK: Manually inject ROS keys if base image keys are expired.
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    break || (echo "Retry $i/5..." && sleep 5); \
    done

# 3. Add MongoDB Repository (Required for Topological Navigation)
RUN curl -fsSL https://www.mongodb.org/static/pgp/server-6.0.asc | gpg --dearmor -o /usr/share/keyrings/mongodb-server-6.0.gpg && \
    echo "deb [ arch=amd64,arm64 signed-by=/usr/share/keyrings/mongodb-server-6.0.gpg ] https://repo.mongodb.org/ubuntu jammy/mongodb-org/6.0 multiverse" | tee /etc/apt/sources.list.d/mongodb-org-6.0.list

# 4. Dependency Install (Consolidated for space efficiency)
RUN for i in {1..5}; do \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-pip python3-serial udev libasio-dev python3-colcon-common-extensions \
    python3-setuptools python3-wheel python3-transforms3d \
    mongodb-org-server mongodb-org-shell python3-pymongo \
    ros-humble-gps-msgs ros-humble-tf-transformations ros-humble-rtcm-msgs \
    ros-humble-usb-cam ros-humble-xacro ros-humble-nmea-msgs \
    ros-humble-joint-state-publisher ros-humble-robot-state-publisher \
    ros-humble-controller-manager ros-humble-diff-drive-controller \
    ros-humble-twist-mux ros-humble-interactive-markers \
    ros-humble-navigation2 ros-humble-nav2-msgs \
    ros-humble-rmw-cyclonedds-cpp ros-humble-iceoryx-binding-c && \
    break || (echo "Retry $i/5 failing, waiting..." && sleep 10); \
    done && rm -rf /var/lib/apt/lists/*

# 5. Python UI & Logic (No-cache to save disk)
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir nicegui lizard 

# 6. Lizard Espresso Firmware
RUN mkdir -p /root/.lizard && cd /root && \
    LATEST_TAG=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    wget -q -O lizard.zip "https://github.com/zauberzeug/lizard/releases/download/${LATEST_TAG}/lizard_firmware_and_devtools_${LATEST_TAG}_esp32.zip" && \
    unzip -qo lizard.zip -d /root/.lizard && rm lizard.zip && \
    chmod +x /root/.lizard/espresso.py

# 7. Workspace Build (Memory-Safe for ThinkPad)
WORKDIR /workspace
COPY ./src ./src
RUN . /opt/ros/humble/setup.sh && \
    # Build core msgs first to prevent dependency races
    colcon build --symlink-install --packages-select ublox_serialization ublox_msgs --executor sequential && \
    . install/setup.sh && \
    # Final build: sequential and single worker to prevent OOM Kill/Disappearing container
    colcon build --symlink-install --executor sequential --parallel-workers 1

# ==========================================
# STAGE 2: The Runtime (Production Image)
# ==========================================
FROM ros:humble-ros-base
WORKDIR /workspace

# Copy build artifacts and lizard scripts
COPY --from=builder /workspace/install /workspace/install
COPY --from=builder /root/.lizard /opt/lizard

# Environment & Entrypoint Configuration
ENV PYTHONPATH="/workspace/install/basekit_ui/lib/python3.10/site-packages:/workspace/install/basekit_driver/lib/python3.10/site-packages:${PYTHONPATH}"

# Final Bash Setup
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/install/setup.bash" >> /root/.bashrc

# Fix permissions for hardware access scripts
RUN chmod -R 755 /opt/lizard

CMD ["/bin/bash"]
