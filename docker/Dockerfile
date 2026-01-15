FROM ros:humble-ros-base

# 1. Resilience & Network Setup
RUN echo 'Acquire::http::Pipeline-Depth "0"; Acquire::Retries "100";' > /etc/apt/apt.conf.d/99-resilience
ENV VIRTUAL_ENV=/opt/agbot_venv
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
ENV PYTHONPATH="/root/.lizard:$PYTHONPATH"

# 2. Resilient System Install (The Loop)
RUN apt-get update && \
    for i in {1..10}; do \
    apt-get install -y --no-install-recommends \
    python3-pip python3-venv libasio-dev wget unzip git nano iputils-ping \
    ros-humble-usb-cam ros-humble-xacro ros-humble-nmea-msgs \
    ros-humble-rtcm-msgs ros-humble-diagnostic-updater \
    ros-humble-twist-mux ros-humble-rmw-cyclonedds-cpp \
    ros-humble-domain-bridge ros-humble-tf-transformations \
    ros-humble-image-transport ros-humble-compressed-image-transport && \
    break || sleep 5; done && rm -rf /var/lib/apt/lists/*

# 3. Lizard Espresso Integration
RUN mkdir -p /root/.lizard && cd /root && \
    LATEST=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    wget https://github.com/zauberzeug/lizard/releases/download/${LATEST}/lizard_firmware_and_devtools_${LATEST}_esp32.zip && \
    unzip *.zip -d /root/.lizard && rm *.zip

# 4. Venv Setup (Avoids pip-as-root issues)
RUN python3 -m venv $VIRTUAL_ENV && \
    pip install --no-cache-dir nicegui pyserial lizard-lib requests pyyaml

# 5. Environment Customization
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source $VIRTUAL_ENV/bin/activate" >> /root/.bashrc && \
    echo "export PYTHONPATH=/root/.lizard:\$PYTHONPATH" >> /root/.bashrc

# 6. Build (Matches your package structure)
WORKDIR /workspace
COPY ./src ./src
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select ublox_serialization && \
    . install/setup.sh && \
    colcon build --symlink-install --packages-select ublox_msgs && \
    . install/setup.sh && \
    colcon build --symlink-install --parallel-workers 1

# Note: Entrypoint is omitted here because Compose 
# uses the 'command:' override to manage the launch sequence.
