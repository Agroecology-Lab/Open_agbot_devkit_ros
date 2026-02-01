# ==========================================
# STAGE 1: env-base (The Dependency Layer)
# ==========================================
FROM ros:humble-ros-base AS env-base

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
SHELL ["/bin/bash", "-c"]

# 1. Locale & Timezone (Stolen from the advanced file - prevents Python crashes)
RUN apt-get update && apt-get install -y locales tzdata && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    ln -fs /usr/share/zoneinfo/UTC /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata
ENV LANG en_US.UTF-8

# 2. Setup L-CAS Lincoln Repository & Core System Headers
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 ca-certificates libasio-dev libgeographic-dev && \
    mkdir -p /usr/share/keyrings && \
    curl -fsSL https://lcas.lincoln.ac.uk/apt/repo_signing.key | \
    gpg --dearmor -o /usr/share/keyrings/lcas-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/lcas-keyring.gpg] https://lcas.lincoln.ac.uk/apt/lcas jammy lcas" > /etc/apt/sources.list.d/lcas.list

    
# 3. System Tools, Middleware, and Nav2 Stack
RUN for i in {1..5}; do \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    wget unzip git build-essential cmake \
    python3-pip python3-rosdep python3-setuptools python3-colcon-common-extensions \
    net-tools \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosbridge-suite \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup && \
    break || (sleep 10); \
    done

# 3.1. Automated Dependency Resolution
WORKDIR /workspace
COPY ./src ./src
# We need to init rosdep if it's the first time in this layer
RUN if [ ! -d /etc/ros/rosdep/sources.list.d ]; then rosdep init; fi && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y \
    --rosdistro humble \
    --skip-keys "pyserial" && \
    rm -rf /var/lib/apt/lists/*

# 3.2. Pip Dependencies
RUN pip3 install --no-cache-dir \
    nicegui lizard transforms3d pymongo pyserial networkx


# 4. Automated Dependency Resolution (Added skip-keys for pyserial)
WORKDIR /workspace
COPY ./src ./src
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -y \
    --rosdistro humble \
    --skip-keys "pyserial" && \
    rm -rf /var/lib/apt/lists/*

# 5. Pip Dependencies (Specific non-ROS Python libs)
RUN pip3 install --no-cache-dir \
    nicegui lizard transforms3d pymongo pyserial networkx

# 6. Lizard Firmware
RUN mkdir -p /opt/lizard && cd /tmp && \
    LATEST_TAG=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")' || echo "v0.1.5") && \
    curl -fsSL -o lizard.zip "https://github.com/zauberzeug/lizard/releases/download/${LATEST_TAG}/lizard_firmware_and_devtools_${LATEST_TAG}_esp32.zip" && \
    unzip -qo lizard.zip -d /opt/lizard && rm lizard.zip

# ==========================================
# STAGE 2: builder
# ==========================================
FROM env-base AS builder
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --executor sequential

# ==========================================
# STAGE 3: runtime
# ==========================================
FROM env-base AS runtime
COPY --from=builder /workspace/install /workspace/install
COPY --from=builder /opt/lizard /opt/lizard

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV PYTHONPATH="/workspace/install/lib/python3.10/site-packages:/opt/ros/humble/lib/python3.10/site-packages"

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && [ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash; exec \"$@\"", "--"]
CMD ["bash"]
