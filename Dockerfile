FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    python3-pip \
    libasio-dev \
    ros-humble-diagnostic-updater \
    ros-humble-gps-msgs \
    ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install pyserial nicegui

WORKDIR /open_agbot_ws
