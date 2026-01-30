#!/bin/bash
# install_deps.sh - Clones missing ROS message dependencies into src

echo "--- Fetching Source Dependencies for Open AgBot ---"

mkdir -p src
cd src

# Clone NMEA messages (for GPS)
if [ ! -d "nmea_msgs" ]; then
    git clone https://github.com/ros-drivers/nmea_msgs.git -b ros2
else
    echo "nmea_msgs already exists, skipping..."
fi

# Clone RTCM messages (for DGPS/RTK)
if [ ! -d "rtcm_msgs" ]; then
    git clone https://github.com/tilk/rtcm_msgs.git -b ros2_test
else
    echo "rtcm_msgs already exists, skipping..."
fi

cd ..
echo "--- Dependencies Ready. You can now run: python3 manage.py rebuild ---"
