#!/bin/bash

# Target the specific container name used in manage.py
CONTAINER_NAME="open_agbot"

# Check if the container is actually running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "------------------------------------------------------"
    echo "❌ ERROR: No running AgBot container found."
    echo "Container '${CONTAINER_NAME}' is not active."
    echo "Make sure you have started the robot with: python3 manage.py"
    echo "------------------------------------------------------"
    exit 1
fi

echo "------------------------------------------------------"
echo "✅ Found AgBot Container: $CONTAINER_NAME"
echo "🚀 Entering Bash environment..."
echo "------------------------------------------------------"

# Enter the container with an interactive bash shell
docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && bash"
