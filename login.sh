#!/bin/bash

# Find the ID of the most recent container running the openagbot:dev image
CONTAINER_ID=$(docker ps -q --filter "ancestor=openagbot:dev" | head -n 1)

if [ -z "$CONTAINER_ID" ]; then
    echo "------------------------------------------------------"
    echo "❌ ERROR: No running AgBot container found."
    echo "Make sure you have started the robot with: python3 manage.py"
    echo "------------------------------------------------------"
    exit 1
fi

# Get the name of the container for a prettier display
NAME=$(docker inspect --format '{{.Name}}' $CONTAINER_ID | sed 's/\///')

echo "------------------------------------------------------"
echo "✅ Found AgBot Container: $NAME ($CONTAINER_ID)"
echo "🚀 Entering Bash environment..."
echo "------------------------------------------------------"

# Enter the container with an interactive bash shell
docker exec -it $CONTAINER_ID bash
