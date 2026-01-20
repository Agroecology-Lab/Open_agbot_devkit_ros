#!/bin/bash

echo "--- Debugging AgBot Login ---"

# 1. Check if Docker is even running
if ! docker info >/dev/null 2>&1; then
    echo "❌ ERROR: Docker daemon is not running or no permissions."
    exit 1
fi

# 2. Try to find the specific ID
RAW_PS=$(docker ps | grep "openagbot:dev")
echo "DEBUG: grep result: $RAW_PS"

CONTAINER_ID=$(echo "$RAW_PS" | awk '{print $1}' | head -n 1)
echo "DEBUG: Extracted ID: $CONTAINER_ID"

# 3. Final check and execution
if [ -z "$CONTAINER_ID" ]; then
    echo "❌ FAIL: No container found with image 'openagbot-basekit'."
    echo "Current running containers:"
    docker ps --format "table {{.ID}}\t{{.Image}}\t{{.Names}}"
    exit 1
fi

echo "🚀 SUCCESS: Entering AgBot [$CONTAINER_ID]..."
echo "-------------------------------------------"
docker exec -it "$CONTAINER_ID" /bin/bash
