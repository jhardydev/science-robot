#!/bin/bash
# Entrypoint script for Duckiebot Science Fair Robot container

set -e

# Source Duckietown environment (should be in base image)
if [ -f /environment.sh ]; then
    source /environment.sh
fi

# Source ROS Noetic setup (daffy-arm64v8 uses Noetic)
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
fi

# Ensure ROS master is set (default to localhost if not provided)
if [ -z "$ROS_MASTER_URI" ]; then
    export ROS_MASTER_URI=http://localhost:11311
fi

# Set ROS hostname if not set
if [ -z "$ROS_HOSTNAME" ]; then
    export ROS_HOSTNAME=$(hostname)
fi

# Wait for ROS master to be available (since ros container uses host network)
echo "Waiting for ROS master at $ROS_MASTER_URI..."
timeout=30
elapsed=0
while ! rostopic list > /dev/null 2>&1; do
    if [ $elapsed -ge $timeout ]; then
        echo "Warning: ROS master not available after ${timeout}s, proceeding anyway..."
        break
    fi
    sleep 1
    elapsed=$((elapsed + 1))
done

# Execute the command
exec "$@"

