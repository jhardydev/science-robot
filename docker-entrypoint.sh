#!/bin/bash
# Entrypoint script for Duckiebot Science Fair Robot container

# Don't exit on error - let the Python script handle errors
# set -e

# Source Duckietown environment (should already be sourced by base image entrypoint)
# The base image's entrypoint.sh already sources /environment.sh, so we skip it
# Check if Duckietown environment variables are already set
if [ -z "$DUCKIETOWN_ROOT" ] && [ -z "$DUCKIETOWN_SHELL" ]; then
    # Only source if not already sourced
    if [ -f /environment.sh ] && [ -z "$DUCKIETOWN_ENV_SOURCED" ]; then
        echo "Sourcing Duckietown environment (not already sourced)..."
        export DUCKIETOWN_ENV_SOURCED=1
        source /environment.sh 2>/dev/null || echo "Warning: Failed to source /environment.sh"
    fi
else
    echo "Duckietown environment already sourced (DUCKIETOWN_ROOT or DUCKIETOWN_SHELL is set)"
fi

# Source ROS Noetic setup (daffy-arm64v8 uses Noetic)
if [ -f /opt/ros/noetic/setup.bash ]; then
    echo "Sourcing ROS Noetic setup..."
    source /opt/ros/noetic/setup.bash || echo "Warning: Failed to source ROS Noetic"
else
    echo "Warning: /opt/ros/noetic/setup.bash not found"
fi

# Ensure ROS master is set (default to localhost if not provided)
if [ -z "$ROS_MASTER_URI" ]; then
    export ROS_MASTER_URI=http://localhost:11311
    echo "Using default ROS_MASTER_URI: $ROS_MASTER_URI"
else
    echo "Using ROS_MASTER_URI: $ROS_MASTER_URI"
fi

# Set ROS hostname if not set
if [ -z "$ROS_HOSTNAME" ]; then
    export ROS_HOSTNAME=$(hostname)
fi
echo "ROS_HOSTNAME: $ROS_HOSTNAME"

# Wait for ROS master to be available (since ros container uses host network)
echo "Waiting for ROS master at $ROS_MASTER_URI..."
timeout=30
elapsed=0
ROS_AVAILABLE=false

while [ $elapsed -lt $timeout ]; do
    if rostopic list > /dev/null 2>&1; then
        echo "ROS master is available!"
        ROS_AVAILABLE=true
        rostopic list | head -5
        break
    fi
    echo "  Waiting for ROS master... (${elapsed}/${timeout}s)"
    sleep 1
    elapsed=$((elapsed + 1))
done

if [ "$ROS_AVAILABLE" = false ]; then
    echo "WARNING: ROS master not available after ${timeout}s, proceeding anyway..."
    echo "The application may fail if ROS topics are not accessible."
fi

# Execute the command
echo "Starting application: $@"
exec "$@"

