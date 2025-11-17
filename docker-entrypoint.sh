#!/bin/bash
# Entrypoint script for Duckiebot Science Fair Robot container

# Don't exit on error - let the Python script handle errors
# set -e

# Skip sourcing /environment.sh - the Duckietown base image entrypoint already does this
# Sourcing it again causes infinite loops since /environment.sh may source our entrypoint
# The environment should already be set up when this script runs
echo "Duckietown environment should already be sourced by base image entrypoint"

# Source ROS Noetic setup (daffy-arm64v8 uses Noetic)
if [ -f /opt/ros/noetic/setup.bash ]; then
    echo "Sourcing ROS Noetic setup..."
    source /opt/ros/noetic/setup.bash || echo "Warning: Failed to source ROS Noetic"
else
    echo "Warning: /opt/ros/noetic/setup.bash not found"
fi

# Source Duckietown workspace to make duckietown_msgs available
# The base image has the workspace at /code/catkin_ws
if [ -f /code/catkin_ws/devel/setup.bash ]; then
    echo "Sourcing Duckietown workspace..."
    source /code/catkin_ws/devel/setup.bash || echo "Warning: Failed to source workspace"
elif [ -f /code/catkin_ws/install/setup.bash ]; then
    echo "Sourcing Duckietown workspace (install space)..."
    source /code/catkin_ws/install/setup.bash || echo "Warning: Failed to source workspace"
else
    echo "Warning: Duckietown workspace setup.bash not found"
    echo "Attempting to add workspace to PYTHONPATH manually..."
    # Add workspace Python packages to PYTHONPATH as fallback
    if [ -d /code/catkin_ws/devel/lib/python3/dist-packages ]; then
        export PYTHONPATH="/code/catkin_ws/devel/lib/python3/dist-packages:$PYTHONPATH"
        echo "Added /code/catkin_ws/devel/lib/python3/dist-packages to PYTHONPATH"
    fi
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

# Virtual display setup (if enabled)
USE_VIRTUAL_DISPLAY=${USE_VIRTUAL_DISPLAY:-false}
VIRTUAL_DISPLAY_NUM=${VIRTUAL_DISPLAY_NUM:-99}
VIRTUAL_DISPLAY_SIZE=${VIRTUAL_DISPLAY_SIZE:-1024x768x24}

if [ "$USE_VIRTUAL_DISPLAY" = "true" ]; then
    echo "Starting virtual display (Xvfb) on :${VIRTUAL_DISPLAY_NUM}..."
    Xvfb :${VIRTUAL_DISPLAY_NUM} -screen 0 ${VIRTUAL_DISPLAY_SIZE} > /dev/null 2>&1 &
    XVFB_PID=$!
    sleep 1
    
    # Check if Xvfb started successfully
    if kill -0 $XVFB_PID 2>/dev/null; then
        export DISPLAY=:${VIRTUAL_DISPLAY_NUM}
        export DISPLAY_OUTPUT=true
        echo "Virtual display started successfully on ${DISPLAY}"
        echo "Display size: ${VIRTUAL_DISPLAY_SIZE}"
        
        # Optionally start VNC server to share the virtual display
        ENABLE_VNC=${ENABLE_VNC:-false}
        VNC_PORT=${VNC_PORT:-5900}
        if [ "$ENABLE_VNC" = "true" ]; then
            echo "Starting VNC server on port ${VNC_PORT}..."
            # Start x11vnc sharing the virtual display
            x11vnc -display :${VIRTUAL_DISPLAY_NUM} -nopw -forever -shared -rfbport ${VNC_PORT} > /dev/null 2>&1 &
            VNC_PID=$!
            sleep 1
            if kill -0 $VNC_PID 2>/dev/null; then
                echo "VNC server started on port ${VNC_PORT}"
                echo "Connect with: vncviewer <robot-ip>:${VNC_PORT}"
            else
                echo "Warning: Failed to start VNC server"
            fi
        fi
    else
        echo "Warning: Failed to start virtual display, falling back to headless mode"
        export DISPLAY_OUTPUT=false
        export QT_QPA_PLATFORM=offscreen
    fi
elif [ -z "$DISPLAY" ]; then
    # No DISPLAY variable set
    if [ "$DISPLAY_OUTPUT" = "true" ]; then
        # User requested display output but DISPLAY not set
        echo "Warning: DISPLAY_OUTPUT=true but DISPLAY is not set"
        echo "  Make sure you SSH'd with -X or -Y flag"
        echo "  Or set DISPLAY environment variable"
        echo "  Falling back to headless mode"
        export DISPLAY_OUTPUT=false
        export QT_QPA_PLATFORM=offscreen
    else
        # Running in headless mode as expected
        echo "No display available, running in headless mode"
        export DISPLAY_OUTPUT=${DISPLAY_OUTPUT:-false}
        export QT_QPA_PLATFORM=offscreen
    fi
else
    # DISPLAY is set (X11 forwarding or local X11)
    echo "Using X11 display: ${DISPLAY}"
    # Note: /tmp/.X11-unix might not exist with SSH X11 forwarding, that's OK
    if [ "$DISPLAY_OUTPUT" != "false" ]; then
        export DISPLAY_OUTPUT=true
        # Don't set QT_QPA_PLATFORM - let Qt auto-detect (will use xcb)
        # xcb plugin is available and will work with X11 forwarding
    else
        export DISPLAY_OUTPUT=false
        export QT_QPA_PLATFORM=offscreen
    fi
fi

# Set Qt backend based on display availability
# Only set offscreen if we're truly headless (DISPLAY_OUTPUT=false)
# If DISPLAY_OUTPUT=true, let Qt auto-detect the platform (xcb should work)
if [ "$DISPLAY_OUTPUT" = "false" ]; then
    # Try to use offscreen, but don't fail if it's not available
    export QT_QPA_PLATFORM=offscreen
else
    # Display output enabled - unset QT_QPA_PLATFORM to let Qt auto-detect
    # This will use xcb which is available in the container
    unset QT_QPA_PLATFORM
fi

# Setup VPI library paths (if VPI is mounted from host)
# VPI may be mounted in /host paths, need to add to PYTHONPATH and LD_LIBRARY_PATH
if [ -d /host/usr/lib/python3/dist-packages ]; then
    export PYTHONPATH="/host/usr/lib/python3/dist-packages:$PYTHONPATH"
    echo "Added /host/usr/lib/python3/dist-packages to PYTHONPATH for VPI"
fi

# Add VPI library paths to LD_LIBRARY_PATH
if [ -d /host/usr/lib/aarch64-linux-gnu ]; then
    export LD_LIBRARY_PATH="/host/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH"
    echo "Added /host/usr/lib/aarch64-linux-gnu to LD_LIBRARY_PATH for VPI"
fi

# Check if VPI is accessible
if python3 -c "import vpi" 2>/dev/null; then
    echo "✓ VPI is accessible in container"
else
    echo "⚠ VPI not accessible in container (will use CPU fallback)"
    echo "  Note: VPI requires JetPack SDK and may need to be mounted from host"
fi

# Execute the command
echo "Starting application: $@"
exec "$@"

