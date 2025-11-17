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
    
    # Verify X11 connection is ready (for SSH X11 forwarding)
    if command -v xdpyinfo > /dev/null 2>&1; then
        echo "Checking X11 connection..."
        if timeout 2 xdpyinfo > /dev/null 2>&1; then
            echo "✓ X11 connection is ready"
        else
            echo "⚠ X11 connection check timed out (may still work)"
            # Small delay to let X11 forwarding initialize
            sleep 1
        fi
    else
        echo "xdpyinfo not available, skipping X11 connection check"
        # Small delay for X11 forwarding to initialize
        sleep 0.5
    fi
    
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
# Only add specific VPI paths, not entire dist-packages (avoids conflicts)

# Find VPI package in mounted paths
VPI_FOUND=false

# Check if VPI_PARENT_DIR was set by docker-run.sh
if [ -n "$VPI_PARENT_DIR" ]; then
    HOST_VPI_PARENT="/host$VPI_PARENT_DIR"
    if [ -d "$HOST_VPI_PARENT" ]; then
        export PYTHONPATH="$HOST_VPI_PARENT:$PYTHONPATH"
        echo "Added $HOST_VPI_PARENT to PYTHONPATH for VPI (from VPI_PARENT_DIR)"
        VPI_FOUND=true
    fi
fi

# Also try common paths if not found yet
if [ "$VPI_FOUND" = false ]; then
    VPI_PATHS=(
        "/host/usr/lib/python3/dist-packages/vpi"
        "/host/usr/local/lib/python3/dist-packages/vpi"
        "/usr/lib/python3/dist-packages/vpi"
    )
    
    for VPI_PATH in "${VPI_PATHS[@]}"; do
        if [ -d "$VPI_PATH" ] || [ -f "${VPI_PATH}.py" ]; then
            VPI_PARENT=$(dirname "$VPI_PATH" 2>/dev/null || echo "")
            # If path starts with /host, use it directly; otherwise prepend /host
            if [[ "$VPI_PARENT" == /host/* ]]; then
                HOST_PARENT="$VPI_PARENT"
            else
                HOST_PARENT="/host$VPI_PARENT"
            fi
            
            if [ -n "$HOST_PARENT" ] && [ -d "$HOST_PARENT" ]; then
                export PYTHONPATH="$HOST_PARENT:$PYTHONPATH"
                echo "Added $HOST_PARENT to PYTHONPATH for VPI"
                VPI_FOUND=true
                break
            fi
        fi
    done
fi

# Add VPI library paths to LD_LIBRARY_PATH
if [ -d /host/usr/lib/aarch64-linux-gnu ]; then
    # Check if VPI libraries exist
    if ls /host/usr/lib/aarch64-linux-gnu/libnvvpi* > /dev/null 2>&1; then
        export LD_LIBRARY_PATH="/host/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH"
        echo "Added /host/usr/lib/aarch64-linux-gnu to LD_LIBRARY_PATH for VPI"
    fi
fi

# Check if VPI is accessible
if python3 -c "import vpi" 2>/dev/null; then
    echo "✓ VPI is accessible in container"
else
    echo "⚠ VPI not accessible in container (will use CPU fallback)"
    echo "  Note: VPI requires JetPack SDK and may need to be mounted from host"
fi

# Suppress fontconfig warnings (they're harmless but noisy)
# Redirect fontconfig errors to /dev/null
export FONTCONFIG_FILE=/etc/fonts/fonts.conf 2>/dev/null || true
export FC_DEBUG=0 2>/dev/null || true
# Create a minimal fontconfig config if it's broken
if [ -f /etc/fonts/fonts.conf ] && ! xmllint /etc/fonts/fonts.conf >/dev/null 2>&1; then
    echo "Warning: fonts.conf is invalid XML, fontconfig may complain"
fi

# Auto-start wheels driver node if not running
ROBOT_NAME=${VEHICLE_NAME:-robot1}
WHEELS_TOPIC="/${ROBOT_NAME}/wheels_driver_node/wheels_cmd"

echo "Checking if wheels driver node is running..."
# Check if topic has subscribers or if node is running
WHEELS_RUNNING=false

# Check if node exists
if rostopic list > /dev/null 2>&1; then
    if rosnode list 2>/dev/null | grep -q "wheels_driver_node"; then
        echo "✓ wheels_driver_node is already running"
        WHEELS_RUNNING=true
    # Check if topic exists and has subscribers (topic exists means driver is running)
    elif rostopic info "$WHEELS_TOPIC" > /dev/null 2>&1; then
        # Topic exists - check if it has subscribers
        topic_info=$(rostopic info "$WHEELS_TOPIC" 2>/dev/null)
        if echo "$topic_info" | grep -q "Subscribers:"; then
            echo "✓ wheels driver topic exists and has subscribers"
            WHEELS_RUNNING=true
        fi
    fi
fi

if [ "$WHEELS_RUNNING" = false ]; then
    echo "Starting wheels driver node..."
    
    # Try different methods to start the wheels driver
    # Method 1: Try roslaunch (most common for Duckietown)
    if command -v roslaunch > /dev/null 2>&1; then
        # Try different possible launch file paths/names
        LAUNCH_ATTEMPTS=(
            "duckietown wheels_driver.launch veh:=$ROBOT_NAME"
            "duckietown_demos wheels_driver.launch veh:=$ROBOT_NAME"
            "dt-car-interface wheels_driver.launch veh:=$ROBOT_NAME"
        )
        
        for launch_cmd in "${LAUNCH_ATTEMPTS[@]}"; do
            echo "  Trying: roslaunch $launch_cmd"
            roslaunch $launch_cmd > /dev/null 2>&1 &
            WHEELS_PID=$!
            sleep 2
            
            if kill -0 $WHEELS_PID 2>/dev/null; then
                # Process is still running - check if node appeared
                if rosnode list 2>/dev/null | grep -q "wheels_driver_node"; then
                    echo "✓ wheels_driver_node started (PID: $WHEELS_PID)"
                    WHEELS_RUNNING=true
                    break
                fi
            fi
            
            # If process died, try next method
            wait $WHEELS_PID 2>/dev/null || true
        done
    fi
    
    # Method 2: Try starting as ROS node directly (if wheels_driver executable exists)
    if [ "$WHEELS_RUNNING" = false ] && command -v wheels_driver_node > /dev/null 2>&1; then
        echo "  Trying: wheels_driver_node"
        wheels_driver_node > /dev/null 2>&1 &
        WHEELS_PID=$!
        sleep 2
        if kill -0 $WHEELS_PID 2>/dev/null && rosnode list 2>/dev/null | grep -q "wheels_driver_node"; then
            echo "✓ wheels_driver_node started (PID: $WHEELS_PID)"
            WHEELS_RUNNING=true
        else
            wait $WHEELS_PID 2>/dev/null || true
        fi
    fi
    
    # Wait a bit for topic to become available
    if [ "$WHEELS_RUNNING" = true ]; then
        echo "Waiting for wheels driver topic to become available..."
        timeout=10
        elapsed=0
        while [ $elapsed -lt $timeout ]; do
            if rostopic info "$WHEELS_TOPIC" > /dev/null 2>&1; then
                echo "✓ Wheels driver topic is available: $WHEELS_TOPIC"
                break
            fi
            sleep 0.5
            elapsed=$((elapsed + 1))
        done
        
        if [ $elapsed -ge $timeout ]; then
            echo "⚠ Topic not available after ${timeout}s, but node may still be starting"
        fi
    else
        echo "⚠ Could not start wheels driver automatically"
        echo "  You may need to start it manually:"
        echo "  roslaunch duckietown wheels_driver.launch veh:=$ROBOT_NAME"
        echo "  Or: roslaunch duckietown_demos wheels_driver.launch veh:=$ROBOT_NAME"
    fi
else
    echo "✓ Wheels driver is ready"
fi

# Execute the command
echo "Starting application: $@"
exec "$@"

