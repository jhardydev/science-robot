#!/bin/bash
# Helper script to build and run the Duckiebot Science Fair Robot container

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default values
ROBOT_NAME=${VEHICLE_NAME:-robot1}
ROS_MASTER=${ROS_MASTER_URI:-http://localhost:11311}
BUILD=false
USE_VIRTUAL_DISPLAY=${USE_VIRTUAL_DISPLAY:-false}
DISPLAY_OUTPUT=${DISPLAY_OUTPUT:-false}
ENABLE_VNC=${ENABLE_VNC:-false}
VNC_PORT=${VNC_PORT:-5900}
LOG_DIR="${HOME}/science-robot-logs"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --robot-name)
            ROBOT_NAME="$2"
            shift 2
            ;;
        --ros-master)
            ROS_MASTER="$2"
            shift 2
            ;;
        --build)
            BUILD=true
            shift
            ;;
        --virtual-display)
            USE_VIRTUAL_DISPLAY=true
            DISPLAY_OUTPUT=true
            shift
            ;;
        --display-output)
            DISPLAY_OUTPUT=true
            shift
            ;;
        --vnc)
            USE_VIRTUAL_DISPLAY=true
            DISPLAY_OUTPUT=true
            ENABLE_VNC=true
            shift
            ;;
        --vnc-port)
            VNC_PORT="$2"
            shift 2
            ;;
        --log-dir)
            LOG_DIR="$2"
            shift 2
            ;;
        --cleanup)
            # Run cleanup script
            SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
            "$SCRIPT_DIR/docker-cleanup.sh" --all --force
            exit 0
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --robot-name NAME       Set robot name (default: robot1)"
            echo "  --ros-master URI        Set ROS master URI (default: http://localhost:11311)"
            echo "  --build                 Build image before running"
            echo "  --virtual-display       Enable virtual display (Xvfb) for headless video output"
            echo "  --display-output        Enable display output (requires X11 or virtual display)"
            echo "  --vnc                   Enable virtual display with VNC server (combines --virtual-display + VNC)"
            echo "  --vnc-port PORT         Set VNC port (default: 5900)"
            echo "  --log-dir DIR           Set log directory (default: ~/science-robot-logs)"
            echo "  --cleanup               Clean up containers, images, and volumes (then exit)"
            echo "  --help                  Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0 --build"
            echo "  $0 --robot-name duckiebot --ros-master http://192.168.1.100:11311"
            echo "  $0 --cleanup          # Clean up everything before running"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo -e "${GREEN}Duckiebot Science Fair Robot - Docker Runner${NC}"
echo "Robot name: $ROBOT_NAME"
echo "ROS Master: $ROS_MASTER"
echo "Virtual display: $USE_VIRTUAL_DISPLAY"
echo "Display output: $DISPLAY_OUTPUT"
if [ "$USE_VIRTUAL_DISPLAY" = "true" ]; then
    echo "VNC enabled: $ENABLE_VNC"
    if [ "$ENABLE_VNC" = "true" ]; then
        echo "VNC port: $VNC_PORT"
    fi
fi
echo "Log directory: $LOG_DIR"
echo ""

# Create log directory if it doesn't exist
mkdir -p "$LOG_DIR"

# Build if requested
if [ "$BUILD" = true ]; then
    echo -e "${YELLOW}Building Docker image...${NC}"
    docker build -t science-robot:latest .
    echo ""
fi

# Check if image exists
if ! docker image inspect science-robot:latest > /dev/null 2>&1; then
    echo -e "${YELLOW}Image not found. Building...${NC}"
    docker build -t science-robot:latest .
    echo ""
fi

# Stop existing container if running
if docker ps -a --format '{{.Names}}' | grep -q "^science-robot$"; then
    echo -e "${YELLOW}Stopping existing container...${NC}"
    docker stop science-robot > /dev/null 2>&1 || true
    docker rm science-robot > /dev/null 2>&1 || true
    echo ""
fi

# Run container
echo -e "${GREEN}Starting container...${NC}"

# Build volume mounts
VOLUME_MOUNTS=(
    -v "$LOG_DIR:/code/logs"
)

# Add X11 mounts if not using virtual display
if [ "$USE_VIRTUAL_DISPLAY" != "true" ] && [ -n "$DISPLAY" ] && [ -e /tmp/.X11-unix ]; then
    VOLUME_MOUNTS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
    if [ -e "$HOME/.Xauthority" ]; then
        VOLUME_MOUNTS+=(-v "$HOME/.Xauthority:/root/.Xauthority:rw")
    fi
fi

# Build environment variables
ENV_VARS=(
    -e ROS_MASTER_URI="$ROS_MASTER"
    -e ROS_HOSTNAME=science-robot
    -e VEHICLE_NAME="$ROBOT_NAME"
    -e USE_VIRTUAL_DISPLAY="$USE_VIRTUAL_DISPLAY"
    -e DISPLAY_OUTPUT="$DISPLAY_OUTPUT"
    -e ENABLE_VNC="$ENABLE_VNC"
    -e VNC_PORT="$VNC_PORT"
)

# Add DISPLAY if not using virtual display
if [ "$USE_VIRTUAL_DISPLAY" != "true" ] && [ -n "$DISPLAY" ]; then
    ENV_VARS+=(-e DISPLAY="$DISPLAY")
fi

# Add VNC port mapping if VNC is enabled
PORT_MAPPINGS=()
if [ "$ENABLE_VNC" = "true" ]; then
    PORT_MAPPINGS=(-p "${VNC_PORT}:${VNC_PORT}")
    echo "VNC will be accessible on port ${VNC_PORT}"
    echo "  Direct: robot-ip:${VNC_PORT}"
    echo "  SSH tunnel: ssh -L ${VNC_PORT}:localhost:${VNC_PORT} user@robot-ip"
fi

docker run -it --rm \
    --name science-robot \
    --network host \
    "${PORT_MAPPINGS[@]}" \
    "${ENV_VARS[@]}" \
    "${VOLUME_MOUNTS[@]}" \
    science-robot:latest

