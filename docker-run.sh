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
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --robot-name NAME    Set robot name (default: robot1)"
            echo "  --ros-master URI     Set ROS master URI (default: http://localhost:11311)"
            echo "  --build              Build image before running"
            echo "  --help               Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0 --build"
            echo "  $0 --robot-name duckiebot --ros-master http://192.168.1.100:11311"
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
echo ""

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
docker run -it --rm \
    --name science-robot \
    --network host \
    -e ROS_MASTER_URI="$ROS_MASTER" \
    -e ROS_HOSTNAME=science-robot \
    -e VEHICLE_NAME="$ROBOT_NAME" \
    -e DISPLAY="${DISPLAY:-:0}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$HOME/.Xauthority:/root/.Xauthority:rw" \
    science-robot:latest

