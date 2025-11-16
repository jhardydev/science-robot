# Deployment Guide - Duckiebot Science Fair Robot

This guide explains how to deploy the Science Fair Robot as a Docker container on your Duckiebot platform.

## Prerequisites

- Duckiebot running with ROS (Ubuntu 18.04 with ROS Melodic, or Ubuntu 20.04 with ROS Noetic)
- Docker installed on the robot
- ARM64 architecture (tested on Jetson Nano with Ubuntu 18.04.6)
- Access to the robot via SSH or direct terminal
- ROS master running on the robot

## Quick Start

### Option 1: Using the Helper Script (Recommended)

```bash
# Build and run with defaults (robot name: robot1)
./docker-run.sh --build

# Customize robot name
./docker-run.sh --build --robot-name robot1

# Connect to remote ROS master
./docker-run.sh --build --robot-name robot1 --ros-master http://192.168.1.100:11311
```

### Option 2: Using Docker Compose

```bash
# Set environment variables
export VEHICLE_NAME=robot1
export ROS_MASTER_URI=http://localhost:11311

# Build and run
docker-compose up --build
```

### Option 3: Manual Docker Commands

```bash
# Build the image
docker build -t science-robot:latest .

# Run the container
docker run -it --rm \
    --name science-robot \
    --network host \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -e ROS_HOSTNAME=science-robot \
    -e VEHICLE_NAME=robot1 \
    -e DISPLAY=:0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/root/.Xauthority:rw \
    science-robot:latest
```

## Configuration

### Robot Name

Update the robot name in `config.py`:

```python
ROBOT_NAME = 'robot1'  # Change to your robot's name
```

Or set via environment variable:

```bash
export VEHICLE_NAME=robot1
```

The Docker entrypoint will use this to construct ROS topic names.

### ROS Topics

The application uses these ROS topics (based on `ROBOT_NAME`):

- Motor Control: `/{ROBOT_NAME}/wheels_driver_node/wheels_cmd`
- Camera: `/{ROBOT_NAME}/camera_node/image/compressed`

Verify these topics exist on your robot:

```bash
rostopic list | grep robot1
```

## Building the Container

### On the Robot (Recommended)

Build directly on the robot for platform compatibility:

```bash
# SSH into your robot
ssh duckiebot@<robot-ip>

# Clone or copy the project
cd ~/projects/science-robot

# Build
docker build -t science-robot:latest .
```

### Cross-Platform Build

If building on a different machine:

```bash
# For ARM64 (Jetson Nano)
docker buildx build --platform linux/arm64 -t science-robot:latest .

# For AMD64
docker buildx build --platform linux/amd64 -t science-robot:latest .
```

## Running the Container

### Verify ROS Master is Running

Before starting the container, ensure ROS master is running:

```bash
# On the robot
roscore

# Or check if it's already running
rostopic list
```

### Start the Application

```bash
# Using helper script
./docker-run.sh

# Using docker-compose
docker-compose up

# Using docker directly
docker run -it --rm \
    --name science-robot \
    --network host \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -e VEHICLE_NAME=robot1 \
    science-robot:latest
```

## Display Output (Optional)

If you want to see the camera feed with overlays:

1. Enable X11 forwarding (if using SSH):
   ```bash
   ssh -X duckiebot@<robot-ip>
   ```

2. Set DISPLAY variable:
   ```bash
   export DISPLAY=:0
   ```

3. The container already includes X11 mount options in docker-compose.yml

## Troubleshooting

### Container Can't Connect to ROS Master

**Problem**: Container starts but can't find ROS topics.

**Solutions**:
1. Verify ROS master is running:
   ```bash
   rostopic list
   ```

2. Check ROS_MASTER_URI:
   ```bash
   echo $ROS_MASTER_URI
   # Should be: http://localhost:11311 (or your robot's IP)
   ```

3. Ensure `--network host` is used (or proper networking in docker-compose)

### Camera Topic Not Found

**Problem**: No camera frames received.

**Solutions**:
1. Verify camera topic exists:
   ```bash
   rostopic list | grep camera
   rostopic hz /robot1/camera_node/image/compressed
   ```

2. Check robot name matches:
   ```bash
   # In config.py or via VEHICLE_NAME env var
   ```

### Motor Commands Not Working

**Problem**: Robot doesn't move.

**Solutions**:
1. Verify wheels_driver_node is running:
   ```bash
   rostopic echo /robot1/wheels_driver_node/wheels_cmd
   ```

2. Check robot's FSM mode (should allow external commands):
   ```bash
   rostopic echo /robot1/fsm_node/mode
   ```

3. Verify motor topic name in config matches your setup

### Performance Issues

**Problem**: Low FPS or laggy performance.

**Solutions**:
1. Disable display output in `config.py`:
   ```python
   DISPLAY_OUTPUT = False
   ```

2. Reduce camera resolution:
   ```python
   CAMERA_WIDTH = 320
   CAMERA_HEIGHT = 240
   ```

3. Disable GPU acceleration if not available:
   ```python
   USE_CUDA_ACCELERATION = False
   USE_VPI_ACCELERATION = False
   ```

## Development Mode

For development, mount the source code as a volume:

```bash
docker run -it --rm \
    --name science-robot \
    --network host \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -e VEHICLE_NAME=robot1 \
    -v $(pwd):/code \
    -w /code \
    science-robot:latest python3 main.py
```

## Stopping the Container

```bash
# If using docker-compose
docker-compose down

# If using docker directly
docker stop science-robot

# The helper script uses --rm, so container is auto-removed on exit
```

## Cleanup

For development environments, it's easy to clean up after failed attempts or when starting fresh.

### Quick Cleanup

```bash
# Clean up everything (containers, images, volumes)
./docker-cleanup.sh --all

# Clean up with auto-confirmation (no prompts)
./docker-cleanup.sh --all --force

# Clean up using docker-run.sh
./docker-run.sh --cleanup
```

### Selective Cleanup

```bash
# Remove stopped containers only (default)
./docker-cleanup.sh

# Remove Docker image only
./docker-cleanup.sh --image

# Remove containers and image
./docker-cleanup.sh --containers --image

# Remove everything including volumes
./docker-cleanup.sh --all
```

### Manual Cleanup

```bash
# Stop and remove container
docker stop science-robot 2>/dev/null || true
docker rm science-robot 2>/dev/null || true

# Remove image
docker rmi science-robot:latest 2>/dev/null || true

# Remove dangling images (optional)
docker image prune -f

# Remove volumes (if any)
docker volume ls | grep science-robot | awk '{print $2}' | xargs docker volume rm 2>/dev/null || true
```

### Common Cleanup Scenarios

**After a failed build:**
```bash
# Clean up and rebuild
./docker-cleanup.sh --image --force
./docker-run.sh --build
```

**Starting fresh:**
```bash
# Remove everything and start clean
./docker-cleanup.sh --all --force
./docker-run.sh --build
```

**Quick reset during development:**
```bash
# Stop current container and remove image for rebuild
docker stop science-robot 2>/dev/null || true
docker rm science-robot 2>/dev/null || true
docker rmi science-robot:latest 2>/dev/null || true
./docker-run.sh --build
```

## Integration with Duckietown Stack

This container integrates with the Duckietown ROS ecosystem:

- **ROS Topics**: Uses standard Duckietown topic names
- **Message Types**: Uses `duckietown_msgs` for wheel commands
- **Network**: Uses host networking to access ROS master
- **Compatibility**: Based on `duckietown/dt-ros-commons` image

## Updating the Application

To update the code:

1. **Edit code locally** or on robot
2. **Rebuild container**:
   ```bash
   docker build -t science-robot:latest .
   ```
3. **Restart container**:
   ```bash
   docker-compose restart
   # or
   ./docker-run.sh --build
   ```

## Monitoring

View container logs:

```bash
# Using docker-compose
docker-compose logs -f

# Using docker
docker logs -f science-robot
```

Check ROS topics from container:

```bash
docker exec -it science-robot bash
rostopic list
rostopic echo /robot1/wheels_driver_node/wheels_cmd
```

