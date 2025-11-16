#!/bin/bash
# Diagnostic script to test the container and see what's happening

echo "=== Testing Science Robot Container ==="
echo ""

# Check if image exists
if ! docker image inspect science-robot:latest > /dev/null 2>&1; then
    echo "ERROR: Image science-robot:latest not found!"
    echo "Run: ./docker-run.sh --build"
    exit 1
fi

echo "✓ Image exists"
echo ""

# Test 1: Check entrypoint works
echo "=== Test 1: Entrypoint Test ==="
docker run --rm --network host \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -e VEHICLE_NAME=robot1 \
    science-robot:latest \
    bash -c "source /entrypoint.sh && echo 'Entrypoint test passed'"
echo ""

# Test 2: Check ROS connectivity
echo "=== Test 2: ROS Connectivity Test ==="
docker run --rm --network host \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -e VEHICLE_NAME=robot1 \
    science-robot:latest \
    bash -c "source /entrypoint.sh && rostopic list | head -10"
echo ""

# Test 3: Check Python imports
echo "=== Test 3: Python Import Test ==="
docker run --rm --network host \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -e VEHICLE_NAME=robot1 \
    science-robot:latest \
    bash -c "source /entrypoint.sh && python3 -c 'import rospy; import cv2; import mediapipe; import duckietown_msgs; print(\"All imports successful\")'"
echo ""

# Test 4: Run main.py with error capture
echo "=== Test 4: Main Script Test (with error capture) ==="
docker run --rm --network host \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -e VEHICLE_NAME=robot1 \
    science-robot:latest \
    bash -c "source /entrypoint.sh && python3 main.py 2>&1 | head -50 || echo 'Script exited with error'"
echo ""

# Test 5: Interactive shell for manual testing
echo "=== Test 5: Interactive Shell (for manual testing) ==="
echo "Run this to get an interactive shell in the container:"
echo "docker run -it --rm --network host -e ROS_MASTER_URI=http://localhost:11311 -e VEHICLE_NAME=robot1 science-robot:latest bash"
echo ""

echo "=== Tests Complete ==="

