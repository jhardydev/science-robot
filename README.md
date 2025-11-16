# Duckiebot Science Fair Robot

A gesture-controlled robot that detects waving children, steers toward them, and performs dance routines when triggered by hand gestures. Built for Duckiebot platforms using ROS.

## Features

- **Wave Detection**: Uses computer vision to detect waving children and steer the robot toward them
- **Gesture Recognition**: Recognizes hand gestures using MediaPipe
- **Dance Routines**: Performs predefined dance sequences when a special gesture is detected
- **Treat Dispensing**: Architecture ready for future treat dispensing mechanism
- **NVIDIA GPU Acceleration**: CUDA and VPI acceleration for improved performance (optional)
- **ROS Integration**: Uses ROS topics for motor control and camera access
- **Docker Support**: Containerized for easy deployment on Duckiebot platforms

## Hardware Requirements

- Duckiebot with ROS running
- Camera (access via ROS camera node)
- Motor control via Duckietown's wheels_driver_node

## Software Requirements

- ROS Melodic (Ubuntu 18.04) or ROS Noetic (Ubuntu 20.04)
- Python 3.6+
- Docker (for containerized deployment)
- ARM64 architecture (tested on Jetson Nano with Ubuntu 18.04)

## Quick Start with Docker (Recommended)

```bash
# Build and run with helper script
./docker-run.sh --build

# Or use docker-compose
docker-compose up --build
```

See [DEPLOYMENT.md](DEPLOYMENT.md) for detailed deployment instructions.

## Manual Installation

1. **Clone or navigate to the project directory**

2. **Install Python dependencies**:
   ```bash
   pip3 install -r requirements.txt
   ```

3. **Install ROS dependencies** (on robot):
   ```bash
   # For Ubuntu 18.04 (ROS Melodic):
   sudo apt-get install ros-melodic-rospy ros-melodic-duckietown-msgs ros-melodic-cv-bridge ros-melodic-sensor-msgs ros-melodic-std-msgs
   # Or for Ubuntu 20.04 (ROS Noetic):
   sudo apt-get install ros-noetic-rospy ros-noetic-duckietown-msgs ros-noetic-cv-bridge ros-noetic-sensor-msgs ros-noetic-std-msgs
   ```

4. **Configure settings** in `config.py`:
   - Robot name (default: robot1)
   - ROS topics (auto-configured based on robot name)
   - Motor speeds and thresholds

## Configuration

Edit `config.py` to adjust:

- **ROS settings**: Robot name and topic names (auto-configured)
- **Camera settings**: Resolution, FPS (source is ROS topic)
- **Motor control**: Speeds and thresholds (control via ROS topics)
- **Wave detection**: Sensitivity, motion thresholds
- **Navigation**: Steering gain, dead zones
- **Gestures**: Confidence thresholds, hold times
- **NVIDIA acceleration**: CUDA, VPI, and GPU preprocessing options (optional)
- **Performance monitoring**: Enable FPS and processing time logging

## Usage

### Running in Container (Recommended)

```bash
# Build and run
./docker-run.sh --build

# Clean up before running (if needed)
./docker-cleanup.sh --all --force
./docker-run.sh --build
```

### Running Directly

Ensure ROS master is running, then:

```bash
# Set robot name if different from default
export VEHICLE_NAME=robot1

# Run
python3 main.py
```

### Controls

- **'q'**: Quit the program
- **'s'**: Emergency stop (stops motors immediately)

### Gestures

- **Wave**: Side-to-side hand motion to attract robot's attention
- **Dance Trigger**: Open hand (all fingers extended) held for 1 second
- **Treat Trigger** (future): Thumb and pinky extended (shaka gesture) held for 2 seconds

## Project Structure

```
science-robot/
├── main.py                 # Main control loop with ROS integration
├── config.py               # Configuration parameters
├── requirements.txt        # Python dependencies
├── Dockerfile              # Docker container definition
├── docker-compose.yml      # Docker Compose configuration
├── docker-entrypoint.sh    # Container entrypoint script
├── docker-run.sh           # Helper script for building/running
├── docker-cleanup.sh       # Cleanup script for development
├── DEPLOYMENT.md           # Detailed deployment guide
├── src/
│   ├── camera.py          # ROS camera subscriber
│   ├── gesture_detector.py # Hand/gesture detection
│   ├── wave_detector.py    # Wave detection logic
│   ├── motor_controller.py # ROS-based motor control
│   ├── navigation.py       # Steering logic
│   ├── dance.py           # Dance routines
│   ├── treat_dispenser.py  # Treat dispensing (future)
│   └── vpi_processor.py    # VPI GPU-accelerated image processing
└── README.md              # This file
```

## Calibration

After initial setup, you may need to calibrate:

1. **Robot name**: Set `ROBOT_NAME` in `config.py` to match your Duckiebot
2. **Motor speeds**: Adjust `MOTOR_BASE_SPEED`, `MOTOR_TURN_SPEED` in `config.py`
3. **Wave detection**: Tune `WAVE_MOTION_THRESHOLD` and `WAVE_SENSITIVITY`
4. **Steering**: Adjust `STEERING_GAIN` and `STEERING_DEAD_ZONE`
5. **ROS topics**: Verify topic names match your Duckiebot setup

## NVIDIA GPU Acceleration

This project includes NVIDIA GPU acceleration support for improved performance:

- **CUDA Acceleration**: Automatically detected and used if OpenCV is built with CUDA support
- **VPI (Vision Programming Interface)**: GPU-accelerated image processing operations
- **Automatic Fallback**: If GPU acceleration isn't available, the system automatically falls back to CPU

### Configuration

Enable/disable acceleration features in `config.py`:

```python
USE_CUDA_ACCELERATION = True   # Use CUDA if available
USE_VPI_ACCELERATION = True    # Use VPI for GPU processing
VPI_BACKEND = 'GPU'            # VPI backend: 'GPU', 'CPU', or 'VIC'
GPU_PREPROCESSING = True       # Use GPU for image preprocessing
ENABLE_PERFORMANCE_MONITORING = False  # Log FPS and processing times
```

### Verifying GPU Acceleration

On startup, the robot will display which acceleration methods are available:
- "CUDA acceleration: Available" - OpenCV CUDA is working
- "VPI acceleration: Available" - VPI library is available
- The video overlay shows "CUDA" and "VPI" indicators when active

## Troubleshooting

### Camera not detected
- Verify ROS camera node is running: `rostopic list | grep camera`
- Check camera topic is publishing: `rostopic hz /robot1/camera_node/image/compressed`
- Verify robot name in `config.py` matches your Duckiebot
- Check camera node is enabled in Duckiebot's configuration

### Motors not responding
- Verify ROS wheels_driver_node is running: `rostopic list | grep wheels`
- Check robot's FSM mode allows external commands: `rostopic echo /robot1/fsm_node/mode`
- Verify motor topic name matches your setup: `rostopic info /robot1/wheels_driver_node/wheels_cmd`
- Test publishing to motor topic manually to verify connectivity

### Poor gesture recognition
- Improve lighting conditions
- Adjust `GESTURE_CONFIDENCE_THRESHOLD` in `config.py`
- Ensure hand is clearly visible in frame

### GPU acceleration not working
- Ensure JetPack SDK is installed on Jetson Nano
- Check that OpenCV was built with CUDA support (usually included with JetPack)
- Verify VPI is installed (included with JetPack)
- If unavailable, the system will automatically fall back to CPU processing

## Future Enhancements

- Treat dispensing hardware integration
- Obstacle avoidance
- Multiple gesture patterns
- Audio feedback
- LED indicators

## License

This project is created for educational purposes (science fair project).

## Deployment

See [DEPLOYMENT.md](DEPLOYMENT.md) for detailed instructions on:
- Building Docker containers
- Deploying on Duckiebot platforms
- Troubleshooting common issues
- Integration with Duckietown stack

## Credits

Built using:
- OpenCV for computer vision (with CUDA acceleration)
- MediaPipe for hand detection
- ROS (Robot Operating System) for hardware interface
- Duckietown ROS stack for robot integration
- NVIDIA VPI for GPU-accelerated image processing (optional)

