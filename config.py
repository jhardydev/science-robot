"""
Configuration parameters for Duckiebot Science Fair Robot
"""
import os

# ROS configuration
ROBOT_NAME = 'robot1'  # Your robot's name
MOTOR_TOPIC = f'/{ROBOT_NAME}/wheels_driver_node/wheels_cmd'
CAMERA_TOPIC = f'/{ROBOT_NAME}/camera_node/image/compressed'
EMERGENCY_STOP_TOPIC = f'/{ROBOT_NAME}/wheels_driver_node/emergency_stop'

# Camera settings (for resizing/processing, actual source is ROS topic)
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 60  # Increased for better responsiveness

# Motor control settings (legacy GPIO settings - not used with ROS)
# MOTOR_LEFT_PIN1 = 18
# MOTOR_LEFT_PIN2 = 19
# MOTOR_RIGHT_PIN1 = 20
# MOTOR_RIGHT_PIN2 = 21
# PWM_FREQUENCY = 1000  # Hz

# Motor speeds (0.0 to 1.0)
MOTOR_BASE_SPEED = 0.5
MOTOR_TURN_SPEED = 0.6
MOTOR_MAX_SPEED = 0.8
MOTOR_DANCE_SPEED = 0.7

# Wave detection parameters
WAVE_DETECTION_FRAMES = 15  # Number of frames to analyze for wave detection
WAVE_MOTION_THRESHOLD = 30  # Minimum pixel movement to consider as waving
WAVE_MIN_DURATION = 0.5  # Minimum seconds of waving to trigger
WAVE_SENSITIVITY = 0.3  # Sensitivity for wave detection (0.0 to 1.0)

# Navigation parameters
STEERING_GAIN = 1.5  # Proportional gain for steering control
STEERING_DEAD_ZONE = 0.1  # Dead zone in normalized coordinates (0.0 to 1.0)
MAX_STEERING_ANGLE = 1.0  # Maximum steering angle (normalized)

# Gesture recognition thresholds
GESTURE_CONFIDENCE_THRESHOLD = 0.7
DANCE_GESTURE_HOLD_TIME = 1.0  # Seconds gesture must be held
TREAT_GESTURE_HOLD_TIME = 2.0  # Seconds gesture must be held (future)
# Distance thresholds for clap (normalized coordinates 0.0 - 1.0)
DANCE_CLAP_FINGER_THRESHOLD = 0.12  # Max distance between corresponding fingertips
DANCE_CLAP_PALM_THRESHOLD = 0.18    # Max distance between palms/wrists

# Dance routine settings
DANCE_DURATION = 5.0  # Total dance duration in seconds
DANCE_MOVE_DURATION = 1.0  # Duration of each dance move in seconds

# Main loop settings
MAIN_LOOP_FPS = 60  # Increased frame rate for smoother operation
# Disable display output by default in container (headless mode)
# Set to True only if you have X11 forwarding set up or virtual display enabled
DISPLAY_OUTPUT = os.getenv('DISPLAY_OUTPUT', 'False').lower() == 'true'  # Auto-detect or use env var

# Virtual display settings (for headless testing with video output)
USE_VIRTUAL_DISPLAY = os.getenv('USE_VIRTUAL_DISPLAY', 'False').lower() == 'true'  # Start Xvfb virtual display
VIRTUAL_DISPLAY_NUM = int(os.getenv('VIRTUAL_DISPLAY_NUM', '99'))  # Display number (e.g., :99)
VIRTUAL_DISPLAY_SIZE = os.getenv('VIRTUAL_DISPLAY_SIZE', '1024x768x24')  # Resolution and depth

# Debug settings
DEBUG_MODE = os.getenv('DEBUG_MODE', 'False').lower() == 'true'
LOG_GESTURES = True

# Logging settings
LOG_DIR = os.getenv('LOG_DIR', '/code/logs')
LOG_LEVEL = os.getenv('LOG_LEVEL', 'INFO').upper()  # DEBUG, INFO, WARNING, ERROR
ENABLE_FILE_LOGGING = os.getenv('ENABLE_FILE_LOGGING', 'True').lower() == 'true'

# NVIDIA acceleration settings
USE_CUDA_ACCELERATION = True  # Use CUDA-accelerated OpenCV if available
USE_VPI_ACCELERATION = True   # Use VPI for GPU-accelerated image processing
VPI_BACKEND = 'CUDA'          # VPI backend: 'CUDA' (GPU), 'CPU', 'VIC', 'PVA', or 'OFA'
                              # Note: 'GPU' is accepted as alias for 'CUDA'
GPU_PREPROCESSING = True       # Use GPU for image preprocessing

# Performance monitoring
ENABLE_PERFORMANCE_MONITORING = False  # Log FPS and processing times

