# Dockerfile for Duckiebot Science Fair Robot
# For Ubuntu 18.04 ARM64 (ROS Melodic)
# Uses standard Ubuntu base since ROS runs on host

FROM arm64v8/ubuntu:18.04

# Set working directory
WORKDIR /code

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies for OpenCV and MediaPipe
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    libopencv-dev \
    python3-opencv \
    libgl1-mesa-glx \
    libglib2.0-0 \
    curl \
    lsb-release \
    gnupg2 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements first for better caching
COPY requirements.txt .

# Install Python dependencies
RUN pip3 install --no-cache-dir -r requirements.txt

# Install ROS Melodic (Ubuntu 18.04)
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get install -y \
    ros-melodic-ros-base \
    ros-melodic-rospy \
    ros-melodic-cv-bridge \
    ros-melodic-sensor-msgs \
    ros-melodic-std-msgs \
    python-rosdep \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Note: duckietown-msgs needs to be installed separately
# Option 1: Install from Duckietown repos (if available)
# Option 2: Build from source in container
# Option 3: Mount host ROS workspace with duckietown-msgs
# For now, we'll try to install from apt (may fail - see DEPLOYMENT.md)
RUN apt-get update && \
    (apt-get install -y ros-melodic-duckietown-msgs 2>/dev/null || \
     echo "Warning: duckietown-msgs not available in apt - will need to install manually") && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Copy application code
COPY . .

# Make main script executable
RUN chmod +x main.py

# Set ROS environment variables (these will be set by Duckietown's entrypoint)
# ENV ROS_MASTER_URI=http://localhost:11311
# ENV ROS_HOSTNAME=localhost

# Entrypoint script
COPY docker-entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Use Duckietown's environment setup
ENTRYPOINT ["/entrypoint.sh"]
CMD ["python3", "main.py"]

