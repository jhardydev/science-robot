# Dockerfile for Duckiebot Science Fair Robot
# Uses Duckietown ROS commons base image for compatibility

FROM duckietown/dt-ros-commons:daffy-amd64

# Set working directory
WORKDIR /code

# Install system dependencies for OpenCV and MediaPipe
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    libopencv-dev \
    python3-opencv \
    libgl1-mesa-glx \
    libglib2.0-0 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements first for better caching
COPY requirements.txt .

# Install Python dependencies
RUN pip3 install --no-cache-dir -r requirements.txt

# Install ROS dependencies (duckietown-msgs should be in base image, but ensure they're there)
RUN apt-get update && apt-get install -y \
    ros-noetic-rospy \
    ros-noetic-duckietown-msgs \
    ros-noetic-cv-bridge \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    || (apt-get update && apt-get install -y \
        ros-kinetic-rospy \
        ros-kinetic-duckietown-msgs \
        ros-kinetic-cv-bridge \
        ros-kinetic-sensor-msgs \
        ros-kinetic-std-msgs) \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

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

