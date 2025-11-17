# Dockerfile for Duckiebot Science Fair Robot
# For Ubuntu 20.04 ARM64 (ROS Noetic)
# Uses Duckietown ROS commons base image

FROM duckietown/dt-ros-commons:daffy-arm64v8

# Set working directory
WORKDIR /code

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies for OpenCV and MediaPipe
# ROS and duckietown-msgs should already be in the base image
# Note: VPI (Vision Programming Interface) is typically part of JetPack SDK
# and may not be available in all Docker images. It will gracefully fall back to CPU.
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    libopencv-dev \
    python3-opencv \
    libgl1-mesa-glx \
    libglib2.0-0 \
    xvfb \
    x11vnc \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Try to install VPI if available (may not be in all base images)
# VPI is typically part of NVIDIA JetPack SDK on Jetson devices
RUN python3 -c "import vpi" 2>/dev/null && echo "VPI is available" || \
    (echo "VPI not found in base image - will use CPU fallback" && \
     echo "Note: VPI requires JetPack SDK on Jetson devices")

# Copy requirements first for better caching
COPY requirements.txt .

# Install Python dependencies with increased timeout and retry logic
# Large packages like opencv-python can timeout, so we increase timeout and retry
RUN pip3 install --no-cache-dir --default-timeout=300 --retries=3 \
    --upgrade pip setuptools wheel && \
    pip3 install --no-cache-dir --default-timeout=300 --retries=3 -r requirements.txt

# Copy application code
COPY . .

# Make main script executable
RUN chmod +x main.py

# Create logs directory
RUN mkdir -p /code/logs

# Set ROS environment variables (these will be set by Duckietown's entrypoint)
# ENV ROS_MASTER_URI=http://localhost:11311
# ENV ROS_HOSTNAME=localhost

# Entrypoint script
COPY docker-entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Use Duckietown's environment setup
ENTRYPOINT ["/entrypoint.sh"]
CMD ["python3", "main.py"]

