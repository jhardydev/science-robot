# Clone Repository to Duckiebot

This guide shows you how to clone the `science-robot` repository to your Duckiebot.

## Quick Method: Using the Helper Script

```bash
# Basic usage (uses default robot1 and ~/science-robot)
./clone-to-robot.sh

# Specify robot hostname/IP
./clone-to-robot.sh 192.168.1.100

# Specify robot and target directory
./clone-to-robot.sh robot1 ~/projects/science-robot

# Use different SSH user
export DUCKIEBOT_USER=ubuntu
./clone-to-robot.sh robot1
```

## Manual Method: SSH and Clone

### Step 1: Connect to Robot

```bash
ssh duckiebot@robot1
# Or with IP:
ssh duckiebot@192.168.1.100
```

### Step 2: Clone Repository

```bash
# Navigate to desired location
cd ~

# Clone repository
git clone https://github.com/jhardydev/science-robot.git

# Or clone to specific directory
git clone https://github.com/jhardydev/science-robot.git ~/projects/science-robot

# Enter repository
cd science-robot
```

### Step 3: Verify Clone

```bash
# Check files
ls -la

# Verify git remote
git remote -v

# Check out main branch (should already be checked out)
git branch
```

## Setup After Cloning

### Option 1: Build and Run with Docker (Recommended)

```bash
cd ~/science-robot

# Build and run
./docker-run.sh --build

# Or use docker-compose
docker-compose up --build
```

### Option 2: Manual Setup (Without Docker)

```bash
cd ~/science-robot

# Install Python dependencies
pip3 install -r requirements.txt

# Install ROS dependencies
# For Ubuntu 18.04 (ROS Melodic):
sudo apt-get update
sudo apt-get install -y \
    ros-melodic-rospy \
    ros-melodic-duckietown-msgs \
    ros-melodic-cv-bridge \
    ros-melodic-sensor-msgs \
    ros-melodic-std-msgs
# For Ubuntu 20.04 (ROS Noetic), use ros-noetic-* packages instead

# Ensure ROS master is running
roscore &

# Run application
python3 main.py
```

## Configuration

After cloning, update configuration if needed:

```bash
cd ~/science-robot

# Edit config.py to set robot name if different
nano config.py
# Change: ROBOT_NAME = 'robot1' to match your robot
```

## Updating the Repository

To get the latest changes from GitHub:

```bash
cd ~/science-robot

# Pull latest changes
git pull origin main

# If you have local changes, stash first:
git stash
git pull origin main
git stash pop
```

## Troubleshooting

### SSH Connection Issues

**Problem**: Can't connect to robot via SSH

**Solutions**:
- Verify robot is on network: `ping robot1` or `ping <robot-ip>`
- Check SSH service is running on robot
- Verify SSH keys are set up (or use password authentication)
- Try connecting with IP address instead of hostname

### Git Clone Fails

**Problem**: Git clone fails with permission or network error

**Solutions**:
- Ensure robot has internet access
- Verify GitHub repository is accessible: `curl -I https://github.com/jhardydev/science-robot`
- Check git is installed: `git --version`
- Try cloning to a directory you have write permissions for

### Missing Files After Clone

**Problem**: Some files seem to be missing

**Solutions**:
- Check `.gitignore` to see if files are intentionally ignored
- Verify you're on the `main` branch: `git branch`
- Check for untracked files: `git status`

### Docker Build Fails

**Problem**: Docker build fails on robot

**Solutions**:
- Ensure Docker is installed: `docker --version`
- Check Docker daemon is running: `sudo systemctl status docker`
- Verify robot has sufficient disk space: `df -h`
- Check Docker build logs for specific errors

## Alternative: Direct File Transfer

If git clone doesn't work, you can transfer files directly:

### From Local Machine

```bash
# From your local machine, in the science-robot directory
cd "/Users/jasonhardy/Desktop/working/Coding Projects/science-robot"

# Create tarball
tar czf science-robot.tar.gz --exclude='.git' --exclude='__pycache__' .

# Transfer to robot
scp science-robot.tar.gz duckiebot@robot1:~/

# SSH to robot and extract
ssh duckiebot@robot1
cd ~
tar xzf science-robot.tar.gz -C science-robot
```

## Next Steps

After cloning:

1. **Review configuration**: Edit `config.py` if needed
2. **Build container**: `./docker-run.sh --build`
3. **Test connection**: Verify ROS topics are accessible
4. **Run application**: Start the robot controller

For detailed deployment instructions, see [DEPLOYMENT.md](DEPLOYMENT.md).

