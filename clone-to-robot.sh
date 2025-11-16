#!/bin/bash
# Script to clone science-robot repository to Duckiebot
# Usage: ./clone-to-robot.sh [robot-hostname-or-ip] [target-directory]

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
ROBOT_HOST="${1:-robot1}"
ROBOT_USER="${DUCKIEBOT_USER:-duckiebot}"
TARGET_DIR="${2:-~/science-robot}"
REPO_URL="https://github.com/jhardydev/science-robot.git"

echo -e "${BLUE}Cloning science-robot to Duckiebot${NC}"
echo "Robot: ${ROBOT_USER}@${ROBOT_HOST}"
echo "Target: ${TARGET_DIR}"
echo "Repository: ${REPO_URL}"
echo ""

# Test SSH connection
echo -e "${YELLOW}Testing SSH connection...${NC}"
if ! ssh -o ConnectTimeout=5 -o BatchMode=yes "${ROBOT_USER}@${ROBOT_HOST}" exit 2>/dev/null; then
    echo -e "${YELLOW}SSH connection test failed. You may need to:${NC}"
    echo "  1. Ensure SSH keys are set up (or use password authentication)"
    echo "  2. Verify the robot hostname/IP: ${ROBOT_HOST}"
    echo "  3. Check robot is accessible on network"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Clone repository
echo -e "${GREEN}Cloning repository...${NC}"
ssh "${ROBOT_USER}@${ROBOT_HOST}" << EOF
    # Remove existing directory if it exists
    if [ -d "${TARGET_DIR}" ]; then
        echo "Removing existing directory..."
        rm -rf "${TARGET_DIR}"
    fi
    
    # Create parent directory if needed
    mkdir -p "$(dirname "${TARGET_DIR}")"
    
    # Clone repository
    echo "Cloning ${REPO_URL} to ${TARGET_DIR}..."
    git clone ${REPO_URL} "${TARGET_DIR}"
    
    # Verify clone
    if [ -d "${TARGET_DIR}" ]; then
        cd "${TARGET_DIR}"
        echo ""
        echo "Repository cloned successfully!"
        echo "Location: ${TARGET_DIR}"
        echo ""
        echo "Files in repository:"
        ls -la
        echo ""
        echo "To build and run:"
        echo "  cd ${TARGET_DIR}"
        echo "  ./docker-run.sh --build"
    else
        echo "Error: Repository clone failed!"
        exit 1
    fi
EOF

echo ""
echo -e "${GREEN}Done!${NC}"
echo ""
echo "To connect to robot and work with the repository:"
echo "  ssh ${ROBOT_USER}@${ROBOT_HOST}"
echo "  cd ${TARGET_DIR}"

