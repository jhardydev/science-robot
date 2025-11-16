#!/bin/bash
# Cleanup script for Duckiebot Science Fair Robot Docker containers and images
# Usage: ./docker-cleanup.sh [OPTIONS]

set -e

# Colors for output
RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

# Default values
CLEAN_CONTAINERS=true
CLEAN_IMAGE=false
CLEAN_VOLUMES=false
CLEAN_ALL=false
FORCE=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --containers)
            CLEAN_CONTAINERS=true
            shift
            ;;
        --image)
            CLEAN_IMAGE=true
            shift
            ;;
        --volumes)
            CLEAN_VOLUMES=true
            shift
            ;;
        --all)
            CLEAN_ALL=true
            shift
            ;;
        --force)
            FORCE=true
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Cleanup Docker resources for science-robot"
            echo ""
            echo "Options:"
            echo "  --containers    Remove stopped containers (default)"
            echo "  --image         Remove science-robot Docker image"
            echo "  --volumes       Remove associated volumes"
            echo "  --all           Remove containers, image, and volumes"
            echo "  --force         Skip confirmation prompts"
            echo "  --help          Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                    # Remove stopped containers only"
            echo "  $0 --all              # Remove everything"
            echo "  $0 --image --force    # Remove image without confirmation"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo -e "${YELLOW}Duckiebot Science Fair Robot - Docker Cleanup${NC}"
echo ""

# If --all is specified, enable everything
if [ "$CLEAN_ALL" = true ]; then
    CLEAN_CONTAINERS=true
    CLEAN_IMAGE=true
    CLEAN_VOLUMES=true
fi

# Clean up containers
if [ "$CLEAN_CONTAINERS" = true ]; then
    echo -e "${YELLOW}Checking for containers...${NC}"
    
    # Check for running containers
    if docker ps --format '{{.Names}}' | grep -q "^science-robot$"; then
        echo -e "${RED}Found running container 'science-robot'${NC}"
        if [ "$FORCE" = false ]; then
            read -p "Stop and remove running container? (y/n) " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                echo "Skipping container cleanup"
                CLEAN_CONTAINERS=false
            fi
        fi
        
        if [ "$CLEAN_CONTAINERS" = true ]; then
            echo -e "${YELLOW}Stopping container...${NC}"
            docker stop science-robot 2>/dev/null || true
            echo -e "${GREEN}Container stopped${NC}"
        fi
    fi
    
    # Remove stopped containers
    if [ "$CLEAN_CONTAINERS" = true ]; then
        CONTAINERS=$(docker ps -a --filter "name=science-robot" --format "{{.ID}}" 2>/dev/null || true)
        if [ -n "$CONTAINERS" ]; then
            echo -e "${YELLOW}Removing stopped containers...${NC}"
            echo "$CONTAINERS" | xargs -r docker rm 2>/dev/null || true
            echo -e "${GREEN}Containers removed${NC}"
        else
            echo -e "${GREEN}No containers to remove${NC}"
        fi
    fi
fi

# Clean up image
if [ "$CLEAN_IMAGE" = true ]; then
    echo ""
    echo -e "${YELLOW}Checking for Docker image...${NC}"
    
    if docker image inspect science-robot:latest > /dev/null 2>&1; then
        if [ "$FORCE" = false ]; then
            read -p "Remove Docker image 'science-robot:latest'? (y/n) " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                echo "Skipping image cleanup"
                CLEAN_IMAGE=false
            fi
        fi
        
        if [ "$CLEAN_IMAGE" = true ]; then
            echo -e "${YELLOW}Removing Docker image...${NC}"
            docker rmi science-robot:latest 2>/dev/null || true
            echo -e "${GREEN}Image removed${NC}"
        fi
    else
        echo -e "${GREEN}No image to remove${NC}"
    fi
    
    # Also check for dangling images
    DANGLING=$(docker images -f "dangling=true" -q 2>/dev/null | wc -l)
    if [ "$DANGLING" -gt 0 ]; then
        if [ "$FORCE" = false ]; then
            read -p "Remove $DANGLING dangling images? (y/n) " -n 1 -r
            echo
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                docker image prune -f
                echo -e "${GREEN}Dangling images removed${NC}"
            fi
        else
            docker image prune -f
            echo -e "${GREEN}Dangling images removed${NC}"
        fi
    fi
fi

# Clean up volumes
if [ "$CLEAN_VOLUMES" = true ]; then
    echo ""
    echo -e "${YELLOW}Checking for volumes...${NC}"
    
    VOLUMES=$(docker volume ls --filter "name=science-robot" --format "{{.Name}}" 2>/dev/null || true)
    if [ -n "$VOLUMES" ]; then
        if [ "$FORCE" = false ]; then
            echo "Found volumes:"
            echo "$VOLUMES"
            read -p "Remove these volumes? (y/n) " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                echo "Skipping volume cleanup"
                CLEAN_VOLUMES=false
            fi
        fi
        
        if [ "$CLEAN_VOLUMES" = true ]; then
            echo -e "${YELLOW}Removing volumes...${NC}"
            echo "$VOLUMES" | xargs -r docker volume rm 2>/dev/null || true
            echo -e "${GREEN}Volumes removed${NC}"
        fi
    else
        echo -e "${GREEN}No volumes to remove${NC}"
    fi
fi

echo ""
echo -e "${GREEN}Cleanup complete!${NC}"
echo ""
echo "Summary:"
docker ps -a --filter "name=science-robot" --format "table {{.Names}}\t{{.Status}}" 2>/dev/null || echo "  No containers found"
docker image ls science-robot 2>/dev/null || echo "  No images found"

