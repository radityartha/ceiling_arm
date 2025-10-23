#!/bin/bash

# Exit on any error
set -e

# Define variables
IMAGE_NAME="moonshot_project_ros2"
TAG="latest"
CONTAINER_NAME="moonshot_project_ros2_container"
PROJECT_DIR=$(pwd)
WORKSPACE="/moonshot_project/ros2_ws"

# Check if the Docker image exists
if ! docker image inspect $IMAGE_NAME:$TAG > /dev/null 2>&1; then
    echo "Error: Docker image $IMAGE_NAME:$TAG not found. Please run build.sh first."
    exit 1
fi

# Check for X11 display (for GUI tools like RViz/Gazebo)
if [ -z "$DISPLAY" ]; then
    echo "Warning: DISPLAY environment variable not set. GUI tools may not work."
fi

# Run the Docker container
echo "Starting Docker container: $CONTAINER_NAME"
docker run -it \
    --name $CONTAINER_NAME \
    --network host \
    --privileged \
    --env DISPLAY=$DISPLAY \
    --env ROS_DOMAIN_ID=0 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume $PROJECT_DIR/volumes/data:/moonshot_project/volumes/data \
    --volume /dev:/dev \
    $IMAGE_NAME:$TAG

# Remove the container after it exits
echo "Cleaning up: Removing container $CONTAINER_NAME"
docker rm $CONTAINER_NAME || true