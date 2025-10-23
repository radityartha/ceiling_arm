#!/bin/bash

# Exit on any error
set -e

# Define variables
PROJECT_DIR=$(pwd)
IMAGE_NAME="moonshot_project_ros2"
TAG="latest"

# Check if dependencies folder and kortex_api wheel exist
if [ ! -f "$PROJECT_DIR/dependencies/kortex_api-2.6.0.post3-py3-none-any.whl" ]; then
    echo "Error: kortex_api-2.6.0.post3-py3-none-any.whl not found in $PROJECT_DIR/dependencies/"
    exit 1
fi

# Build the Docker image
echo "Building Docker image: $IMAGE_NAME:$TAG"
docker build -t $IMAGE_NAME:$TAG -f $PROJECT_DIR/Dockerfile $PROJECT_DIR

# Verify the image was built
if [ $? -eq 0 ]; then
    echo "Docker image $IMAGE_NAME:$TAG built successfully!"
else
    echo "Error: Failed to build Docker image"
    exit 1
fi