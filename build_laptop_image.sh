#!/bin/bash

IMAGE_NAME="bin_boy_laptop"
TAG="foxy-qt-joystick"

echo "Building Docker image: ${IMAGE_NAME}:${TAG}..."
docker build -t "${IMAGE_NAME}:${TAG}" -f Dockerfile.laptop .

if [ $? -eq 0 ]; then
  echo "Docker image built successfully: ${IMAGE_NAME}:${TAG}"
else
  echo "Error: Docker image build failed."
  exit 1
fi