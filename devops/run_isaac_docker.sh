#!/bin/bash
# Isaac Sim 4.2.0 Docker 실행 스크립트

set -e

IMAGE="nvcr.io/nvidia/isaac-sim:4.2.0"
CONTAINER_NAME="isaac-sim-4.2"
WORKSPACE="$HOME/roarm_isaac_clean"

echo "======================================"
echo "Isaac Sim 4.2.0 Docker 실행"
echo "======================================"
echo ""

# X11 forwarding 활성화
xhost +local:docker > /dev/null 2>&1 || echo "Warning: xhost command failed"

# Docker run
docker run -it --rm \
    --name ${CONTAINER_NAME} \
    --runtime=nvidia \
    --gpus all \
    -e ACCEPT_EULA=Y \
    -e DISPLAY=${DISPLAY} \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v ${WORKSPACE}:/workspace \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v ~/.Xauthority:/root/.Xauthority:rw \
    -w /workspace \
    --network host \
    ${IMAGE} \
    /bin/bash
