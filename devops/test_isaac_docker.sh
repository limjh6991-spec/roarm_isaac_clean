#!/bin/bash
# Isaac Sim 4.2.0 Docker - 빠른 테스트 스크립트

IMAGE="nvcr.io/nvidia/isaac-sim:4.2.0"

echo "=========================================="
echo "Isaac Sim 4.2.0 기본 테스트 (Headless)"
echo "=========================================="
echo ""

# X11 forwarding
xhost +local:docker > /dev/null 2>&1

# 기본 Isaac Sim 부팅 테스트
docker run --rm \
    --runtime=nvidia \
    --gpus all \
    -e ACCEPT_EULA=Y \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v ~/roarm_isaac_clean:/workspace \
    -w /workspace \
    ${IMAGE} \
    python scripts/test/test_isaac_basic.py --headless

echo ""
echo "=========================================="
echo "테스트 완료!"
echo "=========================================="
