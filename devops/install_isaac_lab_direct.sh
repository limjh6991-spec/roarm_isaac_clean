#!/bin/bash

echo "=========================================="
echo "Isaac Lab 설치 (Isaac Sim 4.2.0 직접 설치)"
echo "=========================================="

# 컨테이너에서 직접 명령 실행
echo ""
echo "Step 1: apt-get update"
docker run --rm --gpus all \
  --env ACCEPT_EULA=Y \
  nvcr.io/nvidia/isaac-sim:4.2.0 \
  bash -c "apt-get update -y"

echo ""
echo "Step 2: git 설치"
docker run --rm --gpus all \
  --env ACCEPT_EULA=Y \
  nvcr.io/nvidia/isaac-sim:4.2.0 \
  bash -c "apt-get install -y git"

echo ""
echo "Step 3: Isaac Lab 클론"
docker run --rm --gpus all \
  --env ACCEPT_EULA=Y \
  -v /tmp/isaac_lab:/workspace \
  nvcr.io/nvidia/isaac-sim:4.2.0 \
  bash -c "cd /workspace && git clone https://github.com/isaac-sim/IsaacLab.git"

echo ""
echo "Step 4: v1.0.0 체크아웃"
docker run --rm --gpus all \
  --env ACCEPT_EULA=Y \
  -v /tmp/isaac_lab:/workspace \
  nvcr.io/nvidia/isaac-sim:4.2.0 \
  bash -c "cd /workspace/IsaacLab && git checkout v1.0.0"

echo ""
echo "Step 5: Isaac Lab 설치 실행"
docker run --rm --gpus all \
  --env ACCEPT_EULA=Y \
  -v /tmp/isaac_lab:/workspace \
  nvcr.io/nvidia/isaac-sim:4.2.0 \
  bash -c "cd /workspace/IsaacLab && ./isaaclab.sh --install"

echo ""
echo "Step 6: 설치 검증"
docker run --rm --gpus all \
  --env ACCEPT_EULA=Y \
  -v /tmp/isaac_lab:/workspace \
  nvcr.io/nvidia/isaac-sim:4.2.0 \
  bash -c "cd /workspace/IsaacLab && python -c 'import isaaclab; print(\"✅ Isaac Lab 설치 완료:\", isaaclab.__version__)'"

echo ""
echo "=========================================="
echo "설치 완료!"
echo "=========================================="
