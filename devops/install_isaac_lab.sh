#!/bin/bash
# Isaac Lab 1.0.0 설치 스크립트 (Isaac Sim 4.2.0용)

set -e

IMAGE="nvcr.io/nvidia/isaac-sim:4.2.0"
ISAAC_LAB_VERSION="v1.0.0"

echo "=========================================="
echo "Isaac Lab 설치 (Isaac Sim 4.2.0)"
echo "=========================================="
echo ""

echo "1. Isaac Lab 클론 및 설치..."
docker run -it --rm \
    --runtime=nvidia \
    --gpus all \
    -e ACCEPT_EULA=Y \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v ~/roarm_isaac_clean:/workspace \
    -w /workspace \
    ${IMAGE} \
    bash -c "
set -e

echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
echo '📦 Step 1: 시스템 패키지 업데이트'
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
apt-get update -qq

echo ''
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
echo '📥 Step 2: Git 설치'
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
apt-get install -y git wget curl

echo ''
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
echo '🔽 Step 3: Isaac Lab 클론 (${ISAAC_LAB_VERSION})'
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
if [ -d '/isaac-sim/IsaacLab' ]; then
    echo 'Isaac Lab 디렉토리가 이미 존재합니다. 제거 중...'
    rm -rf /isaac-sim/IsaacLab
fi

cd /isaac-sim
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
git checkout ${ISAAC_LAB_VERSION}

echo ''
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
echo '⚙️  Step 4: Isaac Lab 설치'
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
./isaaclab.sh --install

echo ''
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
echo '✅ Step 5: 설치 확인'
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
python -c 'import isaaclab; print(\"Isaac Lab 버전:\", isaaclab.__version__)' || \
python -c 'import omni.isaac.lab; print(\"Isaac Lab import 성공!\")'

echo ''
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
echo '✅ Isaac Lab 설치 완료!'
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
echo ''
echo 'Isaac Lab 경로: /isaac-sim/IsaacLab'
echo 'Isaac Sim 경로: /isaac-sim'
echo ''
"

echo ""
echo "=========================================="
echo "✅ 설치 완료!"
echo "=========================================="
