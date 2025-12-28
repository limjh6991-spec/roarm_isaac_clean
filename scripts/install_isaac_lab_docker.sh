#!/bin/bash

# Isaac Lab Docker 설치 및 실행 스크립트
# RTX 5090 + Isaac Sim 5.x 지원

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║         Isaac Lab Docker 설치 (Isaac Sim 5.x)             ║${NC}"
echo -e "${BLUE}║              RTX 5090 완벽 지원 버전                        ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

# 1. Docker 확인
echo -e "${YELLOW}[1/5] Docker 설치 확인...${NC}"
if ! command -v docker &> /dev/null; then
    echo -e "${RED}❌ Docker가 설치되어 있지 않습니다.${NC}"
    echo "설치 방법: sudo apt-get install docker.io docker-compose"
    exit 1
fi
echo -e "${GREEN}✅ Docker $(docker --version | cut -d ' ' -f3)${NC}"

# 2. NVIDIA Docker 확인
echo -e "${YELLOW}[2/5] NVIDIA Docker 지원 확인...${NC}"
if ! docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi &> /dev/null; then
    echo -e "${RED}❌ NVIDIA Docker 지원이 없습니다.${NC}"
    echo "설치 방법: sudo apt-get install nvidia-container-toolkit"
    exit 1
fi
echo -e "${GREEN}✅ NVIDIA Docker 정상 작동${NC}"

# 3. GPU 정보 확인
echo -e "${YELLOW}[3/5] GPU 정보 확인...${NC}"
GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)
DRIVER_VERSION=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)
echo -e "${GREEN}✅ GPU: ${GPU_NAME}${NC}"
echo -e "${GREEN}✅ 드라이버: ${DRIVER_VERSION}${NC}"

# 4. X11 권한 설정
echo -e "${YELLOW}[4/5] X11 디스플레이 권한 설정...${NC}"
xhost +local:docker > /dev/null 2>&1 || true
echo -e "${GREEN}✅ X11 권한 설정 완료${NC}"

# 5. Isaac Lab Docker 이미지 다운로드
echo -e "${YELLOW}[5/5] Isaac Lab Docker 이미지 다운로드...${NC}"
echo -e "${BLUE}📦 약 10-15GB 다운로드 예상 (최초 1회만)${NC}"
echo ""

# 이미지 확인
if docker images | grep -q "nvcr.io/nvidia/isaac-sim"; then
    echo -e "${GREEN}✅ Isaac Sim 이미지가 이미 존재합니다.${NC}"
    read -p "다시 다운로드하시겠습니까? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${BLUE}기존 이미지를 사용합니다.${NC}"
    else
        docker pull nvcr.io/nvidia/isaac-sim:4.2.0
    fi
else
    docker pull nvcr.io/nvidia/isaac-sim:4.2.0
fi

echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║              ✅ Isaac Lab Docker 설치 완료!                ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

# 실행 방법 안내
echo -e "${BLUE}📚 사용 방법:${NC}"
echo ""
echo -e "${YELLOW}방법 1: Docker Compose 사용 (권장)${NC}"
echo "  $ docker-compose -f docker-compose.isaac-lab.yml up -d"
echo "  $ docker exec -it isaac-lab-rtx5090 /bin/bash"
echo ""
echo -e "${YELLOW}방법 2: 직접 실행${NC}"
echo "  $ bash scripts/run_isaac_lab_docker.sh"
echo ""
echo -e "${YELLOW}방법 3: Docker 명령어${NC}"
echo "  $ docker run --rm -it --gpus all \\"
echo "      -e DISPLAY=\$DISPLAY \\"
echo "      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \\"
echo "      -v \$(pwd):/workspace \\"
echo "      isaac-sim.nvidia.com/isaac-lab:latest"
echo ""

# 버전 확인 스크립트 제공
echo -e "${BLUE}🔍 Isaac Sim 버전 확인:${NC}"
echo "  $ docker run --rm --gpus all isaac-sim.nvidia.com/isaac-lab:latest \\"
echo "      python -c \"from isaacsim import SimulationApp; \\"
echo "      app = SimulationApp({'headless': True}); \\"
echo "      print(f'Isaac Sim Version: {app.version}'); \\"
echo "      app.close()\""
echo ""

# 다음 단계
echo -e "${GREEN}🎯 다음 단계:${NC}"
echo "  1. Docker 컨테이너 실행"
echo "  2. 프로젝트 파일이 /workspace에 마운트됨"
echo "  3. Vision RL 학습 시작: python scripts/train/train_vision_sac.py"
echo ""
