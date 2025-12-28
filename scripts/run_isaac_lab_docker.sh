#!/bin/bash

# Isaac Lab Docker 실행 스크립트

set -e

# 색상 정의
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║            Isaac Lab Docker 컨테이너 실행                   ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

# X11 권한 설정
echo -e "${YELLOW}🔧 X11 디스플레이 권한 설정...${NC}"
xhost +local:docker > /dev/null 2>&1 || true

# 현재 디렉토리
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo -e "${GREEN}📁 워크스페이스: ${WORKSPACE_DIR}${NC}"
echo ""

# 컨테이너 이름
CONTAINER_NAME="isaac-lab-rtx5090"

# 기존 컨테이너 확인 및 정리
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo -e "${YELLOW}⚠️  기존 컨테이너 발견: ${CONTAINER_NAME}${NC}"
    read -p "기존 컨테이너를 삭제하고 새로 시작하시겠습니까? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        docker rm -f ${CONTAINER_NAME}
        echo -e "${GREEN}✅ 기존 컨테이너 삭제 완료${NC}"
    else
        echo -e "${BLUE}기존 컨테이너 재시작...${NC}"
        docker start ${CONTAINER_NAME}
        docker exec -it ${CONTAINER_NAME} /bin/bash
        exit 0
    fi
fi

echo -e "${BLUE}🚀 Isaac Lab 컨테이너 시작...${NC}"
echo ""

# Docker 실행
docker run --rm -it \
  --name ${CONTAINER_NAME} \
  --gpus all \
  --network host \
  -e DISPLAY=${DISPLAY} \
  -e ACCEPT_EULA=Y \
  -e OMNI_KIT_ACCEPT_EULA=YES \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ${HOME}/.Xauthority:/root/.Xauthority:rw \
  -v ${WORKSPACE_DIR}:/workspace:rw \
  -w /workspace \
  nvcr.io/nvidia/isaac-sim:4.2.0 \
  /bin/bash

echo ""
echo -e "${GREEN}✅ 컨테이너 종료됨${NC}"
