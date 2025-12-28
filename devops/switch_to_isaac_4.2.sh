#!/bin/bash
# Isaac Sim 4.2.0 다운로드 및 설치 스크립트

set -e

echo "=========================================="
echo "Isaac Sim 4.2.0 LTS 설치 스크립트"
echo "=========================================="
echo ""

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 설치 디렉토리
INSTALL_DIR="$HOME"
ISAAC_SIM_VERSION="4.2.0"
ISAAC_SIM_DIR="${INSTALL_DIR}/isaac-sim-${ISAAC_SIM_VERSION}"
BACKUP_DIR="${HOME}/isaacsim_5.0_rc_backup"

echo -e "${YELLOW}1. 현재 Isaac Sim 5.0 RC 백업...${NC}"
if [ -L "${HOME}/isaacsim" ]; then
    echo "심볼릭 링크 감지"
    TARGET=$(readlink -f "${HOME}/isaacsim")
    echo "현재 링크 대상: ${TARGET}"
    rm "${HOME}/isaacsim"
    echo "기존 링크 제거됨"
elif [ -d "${HOME}/isaacsim" ]; then
    if [ ! -d "${BACKUP_DIR}" ]; then
        echo "백업 생성: ${BACKUP_DIR}"
        mv "${HOME}/isaacsim" "${BACKUP_DIR}"
    else
        echo -e "${RED}경고: 백업이 이미 존재합니다. 건너뜁니다.${NC}"
    fi
fi

echo ""
echo -e "${YELLOW}2. Isaac Sim 4.2.0 다운로드 확인...${NC}"
echo ""
echo "⚠️  Isaac Sim 4.2.0은 수동으로 다운로드해야 합니다:"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📥 다운로드 방법 (3가지 옵션):"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "▶ 옵션 1: NVIDIA NGC (권장)"
echo "   URL: https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim"
echo "   버전: 4.2.0"
echo ""
echo "▶ 옵션 2: Omniverse Launcher"
echo "   1. Launcher 설치: https://www.nvidia.com/en-us/omniverse/download/"
echo "   2. Exchange > Isaac Sim 4.2.0 선택"
echo "   3. 다운로드 및 설치"
echo ""
echo "▶ 옵션 3: Docker (가장 빠름)"
echo "   $ docker pull nvcr.io/nvidia/isaac-sim:4.2.0"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Docker 옵션 제안
echo -e "${GREEN}🐳 Docker를 사용하면 가장 빠르고 안전합니다!${NC}"
echo ""
read -p "Docker 사용 설정 파일을 생성하시겠습니까? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    cat > "${HOME}/roarm_isaac_clean/docker-compose.isaac-sim-4.2.yml" << 'DOCKER_EOF'
version: '3.8'

services:
  isaac-sim:
    image: nvcr.io/nvidia/isaac-sim:4.2.0
    container_name: isaac-sim-4.2
    runtime: nvidia
    environment:
      - DISPLAY=${DISPLAY}
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
    volumes:
      - ./:/workspace
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ~/.Xauthority:/root/.Xauthority:rw
    working_dir: /workspace
    stdin_open: true
    tty: true
    network_mode: host
    command: /bin/bash
DOCKER_EOF

    echo -e "${GREEN}✅ Docker Compose 파일 생성됨: ~/roarm_isaac_clean/docker-compose.isaac-sim-4.2.yml${NC}"
    echo ""
    echo "Docker 사용 방법:"
    echo "  1. cd ~/roarm_isaac_clean"
    echo "  2. docker-compose -f docker-compose.isaac-sim-4.2.yml pull"
    echo "  3. docker-compose -f docker-compose.isaac-sim-4.2.yml run --rm isaac-sim"
    echo "  4. 컨테이너 내에서: python scripts/test/test_vision_env.py --headless"
fi

echo ""
echo -e "${YELLOW}3. Omniverse에서 설치된 버전 확인...${NC}"
OV_PKG_DIR="${HOME}/.local/share/ov/pkg"
if [ -d "${OV_PKG_DIR}" ]; then
    echo "Omniverse 패키지 디렉토리 발견:"
    ls -d ${OV_PKG_DIR}/isaac* 2>/dev/null || echo "  Isaac Sim 패키지 없음"
    echo ""
    
    # 4.2.0 버전 찾기
    ISAAC_42_DIRS=$(find ${OV_PKG_DIR} -maxdepth 1 -type d -name "isaac*4.2*" -o -name "isaac*2024.1*")
    if [ ! -z "$ISAAC_42_DIRS" ]; then
        echo -e "${GREEN}✅ Isaac Sim 4.2.0 발견!${NC}"
        echo "$ISAAC_42_DIRS"
        
        for dir in $ISAAC_42_DIRS; do
            if [ -f "${dir}/python.sh" ]; then
                echo ""
                read -p "이 버전을 사용하시겠습니까? ${dir} (y/n): " -n 1 -r
                echo
                if [[ $REPLY =~ ^[Yy]$ ]]; then
                    ln -s "${dir}" "${HOME}/isaacsim"
                    echo -e "${GREEN}✅ 심볼릭 링크 생성됨: ~/isaacsim -> ${dir}${NC}"
                    
                    # 버전 확인
                    if [ -f "${HOME}/isaacsim/VERSION" ]; then
                        echo ""
                        echo "설치된 버전:"
                        cat "${HOME}/isaacsim/VERSION"
                    fi
                    
                    echo ""
                    echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
                    echo -e "${GREEN}✅ Isaac Sim 4.2.0 전환 완료!${NC}"
                    echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
                    echo ""
                    echo "다음 명령으로 테스트하세요:"
                    echo "  ~/isaacsim/python.sh --version"
                    echo "  ~/isaacsim/python.sh scripts/test/test_isaac_basic.py --headless"
                    exit 0
                fi
            fi
        done
    fi
fi

echo ""
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${YELLOW}⚠️  수동 설치 필요${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo "Isaac Sim 4.2.0이 설치되지 않았습니다."
echo ""
echo "권장 설치 방법:"
echo ""
echo "🔹 방법 1: Omniverse Launcher 사용"
echo "   1. https://www.nvidia.com/en-us/omniverse/download/ 접속"
echo "   2. Launcher 다운로드 및 설치"
echo "   3. Exchange > Isaac Sim > 버전 4.2.0 선택"
echo "   4. 설치 후 이 스크립트 다시 실행"
echo ""
echo "🔹 방법 2: Docker 사용 (가장 빠름)"
echo "   $ cd ~/roarm_isaac_clean"
echo "   $ docker-compose -f docker-compose.isaac-sim-4.2.yml pull"
echo "   $ docker-compose -f docker-compose.isaac-sim-4.2.yml run --rm isaac-sim"
echo ""
echo "🔹 방법 3: 직접 다운로드 (고급 사용자)"
echo "   NGC에서 Isaac Sim 4.2.0 tarball 다운로드"
echo "   압축 해제 및 심볼릭 링크 생성"
echo ""

echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo "자세한 내용은 다음 문서를 참조하세요:"
echo "  ~/roarm_isaac_clean/ISAAC_SIM_VERSION_SWITCH.md"
echo ""
