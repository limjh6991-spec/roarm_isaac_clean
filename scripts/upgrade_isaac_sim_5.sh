#!/bin/bash

# Isaac Sim 5.x 업그레이드 스크립트
# NGC 또는 로컬 빌드 방식

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║            Isaac Sim 5.x 업그레이드 옵션                   ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

echo -e "${YELLOW}선택 가능한 설치 방법:${NC}"
echo ""
echo "1. Omniverse Launcher로 Isaac Sim 5.x 설치 (권장)"
echo "2. NGC에서 Docker 이미지 다운로드 (NGC 계정 필요)"
echo "3. 기존 Isaac Sim 5.0 RC 사용 (백업에서 복원)"
echo "4. Isaac Sim pip 패키지 설치 (실험적)"
echo ""
read -p "선택하세요 (1-4): " choice

case $choice in
    1)
        echo -e "${BLUE}옵션 1: Omniverse Launcher 설치${NC}"
        echo ""
        
        # Launcher 다운로드
        if [ ! -f "omniverse-launcher-linux.AppImage" ]; then
            echo -e "${YELLOW}Omniverse Launcher 다운로드 중...${NC}"
            wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage
            chmod +x omniverse-launcher-linux.AppImage
        fi
        
        echo -e "${GREEN}✅ Launcher 다운로드 완료${NC}"
        echo ""
        echo -e "${YELLOW}다음 단계:${NC}"
        echo "1. ./omniverse-launcher-linux.AppImage 실행"
        echo "2. NVIDIA 계정으로 로그인"
        echo "3. Exchange 탭에서 'Isaac Sim' 검색"
        echo "4. 버전 5.0 이상 선택하여 설치"
        echo "5. 설치 후: ln -s ~/.local/share/ov/pkg/isaac-sim-5.x.x ~/isaacsim"
        echo ""
        
        read -p "지금 Launcher를 실행하시겠습니까? (y/N): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            ./omniverse-launcher-linux.AppImage
        fi
        ;;
        
    2)
        echo -e "${BLUE}옵션 2: NGC Docker 이미지${NC}"
        echo ""
        echo -e "${YELLOW}NGC 계정이 필요합니다: https://ngc.nvidia.com${NC}"
        echo ""
        
        read -p "NGC API Key를 입력하세요: " NGC_API_KEY
        
        if [ -z "$NGC_API_KEY" ]; then
            echo -e "${RED}❌ API Key가 필요합니다.${NC}"
            exit 1
        fi
        
        # NGC 로그인
        echo "$NGC_API_KEY" | docker login nvcr.io -u '$oauthtoken' --password-stdin
        
        # Isaac Sim 다운로드
        echo -e "${YELLOW}Isaac Sim Docker 이미지 다운로드 중...${NC}"
        docker pull nvcr.io/nvidia/isaac-sim:4.2.0
        
        echo -e "${GREEN}✅ Docker 이미지 다운로드 완료${NC}"
        echo ""
        echo "실행 방법:"
        echo "  bash scripts/run_isaac_lab_docker.sh"
        ;;
        
    3)
        echo -e "${BLUE}옵션 3: 백업된 Isaac Sim 5.0 RC 복원${NC}"
        echo ""
        
        if [ ! -d "$HOME/isaacsim_5.0_rc_backup" ]; then
            echo -e "${RED}❌ 백업 폴더를 찾을 수 없습니다.${NC}"
            exit 1
        fi
        
        # 기존 심볼릭 링크 제거
        if [ -L "$HOME/isaacsim" ]; then
            rm "$HOME/isaacsim"
            echo -e "${GREEN}✅ 기존 링크 제거${NC}"
        fi
        
        # 5.0 RC로 링크
        ln -s "$HOME/isaacsim_5.0_rc_backup" "$HOME/isaacsim"
        
        echo -e "${GREEN}✅ Isaac Sim 5.0 RC로 복원 완료${NC}"
        echo ""
        echo -e "${YELLOW}⚠️  주의: 이것은 RC 버전으로 Warp CUDA 이슈가 있을 수 있습니다.${NC}"
        echo ""
        
        # 버전 확인
        cat "$HOME/isaacsim/VERSION"
        ;;
        
    4)
        echo -e "${BLUE}옵션 4: Isaac Sim pip 패키지 (실험적)${NC}"
        echo ""
        echo -e "${YELLOW}⚠️  이 방법은 아직 실험적이며 제한적입니다.${NC}"
        echo ""
        
        # venv 활성화
        source ~/isaacsim-venv/bin/activate
        
        # isaacsim 패키지 설치
        pip install isaacsim --upgrade
        
        echo -e "${GREEN}✅ isaacsim 패키지 설치 완료${NC}"
        echo ""
        echo "이 방법은 전체 기능을 제공하지 않을 수 있습니다."
        echo "프로덕션 환경에는 옵션 1 또는 2를 권장합니다."
        ;;
        
    *)
        echo -e "${RED}❌ 잘못된 선택${NC}"
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║                   설치 프로세스 완료                        ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}"
