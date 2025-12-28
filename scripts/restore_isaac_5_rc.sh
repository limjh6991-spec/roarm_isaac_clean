#!/bin/bash

# Isaac Sim 5.0 RC 복원 및 Warp 문제 해결 스크립트

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║         Isaac Sim 5.0 RC 복원 + Warp 문제 해결            ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

# 1. 백업 확인
echo -e "${YELLOW}[1/5] 백업 폴더 확인...${NC}"
if [ ! -d "$HOME/isaacsim_5.0_rc_backup" ]; then
    echo -e "${RED}❌ 백업 폴더를 찾을 수 없습니다.${NC}"
    exit 1
fi
echo -e "${GREEN}✅ Isaac Sim 5.0.0-rc.45 발견${NC}"
echo ""

# 2. 기존 심볼릭 링크 백업
echo -e "${YELLOW}[2/5] 기존 설정 백업...${NC}"
if [ -L "$HOME/isaacsim" ]; then
    LINK_TARGET=$(readlink -f "$HOME/isaacsim")
    echo -e "${BLUE}현재 링크: $LINK_TARGET${NC}"
    rm "$HOME/isaacsim"
    echo -e "${GREEN}✅ 기존 링크 제거${NC}"
elif [ -d "$HOME/isaacsim" ]; then
    mv "$HOME/isaacsim" "$HOME/isaacsim_old_$(date +%Y%m%d_%H%M%S)"
    echo -e "${GREEN}✅ 기존 디렉토리 백업${NC}"
fi
echo ""

# 3. Isaac Sim 5.0 RC로 링크 생성
echo -e "${YELLOW}[3/5] Isaac Sim 5.0 RC 활성화...${NC}"
ln -s "$HOME/isaacsim_5.0_rc_backup" "$HOME/isaacsim"
echo -e "${GREEN}✅ 심볼릭 링크 생성 완료${NC}"
echo ""

# 4. Warp 재설치 (CUDA 12.6/드라이버 580 호환)
echo -e "${YELLOW}[4/5] Warp CUDA 문제 해결...${NC}"
echo -e "${BLUE}Python venv에서 Warp 업데이트 중...${NC}"

source ~/isaacsim-venv/bin/activate

# 현재 Warp 버전 확인
pip show warp-lang 2>/dev/null || echo "Warp not installed in venv"

# Warp 최신 버전으로 업그레이드 (CUDA 12.x 지원)
pip install --upgrade warp-lang

echo -e "${GREEN}✅ Warp 업그레이드 완료${NC}"
echo ""

# 5. Isaac Sim Python에서도 Warp 업데이트
echo -e "${YELLOW}[5/5] Isaac Sim 내부 Warp 업데이트...${NC}"

# Isaac Sim의 Python으로 Warp 설치
~/isaacsim/python.sh -m pip install --upgrade warp-lang

echo -e "${GREEN}✅ Isaac Sim Warp 업데이트 완료${NC}"
echo ""

# 버전 정보 출력
echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║                     설치 완료                               ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

echo -e "${GREEN}📦 Isaac Sim 버전:${NC}"
cat ~/isaacsim/VERSION
echo ""

echo -e "${GREEN}🐍 Warp 버전 (venv):${NC}"
source ~/isaacsim-venv/bin/activate
python -c "import warp as wp; print(f'  Warp: {wp.__version__}')" 2>/dev/null || echo "  설치 확인 필요"
echo ""

echo -e "${GREEN}🐍 Warp 버전 (Isaac Sim):${NC}"
~/isaacsim/python.sh -c "import warp as wp; print(f'  Warp: {wp.__version__}')" 2>/dev/null || echo "  설치 확인 필요"
echo ""

# 테스트 스크립트
echo -e "${YELLOW}🧪 다음 명령어로 테스트하세요:${NC}"
echo ""
echo "  # Warp CUDA 테스트"
echo "  ~/isaacsim/python.sh -c 'import warp as wp; wp.init(); print(wp.get_device())'"
echo ""
echo "  # Isaac Sim 부팅 테스트"
echo "  ~/isaacsim/python.sh -c 'from isaacsim import SimulationApp; app = SimulationApp({\"headless\": True}); print(\"Success!\"); app.close()'"
echo ""
echo "  # Vision RL 환경 테스트"
echo "  source ~/isaacsim-venv/bin/activate && python envs/simple_vision_env.py"
echo ""

echo -e "${GREEN}✅ 모든 설정이 완료되었습니다!${NC}"
