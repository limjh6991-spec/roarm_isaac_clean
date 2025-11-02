#!/bin/bash
################################################################################
# Vision RL Training - Quick Start Guide
# 
# 이 스크립트는 Vision RL 학습을 단계별로 안내합니다.
# 
# 날짜: 2025-11-02
# 작성자: Jarvis AI Assistant
################################################################################

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

clear

echo -e "${BLUE}"
cat << "EOF"
╔══════════════════════════════════════════════════════════════════╗
║                                                                  ║
║           🤖 Vision RL Training - Quick Start Guide              ║
║                                                                  ║
║                    RoArm-M3 + Intel D405                         ║
║                                                                  ║
╚══════════════════════════════════════════════════════════════════╝
EOF
echo -e "${NC}"

echo ""
echo -e "${CYAN}📋 오늘의 작업 계획${NC}"
echo ""
echo -e "${YELLOW}Phase 1: 환경 검증 (필수)${NC}"
echo "  ├─ Pre-flight check 실행"
echo "  ├─ Environment test (10 episodes)"
echo "  └─ 예상 시간: 5분"
echo ""
echo -e "${YELLOW}Phase 2: Quick Training (권장)${NC}"
echo "  ├─ SAC 50K steps 학습"
echo "  ├─ 파이프라인 검증"
echo "  └─ 예상 시간: 30분"
echo ""
echo -e "${YELLOW}Phase 3: Full Training (선택)${NC}"
echo "  ├─ SAC 500K steps 학습"
echo "  ├─ Pick & Place 마스터"
echo "  └─ 예상 시간: 5-10시간"
echo ""

# Wait for user
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
read -p "계속하려면 Enter를 누르세요..."
clear

################################################################################
# Step 1: Environment Check
################################################################################

echo -e "${BLUE}"
echo "╔══════════════════════════════════════════════════════════════════╗"
echo "║  Step 1: Environment Check                                       ║"
echo "╚══════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo ""

# Check Isaac Sim
ISAAC_PYTHON="/home/roarm_m3/isaacsim/python.sh"
if [ ! -f "$ISAAC_PYTHON" ]; then
    echo -e "${RED}❌ Isaac Sim not found: $ISAAC_PYTHON${NC}"
    exit 1
fi
echo -e "${GREEN}✅ Isaac Sim found${NC}"

# Check project directory
PROJECT_DIR="/home/roarm_m3/roarm_isaac_clean"
if [ ! -d "$PROJECT_DIR" ]; then
    echo -e "${RED}❌ Project directory not found: $PROJECT_DIR${NC}"
    exit 1
fi
echo -e "${GREEN}✅ Project directory found${NC}"

# Check key files
echo ""
echo -e "${CYAN}Checking key files...${NC}"
FILES=(
    "envs/simple_vision_env.py"
    "models/cnn_extractor.py"
    "scripts/train/train_vision_sac.py"
    "scripts/launch_vision_rl.sh"
)

for file in "${FILES[@]}"; do
    if [ -f "$PROJECT_DIR/$file" ]; then
        echo -e "  ${GREEN}✅${NC} $file"
    else
        echo -e "  ${RED}❌${NC} $file ${RED}(MISSING!)${NC}"
        exit 1
    fi
done

echo ""
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
read -p "환경 체크 완료! Enter를 눌러 계속..."
clear

################################################################################
# Step 2: Pre-flight Check
################################################################################

echo -e "${BLUE}"
echo "╔══════════════════════════════════════════════════════════════════╗"
echo "║  Step 2: Pre-flight Check (필수!)                                ║"
echo "╚══════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo ""
echo -e "${YELLOW}다음 테스트를 실행합니다:${NC}"
echo "  1. Environment 초기화"
echo "  2. Observation/Action space 검증"
echo "  3. CNN extractor 테스트"
echo "  4. gym.check_env 실행"
echo ""
echo -e "${CYAN}💡 Tip: 첫 실행 시 시간이 걸릴 수 있습니다.${NC}"
echo ""

read -p "Pre-flight check를 실행하시겠습니까? [Y/n] " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Nn]$ ]]; then
    cd "$PROJECT_DIR"
    $ISAAC_PYTHON scripts/test/preflight_vision_rl.py
    
    if [ $? -eq 0 ]; then
        echo ""
        echo -e "${GREEN}✅ Pre-flight check 통과!${NC}"
    else
        echo ""
        echo -e "${RED}❌ Pre-flight check 실패!${NC}"
        echo -e "${RED}   로그를 확인하고 문제를 해결해주세요.${NC}"
        exit 1
    fi
else
    echo -e "${YELLOW}⚠️  Pre-flight check를 건너뛰었습니다.${NC}"
fi

echo ""
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
read -p "Enter를 눌러 계속..."
clear

################################################################################
# Step 3: Choose Training Mode
################################################################################

echo -e "${BLUE}"
echo "╔══════════════════════════════════════════════════════════════════╗"
echo "║  Step 3: Training Mode Selection                                 ║"
echo "╚══════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo ""
echo -e "${CYAN}어떤 모드로 실행하시겠습니까?${NC}"
echo ""
echo -e "${YELLOW}1)${NC} Test Mode (환경 테스트만)"
echo "   ├─ 10 episodes random policy"
echo "   ├─ RGB-D 이미지 저장"
echo "   └─ 시간: ~2분"
echo ""
echo -e "${YELLOW}2)${NC} Quick Training (추천!)"
echo "   ├─ SAC 50K steps"
echo "   ├─ 파이프라인 검증용"
echo "   └─ 시간: ~30분"
echo ""
echo -e "${YELLOW}3)${NC} Full Training"
echo "   ├─ SAC 500K steps"
echo "   ├─ 본격 학습"
echo "   └─ 시간: ~5-10시간"
echo ""
echo -e "${YELLOW}4)${NC} Exit"
echo ""

read -p "선택 [1-4]: " choice

case $choice in
    1)
        echo ""
        echo -e "${GREEN}🧪 Test Mode 시작...${NC}"
        echo ""
        cd "$PROJECT_DIR"
        bash scripts/launch_vision_rl.sh --test
        ;;
    2)
        echo ""
        echo -e "${GREEN}🚀 Quick Training 시작...${NC}"
        echo ""
        echo -e "${CYAN}💡 TensorBoard 실행 (다른 터미널):${NC}"
        echo -e "   cd $PROJECT_DIR"
        echo -e "   tensorboard --logdir output/train_vision_sac/"
        echo ""
        read -p "Enter를 눌러 시작..."
        
        cd "$PROJECT_DIR"
        bash scripts/launch_vision_rl.sh --quick
        ;;
    3)
        echo ""
        echo -e "${GREEN}🚀 Full Training 시작...${NC}"
        echo ""
        echo -e "${YELLOW}⚠️  경고: 5-10시간이 소요됩니다!${NC}"
        echo ""
        echo -e "${CYAN}💡 TensorBoard 실행 (다른 터미널):${NC}"
        echo -e "   cd $PROJECT_DIR"
        echo -e "   tensorboard --logdir output/train_vision_sac/"
        echo ""
        read -p "정말 시작하시겠습니까? [y/N] " -n 1 -r
        echo
        
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            cd "$PROJECT_DIR"
            bash scripts/launch_vision_rl.sh --train
        else
            echo -e "${YELLOW}취소되었습니다.${NC}"
            exit 0
        fi
        ;;
    4)
        echo ""
        echo -e "${YELLOW}종료합니다.${NC}"
        exit 0
        ;;
    *)
        echo ""
        echo -e "${RED}잘못된 선택입니다.${NC}"
        exit 1
        ;;
esac

################################################################################
# Completion
################################################################################

echo ""
echo -e "${GREEN}"
cat << "EOF"
╔══════════════════════════════════════════════════════════════════╗
║                                                                  ║
║                    ✅ 작업이 완료되었습니다!                     ║
║                                                                  ║
╚══════════════════════════════════════════════════════════════════╝
EOF
echo -e "${NC}"

echo ""
echo -e "${CYAN}📊 결과 확인:${NC}"
echo "  ├─ 로그: output/train_vision_sac/"
echo "  ├─ 체크포인트: output/train_vision_sac/.../checkpoints/"
echo "  └─ Best model: output/train_vision_sac/.../best_model/"
echo ""
echo -e "${CYAN}📚 문서:${NC}"
echo "  └─ VISION_RL_STATUS.md"
echo ""
