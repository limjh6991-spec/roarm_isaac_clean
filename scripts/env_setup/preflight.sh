#!/bin/bash
###############################################################################
# RoArm M3 Preflight Check
# 
# 목적: 매일 작업 시작 전 환경 검증
# 사용: make preflight 또는 ./scripts/env_setup/preflight.sh
###############################################################################

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 체크 카운터
CHECKS_PASSED=0
CHECKS_FAILED=0

echo -e "${BLUE}╔═══════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║     RoArm M3 Environment Preflight Check (2025-10-20)    ║${NC}"
echo -e "${BLUE}╚═══════════════════════════════════════════════════════════╝${NC}"
echo ""

# 프로젝트 루트 찾기
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$PROJECT_ROOT"

echo -e "${BLUE}📁 Project Root: ${PROJECT_ROOT}${NC}"
echo ""

###############################################################################
# 1. 환경 변수 로드
###############################################################################
echo -e "${BLUE}[1/10] Loading environment variables...${NC}"

if [ -f ".env" ]; then
    set -a
    source .env
    set +a
    echo -e "${GREEN}✅ Loaded .env${NC}"
    ((CHECKS_PASSED++))
else
    echo -e "${RED}❌ .env file not found!${NC}"
    echo -e "${YELLOW}   Run: cp .env.example .env${NC}"
    ((CHECKS_FAILED++))
fi

if [ -f ".env.local" ]; then
    set -a
    source .env.local
    set +a
    echo -e "${GREEN}✅ Loaded .env.local${NC}"
fi

echo ""

###############################################################################
# 2. GPU 확인
###############################################################################
echo -e "${BLUE}[2/10] Checking GPU...${NC}"

if command -v nvidia-smi &> /dev/null; then
    GPU_INFO=$(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader | head -1)
    echo -e "${GREEN}✅ GPU: ${GPU_INFO}${NC}"
    
    # GPU 메모리 확인
    GPU_MEM=$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits | head -1)
    if [ "$GPU_MEM" -lt 4000 ]; then
        echo -e "${YELLOW}⚠️  GPU memory low: ${GPU_MEM}MB (recommend > 4GB)${NC}"
    fi
    ((CHECKS_PASSED++))
else
    echo -e "${RED}❌ nvidia-smi not found!${NC}"
    echo -e "${YELLOW}   Install NVIDIA drivers${NC}"
    ((CHECKS_FAILED++))
fi

echo ""

###############################################################################
# 3. Vulkan 확인
###############################################################################
echo -e "${BLUE}[3/10] Checking Vulkan...${NC}"

if command -v vulkaninfo &> /dev/null; then
    VULKAN_VERSION=$(vulkaninfo | grep "Vulkan Instance Version:" | awk '{print $4}' || echo "Unknown")
    echo -e "${GREEN}✅ Vulkan: Version ${VULKAN_VERSION}${NC}"
    ((CHECKS_PASSED++))
else
    echo -e "${YELLOW}⚠️  vulkaninfo not found (optional for headless)${NC}"
    echo -e "${YELLOW}   Install: sudo apt install vulkan-tools${NC}"
fi

echo ""

###############################################################################
# 4. Isaac Sim 경로 확인
###############################################################################
echo -e "${BLUE}[4/10] Checking Isaac Sim...${NC}"

if [ -z "$ISAAC_SIM_ROOT" ]; then
    ISAAC_SIM_ROOT="/home/roarm_m3/isaacsim"  # 기본값
fi

if [ -d "$ISAAC_SIM_ROOT" ]; then
    echo -e "${GREEN}✅ Isaac Sim: ${ISAAC_SIM_ROOT}${NC}"
    
    # Python 버전 확인
    if [ -f "$ISAAC_SIM_ROOT/python.sh" ]; then
        ISAAC_PYTHON_VERSION=$("$ISAAC_SIM_ROOT/python.sh" --version 2>&1 | awk '{print $2}')
        echo -e "${GREEN}✅ Isaac Python: ${ISAAC_PYTHON_VERSION}${NC}"
        ((CHECKS_PASSED++))
    else
        echo -e "${RED}❌ python.sh not found in Isaac Sim${NC}"
        ((CHECKS_FAILED++))
    fi
else
    echo -e "${RED}❌ Isaac Sim directory not found: ${ISAAC_SIM_ROOT}${NC}"
    echo -e "${YELLOW}   Check ISAAC_SIM_ROOT in .env${NC}"
    ((CHECKS_FAILED++))
fi

echo ""

###############################################################################
# 5. Python 환경 확인
###############################################################################
echo -e "${BLUE}[5/10] Checking Python environments...${NC}"

# 시스템 Python 3.11
if command -v python3.11 &> /dev/null; then
    PY311_VERSION=$(python3.11 --version)
    echo -e "${GREEN}✅ ${PY311_VERSION}${NC}"
else
    echo -e "${YELLOW}⚠️  Python 3.11 not found (needed for Isaac venv)${NC}"
fi

# 시스템 Python 3.12
if command -v python3.12 &> /dev/null; then
    PY312_VERSION=$(python3.12 --version)
    echo -e "${GREEN}✅ ${PY312_VERSION}${NC}"
else
    echo -e "${YELLOW}⚠️  Python 3.12 not found (needed for RL venv)${NC}"
fi

((CHECKS_PASSED++))
echo ""

###############################################################################
# 6. 필수 디렉토리 확인
###############################################################################
echo -e "${BLUE}[6/10] Checking required directories...${NC}"

REQUIRED_DIRS=(
    "assets/roarm_m3/urdf"
    "envs"
    "scripts/rl"
    "configs"
    "docs/rl"
)

ALL_DIRS_OK=true
for dir in "${REQUIRED_DIRS[@]}"; do
    if [ -d "$dir" ]; then
        echo -e "${GREEN}✅ ${dir}${NC}"
    else
        echo -e "${RED}❌ Missing: ${dir}${NC}"
        ALL_DIRS_OK=false
    fi
done

if [ "$ALL_DIRS_OK" = true ]; then
    ((CHECKS_PASSED++))
else
    ((CHECKS_FAILED++))
fi

echo ""

###############################################################################
# 7. 디스크 공간 확인
###############################################################################
echo -e "${BLUE}[7/10] Checking disk space...${NC}"

DISK_FREE=$(df -BG . | tail -1 | awk '{print $4}' | sed 's/G//')
if [ "$DISK_FREE" -gt 10 ]; then
    echo -e "${GREEN}✅ Disk space: ${DISK_FREE}GB free (sufficient)${NC}"
    ((CHECKS_PASSED++))
else
    echo -e "${YELLOW}⚠️  Disk space: ${DISK_FREE}GB free (recommend > 10GB)${NC}"
    echo -e "${YELLOW}   Clean up: make clean${NC}"
fi

echo ""

###############################################################################
# 8. 메모리 확인
###############################################################################
echo -e "${BLUE}[8/10] Checking memory...${NC}"

TOTAL_RAM=$(free -g | awk '/^Mem:/{print $2}')
FREE_RAM=$(free -g | awk '/^Mem:/{print $7}')

if [ "$TOTAL_RAM" -ge 16 ]; then
    echo -e "${GREEN}✅ RAM: ${TOTAL_RAM}GB total, ${FREE_RAM}GB available${NC}"
    ((CHECKS_PASSED++))
else
    echo -e "${YELLOW}⚠️  RAM: ${TOTAL_RAM}GB (recommend > 16GB)${NC}"
fi

echo ""

###############################################################################
# 9. Git 상태 확인
###############################################################################
echo -e "${BLUE}[9/10] Checking Git status...${NC}"

if git rev-parse --git-dir > /dev/null 2>&1; then
    BRANCH=$(git branch --show-current)
    UNCOMMITTED=$(git status --porcelain | wc -l)
    
    echo -e "${GREEN}✅ Git branch: ${BRANCH}${NC}"
    
    if [ "$UNCOMMITTED" -gt 0 ]; then
        echo -e "${YELLOW}⚠️  ${UNCOMMITTED} uncommitted changes${NC}"
    else
        echo -e "${GREEN}✅ Working tree clean${NC}"
    fi
    ((CHECKS_PASSED++))
else
    echo -e "${YELLOW}⚠️  Not a git repository${NC}"
fi

echo ""

###############################################################################
# 10. 필수 파일 확인
###############################################################################
echo -e "${BLUE}[10/10] Checking required files...${NC}"

REQUIRED_FILES=(
    "assets/roarm_m3/urdf/roarm_m3_multiprim.urdf"
    "envs/roarm_pick_place_env.py"
    "scripts/rl/train_dense_reward.py"
    "docs/rl/RL_TRAINING_PLAN_V2.md"
)

ALL_FILES_OK=true
for file in "${REQUIRED_FILES[@]}"; do
    if [ -f "$file" ]; then
        echo -e "${GREEN}✅ ${file}${NC}"
    else
        echo -e "${RED}❌ Missing: ${file}${NC}"
        ALL_FILES_OK=false
    fi
done

if [ "$ALL_FILES_OK" = true ]; then
    ((CHECKS_PASSED++))
else
    ((CHECKS_FAILED++))
fi

echo ""

###############################################################################
# 결과 요약
###############################################################################
echo -e "${BLUE}╔═══════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║                    Preflight Results                      ║${NC}"
echo -e "${BLUE}╚═══════════════════════════════════════════════════════════╝${NC}"
echo ""

TOTAL_CHECKS=$((CHECKS_PASSED + CHECKS_FAILED))
echo -e "${BLUE}Total checks: ${TOTAL_CHECKS}${NC}"
echo -e "${GREEN}Passed: ${CHECKS_PASSED} ✅${NC}"

if [ "$CHECKS_FAILED" -gt 0 ]; then
    echo -e "${RED}Failed: ${CHECKS_FAILED} ❌${NC}"
    echo ""
    echo -e "${RED}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${RED}║  ⚠️  PREFLIGHT FAILED - Fix issues before proceeding  ⚠️  ║${NC}"
    echo -e "${RED}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${YELLOW}📚 See: docs/environment/TROUBLESHOOTING.md${NC}"
    exit 1
else
    echo -e "${GREEN}Failed: 0${NC}"
    echo ""
    echo -e "${GREEN}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║        ✅ ALL CHECKS PASSED - Ready to work! ✅           ║${NC}"
    echo -e "${GREEN}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${BLUE}📋 Next steps:${NC}"
    echo -e "  ${GREEN}•${NC} Quick test:  ${YELLOW}make train-quick${NC}  (10K steps, ~15min)"
    echo -e "  ${GREEN}•${NC} Full train:  ${YELLOW}make train${NC}        (50K steps, ~1hr)"
    echo -e "  ${GREEN}•${NC} Isaac GUI:   ${YELLOW}make isaac-gui${NC}"
    echo -e "  ${GREEN}•${NC} Diagnose:    ${YELLOW}make diagnose${NC}"
    echo ""
    echo -e "${BLUE}📚 Documentation:${NC}"
    echo -e "  ${GREEN}•${NC} Training plan:  ${YELLOW}docs/rl/RL_TRAINING_PLAN_V2.md${NC}"
    echo -e "  ${GREEN}•${NC} Environment:    ${YELLOW}docs/environment/ENVIRONMENT_SETUP.md${NC}"
    echo -e "  ${GREEN}•${NC} Daily checklist: ${YELLOW}docs/environment/DAILY_CHECKLIST.md${NC}"
    echo ""
fi

exit 0
