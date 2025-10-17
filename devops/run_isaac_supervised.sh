#!/usr/bin/env bash
# ==============================================================================
# Isaac Sim Supervised Runner
# 프로세스 격리 + 강제 로그화 + 코어덤프 수집
# ==============================================================================

set -Eeuo pipefail

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 프로젝트 루트
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

# 타임스탬프 생성
TIMESTAMP=$(date +'%Y%m%d_%H%M%S')

# 로그 디렉토리
LOG_DIR="logs/isaac"
CORE_DIR="logs/core"
mkdir -p "$LOG_DIR" "$CORE_DIR"

# 로그 파일
LOG_FILE="$LOG_DIR/isaac_${TIMESTAMP}.log"
ENV_SNAPSHOT="$LOG_DIR/env_${TIMESTAMP}.txt"

echo -e "${GREEN}========================================================================"
echo "       Isaac Sim Supervised Runner"
echo "========================================================================${NC}"
echo ""
echo "Timestamp: $TIMESTAMP"
echo "Log File:  $LOG_FILE"
echo "Core Dump: $CORE_DIR (if crash occurs)"
echo ""

# ------------------------------------------------------------------------------
# 1. Preflight Checks
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[Preflight] Running system checks...${NC}"
if ! bash devops/preflight/check_system.sh; then
    echo -e "${RED}Preflight failed! Aborting.${NC}"
    exit 1
fi
echo ""

# ------------------------------------------------------------------------------
# 2. 환경 스냅샷 저장
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[Snapshot] Saving environment snapshot...${NC}"
{
    echo "=== Environment Snapshot ==="
    echo "Timestamp: $(date)"
    echo "User: $USER"
    echo "Hostname: $(hostname)"
    echo "PWD: $PWD"
    echo ""
    echo "=== Python ==="
    which python
    python --version
    echo ""
    echo "=== pip freeze ==="
    pip freeze
    echo ""
    echo "=== Environment Variables ==="
    env | sort
} > "$ENV_SNAPSHOT"
echo "  Saved to: $ENV_SNAPSHOT"
echo ""

# ------------------------------------------------------------------------------
# 3. 코어덤프 설정
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[Core Dump] Enabling core dumps...${NC}"
ulimit -c unlimited
CORE_PATTERN="/tmp/core.isaac.%e.%p.%t"
echo "  Core pattern: $CORE_PATTERN"
# Note: Requires root to change /proc/sys/kernel/core_pattern
# sudo sysctl -w kernel.core_pattern="$CORE_DIR/core.%e.%p.%t"
echo ""

# ------------------------------------------------------------------------------
# 4. Isaac Sim 스크립트 실행 (인자 전달)
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[Execution] Starting Isaac Sim script...${NC}"
echo "  Script: $*"
echo "  Log: $LOG_FILE"
echo ""

# stdbuf: 버퍼링 비활성화로 실시간 로그 출력
# tee: 터미널 출력 + 파일 저장
if stdbuf -oL -eL python "$@" 2>&1 | tee -a "$LOG_FILE"; then
    EXIT_CODE=0
    echo ""
    echo -e "${GREEN}========================================================================"
    echo "       Isaac Sim Execution SUCCESSFUL"
    echo "========================================================================${NC}"
else
    EXIT_CODE=$?
    echo ""
    echo -e "${RED}========================================================================"
    echo "       Isaac Sim Execution FAILED (Exit Code: $EXIT_CODE)"
    echo "========================================================================${NC}"
    echo ""
    echo -e "${YELLOW}Log saved to: $LOG_FILE${NC}"
    echo -e "${YELLOW}Check for core dumps in: $CORE_DIR${NC}"
fi

echo ""
echo "Log file: $LOG_FILE"
echo "Size: $(du -h "$LOG_FILE" | cut -f1)"
echo ""

exit $EXIT_CODE
