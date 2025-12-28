#!/bin/bash
################################################################################
# VRL Debug Launcher - GPU/메모리 모니터링 및 로그 생성
#
# Usage:
#   bash scripts/debug_vrl_launch.sh [--test|--quick|--train]
#
# 모든 출력과 시스템 상태를 logs/debug/ 폴더에 기록
################################################################################

set -o pipefail

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Paths
PROJECT_DIR="/home/roarm_m3/roarm_isaac_clean"
ISAAC_PYTHON="/home/roarm_m3/isaacsim/python.sh"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
DEBUG_DIR="$PROJECT_DIR/logs/debug/$TIMESTAMP"

# Create debug directory
mkdir -p "$DEBUG_DIR"

# Log files
MAIN_LOG="$DEBUG_DIR/main_output.log"
GPU_LOG="$DEBUG_DIR/gpu_monitor.log"
MEM_LOG="$DEBUG_DIR/memory_monitor.log"
SYS_LOG="$DEBUG_DIR/system_info.log"
DMESG_LOG="$DEBUG_DIR/dmesg_before.log"
DMESG_AFTER_LOG="$DEBUG_DIR/dmesg_after.log"

echo -e "${BLUE}==========================================================${NC}"
echo -e "${BLUE}🔍 VRL Debug Launcher${NC}"
echo -e "${BLUE}==========================================================${NC}"
echo -e "Debug directory: ${GREEN}$DEBUG_DIR${NC}"
echo ""

# Capture initial system state
echo "Capturing initial system state..."

# System info
{
    echo "=== System Info ==="
    echo "Date: $(date)"
    echo "Hostname: $(hostname)"
    echo "Kernel: $(uname -r)"
    echo ""
    echo "=== CPU ==="
    lscpu | head -20
    echo ""
    echo "=== Memory ==="
    free -h
    echo ""
    echo "=== GPU ==="
    nvidia-smi
    echo ""
    echo "=== Disk ==="
    df -h $PROJECT_DIR
    echo ""
    echo "=== Isaac Sim Version ==="
    cat /home/roarm_m3/isaacsim/VERSION 2>/dev/null || echo "VERSION file not found"
    echo ""
    echo "=== Environment Variables ==="
    env | grep -E "(ISAAC|CUDA|NVIDIA|OMNI|LD_)" | sort
} > "$SYS_LOG" 2>&1

# Capture dmesg before
sudo dmesg -T | tail -100 > "$DMESG_LOG" 2>/dev/null || echo "Cannot capture dmesg (need sudo)" > "$DMESG_LOG"

# Start GPU monitoring in background
echo "Starting GPU monitor (logs every 2 seconds)..."
(
    while true; do
        echo "--- $(date '+%H:%M:%S') ---" >> "$GPU_LOG"
        nvidia-smi --query-gpu=timestamp,memory.used,memory.total,utilization.gpu,utilization.memory,temperature.gpu,power.draw --format=csv,noheader >> "$GPU_LOG" 2>&1
        sleep 2
    done
) &
GPU_MONITOR_PID=$!

# Start memory monitoring in background
echo "Starting memory monitor (logs every 2 seconds)..."
(
    while true; do
        echo "--- $(date '+%H:%M:%S') ---" >> "$MEM_LOG"
        free -h | head -2 >> "$MEM_LOG"
        echo "" >> "$MEM_LOG"
        sleep 2
    done
) &
MEM_MONITOR_PID=$!

# Cleanup function
cleanup() {
    echo ""
    echo -e "${YELLOW}Cleaning up monitors...${NC}"
    kill $GPU_MONITOR_PID 2>/dev/null
    kill $MEM_MONITOR_PID 2>/dev/null
    
    # Capture dmesg after
    sudo dmesg -T | tail -100 > "$DMESG_AFTER_LOG" 2>/dev/null || echo "Cannot capture dmesg (need sudo)" > "$DMESG_AFTER_LOG"
    
    echo ""
    echo -e "${GREEN}==========================================================${NC}"
    echo -e "${GREEN}📁 Debug logs saved to:${NC}"
    echo -e "   $DEBUG_DIR/"
    echo ""
    echo -e "   ${BLUE}main_output.log${NC} - VRL stdout/stderr"
    echo -e "   ${BLUE}gpu_monitor.log${NC} - GPU usage timeline"
    echo -e "   ${BLUE}memory_monitor.log${NC} - RAM usage timeline"
    echo -e "   ${BLUE}system_info.log${NC} - Initial system state"
    echo -e "   ${BLUE}dmesg_*.log${NC} - Kernel messages (OOM killer, GPU errors)"
    echo -e "${GREEN}==========================================================${NC}"
}
trap cleanup EXIT

# Parse mode
MODE="${1:---test}"

echo ""
echo -e "${YELLOW}Mode: $MODE${NC}"
echo -e "${YELLOW}Starting VRL with full logging...${NC}"
echo ""

# Check Isaac Sim
if [ ! -f "$ISAAC_PYTHON" ]; then
    echo -e "${RED}❌ Error: Isaac Sim not found at $ISAAC_PYTHON${NC}" | tee -a "$MAIN_LOG"
    exit 1
fi

# Run VRL with output capture
cd "$PROJECT_DIR"

case "$MODE" in
    --test)
        echo "Running: $ISAAC_PYTHON scripts/test/test_vision_env.py"
        echo "Command: $ISAAC_PYTHON scripts/test/test_vision_env.py" >> "$MAIN_LOG"
        echo "Started at: $(date)" >> "$MAIN_LOG"
        echo "---" >> "$MAIN_LOG"
        
        $ISAAC_PYTHON scripts/test/test_vision_env.py 2>&1 | tee -a "$MAIN_LOG"
        EXIT_CODE=${PIPESTATUS[0]}
        ;;
    
    --quick)
        echo "Running: $ISAAC_PYTHON scripts/train/train_vision_sac.py --steps 50000"
        echo "Command: $ISAAC_PYTHON scripts/train/train_vision_sac.py --steps 50000" >> "$MAIN_LOG"
        echo "Started at: $(date)" >> "$MAIN_LOG"
        echo "---" >> "$MAIN_LOG"
        
        $ISAAC_PYTHON scripts/train/train_vision_sac.py --steps 50000 2>&1 | tee -a "$MAIN_LOG"
        EXIT_CODE=${PIPESTATUS[0]}
        ;;
    
    --train)
        echo "Running: $ISAAC_PYTHON scripts/train/train_vision_sac.py"
        echo "Command: $ISAAC_PYTHON scripts/train/train_vision_sac.py" >> "$MAIN_LOG"
        echo "Started at: $(date)" >> "$MAIN_LOG"
        echo "---" >> "$MAIN_LOG"
        
        $ISAAC_PYTHON scripts/train/train_vision_sac.py 2>&1 | tee -a "$MAIN_LOG"
        EXIT_CODE=${PIPESTATUS[0]}
        ;;
        
    --headless)
        echo "Running: DISPLAY= $ISAAC_PYTHON scripts/train/train_vision_sac.py (headless)"
        echo "Command: DISPLAY= $ISAAC_PYTHON scripts/train/train_vision_sac.py (headless)" >> "$MAIN_LOG"
        echo "Started at: $(date)" >> "$MAIN_LOG"
        echo "---" >> "$MAIN_LOG"
        
        # Force headless by unsetting DISPLAY
        DISPLAY= $ISAAC_PYTHON scripts/train/train_vision_sac.py 2>&1 | tee -a "$MAIN_LOG"
        EXIT_CODE=${PIPESTATUS[0]}
        ;;
    
    *)
        echo -e "${RED}Unknown mode: $MODE${NC}"
        echo "Usage: bash scripts/debug_vrl_launch.sh [--test|--quick|--train|--headless]"
        exit 1
        ;;
esac

echo "" >> "$MAIN_LOG"
echo "---" >> "$MAIN_LOG"
echo "Finished at: $(date)" >> "$MAIN_LOG"
echo "Exit code: $EXIT_CODE" >> "$MAIN_LOG"

echo ""
echo -e "Exit code: $EXIT_CODE"

if [ $EXIT_CODE -ne 0 ]; then
    echo -e "${RED}❌ VRL exited with error code $EXIT_CODE${NC}"
else
    echo -e "${GREEN}✅ VRL completed successfully${NC}"
fi
