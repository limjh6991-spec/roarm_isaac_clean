#!/usr/bin/env bash
# ==============================================================================
# System Preflight Check for Isaac Sim
# 실행 전 필수 환경 검사 - GPU, Driver, Vulkan, Python
# ==============================================================================

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

PASS=0
FAIL=0

echo "========================================================================"
echo "          Isaac Sim System Preflight Check"
echo "========================================================================"
echo "" || true

# ------------------------------------------------------------------------------
# 1. GPU & Driver 검사
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[1/5] GPU & Driver Check${NC}"
if command -v nvidia-smi &> /dev/null; then
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
    DRIVER_VERSION=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)
    echo "Driver Version: $DRIVER_VERSION"
    
    # Isaac Sim 5.0 requires Driver >= 550.x
    MAJOR_VERSION=$(echo "$DRIVER_VERSION" | cut -d'.' -f1)
    if [ "$MAJOR_VERSION" -ge 550 ]; then
        echo -e "${GREEN}✓ Driver version OK (>= 550)${NC}"
        ((PASS++))
    else
        echo -e "${RED}✗ Driver version too old! Requires >= 550, found $DRIVER_VERSION${NC}"
        ((FAIL++))
    fi
else
    echo -e "${RED}✗ nvidia-smi not found! NVIDIA driver not installed${NC}"
    ((FAIL++))
fi
echo "" || true

# ------------------------------------------------------------------------------
# 2. CUDA 검사
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[2/5] CUDA Check${NC}"
if command -v nvcc &> /dev/null; then
    nvcc --version | grep "release"
    echo -e "${GREEN}✓ CUDA toolkit found${NC}"
    ((PASS++))
else
    echo -e "${YELLOW}⚠ nvcc not in PATH (optional for runtime)${NC}"
    # nvcc가 없어도 통과 (runtime만 필요)
fi
echo "" || true

# ------------------------------------------------------------------------------
# 3. Vulkan 검사
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[3/5] Vulkan Check${NC}"
if command -v vulkaninfo &> /dev/null; then
    echo "Vulkan Instance Version:"
    vulkaninfo | grep -i "instance version" | head -1 || echo "N/A"
    echo "GPU Devices:"
    vulkaninfo | grep -i "GPU id" | head -3 || echo "N/A"
    echo -e "${GREEN}✓ Vulkan available${NC}"
    ((PASS++))
else
    echo -e "${RED}✗ vulkaninfo not found! Install vulkan-utils${NC}"
    ((FAIL++))
fi
echo "" || true

# ------------------------------------------------------------------------------
# 4. Python 환경 검사
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[4/5] Python Environment Check${NC}"
if [ -n "${VIRTUAL_ENV:-}" ]; then
    echo "Virtual Environment: $VIRTUAL_ENV"
else
    echo -e "${YELLOW}⚠ Not in a virtual environment${NC}"
fi

PYTHON_PATH=$(which python)
PYTHON_VERSION=$(python --version 2>&1)
echo "Python Path: $PYTHON_PATH"
echo "Python Version: $PYTHON_VERSION"

# Isaac Sim 5.0 prefers Python 3.10 or 3.11
if [[ "$PYTHON_VERSION" =~ "Python 3.1"[01] ]]; then
    echo -e "${GREEN}✓ Python version compatible (3.10 or 3.11)${NC}"
    ((PASS++))
else
    echo -e "${YELLOW}⚠ Python version may be incompatible. Recommended: 3.10 or 3.11${NC}"
fi
echo "" || true

# ------------------------------------------------------------------------------
# 5. 환경변수 검사
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[5/5] Environment Variables Check${NC}"
echo "LD_LIBRARY_PATH: ${LD_LIBRARY_PATH:-<not set>}"
echo "CUDA_VISIBLE_DEVICES: ${CUDA_VISIBLE_DEVICES:-<not set>}"
echo "VK_ICD_FILENAMES: ${VK_ICD_FILENAMES:-<not set>}"
echo "DISPLAY: ${DISPLAY:-<not set>}"

if [ -z "${CUDA_VISIBLE_DEVICES:-}" ]; then
    echo -e "${GREEN}✓ CUDA_VISIBLE_DEVICES not set (good - avoid conflicts)${NC}"
    ((PASS++))
else
    echo -e "${YELLOW}⚠ CUDA_VISIBLE_DEVICES is set: $CUDA_VISIBLE_DEVICES${NC}"
    echo -e "${YELLOW}  This may cause device enumeration issues!${NC}"
fi
echo "" || true

# ------------------------------------------------------------------------------
# Summary
# ------------------------------------------------------------------------------
echo "========================================================================"
echo -e "PREFLIGHT SUMMARY: ${GREEN}PASS=$PASS${NC} ${RED}FAIL=$FAIL${NC}"
echo "========================================================================"

if [ $FAIL -gt 0 ]; then
    echo -e "${RED}PREFLIGHT FAILED!${NC} Fix errors above before running Isaac Sim."
    exit 1
else
    echo -e "${GREEN}PREFLIGHT PASSED!${NC} System ready for Isaac Sim."
    exit 0
fi
