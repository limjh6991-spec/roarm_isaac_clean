#!/usr/bin/env bash
# ==============================================================================
# Isaac Sim Python Environment Diagnostic
# pxr 모듈 환경 확인 및 진단
# ==============================================================================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${GREEN}========================================================================"
echo "       Isaac Sim Python Environment Diagnostic"
echo "========================================================================${NC}"
echo ""

# ------------------------------------------------------------------------------
# 1. Current Python Environment
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[1/4] Current Python Environment${NC}"
echo "Python Version: $(python --version 2>&1)"
echo "Python Executable: $(which python)"
echo "Virtual Environment: ${VIRTUAL_ENV:-<not in venv>}"
echo ""

# ------------------------------------------------------------------------------
# 2. Check pxr in Current Environment
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[2/4] pxr Module Check in Current Environment${NC}"
if python -c "import pxr; print('✓ pxr found:', pxr.__file__)" 2>/dev/null; then
    echo -e "${GREEN}✓ pxr is available in current environment${NC}"
else
    echo -e "${RED}✗ pxr NOT available in current environment${NC}"
    echo "  This is EXPECTED for pip-installed Isaac Sim."
fi
echo ""

# ------------------------------------------------------------------------------
# 3. Find Isaac Sim pxr Module
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[3/4] Locating Isaac Sim pxr Module${NC}"

ISAAC_VENV="$HOME/isaacsim-venv"
PXR_PATH=$(find "$ISAAC_VENV/lib/python3.11/site-packages" -path "*/omni.usd.libs*/pxr" -type d 2>/dev/null | head -1)

if [ -n "$PXR_PATH" ]; then
    echo -e "${GREEN}✓ Found Isaac Sim pxr:${NC}"
    echo "  $PXR_PATH"
    
    # Parent directory (contains pxr and libs)
    USD_LIBS_DIR=$(dirname "$PXR_PATH")
    echo ""
    echo "USD Libs Directory:"
    echo "  $USD_LIBS_DIR"
    
    # Check for lib64 or lib
    if [ -d "$USD_LIBS_DIR/lib64" ]; then
        LIB_DIR="$USD_LIBS_DIR/lib64"
    elif [ -d "$USD_LIBS_DIR/lib" ]; then
        LIB_DIR="$USD_LIBS_DIR/lib"
    else
        LIB_DIR=""
    fi
    
    if [ -n "$LIB_DIR" ]; then
        echo "Library Directory:"
        echo "  $LIB_DIR"
    fi
else
    echo -e "${RED}✗ Isaac Sim pxr NOT found!${NC}"
    echo "  Expected location: $ISAAC_VENV/lib/.../omni.usd.libs*/pxr"
    exit 1
fi
echo ""

# ------------------------------------------------------------------------------
# 4. Test pxr Import with PYTHONPATH
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[4/4] Testing pxr Import with PYTHONPATH${NC}"

export PYTHONPATH="$USD_LIBS_DIR:$PYTHONPATH"
if [ -n "$LIB_DIR" ]; then
    export LD_LIBRARY_PATH="$LIB_DIR:$LD_LIBRARY_PATH"
fi

echo "PYTHONPATH: $PYTHONPATH"
echo "LD_LIBRARY_PATH: ${LD_LIBRARY_PATH:-<not set>}"
echo ""

if python -c "import pxr; import sys; print('✓ pxr loaded:', pxr.__file__); print('  Python version:', sys.version)" 2>&1; then
    echo ""
    echo -e "${GREEN}✓ pxr successfully imported with environment variables!${NC}"
else
    echo ""
    echo -e "${RED}✗ pxr import failed even with PYTHONPATH${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}========================================================================"
echo "DIAGNOSTIC COMPLETE"
echo "========================================================================${NC}"
echo ""
echo "To use pxr in scripts, source this environment setup:"
echo -e "${BLUE}  source devops/setup_isaac_python_env.sh${NC}"
echo ""
