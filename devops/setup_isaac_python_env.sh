#!/usr/bin/env bash
# ==============================================================================
# Isaac Sim Python Environment Setup
# pxr 모듈을 사용하기 위한 환경변수 설정
#
# Usage:
#   source devops/setup_isaac_python_env.sh
#   python your_script_using_pxr.py
# ==============================================================================

# Isaac Sim venv 경로
ISAAC_VENV="$HOME/isaacsim-venv"

# omni.usd.libs 경로 찾기
USD_LIBS_DIR=$(find "$ISAAC_VENV/lib/python3.11/site-packages" -path "*/omni.usd.libs*/pxr" -type d 2>/dev/null | head -1)

if [ -z "$USD_LIBS_DIR" ]; then
    echo "ERROR: Isaac Sim pxr module not found!"
    echo "Expected location: $ISAAC_VENV/lib/.../omni.usd.libs*/pxr"
    return 1
fi

# pxr의 부모 디렉토리 (omni.usd.libs-xxx)
USD_LIBS_DIR=$(dirname "$USD_LIBS_DIR")

# PYTHONPATH에 추가
export PYTHONPATH="$USD_LIBS_DIR:$PYTHONPATH"

# LD_LIBRARY_PATH 설정 (lib64 또는 lib)
if [ -d "$USD_LIBS_DIR/lib64" ]; then
    export LD_LIBRARY_PATH="$USD_LIBS_DIR/lib64:$LD_LIBRARY_PATH"
elif [ -d "$USD_LIBS_DIR/lib" ]; then
    export LD_LIBRARY_PATH="$USD_LIBS_DIR/lib:$LD_LIBRARY_PATH"
fi

echo "✓ Isaac Sim Python environment configured"
echo "  PYTHONPATH: $USD_LIBS_DIR"
echo "  LD_LIBRARY_PATH: ${LD_LIBRARY_PATH:-<not set>}"
echo ""
echo "You can now use 'import pxr' in Python scripts."
