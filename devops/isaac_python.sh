#!/usr/bin/env bash
# ==============================================================================
# Isaac Python Wrapper
# pxr 모듈을 사용하는 스크립트를 위한 래퍼
#
# Usage:
#   bash devops/isaac_python.sh your_script.py [args...]
# ==============================================================================

set -euo pipefail

# Isaac Sim venv 경로
ISAAC_VENV="$HOME/isaacsim-venv"

# omni.usd.libs 경로 찾기
USD_LIBS_DIR=$(find "$ISAAC_VENV/lib/python3.11/site-packages" -path "*/omni.usd.libs*/pxr" -type d 2>/dev/null | head -1)

if [ -z "$USD_LIBS_DIR" ]; then
    echo "ERROR: Isaac Sim pxr module not found!"
    echo "Expected location: $ISAAC_VENV/lib/.../omni.usd.libs*/pxr"
    exit 1
fi

# pxr의 부모 디렉토리 (omni.usd.libs-xxx)
USD_LIBS_DIR=$(dirname "$USD_LIBS_DIR")

# PYTHONPATH 설정
export PYTHONPATH="$USD_LIBS_DIR:${PYTHONPATH:-}"

# LD_LIBRARY_PATH 설정
if [ -d "$USD_LIBS_DIR/lib64" ]; then
    export LD_LIBRARY_PATH="$USD_LIBS_DIR/lib64:${LD_LIBRARY_PATH:-}"
elif [ -d "$USD_LIBS_DIR/lib" ]; then
    export LD_LIBRARY_PATH="$USD_LIBS_DIR/lib:${LD_LIBRARY_PATH:-}"
fi

# Python 스크립트 실행
exec python "$@"
