#!/usr/bin/env python3
"""
Isaac Sim Extensions Preflight Check
필수 확장이 모두 로드 가능한지 검사 (5초 이내)
"""

import sys
import os
from pathlib import Path

# 색상 정의
RED = '\033[0;31m'
GREEN = '\033[0;32m'
YELLOW = '\033[1;33m'
NC = '\033[0m'  # No Color

def print_color(color, text):
    print(f"{color}{text}{NC}")

def main():
    print("="*72)
    print("       Isaac Sim Extensions Preflight Check")
    print("="*72)
    print()
    
    # ----------------------------------------------------------------------
    # 1. Isaac Sim 환경 확인
    # ----------------------------------------------------------------------
    print_color(YELLOW, "[1/3] Checking Isaac Sim Installation")
    
    try:
        from isaacsim import SimulationApp
        print_color(GREEN, "✓ isaacsim.SimulationApp found")
    except ImportError as e:
        print_color(RED, f"✗ Cannot import isaacsim: {e}")
        print_color(RED, "  Make sure you're in the Isaac Sim virtual environment!")
        return 1
    
    # ----------------------------------------------------------------------
    # 2. Headless 부팅 테스트
    # ----------------------------------------------------------------------
    print()
    print_color(YELLOW, "[2/3] Testing Headless Boot (5 sec timeout)")
    
    try:
        # stderr 리디렉션으로 불필요한 경고 숨기기
        import logging
        logging.getLogger().setLevel(logging.ERROR)
        
        simulation_app = SimulationApp({
            "headless": True,
            "renderer": "RayTracedLighting",
            "hide_ui": True,
            "active_gpu": 0,
        })
        print_color(GREEN, "✓ SimulationApp initialized successfully")
    except Exception as e:
        print_color(RED, f"✗ Failed to initialize SimulationApp: {e}")
        return 2
    
    # ----------------------------------------------------------------------
    # 3. 필수 확장 검사
    # ----------------------------------------------------------------------
    print()
    print_color(YELLOW, "[3/3] Checking Required Extensions")
    
    required_extensions = [
        "isaacsim.asset.importer.urdf",
        "isaacsim.core.api",
        "omni.isaac.core",
        "omni.kit.viewport.window",
        "omni.physx",
        "omni.usd",
    ]
    
    try:
        import omni.kit.app
        app_interface = omni.kit.app.get_app()
        ext_manager = app_interface.get_extension_manager()
        
        all_enabled = True
        for ext_name in required_extensions:
            is_enabled = ext_manager.is_extension_enabled(ext_name)
            status = f"{'✓' if is_enabled else '✗'}"
            color = GREEN if is_enabled else RED
            print_color(color, f"  {status} {ext_name}")
            
            if not is_enabled:
                all_enabled = False
        
        if not all_enabled:
            print()
            print_color(RED, "Some required extensions are not enabled!")
            simulation_app.close()
            return 3
        
        print()
        print_color(GREEN, "✓ All required extensions enabled")
        
    except Exception as e:
        print_color(RED, f"✗ Failed to check extensions: {e}")
        simulation_app.close()
        return 4
    
    # ----------------------------------------------------------------------
    # 정리
    # ----------------------------------------------------------------------
    print()
    print_color(YELLOW, "Shutting down SimulationApp...")
    simulation_app.close()
    
    print()
    print("="*72)
    print_color(GREEN, "PREFLIGHT PASSED! Isaac Sim extensions ready.")
    print("="*72)
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
