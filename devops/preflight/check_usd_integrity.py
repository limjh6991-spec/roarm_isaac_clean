#!/usr/bin/env python3
"""
USD Model Integrity Preflight Check
USD 파일의 물리 시뮬레이션 준비 상태 검사
"""

import sys
from pathlib import Path

# 색상 정의
RED = '\033[0;31m'
GREEN = '\033[0;32m'
YELLOW = '\033[1;33m'
NC = '\033[0m'

def print_color(color, text):
    print(f"{color}{text}{NC}")

def check_usd_integrity(usd_path: Path) -> int:
    """
    USD 파일의 무결성 검사
    
    Returns:
        0: 성공
        1: USD 파일 없음
        2: Mass/Inertia 누락
        3: Articulation 문제
        4: Drive 설정 문제
    """
    print("="*72)
    print(f"       USD Integrity Check: {usd_path.name}")
    print("="*72)
    print()
    
    # ----------------------------------------------------------------------
    # 1. 파일 존재 확인
    # ----------------------------------------------------------------------
    print_color(YELLOW, "[1/5] File Existence Check")
    if not usd_path.exists():
        print_color(RED, f"✗ USD file not found: {usd_path}")
        return 1
    
    file_size = usd_path.stat().st_size
    print(f"  File: {usd_path}")
    print(f"  Size: {file_size:,} bytes ({file_size/1024:.2f} KB)")
    print_color(GREEN, "✓ USD file exists")
    print()
    
    # ----------------------------------------------------------------------
    # 2. USD Stage 로드
    # ----------------------------------------------------------------------
    print_color(YELLOW, "[2/5] USD Stage Load")
    try:
        from pxr import Usd, UsdPhysics, PhysxSchema, UsdGeom
        stage = Usd.Stage.Open(str(usd_path))
        if not stage:
            print_color(RED, "✗ Failed to open USD stage")
            return 1
        print_color(GREEN, f"✓ Stage loaded successfully")
    except ImportError:
        print_color(RED, "✗ pxr (USD) module not available")
        print("  Install: pip install usd-core")
        return 1
    except Exception as e:
        print_color(RED, f"✗ Failed to load stage: {e}")
        return 1
    print()
    
    # ----------------------------------------------------------------------
    # 3. Mass/Inertia 검사
    # ----------------------------------------------------------------------
    print_color(YELLOW, "[3/5] Mass & Inertia Check")
    errors = []
    warnings = []
    
    rigid_bodies = []
    for prim in stage.Traverse():
        # RigidBodyAPI가 적용된 prim 찾기
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_bodies.append(prim)
    
    print(f"  Found {len(rigid_bodies)} rigid bodies")
    
    for prim in rigid_bodies:
        # MassAPI 확인
        if not prim.HasAPI(UsdPhysics.MassAPI):
            msg = f"    ✗ {prim.GetPath()}: No MassAPI (inertia missing)"
            errors.append(msg)
            print_color(RED, msg)
        else:
            mass_api = UsdPhysics.MassAPI(prim)
            mass = mass_api.GetMassAttr().Get()
            if mass is None or mass <= 0:
                msg = f"    ⚠ {prim.GetPath()}: Invalid mass ({mass})"
                warnings.append(msg)
                print_color(YELLOW, msg)
    
    if not errors and not warnings:
        print_color(GREEN, "✓ All rigid bodies have valid mass/inertia")
    print()
    
    # ----------------------------------------------------------------------
    # 4. Articulation 검사
    # ----------------------------------------------------------------------
    print_color(YELLOW, "[4/5] Articulation Check")
    
    articulation_roots = []
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            articulation_roots.append(prim)
    
    print(f"  Found {len(articulation_roots)} articulation root(s)")
    
    if len(articulation_roots) == 0:
        msg = "  ⚠ No ArticulationRootAPI found - is this a robot?"
        warnings.append(msg)
        print_color(YELLOW, msg)
    elif len(articulation_roots) > 1:
        msg = f"  ✗ Multiple ArticulationRootAPI found ({len(articulation_roots)}) - conflict possible!"
        errors.append(msg)
        print_color(RED, msg)
    else:
        root_path = articulation_roots[0].GetPath()
        print_color(GREEN, f"✓ Single articulation root: {root_path}")
    print()
    
    # ----------------------------------------------------------------------
    # 5. Joint Drive 검사
    # ----------------------------------------------------------------------
    print_color(YELLOW, "[5/5] Joint Drive Check")
    
    joints = []
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint):
            joints.append(prim)
    
    print(f"  Found {len(joints)} joint(s)")
    
    for joint_prim in joints:
        joint_name = joint_prim.GetPath()
        
        # DriveAPI 확인 (angular 또는 linear)
        has_drive = False
        for axis in ["angular", "linear", "transX", "transY", "transZ", "rotX", "rotY", "rotZ"]:
            if UsdPhysics.DriveAPI.Get(joint_prim, axis):
                has_drive = True
                break
        
        if not has_drive:
            msg = f"    ⚠ {joint_name}: No DriveAPI configured"
            warnings.append(msg)
            print_color(YELLOW, msg)
    
    if not warnings and joints:
        print_color(GREEN, "✓ All joints have drive configuration")
    print()
    
    # ----------------------------------------------------------------------
    # Summary
    # ----------------------------------------------------------------------
    print("="*72)
    print(f"ERRORS: {len(errors)}, WARNINGS: {len(warnings)}")
    
    if errors:
        print_color(RED, "PREFLIGHT FAILED! Fix critical errors above.")
        return 2
    elif warnings:
        print_color(YELLOW, "PREFLIGHT PASSED with warnings. Review above.")
        return 0
    else:
        print_color(GREEN, "PREFLIGHT PASSED! USD model ready for simulation.")
        return 0

def main():
    if len(sys.argv) < 2:
        print("Usage: python check_usd_integrity.py <path_to_usd_file>")
        return 1
    
    usd_path = Path(sys.argv[1])
    return check_usd_integrity(usd_path)

if __name__ == "__main__":
    sys.exit(main())
