#!/usr/bin/env bash
# ==============================================================================
# USD Integrity Check
# Isaac Sim의 python.sh를 사용하여 USD 무결성 검사
# ==============================================================================

set -Eeuo pipefail

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
USD_FILE="${1:-assets/roarm_m3/usd/roarm_m3.usd}"

echo -e "========================================================================"
echo -e "       USD Integrity Check: $(basename "$USD_FILE")"
echo -e "========================================================================"
echo ""

# 1) 파일 존재
if [[ ! -f "$USD_FILE" ]]; then
  echo -e "${RED}✗ USD file not found:${NC} $USD_FILE"
  exit 1
fi
SZ=$(stat -c%s "$USD_FILE" 2>/dev/null || echo 0)
echo -e "[1/5] File Existence Check"
echo -e "  File: $(readlink -f "$USD_FILE")"
echo -e "  Size: $SZ bytes"
echo -e "${GREEN}✓ USD file exists${NC}"
echo ""

# Isaac python 런처 또는 venv 탐색
find_isaac_python() {
  # 1. 표준 Isaac Sim 설치 (python.sh가 있는 경우)
  local cands=(
    "$HOME/.local/share/ov/pkg"/*isaac-sim*"/kit/python.sh"
    "$HOME/.local/share/ov/pkg"/*isaac-sim*"/python.sh"
    "$HOME/isaac-sim/kit/python.sh"
    "$HOME/isaac-sim/python.sh"
    "/opt/NVIDIA/isaac-sim/kit/python.sh"
    "/opt/NVIDIA/isaac-sim/python.sh"
  )
  for p in "${cands[@]}"; do
    [[ -x "$p" ]] && { echo "$p"; return 0; }
  done
  
  # 2. pip 설치 방식 (venv python + PYTHONPATH 설정 필요)
  if [[ -n "${ISAAC_VENV:-}" ]] && [[ -x "$ISAAC_VENV/bin/python" ]]; then
    echo "venv:$ISAAC_VENV/bin/python"
    return 0
  fi
  
  # 3. 현재 venv가 Isaac venv인지 확인
  if [[ -n "${VIRTUAL_ENV:-}" ]] && python -c "import isaacsim" 2>/dev/null; then
    echo "venv:$VIRTUAL_ENV/bin/python"
    return 0
  fi
  
  return 1
}

run_with_isaac_python() {
  local isaac_py="$1"
  echo -e "[2/5] USD Stage Load"
  
  # venv 방식인 경우 PYTHONPATH 설정
  if [[ "$isaac_py" == venv:* ]]; then
    local py_exec="${isaac_py#venv:}"
    local venv_root="$(dirname "$(dirname "$py_exec")")"
    
    # pxr 경로 찾기
    local pxr_path=$("$py_exec" -c "
import sys, os
try:
    import isaacsim
    site_packages = os.path.dirname(os.path.dirname(isaacsim.__file__))
    # omni.usd.libs 찾기
    import glob
    usd_libs = glob.glob(os.path.join(site_packages, 'isaacsim/extscache/omni.usd.libs*/'))
    if usd_libs:
        print(usd_libs[0])
    else:
        sys.exit(1)
except:
    sys.exit(1)
" 2>/dev/null)
    
    if [[ -z "$pxr_path" ]]; then
      echo -e "${RED}✗ Could not find omni.usd.libs in venv${NC}"
      return 2
    fi
    
    export PYTHONPATH="$pxr_path:${PYTHONPATH:-}"
    export LD_LIBRARY_PATH="$pxr_path/bin:${LD_LIBRARY_PATH:-}"
    
    "$py_exec" - "$USD_FILE" <<PY
import sys, os, importlib.util

# USD 파일 경로를 argv로 받기
if len(sys.argv) < 2:
    print("ERROR: USD file path not provided")
    sys.exit(1)
usd_path = os.path.abspath(sys.argv[1])

print("Using interpreter:", sys.executable)

# pxr 경로/버전 로깅 보강
try:
    from pxr import Usd, UsdGeom, Tf, Sdf, UsdPhysics
    spec = importlib.util.find_spec("pxr.Usd")
    origin = getattr(spec, "origin", None) if spec else None
    print("pxr.Usd origin:", origin if origin else "unknown (namespace package)")
except Exception as e:
    print("PXRMOD_FAIL:", e); sys.exit(2)
try:
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print("STAGE_OPEN_FAIL: failed to open stage"); sys.exit(3)
    print("Stage opened:", usd_path)
    roots = [p for p in stage.TraverseAll() if p.GetPath().pathString.count('/') == 1]
    print("Root prims:", len(roots))
except Exception as e:
    print("STAGE_OPEN_EXC:", e); sys.exit(4)

# [3/5] Basic Schema Checks
print("\n[3/5] Basic Schema Checks")
warns = []
fails = []

# 1) Stage metadata
try:
    meters = UsdGeom.GetStageMetersPerUnit(stage)
    up = UsdGeom.GetStageUpAxis(stage)
    print(f"  Stage metersPerUnit: {meters}, upAxis: {up}")
    if abs(meters - 1.0) > 1e-6:
        warns.append(f"metersPerUnit={meters} (recommended 1.0 for Isaac)")
    if up not in ("Z", "Y"):
        warns.append(f"upAxis={up} (expected Y or Z)")
except Exception as e:
    warns.append(f"Stage metadata read failed: {e}")

# 2) defaultPrim
default_prim = stage.GetDefaultPrim()
if default_prim:
    print(f"  defaultPrim: {default_prim.GetPath()}")
else:
    warns.append("defaultPrim not set")

# 3) Sublayers & references existence
try:
    root = stage.GetRootLayer()
    if root.subLayerPaths:
        print(f"  SubLayers: {len(root.subLayerPaths)}")
        for i, sub in enumerate(root.subLayerPaths):
            sub_abs = root.ComputeAbsolutePath(sub)
            if not os.path.exists(sub_abs):
                fails.append(f"Missing sublayer: {sub}")
except Exception as e:
    warns.append(f"SubLayer check error: {e}")

# 4) PhysX articulation root
try:
    from pxr import PhysxSchema
    art_roots = []
    for prim in stage.Traverse():
        if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
            art_roots.append(prim.GetPath().pathString)
    
    print(f"  PhysX Articulation roots: {len(art_roots)}")
    if len(art_roots) == 0:
        warns.append("No PhysxArticulationRoot found")
    elif len(art_roots) > 1:
        fails.append(f"Multiple articulation roots: {art_roots}")
    
    # Nested check
    for i in range(len(art_roots)):
        for j in range(len(art_roots)):
            if i!=j and art_roots[j].startswith(art_roots[i] + "/"):
                fails.append(f"Nested articulation roots: {art_roots[i]} > {art_roots[j]}")
except Exception as e:
    warns.append(f"Physx articulation check error: {e}")

# 5) Rigid bodies & collisions & mass
try:
    rb_count = 0
    mass_missing = []
    col_missing = []
    
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rb_count += 1
            # Mass
            if not prim.HasAPI(UsdPhysics.MassAPI):
                mass_missing.append(prim.GetPath().pathString)
            
            # Collision (간단 체크: Collision 관련 자식 prim 존재)
            has_collision = False
            for child in prim.GetChildren():
                child_name = child.GetName().lower()
                if "collision" in child_name or "coll" in child_name:
                    has_collision = True
                    break
            if not has_collision:
                col_missing.append(prim.GetPath().pathString)
    
    print(f"  Rigid bodies: {rb_count}")
    if mass_missing:
        warns.append(f"Rigid bodies missing MassAPI: {mass_missing[:3]}{' ...' if len(mass_missing)>3 else ''}")
    if col_missing:
        warns.append(f"Rigid bodies likely missing Collision: {col_missing[:3]}{' ...' if len(col_missing)>3 else ''}")
except Exception as e:
    warns.append(f"Rigid/Collision check error: {e}")

# 6) Joints: DriveAPI check
try:
    joint_types = ["RevoluteJoint", "PrismaticJoint", "SphericalJoint", "FixedJoint"]
    joint_count = 0
    no_drive = []
    
    for prim in stage.Traverse():
        type_name = prim.GetTypeName()
        if type_name in joint_types:
            joint_count += 1
            # DriveAPI check (angular or linear)
            has_drive = (prim.HasAPI(UsdPhysics.DriveAPI, "angular") or 
                        prim.HasAPI(UsdPhysics.DriveAPI, "linear"))
            if not has_drive and type_name != "FixedJoint":
                no_drive.append(prim.GetPath().pathString)
    
    print(f"  Joints: {joint_count}")
    if no_drive:
        warns.append(f"Joints missing DriveAPI: {no_drive[:3]}{' ...' if len(no_drive)>3 else ''}")
except Exception as e:
    warns.append(f"Joint check error: {e}")

# 결과 집계
if fails:
    print("\n${RED}✗ Schema FAILS:${NC}")
    for f in fails: print(f"  - {f}")
    sys.exit(5)
elif warns:
    print("\n${YELLOW}⚠ Schema WARNINGS:${NC}")
    for w in warns: print(f"  - {w}")
    print("\n${GREEN}✓ Schema validation PASS (with warnings)${NC}")
    sys.exit(0)
else:
    print(f"\n${GREEN}✓ Schema validation PASS${NC}")
    sys.exit(0)
PY
  else
    # 표준 python.sh 사용
    echo -e "Using Isaac launcher: $isaac_py"
    "$isaac_py" - "$USD_FILE" <<PY
import sys, os, importlib.util

# USD 파일 경로를 argv로 받기
if len(sys.argv) < 2:
    print("ERROR: USD file path not provided")
    sys.exit(1)
usd_path = os.path.abspath(sys.argv[1])

print("Using interpreter:", sys.executable)

# pxr 경로/버전 로깅 보강
try:
    from pxr import Usd, UsdGeom, Tf, Sdf, UsdPhysics
    spec = importlib.util.find_spec("pxr.Usd")
    origin = getattr(spec, "origin", None) if spec else None
    print("pxr.Usd origin:", origin if origin else "unknown (namespace package)")
except Exception as e:
    print("PXRMOD_FAIL:", e); sys.exit(2)
try:
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print("STAGE_OPEN_FAIL: failed to open stage"); sys.exit(3)
    print("Stage opened:", usd_path)
    roots = [p for p in stage.TraverseAll() if p.GetPath().pathString.count('/') == 1]
    print("Root prims:", len(roots))
except Exception as e:
    print("STAGE_OPEN_EXC:", e); sys.exit(4)

# [3/5] Basic Schema Checks
print("\n[3/5] Basic Schema Checks")
warns = []
fails = []

# 1) Stage metadata
try:
    meters = UsdGeom.GetStageMetersPerUnit(stage)
    up = UsdGeom.GetStageUpAxis(stage)
    print(f"  Stage metersPerUnit: {meters}, upAxis: {up}")
    if abs(meters - 1.0) > 1e-6:
        warns.append(f"metersPerUnit={meters} (recommended 1.0 for Isaac)")
    if up not in ("Z", "Y"):
        warns.append(f"upAxis={up} (expected Y or Z)")
except Exception as e:
    warns.append(f"Stage metadata read failed: {e}")

# 2) defaultPrim
default_prim = stage.GetDefaultPrim()
if default_prim:
    print(f"  defaultPrim: {default_prim.GetPath()}")
else:
    warns.append("defaultPrim not set")

# 3) Sublayers & references existence
try:
    root = stage.GetRootLayer()
    if root.subLayerPaths:
        print(f"  SubLayers: {len(root.subLayerPaths)}")
        for i, sub in enumerate(root.subLayerPaths):
            sub_abs = root.ComputeAbsolutePath(sub)
            if not os.path.exists(sub_abs):
                fails.append(f"Missing sublayer: {sub}")
except Exception as e:
    warns.append(f"SubLayer check error: {e}")

# 4) PhysX articulation root
try:
    from pxr import PhysxSchema
    art_roots = []
    for prim in stage.Traverse():
        if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
            art_roots.append(prim.GetPath().pathString)
    
    print(f"  PhysX Articulation roots: {len(art_roots)}")
    if len(art_roots) == 0:
        warns.append("No PhysxArticulationRoot found")
    elif len(art_roots) > 1:
        fails.append(f"Multiple articulation roots: {art_roots}")
    
    # Nested check
    for i in range(len(art_roots)):
        for j in range(len(art_roots)):
            if i!=j and art_roots[j].startswith(art_roots[i] + "/"):
                fails.append(f"Nested articulation roots: {art_roots[i]} > {art_roots[j]}")
except Exception as e:
    warns.append(f"Physx articulation check error: {e}")

# 5) Rigid bodies & collisions & mass
try:
    rb_count = 0
    mass_missing = []
    col_missing = []
    
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rb_count += 1
            # Mass
            if not prim.HasAPI(UsdPhysics.MassAPI):
                mass_missing.append(prim.GetPath().pathString)
            
            # Collision (간단 체크: Collision 관련 자식 prim 존재)
            has_collision = False
            for child in prim.GetChildren():
                child_name = child.GetName().lower()
                if "collision" in child_name or "coll" in child_name:
                    has_collision = True
                    break
            if not has_collision:
                col_missing.append(prim.GetPath().pathString)
    
    print(f"  Rigid bodies: {rb_count}")
    if mass_missing:
        warns.append(f"Rigid bodies missing MassAPI: {mass_missing[:3]}{' ...' if len(mass_missing)>3 else ''}")
    if col_missing:
        warns.append(f"Rigid bodies likely missing Collision: {col_missing[:3]}{' ...' if len(col_missing)>3 else ''}")
except Exception as e:
    warns.append(f"Rigid/Collision check error: {e}")

# 6) Joints: DriveAPI check
try:
    joint_types = ["RevoluteJoint", "PrismaticJoint", "SphericalJoint", "FixedJoint"]
    joint_count = 0
    no_drive = []
    
    for prim in stage.Traverse():
        type_name = prim.GetTypeName()
        if type_name in joint_types:
            joint_count += 1
            # DriveAPI check (angular or linear)
            has_drive = (prim.HasAPI(UsdPhysics.DriveAPI, "angular") or 
                        prim.HasAPI(UsdPhysics.DriveAPI, "linear"))
            if not has_drive and type_name != "FixedJoint":
                no_drive.append(prim.GetPath().pathString)
    
    print(f"  Joints: {joint_count}")
    if no_drive:
        warns.append(f"Joints missing DriveAPI: {no_drive[:3]}{' ...' if len(no_drive)>3 else ''}")
except Exception as e:
    warns.append(f"Joint check error: {e}")

# 결과 집계
if fails:
    print("\n${RED}✗ Schema FAILS:${NC}")
    for f in fails: print(f"  - {f}")
    sys.exit(5)
elif warns:
    print("\n${YELLOW}⚠ Schema WARNINGS:${NC}")
    for w in warns: print(f"  - {w}")
    print("\n${GREEN}✓ Schema validation PASS (with warnings)${NC}")
    sys.exit(0)
else:
    print(f"\n${GREEN}✓ Schema validation PASS${NC}")
    sys.exit(0)
PY
  fi
}

run_with_fallback_injection() {
  echo -e "[2/5] USD Stage Load (Fallback - Legacy Isaac installation)"
  local root="${ISAAC_ROOT:-}"
  if [[ -z "$root" ]]; then
    # 흔한 루트 추정
    for d in "$HOME/.local/share/ov/pkg"/*isaac-sim* "$HOME/isaac-sim" "/opt/NVIDIA/isaac-sim"; do
      [[ -d "$d" ]] && { root="$d"; break; }
    done
  fi
  if [[ -z "$root" ]]; then
    echo -e "${RED}✗ Could not locate ISAAC_ROOT and Isaac python launcher.${NC}"
    echo -e "  Do NOT 'pip install usd-core' for Isaac Sim. Use Isaac's python.sh or pip install isaac-sim."
    return 2
  fi
  export PYTHONPATH="$root/python:$root/exts/omni.usd.libs/bin:${PYTHONPATH:-}"
  export LD_LIBRARY_PATH="$root/kit/lib:$root/exts/omni.usd.libs/bin:${LD_LIBRARY_PATH:-}"
  echo "Using interpreter: python (fallback root: $root)"
  python - "$USD_FILE" <<PY
import sys, os, importlib.util

# USD 파일 경로를 argv로 받기
if len(sys.argv) < 2:
    print("ERROR: USD file path not provided")
    sys.exit(1)
usd_path = os.path.abspath(sys.argv[1])

print("Python executable:", sys.executable)
try:
    from pxr import Usd, UsdGeom
    spec = importlib.util.find_spec("pxr.Usd")
    origin = getattr(spec, "origin", None) if spec else None
    print("pxr.Usd origin:", origin if origin else "unknown")
except Exception as e:
    print("PXRMOD_FAIL:", e); sys.exit(2)
try:
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print("STAGE_OPEN_FAIL: failed to open stage"); sys.exit(3)
    print("Stage opened:", usd_path)
    roots = [p for p in stage.TraverseAll() if p.GetPath().pathString.count('/') == 1]
    print("Root prims:", len(roots))
    print()
    print("${YELLOW}⚠ Fallback mode: Schema validation not available${NC}")
except Exception as e:
    print("STAGE_OPEN_EXC:", e); sys.exit(4)
PY
}

# 실행 경로 선택: Isaac python -> fallback
if ISAAC_PY="$(find_isaac_python)"; then
  if ! run_with_isaac_python "$ISAAC_PY"; then
    echo -e "${YELLOW}⚠ Isaac launcher path found but execution failed. Trying fallback...${NC}"
    run_with_fallback_injection || { echo -e "${RED}✗ USD Integrity FAIL${NC}"; exit 1; }
  fi
else
  echo -e "${YELLOW}⚠ Isaac launcher not found. Trying fallback injection...${NC}"
  run_with_fallback_injection || { echo -e "${RED}✗ USD Integrity FAIL${NC}"; exit 1; }
fi

echo ""
echo -e "${GREEN}✓ USD Integrity PASS${NC}"
exit 0
