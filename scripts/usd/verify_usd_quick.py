#!/usr/bin/env python3
"""
USD Quick Verification using Isaac Sim
SimulationApp을 통해 USD 파일 검증
"""

import sys
from pathlib import Path

def main():
    if len(sys.argv) < 2:
        print("Usage: python verify_usd_quick.py <path_to_usd>")
        return 1
    
    usd_path = Path(sys.argv[1])
    
    if not usd_path.exists():
        print(f"❌ USD file not found: {usd_path}")
        return 1
    
    print("="*72)
    print(f"       USD Quick Verification: {usd_path.name}")
    print("="*72)
    print()
    
    print(f"File: {usd_path}")
    print(f"Size: {usd_path.stat().st_size:,} bytes")
    print()
    
    # Isaac Sim 초기화
    print("🚀 Initializing Isaac Sim (headless)...")
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": True})
    
    print("✓ SimulationApp initialized")
    print()
    
    # USD 모듈 import (SimulationApp 초기화 후에만 가능)
    from pxr import Usd, UsdPhysics, PhysxSchema, UsdGeom
    import omni.usd
    
    # Stage 열기
    print(f"📂 Opening USD stage...")
    stage = Usd.Stage.Open(str(usd_path.absolute()))
    
    if not stage:
        print("❌ Failed to open USD stage")
        simulation_app.close()
        return 1
    
    print(f"✓ Stage opened: {stage.GetRootLayer().identifier}")
    print()
    
    # 기본 정보 수집
    print("="*72)
    print("STAGE INFORMATION")
    print("="*72)
    
    # 모든 Prim 수집
    all_prims = list(stage.Traverse())
    print(f"Total Prims: {len(all_prims)}")
    
    # Rigid Bodies
    rigid_bodies = [p for p in all_prims if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    print(f"Rigid Bodies: {len(rigid_bodies)}")
    
    # Articulation Roots
    articulation_roots = [p for p in all_prims if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    print(f"Articulation Roots: {len(articulation_roots)}")
    
    # Joints
    joints = [p for p in all_prims if p.IsA(UsdPhysics.Joint)]
    print(f"Joints: {len(joints)}")
    
    # Collision Shapes
    collision_shapes = [p for p in all_prims if p.HasAPI(UsdPhysics.CollisionAPI)]
    print(f"Collision Shapes: {len(collision_shapes)}")
    
    # Meshes
    meshes = [p for p in all_prims if p.IsA(UsdGeom.Mesh)]
    print(f"Meshes: {len(meshes)}")
    
    print()
    
    # ArticulationRoot 상세
    if articulation_roots:
        print("="*72)
        print("ARTICULATION ROOTS")
        print("="*72)
        for art_root in articulation_roots:
            print(f"  Path: {art_root.GetPath()}")
            
            # PhysxArticulationAPI 확인
            if art_root.HasAPI(PhysxSchema.PhysxArticulationAPI):
                print(f"  ✓ PhysxArticulationAPI: Applied")
            else:
                print(f"  ⚠ PhysxArticulationAPI: Not Applied")
        print()
    
    # Joints 상세
    if joints:
        print("="*72)
        print("JOINTS")
        print("="*72)
        for joint in joints:
            print(f"  {joint.GetPath().name}:")
            print(f"    Type: {joint.GetTypeName()}")
            
            # DriveAPI 확인
            drive_apis = []
            for axis in ["angular", "linear", "transX", "transY", "transZ", "rotX", "rotY", "rotZ"]:
                if UsdPhysics.DriveAPI.Get(joint, axis):
                    drive_apis.append(axis)
            
            if drive_apis:
                print(f"    DriveAPI: {', '.join(drive_apis)}")
            else:
                print(f"    DriveAPI: None")
        print()
    
    # Collision 상세
    print("="*72)
    print("COLLISION")
    print("="*72)
    if collision_shapes:
        print(f"✓ {len(collision_shapes)} collision shape(s) found:")
        for col in collision_shapes[:10]:  # 처음 10개만 출력
            print(f"  - {col.GetPath()}")
        if len(collision_shapes) > 10:
            print(f"  ... and {len(collision_shapes) - 10} more")
    else:
        print("⚠ No collision shapes found!")
        print("  This may cause issues in physics simulation.")
    print()
    
    # RigidBody with Mass 확인
    print("="*72)
    print("MASS & INERTIA")
    print("="*72)
    bodies_with_mass = 0
    bodies_without_mass = 0
    
    for rb in rigid_bodies:
        if rb.HasAPI(UsdPhysics.MassAPI):
            mass_api = UsdPhysics.MassAPI(rb)
            mass = mass_api.GetMassAttr().Get()
            if mass and mass > 0:
                bodies_with_mass += 1
            else:
                bodies_without_mass += 1
                print(f"  ⚠ {rb.GetPath()}: Invalid mass ({mass})")
        else:
            bodies_without_mass += 1
            print(f"  ⚠ {rb.GetPath()}: No MassAPI")
    
    print(f"Bodies with valid mass: {bodies_with_mass}/{len(rigid_bodies)}")
    if bodies_without_mass > 0:
        print(f"⚠ Bodies without mass: {bodies_without_mass}")
    print()
    
    # 요약
    print("="*72)
    print("SUMMARY")
    print("="*72)
    
    issues = []
    
    if len(articulation_roots) == 0:
        issues.append("❌ No ArticulationRootAPI found")
    elif len(articulation_roots) > 1:
        issues.append(f"⚠ Multiple ArticulationRootAPI ({len(articulation_roots)})")
    else:
        print("✓ Single articulation root found")
    
    if len(collision_shapes) == 0:
        issues.append("❌ No collision shapes found")
    else:
        print(f"✓ {len(collision_shapes)} collision shapes present")
    
    if bodies_without_mass > 0:
        issues.append(f"⚠ {bodies_without_mass} bodies missing mass/inertia")
    else:
        print(f"✓ All {len(rigid_bodies)} bodies have valid mass")
    
    if len(joints) == 0:
        issues.append("⚠ No joints found")
    else:
        print(f"✓ {len(joints)} joints defined")
    
    print()
    
    if issues:
        print("ISSUES FOUND:")
        for issue in issues:
            print(f"  {issue}")
        print()
        print("⚠ USD has issues that may affect simulation")
        exit_code = 1
    else:
        print("✅ USD appears valid for simulation!")
        exit_code = 0
    
    # 정리
    print()
    print("🛑 Closing SimulationApp...")
    simulation_app.close()
    
    return exit_code

if __name__ == "__main__":
    sys.exit(main())
