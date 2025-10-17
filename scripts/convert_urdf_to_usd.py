#!/usr/bin/env python3
"""
URDF to USD 변환 스크립트 (Isaac Sim 5.0)

이 스크립트는 RoArm M3 URDF를 USD 포맷으로 변환합니다.
Isaac Sim의 URDF importer를 사용하여 CollisionAPI를 포함한 USD를 생성합니다.

사용법:
    python scripts/convert_urdf_to_usd.py

출력:
    assets/roarm_m3/usd/roarm_m3.usd
"""

import os
import sys
from pathlib import Path

# 프로젝트 루트 디렉토리 설정
PROJECT_ROOT = Path(__file__).parent.parent
URDF_PATH = PROJECT_ROOT / "assets" / "roarm_m3" / "urdf" / "roarm_m3.urdf"
USD_OUTPUT_PATH = PROJECT_ROOT / "assets" / "roarm_m3" / "usd" / "roarm_m3.usd"

# USD 출력 디렉토리 생성
USD_OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)


def main():
    """URDF를 USD로 변환"""
    print("=" * 70)
    print("RoArm M3 URDF → USD 변환 시작")
    print("=" * 70)
    
    # URDF 파일 확인
    if not URDF_PATH.exists():
        print(f"❌ ERROR: URDF 파일을 찾을 수 없습니다: {URDF_PATH}")
        sys.exit(1)
    
    print(f"✅ URDF 경로: {URDF_PATH}")
    print(f"📝 출력 USD: {USD_OUTPUT_PATH}")
    print()
    
    # Isaac Sim 초기화 (headless 모드)
    print("🚀 Isaac Sim 초기화 중...")
    try:
        from isaacsim import SimulationApp
        
        simulation_app = SimulationApp({
            "headless": True,  # GUI 없이 실행
            "renderer": "RayTracedLighting",
        })
        print("✅ Isaac Sim 초기화 완료")
    except Exception as e:
        print(f"❌ Isaac Sim 초기화 실패: {e}")
        print("\n힌트: Isaac Sim venv가 활성화되어 있는지 확인하세요:")
        print("  source ~/isaacsim-venv/bin/activate")
        sys.exit(1)
    
    print()
    
    try:
        # USD Stage 생성
        from pxr import Usd, UsdGeom, UsdPhysics, Sdf
        print("📦 USD Stage 생성 중...")
        
        stage = Usd.Stage.CreateNew(str(USD_OUTPUT_PATH))
        
        # 기본 설정
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        
        # World Xform 생성
        world_prim = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(world_prim.GetPrim())
        print("✅ World Prim 생성 완료")
        
        # URDF Importer 사용
        print()
        print("🔄 URDF Import 시작...")
        print(f"   입력: {URDF_PATH}")
        
        # URDF Importer 사용
        print("   Isaac Sim 5.0 URDF importer 사용")
        
        from isaacsim.asset.importer.urdf import _urdf
        import omni.kit.commands
        
        # URDF Import Config 객체 생성
        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = True
        import_config.fix_base = True  # ⭐ Base 고정!
        import_config.density = 1000.0
        import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY
        import_config.default_drive_strength = 1e7
        import_config.default_position_drive_damping = 1e5
        
        # URDF 임포트 실행
        success, robot_prim_path = omni.kit.commands.execute(
            "URDFCreateImportConfig",
            import_config=import_config,
        )
        
        urdf_interface = _urdf.acquire_urdf_interface()
        imported_robot = urdf_interface.parse_urdf(
            str(URDF_PATH.parent),
            str(URDF_PATH.name),
            import_config
        )
        
        # USD Stage에 import
        success, robot_prim_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=str(URDF_PATH),
            import_config=import_config,
        )
        
        if not success:
            raise RuntimeError("URDF Import 실패")
        
        print(f"✅ URDF Import 성공: {robot_prim_path}")
        
        # CollisionAPI 검증
        print()
        print("🔍 CollisionAPI 검증 중...")
        robot_prim = stage.GetPrimAtPath(robot_prim_path)
        
        collision_count = 0
        missing_collision = []
        
        for prim in Usd.PrimRange(robot_prim):
            if prim.IsA(UsdGeom.Mesh) or prim.GetTypeName() in ["Cylinder", "Box", "Sphere", "Capsule"]:
                # Collision API 확인
                if UsdPhysics.CollisionAPI(prim):
                    collision_count += 1
                    print(f"   ✅ CollisionAPI: {prim.GetPath()}")
                else:
                    missing_collision.append(str(prim.GetPath()))
                    print(f"   ⚠️  CollisionAPI 누락: {prim.GetPath()}")
        
        print()
        print(f"📊 CollisionAPI 통계:")
        print(f"   - 발견: {collision_count}개")
        print(f"   - 누락: {len(missing_collision)}개")
        
        if missing_collision:
            print()
            print("⚠️  CollisionAPI가 누락된 Prim들:")
            for path in missing_collision:
                print(f"     - {path}")
            print()
            print("💡 해결 방법:")
            print("   1. USD 파일을 Isaac Sim GUI에서 열기")
            print("   2. 각 링크에 수동으로 CollisionAPI 추가")
            print("   3. 또는 verify_usd.py 스크립트에서 자동 추가")
        
        # ArticulationRootAPI 확인
        print()
        print("🔍 ArticulationRootAPI 확인...")
        if UsdPhysics.ArticulationRootAPI(robot_prim):
            print("   ✅ ArticulationRootAPI 적용됨")
            
            # Fixed base 확인
            fixed_base_attr = robot_prim.GetAttribute("physics:fixedBase")
            if fixed_base_attr and fixed_base_attr.Get():
                print("   ✅ physics:fixedBase = True")
            else:
                print("   ⚠️  physics:fixedBase가 설정되지 않았습니다")
                print("      Base가 공중에 떠있거나 회전할 수 있습니다!")
        else:
            print("   ⚠️  ArticulationRootAPI가 없습니다")
        
        # Stage 저장
        print()
        print("💾 USD 파일 저장 중...")
        stage.Save()
        print(f"✅ USD 저장 완료: {USD_OUTPUT_PATH}")
        
        # 파일 크기 확인
        file_size = USD_OUTPUT_PATH.stat().st_size / 1024  # KB
        print(f"   파일 크기: {file_size:.2f} KB")
        
    except Exception as e:
        print(f"❌ 변환 중 오류 발생: {e}")
        import traceback
        traceback.print_exc()
        simulation_app.close()
        sys.exit(1)
    
    finally:
        # Isaac Sim 종료
        print()
        print("🛑 Isaac Sim 종료 중...")
        simulation_app.close()
        print("✅ 완료")
    
    print()
    print("=" * 70)
    print("✨ 변환 완료!")
    print("=" * 70)
    print()
    print("다음 단계:")
    print("  1. USD 검증: python scripts/verify_usd.py")
    print("  2. Isaac Sim GUI 테스트:")
    print("     - File → Open → assets/roarm_m3/usd/roarm_m3.usd")
    print("     - Timeline 재생 테스트")
    print()


if __name__ == "__main__":
    main()
