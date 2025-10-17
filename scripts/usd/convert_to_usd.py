#!/usr/bin/env python3
"""
URDF → USD 변환 스크립트 (Isaac Sim 5.0)

Isaac Sim의 URDF importer를 사용하여 USD로 변환합니다.

사용법:
    python scripts/usd/convert_to_usd.py <urdf_file> [output_usd]
    
예시:
    python scripts/usd/convert_to_usd.py \\
        assets/roarm_m3/urdf/roarm_m3_complete.urdf \\
        assets/roarm_m3/usd/roarm_m3.usd
"""

import sys
import os
from pathlib import Path


def convert_urdf_to_usd(urdf_path: Path, output_path: Path, fix_base: bool = True):
    """URDF를 USD로 변환
    
    Args:
        urdf_path: 입력 URDF 파일 경로
        output_path: 출력 USD 파일 경로
        fix_base: Base를 고정할지 여부
    """
    print("=" * 70)
    print("URDF → USD 변환")
    print("=" * 70)
    print(f"입력 URDF: {urdf_path}")
    print(f"출력 USD: {output_path}")
    print(f"Fix Base: {fix_base}")
    print()
    
    if not urdf_path.exists():
        print(f"❌ URDF 파일을 찾을 수 없습니다: {urdf_path}")
        sys.exit(1)
    
    # Isaac Sim 초기화
    print("🚀 Isaac Sim 초기화 중...")
    try:
        from isaacsim import SimulationApp
        simulation_app = SimulationApp({"headless": True})
        print("✅ Isaac Sim 초기화 완료")
    except Exception as e:
        print(f"❌ Isaac Sim 초기화 실패: {e}")
        print("\n힌트: Isaac Sim venv를 활성화하세요:")
        print("  source ~/isaacsim-venv/bin/activate")
        sys.exit(1)
    
    try:
        from pxr import Usd, UsdGeom, UsdPhysics, Sdf
        from isaacsim.asset.importer.urdf import _urdf
        import omni.kit.commands
        
        # USD Stage 생성
        print("\n📦 USD Stage 생성 중...")
        output_path.parent.mkdir(parents=True, exist_ok=True)
        stage = Usd.Stage.CreateNew(str(output_path))
        
        # 기본 설정
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        
        # World Xform 생성
        world_prim = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(world_prim.GetPrim())
        print("✅ World Prim 생성 완료")
        
        # URDF Import Config
        print("\n🔄 URDF Import 설정 중...")
        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = True
        import_config.fix_base = fix_base
        import_config.density = 1000.0
        import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
        import_config.default_drive_strength = 1e7
        import_config.default_position_drive_damping = 1e5
        
        print(f"   - Fix Base: {import_config.fix_base}")
        print(f"   - Merge Fixed Joints: {import_config.merge_fixed_joints}")
        print(f"   - Import Inertia: {import_config.import_inertia_tensor}")
        
        # URDF Parse
        print("\n🔄 URDF Import 시작...")
        urdf_interface = _urdf.acquire_urdf_interface()
        
        # URDF 파일 경로를 절대 경로로 변환
        urdf_abs_path = urdf_path.resolve()
        
        # Import 실행
        result = urdf_interface.parse_urdf(
            str(urdf_abs_path.parent),
            str(urdf_abs_path.name),
            import_config
        )
        
        if result is None:
            raise RuntimeError("URDF Parse 실패")
        
        print("✅ URDF Parse 완료")
        
        # USD로 변환
        print("\n🔄 USD로 변환 중...")
        success = urdf_interface.import_robot(
            str(urdf_abs_path.parent),
            str(urdf_abs_path.name),
            result,
            import_config,
            "/World/roarm_m3"
        )
        
        if not success:
            raise RuntimeError("USD Import 실패")
        
        print("✅ USD Import 완료: /World/roarm_m3")
        
        # CollisionAPI 검증
        print("\n🔍 CollisionAPI 검증 중...")
        robot_prim = stage.GetPrimAtPath("/World/roarm_m3")
        collision_count = 0
        
        for prim in Usd.PrimRange(robot_prim):
            if UsdPhysics.CollisionAPI(prim):
                collision_count += 1
        
        print(f"   ✅ CollisionAPI 적용: {collision_count}개")
        
        # ArticulationRootAPI 확인
        if UsdPhysics.ArticulationRootAPI(robot_prim):
            print("   ✅ ArticulationRootAPI 적용됨")
            fixed_base = robot_prim.GetAttribute("physics:fixedBase")
            if fixed_base and fixed_base.Get():
                print("   ✅ physics:fixedBase = True")
        
        # Stage 저장
        print("\n💾 USD 파일 저장 중...")
        stage.Save()
        print(f"✅ USD 저장 완료: {output_path}")
        
        file_size = output_path.stat().st_size / 1024
        print(f"   파일 크기: {file_size:.2f} KB")
        
    except Exception as e:
        print(f"\n❌ 변환 중 오류 발생: {e}")
        import traceback
        traceback.print_exc()
        simulation_app.close()
        sys.exit(1)
    
    finally:
        print("\n🛑 Isaac Sim 종료 중...")
        simulation_app.close()
        print("✅ 완료")
    
    print("\n" + "=" * 70)
    print("✨ 변환 완료!")
    print("=" * 70)
    print("\n다음 단계:")
    print(f"  1. USD 검증: python scripts/usd/verify_usd.py {output_path}")
    print("  2. Isaac Sim GUI 테스트")
    print()


def main():
    """메인 함수"""
    if len(sys.argv) < 2:
        print("사용법: python convert_to_usd.py <urdf_file> [output_usd]")
        print()
        print("예시:")
        print("  python scripts/usd/convert_to_usd.py \\")
        print("      assets/roarm_m3/urdf/roarm_m3_complete.urdf \\")
        print("      assets/roarm_m3/usd/roarm_m3.usd")
        sys.exit(1)
    
    urdf_path = Path(sys.argv[1])
    
    if len(sys.argv) >= 3:
        output_path = Path(sys.argv[2])
    else:
        # 기본 출력 경로: urdf와 같은 이름, usd 폴더
        output_path = urdf_path.parent.parent / "usd" / (urdf_path.stem + ".usd")
    
    convert_urdf_to_usd(urdf_path, output_path, fix_base=True)


if __name__ == "__main__":
    main()
