#!/usr/bin/env python3
"""
USD 파일 검증 스크립트

이 스크립트는 생성된 USD 파일의 구조를 검증합니다:
- CollisionAPI 존재 확인
- ArticulationRootAPI 확인
- Joint 구조 검증
- Fixed Base 설정 확인

사용법:
    python scripts/verify_usd.py [--fix]
    
옵션:
    --fix: CollisionAPI가 누락된 경우 자동으로 추가
"""

import sys
import argparse
from pathlib import Path

# 프로젝트 루트
PROJECT_ROOT = Path(__file__).parent.parent
USD_PATH = PROJECT_ROOT / "assets" / "roarm_m3" / "usd" / "roarm_m3.usd"


def verify_usd(usd_path: Path, fix: bool = False):
    """USD 파일 검증
    
    Args:
        usd_path: USD 파일 경로
        fix: CollisionAPI 누락 시 자동 추가 여부
    
    Returns:
        bool: 검증 성공 여부
    """
    print("=" * 70)
    print("USD 파일 검증")
    print("=" * 70)
    print(f"파일: {usd_path}")
    print(f"수정 모드: {'ON' if fix else 'OFF'}")
    print()
    
    if not usd_path.exists():
        print(f"❌ ERROR: USD 파일을 찾을 수 없습니다: {usd_path}")
        return False
    
    try:
        from pxr import Usd, UsdGeom, UsdPhysics, Sdf
        
        # USD 로딩
        print("📂 USD Stage 로딩 중...")
        stage = Usd.Stage.Open(str(usd_path))
        print("✅ Stage 로딩 완료")
        print()
        
        # 루트 Prim 찾기
        default_prim = stage.GetDefaultPrim()
        if not default_prim:
            print("⚠️  DefaultPrim이 설정되지 않았습니다")
            # World 찾기
            world_prim = stage.GetPrimAtPath("/World")
            if world_prim:
                print("   /World를 루트로 사용합니다")
                root = world_prim
            else:
                print("❌ /World를 찾을 수 없습니다")
                return False
        else:
            print(f"✅ DefaultPrim: {default_prim.GetPath()}")
            root = default_prim
        
        # 로봇 Prim 찾기
        print()
        print("🤖 로봇 Prim 검색 중...")
        robot_prim = None
        for child in root.GetChildren():
            if "roarm" in child.GetName().lower():
                robot_prim = child
                print(f"✅ 로봇 Prim 발견: {robot_prim.GetPath()}")
                break
        
        if not robot_prim:
            print("❌ 로봇 Prim을 찾을 수 없습니다")
            return False
        
        # ArticulationRootAPI 확인
        print()
        print("🔍 ArticulationRootAPI 확인...")
        art_api = UsdPhysics.ArticulationRootAPI(robot_prim)
        if art_api:
            print("   ✅ ArticulationRootAPI 적용됨")
            
            # Fixed base 확인
            fixed_base_attr = robot_prim.GetAttribute("physics:fixedBase")
            if fixed_base_attr:
                is_fixed = fixed_base_attr.Get()
                if is_fixed:
                    print("   ✅ physics:fixedBase = True (Base 고정)")
                else:
                    print("   ⚠️  physics:fixedBase = False (Base가 움직일 수 있음)")
            else:
                print("   ⚠️  physics:fixedBase 속성이 없습니다")
                if fix:
                    print("      🔧 physics:fixedBase = True 설정 중...")
                    robot_prim.CreateAttribute(
                        "physics:fixedBase", 
                        Sdf.ValueTypeNames.Bool
                    ).Set(True)
                    print("      ✅ 설정 완료")
        else:
            print("   ⚠️  ArticulationRootAPI가 없습니다")
            if fix:
                print("      🔧 ArticulationRootAPI 적용 중...")
                UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
                robot_prim.CreateAttribute(
                    "physics:fixedBase",
                    Sdf.ValueTypeNames.Bool
                ).Set(True)
                print("      ✅ 적용 완료")
        
        # CollisionAPI 검증
        print()
        print("🔍 CollisionAPI 검증 중...")
        print()
        
        links = []
        collision_prims = []
        missing_collision = []
        
        # 모든 링크 찾기
        for prim in Usd.PrimRange(robot_prim):
            prim_name = prim.GetName()
            
            # 링크 이름 패턴 (base_link, link_*)
            if "link" in prim_name.lower():
                links.append(prim)
                
                # Collision 형상 찾기
                has_collision = False
                for child in prim.GetChildren():
                    # Collision 형상 타입
                    if child.GetTypeName() in ["Mesh", "Cylinder", "Box", "Sphere", "Capsule", "Cube"]:
                        # CollisionAPI 확인
                        if UsdPhysics.CollisionAPI(child):
                            collision_prims.append(child)
                            has_collision = True
                            print(f"   ✅ {child.GetPath()}")
                        else:
                            missing_collision.append(child)
                            print(f"   ⚠️  CollisionAPI 누락: {child.GetPath()}")
                
                if not has_collision:
                    print(f"   ⚠️  링크에 Collision이 없음: {prim.GetPath()}")
        
        # 통계
        print()
        print("📊 검증 결과:")
        print(f"   - 총 링크 수: {len(links)}개")
        print(f"   - CollisionAPI 적용: {len(collision_prims)}개")
        print(f"   - CollisionAPI 누락: {len(missing_collision)}개")
        
        # CollisionAPI 자동 추가
        if missing_collision and fix:
            print()
            print("🔧 CollisionAPI 자동 추가 중...")
            for prim in missing_collision:
                UsdPhysics.CollisionAPI.Apply(prim)
                print(f"   ✅ {prim.GetPath()}")
            print("✅ CollisionAPI 추가 완료")
        
        # Joint 검증
        print()
        print("🔍 Joint 구조 검증...")
        joints = []
        for prim in Usd.PrimRange(robot_prim):
            if prim.IsA(UsdPhysics.Joint):
                joints.append(prim)
                joint_name = prim.GetName()
                
                # Joint 타입 확인
                joint_type = "Unknown"
                if UsdPhysics.RevoluteJoint(prim):
                    joint_type = "Revolute"
                elif UsdPhysics.PrismaticJoint(prim):
                    joint_type = "Prismatic"
                
                print(f"   ✅ {joint_name} ({joint_type})")
        
        print()
        print(f"📊 총 Joint 수: {len(joints)}개")
        
        # 파일 저장 (fix 모드인 경우)
        if fix and (missing_collision or not art_api):
            print()
            print("💾 수정사항 저장 중...")
            stage.Save()
            print(f"✅ 저장 완료: {usd_path}")
        
        # 최종 평가
        print()
        print("=" * 70)
        all_good = len(missing_collision) == 0 and art_api
        if all_good:
            print("✅ 검증 통과! USD 파일이 올바르게 구성되었습니다.")
        else:
            print("⚠️  일부 문제가 발견되었습니다.")
            if not fix:
                print()
                print("💡 자동 수정하려면:")
                print("   python scripts/verify_usd.py --fix")
        print("=" * 70)
        
        return all_good
        
    except ImportError as e:
        print(f"❌ ERROR: USD 라이브러리를 import할 수 없습니다: {e}")
        print()
        print("힌트: Isaac Sim venv를 활성화하세요:")
        print("  source ~/isaacsim-venv/bin/activate")
        return False
    except Exception as e:
        print(f"❌ ERROR: 검증 중 오류 발생: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """메인 함수"""
    parser = argparse.ArgumentParser(description="USD 파일 검증")
    parser.add_argument(
        "--fix",
        action="store_true",
        help="CollisionAPI 누락 시 자동으로 추가"
    )
    parser.add_argument(
        "--usd",
        type=Path,
        default=USD_PATH,
        help="USD 파일 경로 (기본값: assets/roarm_m3/usd/roarm_m3.usd)"
    )
    
    args = parser.parse_args()
    
    success = verify_usd(args.usd, args.fix)
    
    if not success:
        sys.exit(1)
    
    print()
    print("다음 단계:")
    if args.fix:
        print("  1. Isaac Sim GUI에서 테스트:")
        print("     isaac-sim")
        print("     File → Open → assets/roarm_m3/usd/roarm_m3.usd")
        print("     Timeline 재생")
    else:
        print("  1. 문제를 수정하려면: python scripts/verify_usd.py --fix")
        print("  2. 수동 수정: Isaac Sim GUI에서 CollisionAPI 추가")
    print()


if __name__ == "__main__":
    main()
