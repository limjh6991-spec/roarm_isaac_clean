#!/usr/bin/env python3
"""
RoArm-M3 조인트 구조 검증 스크립트
joint_6의 정체를 밝히기 위한 진단 도구
"""

import numpy as np
import time
from omni.isaac.kit import SimulationApp

# Isaac Sim 초기화
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage

def analyze_joint_structure():
    """조인트 구조 분석"""
    print("=" * 80)
    print("🔍 RoArm-M3 조인트 구조 분석")
    print("=" * 80)
    
    # World 생성
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    
    # URDF 경로
    urdf_path = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3_multiprim.urdf"
    
    # 로봇 추가
    print("\n📦 로봇 로딩...")
    add_reference_to_stage(
        usd_path=urdf_path,
        prim_path="/World/roarm"
    )
    
    robot = world.scene.add(
        Articulation(
            prim_path="/World/roarm",
            name="roarm"
        )
    )
    
    world.reset()
    
    # 조인트 정보 출력
    print("\n" + "=" * 80)
    print("📋 DOF 목록")
    print("=" * 80)
    
    dof_names = robot.dof_names
    num_dof = robot.num_dof
    
    print(f"\n총 DOF 개수: {num_dof}")
    print(f"\nDOF 이름 목록:")
    for i, name in enumerate(dof_names):
        print(f"  [{i}] {name}")
    
    # 조인트 위치 범위 확인
    print("\n" + "=" * 80)
    print("📏 조인트 범위 (Limits)")
    print("=" * 80)
    
    dof_properties = robot.get_articulation_controller().get_dof_properties()
    
    for i, name in enumerate(dof_names):
        lower = dof_properties["lower"][i] if "lower" in dof_properties else "N/A"
        upper = dof_properties["upper"][i] if "upper" in dof_properties else "N/A"
        print(f"  {name:25s}: [{lower:7.3f}, {upper:7.3f}]")
    
    # joint_6 테스트
    print("\n" + "=" * 80)
    print("🔧 joint_6 움직임 테스트")
    print("=" * 80)
    
    initial_pose = np.zeros(num_dof)
    
    # 초기 자세
    print("\n1️⃣ 초기 자세 (모든 조인트 0)")
    robot.set_joint_positions(initial_pose)
    world.reset()
    for _ in range(10):
        world.step(render=True)
    input("   ▶ Press Enter to continue...")
    
    # joint_6만 회전
    test_angles = [-1.5, -1.0, -0.5, 0, 0.5, 1.0, 1.5]
    
    if num_dof >= 6:
        for angle in test_angles:
            print(f"\n2️⃣ joint_6 = {angle:.2f} rad ({np.degrees(angle):.1f}°)")
            test_pose = initial_pose.copy()
            test_pose[5] = angle  # joint_6은 인덱스 5
            
            robot.set_joint_positions(test_pose)
            for _ in range(30):
                world.step(render=True)
            
            current = robot.get_joint_positions()[5]
            print(f"   명령: {angle:.3f} rad, 실제: {current:.3f} rad")
            input("   ▶ Press Enter to continue...")
    else:
        print("⚠️ joint_6이 없습니다 (DOF < 6)")
    
    # 다른 조인트와 비교
    print("\n" + "=" * 80)
    print("🔄 각 조인트 개별 테스트")
    print("=" * 80)
    
    joint_test_configs = {
        "joint_1 (BASE)": [0, 0, 0, 0, 0, 0, 0, 0],
        "joint_2 (SHOULDER)": [0, 1.0, 0, 0, 0, 0, 0, 0],
        "joint_3 (ELBOW)": [0, 0, 1.0, 0, 0, 0, 0, 0],
        "joint_4 (WRIST)": [0, 0, 0, 1.0, 0, 0, 0, 0],
        "joint_5 (ROLL)": [0, 0, 0, 0, 1.5, 0, 0, 0],
        "joint_6 (???)": [0, 0, 0, 0, 0, 1.5, 0, 0],
    }
    
    for joint_name, config in joint_test_configs.items():
        print(f"\n▶ 테스트: {joint_name}")
        test_pose = np.array(config[:num_dof])
        
        robot.set_joint_positions(test_pose)
        for _ in range(30):
            world.step(render=True)
        
        print("   화면을 보고 움직임을 확인하세요.")
        input("   ▶ Press Enter for next joint...")
    
    # 그리퍼 테스트
    print("\n" + "=" * 80)
    print("✋ 그리퍼 테스트")
    print("=" * 80)
    
    gripper_configs = {
        "완전 닫힘": [0, 0, 0, 0, 0, 0, 0.0, 0.0],
        "절반 열림": [0, 0, 0, 0, 0, 0, 0.0125, 0.0125],
        "완전 열림": [0, 0, 0, 0, 0, 0, 0.025, 0.025],
    }
    
    for gripper_state, config in gripper_configs.items():
        print(f"\n▶ 그리퍼 상태: {gripper_state}")
        test_pose = np.array(config[:num_dof])
        
        robot.set_joint_positions(test_pose)
        for _ in range(30):
            world.step(render=True)
        
        gripper_left = robot.get_joint_positions()[-2] if num_dof >= 7 else 0
        gripper_right = robot.get_joint_positions()[-1] if num_dof >= 8 else 0
        print(f"   왼쪽: {gripper_left*1000:.1f}mm, 오른쪽: {gripper_right*1000:.1f}mm")
        input("   ▶ Press Enter to continue...")
    
    # 최종 분석
    print("\n" + "=" * 80)
    print("📊 분석 결과")
    print("=" * 80)
    
    print(f"""
    검증 항목:
    1. 총 DOF 개수: {num_dof}
       - 실제 RoArm-M3: 6개 (5+1) 
       - URDF: {num_dof}개
       - 차이: {num_dof - 6}개 {'⚠️ 많음' if num_dof > 6 else '✅ 일치'}
    
    2. joint_6 움직임:
       - 관찰된 움직임: (화면에서 확인)
       - 예상 기능: {'그리퍼 회전?' if num_dof >= 6 else 'N/A'}
       - 실제 필요 여부: (판단 필요)
    
    3. 그리퍼 제어:
       - 왼쪽/오른쪽 독립 제어: {'예' if num_dof >= 8 else '아니오'}
       - 타입: {'Prismatic (병진)' if 'prismatic' in str(dof_names).lower() else 'Revolute (회전)'}
    
    다음 단계:
    □ joint_6이 실제로 움직였는가?
    □ 움직임이 의미 있는가? (그리퍼 회전 등)
    □ 실제 RoArm-M3에 해당 기능이 있는가?
    □ URDF 수정이 필요한가?
    """)
    
    simulation_app.close()

if __name__ == "__main__":
    try:
        analyze_joint_structure()
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
        simulation_app.close()
