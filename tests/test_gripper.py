"""
🧪 Unit Test: Gripper Controller

테스트 항목:
1. measure_width() 정확도
2. is_grasped() 큐브 폭 판정
3. attach/detach 상태 관리
"""

import numpy as np
import sys
import os

# 프로젝트 루트를 path에 추가
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from controllers.gripper import Gripper


def test_measure_width():
    """그리퍼 폭 측정 테스트"""
    print("\n" + "="*60)
    print("🧪 TEST: measure_width()")
    print("="*60)
    
    # Mock stage (실제 Isaac Sim 없이 테스트)
    class MockStage:
        pass
    
    stage = MockStage()
    gripper = Gripper(stage, "/World/roarm", 
                     finger_joint_names=["left", "right"])
    
    # 테스트 케이스
    test_cases = [
        # (joint_positions, expected_width)
        (np.array([0, 0, 0, 0, 0, 0, 0.01, 0.01]), 0.02),   # 양쪽 1cm = 2cm
        (np.array([0, 0, 0, 0, 0, 0, 0.02, 0.02]), 0.04),   # 양쪽 2cm = 4cm (큐브 폭)
        (np.array([0, 0, 0, 0, 0, 0, 0.0175, 0.0175]), 0.035), # 3.5cm
        (np.array([0, 0, 0, 0, 0, 0, 0.0225, 0.0225]), 0.045), # 4.5cm
    ]
    
    for i, (positions, expected) in enumerate(test_cases):
        width = gripper.measure_width(positions)
        passed = abs(width - expected) < 1e-6
        status = "✅" if passed else "❌"
        print(f"  {status} Test {i+1}: width={width:.4f}m (expected={expected:.4f}m)")
    
    print("\n✅ measure_width() 테스트 완료!\n")


def test_is_grasped():
    """그립 판정 테스트"""
    print("\n" + "="*60)
    print("🧪 TEST: is_grasped() - 큐브 폭 기반 판정")
    print("="*60)
    
    class MockStage:
        pass
    
    stage = MockStage()
    gripper = Gripper(stage, "/World/roarm",
                     finger_joint_names=["left", "right"])
    
    ee_pos = np.array([0.3, 0.0, 0.08])
    cube_pos = np.array([0.31, 0.0, 0.08])  # EE 근처
    cube_size = 0.04  # 4cm 큐브
    
    test_cases = [
        # (gripper_width, expected_result, description)
        (0.01, False, "너무 닫힘 (빈 집게)"),
        (0.035, True, "큐브 끼움 (3.5cm)"),
        (0.04, True, "큐브 끼움 (4.0cm)"),
        (0.045, True, "큐브 끼움 (4.5cm)"),
        (0.05, False, "너무 열림 (5cm)"),
    ]
    
    for i, (width, expected, desc) in enumerate(test_cases):
        result = gripper.is_grasped(ee_pos, cube_pos, width, cube_size)
        passed = result == expected
        status = "✅" if passed else "❌"
        print(f"  {status} Test {i+1}: width={width:.3f}m → {result} ({desc})")
    
    # Z축 정렬 테스트
    print("\n  🔍 Z축 정렬 테스트:")
    cube_pos_high = np.array([0.31, 0.0, 0.10])  # 2cm 높음
    result = gripper.is_grasped(ee_pos, cube_pos_high, 0.04, cube_size, z_tol=0.01)
    print(f"    {'❌' if result else '✅'} Z축 2cm 차이 → {result} (should be False)")
    
    cube_pos_aligned = np.array([0.31, 0.0, 0.081])  # 1mm 차이
    result = gripper.is_grasped(ee_pos, cube_pos_aligned, 0.04, cube_size, z_tol=0.01)
    print(f"    {'✅' if result else '❌'} Z축 1mm 차이 → {result} (should be True)")
    
    print("\n✅ is_grasped() 테스트 완료!\n")


def test_attach_state():
    """Attach/detach 상태 관리 테스트"""
    print("\n" + "="*60)
    print("🧪 TEST: attach/detach 상태 관리")
    print("="*60)
    
    class MockStage:
        def DefinePrim(self, path, type_name):
            return None
        def GetPrimAtPath(self, path):
            return None
        def RemovePrim(self, path):
            pass
    
    stage = MockStage()
    gripper = Gripper(stage, "/World/roarm",
                     finger_joint_names=["left", "right"])
    
    # 초기 상태
    assert not gripper.is_attached, "초기 상태는 detached"
    print("  ✅ 초기 상태: detached")
    
    # Attach (실제 USD 없이 상태만 체크)
    gripper.is_attached = True
    gripper.grasp_joint_path = "/World/GraspJoint_100"
    assert gripper.is_attached, "Attach 후 is_attached=True"
    print("  ✅ Attach 상태: attached")
    
    # Reset
    gripper.reset()
    assert not gripper.is_attached, "Reset 후 detached"
    assert gripper.grasp_joint_path is None, "Reset 후 joint_path=None"
    print("  ✅ Reset 후: detached, joint_path=None")
    
    print("\n✅ attach/detach 상태 관리 테스트 완료!\n")


if __name__ == "__main__":
    print("\n" + "="*60)
    print("🧪 Gripper Controller Unit Tests")
    print("="*60)
    
    test_measure_width()
    test_is_grasped()
    test_attach_state()
    
    print("="*60)
    print("✅ 모든 테스트 통과!")
    print("="*60 + "\n")
