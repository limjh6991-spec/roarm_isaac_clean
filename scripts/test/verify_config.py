#!/usr/bin/env python3
"""
설정 검증 스크립트 - Stiffness/Damping 값 확인
"""

import sys
from pathlib import Path

# 스크립트 경로 추가
SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

# test_roarm_with_camera_isaaclab.py 임포트
import importlib.util
spec = importlib.util.spec_from_file_location(
    "test_module",
    SCRIPT_DIR / "test_roarm_with_camera_isaaclab.py"
)
test_module = importlib.util.module_from_spec(spec)

print("=" * 80)
print("🔍 설정 검증: Stiffness & Damping")
print("=" * 80)

# 파일 읽기
script_path = SCRIPT_DIR / "test_roarm_with_camera_isaaclab.py"
with open(script_path, 'r') as f:
    content = f.read()

# Stiffness 확인
print("\n✅ Stiffness 설정:")
if '"link2_to_link3": 2000.0' in content:
    print("   ✅ Elbow Stiffness = 2000.0 (MAXIMUM)")
else:
    print("   ❌ Elbow Stiffness ≠ 2000.0")
    
if '"link1_to_link2": 3000.0' in content:
    print("   ✅ Shoulder Stiffness = 3000.0 (MAXIMUM - 카메라 무게 대응!)")
elif '"link1_to_link2": 1000.0' in content:
    print("   ⚠️  Shoulder Stiffness = 1000.0 (부족할 수 있음)")
else:
    print("   ❌ Shoulder Stiffness 미설정")

# Damping 확인
print("\n✅ Damping 설정:")
if '"link2_to_link3": 50.0' in content:
    print("   ✅ Elbow Damping = 50.0 (MAXIMUM)")
else:
    print("   ❌ Elbow Damping ≠ 50.0")
    
if '"link1_to_link2": 60.0' in content:
    print("   ✅ Shoulder Damping = 60.0 (MAXIMUM - 진동 억제!)")
elif '"link1_to_link2": 20.0' in content:
    print("   ⚠️  Shoulder Damping = 20.0 (부족할 수 있음)")
else:
    print("   ❌ Shoulder Damping 미설정")

# 안정화 프레임 확인
print("\n✅ 안정화 루프:")
if 'for _ in range(50):' in content and 'robot.set_joint_position_target(desired_joint_pos)' in content:
    # 매 프레임 재설정 확인
    lines = content.split('\n')
    in_stabilization = False
    has_target_reset = False
    has_write_data = False
    
    for i, line in enumerate(lines):
        if 'for _ in range(50):' in line:
            in_stabilization = True
            continue
        if in_stabilization:
            if 'robot.set_joint_position_target' in line:
                has_target_reset = True
            if 'robot.write_data_to_sim' in line:
                has_write_data = True
            if 'sim.step()' in line:
                break
    
    if has_target_reset and has_write_data:
        print("   ✅ 50프레임 안정화 (매 프레임 target 재설정)")
    else:
        print("   ⚠️  50프레임이지만 매 프레임 재설정 안됨")
else:
    print("   ❌ 안정화 루프 미설정")

# Joint 동작 테스트 확인
print("\n✅ Joint Motion Test:")
if 'robot.set_joint_position_target(target_pos)' in content:
    # write_data_to_sim 확인
    lines = content.split('\n')
    for i, line in enumerate(lines):
        if 'robot.set_joint_position_target(target_pos)' in line:
            # 다음 줄에 write_data_to_sim이 있는지 확인
            if i + 1 < len(lines) and 'robot.write_data_to_sim()' in lines[i+1]:
                print("   ✅ set_joint_position_target() + write_data_to_sim() (정상)")
                break
            else:
                print("   ❌ write_data_to_sim() 누락!")
                break
else:
    print("   ❌ Joint motion test 미설정")

print("\n" + "=" * 80)
print("✅ 검증 완료")
print("=" * 80)
