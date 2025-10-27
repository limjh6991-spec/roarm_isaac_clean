#!/usr/bin/env python3
"""
개선된 URDF 직접 테스트 (Isaac Sim 5.0)
"""
import sys
sys.stdout = sys.stderr  # Buffering 해결

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
import carb
import time

print("=" * 70)
print("    🔍 URDF GUI 테스트")
print("=" * 70)

# World 생성
print("\n✓ World 생성 중...")
world = World()
world.scene.add_default_ground_plane()

# URDF 경로
urdf_path = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3_multiprim.urdf"

print(f"✓ URDF 로드: {urdf_path}")

# 로봇 추가
from isaacsim.core.api.utils import rotations
from pxr import UsdGeom

# URDF 임포트
import omni.kit.commands

# URDF를 USD로 변환 및 추가
omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=urdf_path,
    import_config=omni.isaac.urdf.ImportConfig(),
)

print("✓ 로봇 로드 완료")

# 큐브 추가
from isaacsim.core.api.objects import DynamicCuboid

cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        position=np.array([0.3, 0.0, 0.05]),
        size=0.03,
        color=np.array([1.0, 0.0, 0.0]),
    )
)

print("✓ 큐브 추가 완료")

# 리셋
world.reset()
print("✓ World 리셋 완료")

print("\n" + "=" * 70)
print("📊 테스트 시작")
print("=" * 70)

# 로봇 객체 가져오기
robot_prim_path = "/roarm_m3_multiprim"  # URDF에서 생성된 경로
robot = Articulation(prim_path=robot_prim_path)

# 초기 자세 대기
print("\n1️⃣  초기 자세 확인 (3초)...")
for _ in range(100):
    world.step(render=True)
time.sleep(3)

# 그리퍼 인덱스 찾기 (마지막 2개 조인트가 그리퍼)
dof_names = robot.dof_names
print(f"\n✓ DOF 이름: {dof_names}")
gripper_indices = [len(dof_names) - 2, len(dof_names) - 1]

print(f"\n2️⃣  그리퍼 동작 테스트 (10회)...")
print(f"   그리퍼 인덱스: {gripper_indices}")
print("   👀 그리퍼 형상을 확인하세요 (20x10mm 박스형 팁)")

for i in range(10):
    # 그리퍼 닫기
    positions = robot.get_joint_positions()
    positions[gripper_indices[0]] = 0.02  # 최대치
    positions[gripper_indices[1]] = 0.02
    robot.set_joint_positions(positions)
    
    for _ in range(10):
        world.step(render=True)
    
    # 그리퍼 열기
    positions = robot.get_joint_positions()
    positions[gripper_indices[0]] = 0.0  # 최소치
    positions[gripper_indices[1]] = 0.0
    robot.set_joint_positions(positions)
    
    for _ in range(10):
        world.step(render=True)
    
    if (i + 1) % 3 == 0:
        print(f"   반복 {i+1}/10")

print("\n✓ 그리퍼 테스트 완료")

print("\n" + "=" * 70)
print("✅ 테스트 완료!")
print("=" * 70)
print("\n👀 확인 사항:")
print("   [ ] 그리퍼가 박스형 팁(20x10mm)으로 보이는가?")
print("   [ ] 그리퍼가 부드럽게 열리고 닫히는가?")
print("   [ ] 개폐 범위가 0~25mm인가?")
print("\n창을 계속 열어두려면 아무 키나 누르세요...")

try:
    input()
    print("추가 30초 대기...")
    for _ in range(900):
        world.step(render=True)
except KeyboardInterrupt:
    print("\n✓ 사용자 중단")

simulation_app.close()
print("✓ 종료")
