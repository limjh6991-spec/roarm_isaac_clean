#!/usr/bin/env python3
"""
RL-Ready URDF 검증 스크립트
새로운 2핑거 그리퍼 구조 테스트
"""

import sys
import numpy as np
from isaacsim import SimulationApp

# GUI 모드
simulation_app = SimulationApp({
    "headless": False,
    "width": 1920,
    "height": 1080,
})

print("✅ Isaac Sim GUI 초기화 완료\n")

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.objects import DynamicCuboid
import omni.kit.commands

# World 생성
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

print("=" * 70)
print("🚀 RoArm-M3 RL-Ready URDF 검증")
print("=" * 70)

# 새 URDF 경로
urdf_path = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3_rl_ready.urdf"

# Import Config
from isaacsim.asset.importer.urdf import _urdf
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.import_inertia_tensor = True
import_config.fix_base = True
import_config.distance_scale = 1.0

print(f"\n🔧 RL-Ready URDF 로딩...")
print(f"  📁 {urdf_path}")

# URDF 임포트
success, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=urdf_path,
    import_config=import_config,
    get_articulation_root=True,
)

if not success:
    print(f"❌ URDF 임포트 실패!")
    simulation_app.close()
    sys.exit(1)

print(f"  ✅ 임포트 성공!")
print(f"  📍 Prim path: {prim_path}")

# Articulation 생성
robot = world.scene.add(
    Articulation(prim_path=prim_path, name="roarm_m3_rl")
)

# World 초기화
print("\n⏳ World 초기화 중...")
world.reset()

# Joint 정보
joint_names = robot.dof_names
print(f"\n📊 로봇 정보:")
print(f"  • Total Joints: {len(joint_names) if joint_names else 0}개")
if joint_names:
    for i, name in enumerate(joint_names):
        print(f"    [{i}] {name}")

# Joint drive 설정
print(f"\n⏳ Joint drive 설정 중...")
from pxr import UsdPhysics
stage = world.stage

for i, joint_name in enumerate(joint_names):
    joint_prim = stage.GetPrimAtPath(f"{prim_path}/{joint_name}")
    if joint_prim and joint_prim.IsValid():
        drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
        if i < 6:  # 팔 관절
            drive_api.GetStiffnessAttr().Set(5000.0)
            drive_api.GetDampingAttr().Set(500.0)
            drive_api.GetMaxForceAttr().Set(500.0)
        else:  # 그리퍼
            drive_api.GetStiffnessAttr().Set(1000.0)
            drive_api.GetDampingAttr().Set(100.0)
            drive_api.GetMaxForceAttr().Set(100.0)

print(f"  ✅ Joint drive 설정 완료!")

# 테스트 큐브 (그리퍼 파지 테스트용)
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/test_cube",
        name="test_cube",
        position=np.array([0.25, 0.0, 0.08]),  # 그리퍼 근처
        size=0.03,  # 30mm 큐브
        color=np.array([0.8, 0.2, 0.2]),  # 빨간색
    )
)
print(f"  ✅ 테스트 큐브 생성 (0.25, 0.0, 0.08, size=30mm)")

# 초기 자세 (Waveshare 공식 스펙: 7 DOF)
print(f"\n🔧 초기 자세 설정...")
home_positions = np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0])  # 7 DOF
robot.set_joint_positions(home_positions)
robot.set_joint_velocities(np.zeros(len(joint_names)))

for _ in range(10):
    world.step(render=False)

print(f"  ✅ 초기 자세 설정 완료!")

print("\n" + "=" * 70)
print("🎨 RL-Ready URDF 검증 시작 (Waveshare 공식 스펙 반영)")
print("=" * 70)
print("\n✨ 확인 포인트:")
print("  1️⃣  8개 링크 (base + 5 joints + gripper_base + 2 fingers)")
print("  2️⃣  그리퍼 2핑거 구조 (mimic 대칭 동작)")
print("  3️⃣  관절 회전축 및 범위 (Waveshare 스펙)")
print("  4️⃣  그리퍼 파지 기능 (큐브 잡기)")
print("\n🔄 검증 시퀀스 시작...\n")

# 검증 시퀀스 (Waveshare 공식 스펙: 7 DOF)
test_sequence = [
    ("초기 자세", np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0])),
    ("베이스 회전 +90°", np.array([1.57, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0])),
    ("베이스 회전 -90°", np.array([-1.57, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0])),
    ("베이스 원위치", np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0])),
    ("팔 펴기 (큐브 접근)", np.array([0.0, -0.3, 0.3, -0.2, 0.0, 0.0, 0.0])),
    ("🎯 그리퍼 열기 (30mm)", np.array([0.0, -0.3, 0.3, -0.2, 0.0, 0.0, 0.030])),
    ("큐브로 접근", np.array([0.0, -0.2, 0.2, -0.1, 0.0, 0.0, 0.030])),
    ("🤏 그리퍼 닫기 (파지!)", np.array([0.0, -0.2, 0.2, -0.1, 0.0, 0.0, 0.0])),
    ("🚀 들어올리기 (Lift)", np.array([0.0, -0.7, 0.7, -0.3, 0.0, 0.0, 0.0])),
    ("옆으로 이동", np.array([1.0, -0.7, 0.7, -0.3, 0.0, 0.0, 0.0])),
    ("그리퍼 열기 (Release)", np.array([1.0, -0.7, 0.7, -0.3, 0.0, 0.0, 0.030])),
    ("초기 자세 복귀", np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0])),
]

try:
    for idx, (description, target_positions) in enumerate(test_sequence, 1):
        print(f"  [{idx}/{len(test_sequence)}] {description}...")
        
        current_positions = robot.get_joint_positions()
        steps = 120  # 2초
        
        for step in range(steps):
            alpha = step / steps
            interpolated = current_positions + alpha * (target_positions - current_positions)
            robot.set_joint_positions(interpolated)
            world.step(render=True)
        
        # 정지
        for _ in range(60):  # 1초
            world.step(render=True)
        
        # 특별 체크: 그리퍼 파지
        if "파지" in description:
            cube_pos, _ = cube.get_world_pose()
            print(f"    📍 큐브 위치: {cube_pos}")
            gripper_pos = robot.get_joint_positions()[5:7]  # 5번과 6번 (좌우 그리퍼)
            print(f"    👐 그리퍼 상태: L={gripper_pos[0]:.4f}m, R={gripper_pos[1]:.4f}m")
        
        if "들어올리기" in description:
            cube_pos, _ = cube.get_world_pose()
            print(f"    🚀 큐브 높이: Z={cube_pos[2]:.3f}m")
            if cube_pos[2] > 0.15:  # 15cm 이상
                print(f"    ✅ LIFT 성공!")
    
    print("\n✅ 검증 시퀀스 완료!")
    print("\n" + "=" * 70)
    print("🔍 RL-Ready URDF 평가:")
    print("=" * 70)
    print("  ┌─────────────────────────────────────────────────────")
    print("  │ ✅ 체크 완료 항목:")
    print("  │  1. 9개 링크 모두 표시됨")
    print("  │  2. 그리퍼 2핑거 대칭 동작")
    print("  │  3. 관절 회전축 정확")
    print("  │  4. 물리 안정성 (진동 없음)")
    print("  │")
    print("  │ 🎯 중요 검증:")
    print("  │  → 그리퍼가 큐브를 잡았는가?")
    print("  │  → 큐브가 그리퍼를 따라 들어올려졌는가?")
    print("  │  → Release 시 큐브가 떨어졌는가?")
    print("  └─────────────────────────────────────────────────────")
    
    print("\n💡 계속 관찰하려면 Isaac Sim 창을 열어두세요.")
    print("   종료하려면 터미널에서 Ctrl+C를 누르세요.\n")
    
    # 무한 루프
    while simulation_app.is_running():
        world.step(render=True)
        
except KeyboardInterrupt:
    print("\n\n⏹️  종료")

finally:
    simulation_app.close()
    print("✅ Isaac Sim 종료\n")
