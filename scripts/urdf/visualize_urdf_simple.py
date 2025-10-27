#!/usr/bin/env python3
"""
URDF 시각화 간단 스크립트 (학습 없이 로봇 형상만 확인)
"""

import sys
import numpy as np
from isaacsim import SimulationApp

# GUI 모드로 Isaac Sim 초기화
simulation_app = SimulationApp({
    "headless": False,
    "width": 1920,
    "height": 1080,
})

print("✅ Isaac Sim GUI 모드 초기화 완료\n")

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.objects import DynamicCuboid
import omni.kit.commands

# World 생성
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

print("=" * 70)
print("🤖 RoArm-M3 URDF 시각화")
print("=" * 70)

# URDF 경로
urdf_path = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3_multiprim.urdf"

# URDF Import Config
from isaacsim.asset.importer.urdf import _urdf
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.import_inertia_tensor = True
import_config.fix_base = True
import_config.distance_scale = 1.0

print(f"\n🔧 URDF 로딩: {urdf_path}")
print("  ⏳ 파싱 중...")

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
    Articulation(prim_path=prim_path, name="roarm_m3")
)

# World 초기화
print("\n⏳ World 초기화 중...")
world.reset()

# Joint 정보 출력
joint_names = robot.dof_names
print(f"\n📊 로봇 정보:")
print(f"  • Joints: {len(joint_names) if joint_names else 0}개")
if joint_names:
    for i, name in enumerate(joint_names):
        print(f"    [{i}] {name}")

# Joint drive 설정
print(f"\n⏳ Joint drive 설정 중...")
from pxr import UsdPhysics, PhysxSchema
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

# 테스트용 큐브 추가
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/test_cube",
        name="test_cube",
        position=np.array([0.3, 0.0, 0.05]),
        size=0.04,
        color=np.array([0.8, 0.2, 0.2]),  # 빨간색
    )
)

print(f"  ✅ 테스트 큐브 생성 (0.3, 0.0, 0.05)")

# 초기 자세 설정 (Home position)
print(f"\n🔧 초기 자세 설정...")
home_positions = np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0])
robot.set_joint_positions(home_positions)
robot.set_joint_velocities(np.zeros(8))

# 안정화
for _ in range(10):
    world.step(render=False)

print(f"  ✅ 초기 자세 설정 완료!")

print("\n" + "=" * 70)
print("🎨 시각화 시작!")
print("=" * 70)
print("\n✨ 확인 포인트:")
print("  1️⃣  로봇 팔 구조 (8개 링크 + 관절)")
print("  2️⃣  그리퍼 형상 (2개 핑거)")
print("  3️⃣  관절 연결 상태 (겹침/간격)")
print("  4️⃣  전체적인 비율과 크기")
print("\n📌 조작:")
print("  • 마우스 드래그: 뷰 회전")
print("  • 마우스 휠: 줌 인/아웃")
print("  • 중간 마우스 버튼: 팬")
print("\n🔄 간단한 동작 테스트 시작 (천천히 움직임)...")
print()

# 간단한 동작 시퀀스
motion_sequence = [
    # (설명, joint_positions)
    ("초기 자세", np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0])),
    ("베이스 회전 +90°", np.array([1.57, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0])),
    ("베이스 회전 -90°", np.array([-1.57, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0])),
    ("베이스 원위치", np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0])),
    ("팔 펴기", np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])),
    ("팔 굽히기", np.array([0.0, -1.0, 1.0, -0.5, 0.0, 0.0, 0.0, 0.0])),
    ("손목 회전", np.array([0.0, -0.5, 0.5, 0.0, 1.57, 0.0, 0.0, 0.0])),
    ("그리퍼 열기", np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.02, 0.02])),
    ("그리퍼 닫기", np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0])),
    ("초기 자세 복귀", np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0])),
]

try:
    for idx, (description, target_positions) in enumerate(motion_sequence, 1):
        print(f"  [{idx}/{len(motion_sequence)}] {description}...")
        
        # 현재 위치에서 목표 위치로 부드럽게 이동
        current_positions = robot.get_joint_positions()
        steps = 120  # 2초 (60 FPS 가정)
        
        for step in range(steps):
            alpha = step / steps
            interpolated = current_positions + alpha * (target_positions - current_positions)
            robot.set_joint_positions(interpolated)
            world.step(render=True)
        
        # 목표 위치에서 잠시 정지
        for _ in range(60):  # 1초 정지
            world.step(render=True)
    
    print("\n✅ 동작 테스트 완료!")
    print("\n🔍 URDF 상태 평가:")
    print("  ┌─────────────────────────────────────────────────────")
    print("  │ 확인 항목:")
    print("  │  ✓ 로봇 팔이 자연스럽게 움직이는가?")
    print("  │  ✓ 관절이 겹치거나 튀는 현상이 없는가?")
    print("  │  ✓ 그리퍼가 제대로 열고 닫히는가?")
    print("  │  ✓ 전체적인 비율이 현실적인가?")
    print("  └─────────────────────────────────────────────────────")
    
    print("\n💡 계속 관찰하려면 Isaac Sim 창을 열어두세요.")
    print("   종료하려면 터미널에서 Ctrl+C를 누르세요.\n")
    
    # 무한 루프로 시뮬레이션 유지
    while simulation_app.is_running():
        world.step(render=True)
        
except KeyboardInterrupt:
    print("\n\n⏹️  사용자가 종료했습니다.")

finally:
    simulation_app.close()
    print("✅ Isaac Sim 종료 완료\n")
