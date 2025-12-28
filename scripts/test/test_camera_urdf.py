#!/usr/bin/env python3
"""
RoArm-M3 + D405 Camera URDF 테스트
Isaac Sim에서 카메라가 올바르게 마운트되었는지 확인
"""

import sys
from pathlib import Path

# Isaac Sim 초기화
from isaacsim import SimulationApp
simulation_app = SimulationApp({
    "headless": False,
    "width": 1920,
    "height": 1080,
})

print("=" * 80)
print("🎥 RoArm-M3 + D405 Camera URDF Test")
print("=" * 80)

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.articulations import Articulation
import omni.kit.commands

# World 생성
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# URDF 경로
urdf_path = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3_with_d405.urdf"

print(f"\n📦 Loading URDF: {urdf_path}")

# URDF Import (Isaac Sim 5.0 최소 파라미터)
success, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=urdf_path,
)

if not success:
    print("❌ URDF Import 실패!")
    simulation_app.close()
    sys.exit(1)

print(f"✅ URDF Imported: {prim_path}")

# Robot Articulation
robot_prim_path = "/World/roarm_m3_with_d405"
robot = Articulation(prim_path=robot_prim_path)
world.scene.add(robot)

# World 초기화
print("\n🌍 Initializing World...")
world.reset()

# Joint 이름 확인
joint_names = robot.dof_names
print(f"\n🔗 Robot Joints ({len(joint_names)}):")
for i, name in enumerate(joint_names):
    print(f"   {i}: {name}")

# 카메라 프레임 확인
print("\n📸 Camera Frames:")
camera_frames = [
    f"{robot_prim_path}/camera_link",
    f"{robot_prim_path}/camera_depth_frame",
    f"{robot_prim_path}/camera_depth_optical_frame",
    f"{robot_prim_path}/camera_color_frame",
    f"{robot_prim_path}/camera_color_optical_frame",
]

from pxr import UsdGeom
stage = world.stage

for frame_path in camera_frames:
    prim = stage.GetPrimAtPath(frame_path)
    if prim.IsValid():
        print(f"   ✅ {frame_path.split('/')[-1]}")
        # Get transform
        xform = UsdGeom.Xformable(prim)
        transform_matrix = xform.ComputeLocalToWorldTransform(0)
        translation = transform_matrix.ExtractTranslation()
        print(f"      Position: ({translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f})")
    else:
        print(f"   ❌ {frame_path.split('/')[-1]} - Not Found")

# 테스트 동작: 로봇을 움직여서 카메라가 따라가는지 확인
print("\n🤖 Testing Robot Motion...")
print("   Moving robot to test camera attachment...")

# Home position으로 이동
home_positions = np.array([0.0, 0.5, -0.5, 0.0, 0.0, 0.0, 0.01])
robot.set_joint_positions(home_positions)

# 시뮬레이션 실행
print("\n▶️  Starting Simulation...")
print("   - 로봇이 홈 포지션으로 이동합니다")
print("   - 카메라(검정 박스)가 그리퍼에 부착되어 움직이는지 확인하세요")
print("   - 카메라는 약간 아래를 향해야 합니다 (30도)")
print("\n   창을 닫으면 종료됩니다.")

frame_count = 0
while simulation_app.is_running():
    world.step(render=True)
    
    frame_count += 1
    
    # 100 프레임마다 카메라 위치 출력
    if frame_count % 100 == 0:
        camera_prim = stage.GetPrimAtPath(f"{robot_prim_path}/camera_link")
        if camera_prim.IsValid():
            xform = UsdGeom.Xformable(camera_prim)
            transform_matrix = xform.ComputeLocalToWorldTransform(0)
            translation = transform_matrix.ExtractTranslation()
            print(f"   Frame {frame_count}: Camera at ({translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f})")

simulation_app.close()
print("\n✅ Test Complete!")
