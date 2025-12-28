#!/usr/bin/env python3
"""
RoArm-M3 + D405 Camera URDF Test (Simple Version)
Isaac Sim 5.0 Built-in API만 사용
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
print("🎥 RoArm-M3 + D405 Camera URDF Test (Simple)")
print("=" * 80)

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import XFormPrim
import omni.kit.commands
from pxr import UsdGeom, Gf

# World 생성
world = World()
world.scene.add_default_ground_plane()

# URDF 경로
urdf_path = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3_with_d405.urdf"

print(f"\n📦 Loading URDF: {urdf_path}")

# Extension 활성화
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.asset.importer.urdf")

# URDF Import Config 설정
from isaacsim.asset.importer.urdf import _urdf
import_config = _urdf.ImportConfig()
import_config.set_fix_base(False)  # ⚠️ 중요: Floating base로 설정
import_config.set_make_default_prim(False)
import_config.set_create_physics_scene(False)

# URDF Import (2단계 방식)
try:
    # Step 1: Parse URDF
    success, robot_model = omni.kit.commands.execute(
        "URDFParseFile",
        urdf_path=urdf_path,
        import_config=import_config,
    )
    
    if not success:
        print("❌ URDF Parse 실패!")
        simulation_app.close()
        sys.exit(1)
    
    # Step 2: Import Robot into Scene
    success, prim_path = omni.kit.commands.execute(
        "URDFImportRobot",
        urdf_path=urdf_path,
        urdf_robot=robot_model,
        import_config=import_config,
    )
    
    if success:
        print(f"✅ URDF Imported: {prim_path}")
    else:
        print("❌ URDF Import 실패!")
        simulation_app.close()
        sys.exit(1)
        
except Exception as e:
    print(f"❌ URDF Import 에러: {e}")
    simulation_app.close()
    sys.exit(1)

# Stage 가져오기
from omni.isaac.core.utils.stage import get_current_stage
stage = get_current_stage()

# 카메라 프레임들 확인
print(f"\n📸 Checking Camera Frames...")
camera_frames = [
    "camera_link",
    "camera_depth_frame",
    "camera_depth_optical_frame",
    "camera_color_frame",
    "camera_color_optical_frame",
]

robot_path = prim_path
found_frames = []

for frame_name in camera_frames:
    # 여러 가능한 경로 시도
    possible_paths = [
        f"{robot_path}/{frame_name}",
        f"{robot_path}/camera_link/{frame_name}",
    ]
    
    for path in possible_paths:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            found_frames.append(path)
            print(f"   ✅ Found: {path}")
            break
    else:
        print(f"   ⚠️  Not found: {frame_name}")

# World 초기화
print(f"\n🌍 Initializing World...")
world.reset()

print(f"\n✅ Scene Ready!")
print(f"   Robot Path: {robot_path}")
print(f"   Found {len(found_frames)} camera frames")

# 로봇 Articulation 가져오기
print(f"\n🤖 Setting Robot Pose to 'ㄱ' shape...")
from isaacsim.core.api.articulations import Articulation

robot = Articulation(prim_path=robot_path)
world.scene.add(robot)

# Robot 초기화
robot.initialize()

# Robot이 바닥에 서도록 초기 위치 설정
# RoArm-M3의 베이스 높이를 고려하여 z=0.1m로 설정
default_root_state = robot.data.default_root_state.clone()
default_root_state[:, 2] = 0.1  # z 위치 설정
robot.set_world_pose(position=default_root_state[:, :3], orientation=default_root_state[:, 3:7])

# 'ㄱ'자 포즈 설정 (그리퍼와 카메라가 정면을 향하도록)
# joint 순서: world_to_base, joint1(base회전), joint2(어깨), joint3(팔꿈치), joint4(손목1), joint5(손목2), gripper
target_positions = np.array([
    0.0,      # world_to_base: 고정
    0.0,      # joint1: 정면 향함
    -1.57,    # joint2: 어깨를 앞으로 90도 (수평)
    1.57,     # joint3: 팔꿈치를 위로 90도 (수직) → 'ㄱ'자 형태
    0.0,      # joint4: 손목 중립
    0.0,      # joint5: 손목 중립
    0.01,     # gripper: 약간 열림
])

print(f"   Target Joint Positions:")
joint_names = ["world_to_base", "joint1", "joint2", "joint3", "joint4", "joint5", "gripper"]
for name, pos in zip(joint_names, target_positions):
    print(f"      {name}: {pos:.3f} rad ({np.degrees(pos):.1f}°)")

robot.set_joint_positions(target_positions)

# World reset으로 물리 시뮬레이션 초기화
world.reset()

# 포즈가 적용되도록 몇 프레임 시뮬레이션
print(f"\n   Applying pose...")
for _ in range(100):  # 더 많은 프레임으로 안정화
    world.step(render=True)

print(f"   ✅ Robot pose set to 'ㄱ' shape!")

# Simulation loop
print(f"\n▶️  Running Simulation...")
print(f"   Press Ctrl+C or close window to exit")

frame_count = 0

try:
    while simulation_app.is_running():
        # Step simulation
        world.step(render=True)
        frame_count += 1
        
        # Print info every 100 frames
        if frame_count % 100 == 0:
            print(f"   Frame {frame_count}")
            
            # Get camera_link position if found
            for frame_path in found_frames:
                if "camera_link" in frame_path:
                    prim = stage.GetPrimAtPath(frame_path)
                    if prim.IsValid():
                        xform = UsdGeom.Xformable(prim)
                        world_transform = xform.ComputeLocalToWorldTransform(0)
                        translation = world_transform.ExtractTranslation()
                        print(f"      Camera Position: ({translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f})")
                    break

except KeyboardInterrupt:
    print("\n\n⚠️  Interrupted by user")

finally:
    simulation_app.close()

print(f"\n✅ Test Complete!")
print(f"   Total Frames: {frame_count}")
