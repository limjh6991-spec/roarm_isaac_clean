#!/usr/bin/env python3
"""
RoArm-M3 + D405 Camera RGB-D 캡처 테스트
Isaac Sim Camera Sensor를 추가하고 RGB + Depth 데이터 캡처
"""

import sys
import numpy as np
from pathlib import Path

# Isaac Sim 초기화
from isaacsim import SimulationApp
simulation_app = SimulationApp({
    "headless": False,
    "width": 1920,
    "height": 1080,
})

print("=" * 80)
print("📸 RoArm-M3 + D405 Camera RGB-D Capture Test")
print("=" * 80)

from isaacsim.core.api import World
from isaacsim.core.api.articulations import Articulation
# TODO_5.1: Use isaacsim.core.api.prims or USD prims directly
# from omni.isaac.core.objects import DynamicCuboid
from isaacsim.sensors.camera import Camera
import omni.kit.commands
import cv2
import torch

# World 생성
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# URDF 경로
urdf_path = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3_with_d405.urdf"

print(f"\n📦 Loading URDF: {urdf_path}")

# URDF Import
success, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=urdf_path,
    import_config=omni.isaac.core.utils.extensions.get_extension_path_from_name(
        "omni.importer.urdf"
    ),
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

# 테스트용 물체 추가 (카메라가 볼 수 있도록)
print("\n📦 Adding Test Object...")
test_object = DynamicCuboid(
    prim_path="/World/test_cube",
    name="test_cube",
    position=np.array([0.3, 0.0, 0.1]),  # 로봇 앞쪽
    size=0.05,
    color=np.array([1.0, 0.0, 0.0]),  # Red
)
world.scene.add(test_object)

# Camera Sensor 추가
print("\n🎥 Adding Camera Sensor...")
camera_prim_path = f"{robot_prim_path}/camera_link/Camera"

camera = Camera(
    prim_path=camera_prim_path,
    frequency=20,  # 20 FPS
    resolution=(256, 256),  # 학습용은 나중에 84x84로 resize
    projection_type="pinhole",
)

# World 초기화
print("\n🌍 Initializing World...")
world.reset()
camera.initialize()

print(f"✅ Camera Initialized")
print(f"   Resolution: {camera.get_resolution()}")
print(f"   Position: {camera.get_world_pose()[0]}")

# Home position
home_positions = np.array([0.0, 0.5, -0.5, 0.0, 0.0, 0.0, 0.01])
robot.set_joint_positions(home_positions)

# RGB-D 캡처 함수
def capture_rgbd():
    """RGB + Depth 데이터 캡처"""
    # RGB (H, W, 4) → (H, W, 3)
    rgba = camera.get_rgba()
    if isinstance(rgba, torch.Tensor):
        rgba = rgba.cpu().numpy()
    rgb = rgba[:, :, :3]
    
    # Depth (H, W)
    depth = camera.get_depth()
    if isinstance(depth, torch.Tensor):
        depth = depth.cpu().numpy()
    
    # Normalize depth for visualization (0-1)
    depth_vis = np.clip(depth, 0.1, 2.0)  # 10cm ~ 2m
    depth_vis = (depth_vis - 0.1) / 1.9
    depth_vis = (depth_vis * 255).astype(np.uint8)
    
    return rgb, depth, depth_vis

# 이미지 저장 디렉토리
output_dir = Path("/home/roarm_m3/roarm_isaac_clean/logs/camera_test")
output_dir.mkdir(parents=True, exist_ok=True)

print(f"\n💾 Output Directory: {output_dir}")
print("\n▶️  Starting Simulation...")
print("   - 's' 키: RGB-D 이미지 캡처 및 저장")
print("   - 'q' 키 또는 창 닫기: 종료")
print("   - 로봇과 빨간 큐브가 보이는지 확인하세요")

frame_count = 0
capture_count = 0

# 키보드 입력 처리를 위한 설정
from pynput import keyboard

capture_flag = False
quit_flag = False

def on_press(key):
    global capture_flag, quit_flag
    try:
        if key.char == 's':
            capture_flag = True
        elif key.char == 'q':
            quit_flag = True
    except AttributeError:
        pass

listener = keyboard.Listener(on_press=on_press)
listener.start()

try:
    while simulation_app.is_running() and not quit_flag:
        world.step(render=True)
        frame_count += 1
        
        # 10프레임마다 카메라 업데이트
        if frame_count % 10 == 0:
            camera.update()
        
        # 캡처 플래그 확인
        if capture_flag:
            print(f"\n📸 Capturing RGB-D image...")
            
            # RGB-D 캡처
            rgb, depth, depth_vis = capture_rgbd()
            
            # BGR로 변환 (OpenCV)
            rgb_bgr = cv2.cvtColor((rgb * 255).astype(np.uint8), cv2.COLOR_RGB2BGR)
            
            # 저장
            capture_count += 1
            rgb_path = output_dir / f"rgb_{capture_count:03d}.png"
            depth_path = output_dir / f"depth_{capture_count:03d}.png"
            depth_raw_path = output_dir / f"depth_raw_{capture_count:03d}.npy"
            
            cv2.imwrite(str(rgb_path), rgb_bgr)
            cv2.imwrite(str(depth_path), depth_vis)
            np.save(str(depth_raw_path), depth)
            
            print(f"   ✅ Saved:")
            print(f"      RGB: {rgb_path}")
            print(f"      Depth (vis): {depth_path}")
            print(f"      Depth (raw): {depth_raw_path}")
            print(f"   RGB shape: {rgb.shape}, min: {rgb.min():.3f}, max: {rgb.max():.3f}")
            print(f"   Depth shape: {depth.shape}, min: {depth.min():.3f}, max: {depth.max():.3f}")
            
            capture_flag = False
        
        # 100프레임마다 상태 출력
        if frame_count % 100 == 0:
            print(f"   Frame {frame_count} | Captured: {capture_count} images")

except KeyboardInterrupt:
    print("\n\n⚠️  Interrupted by user")

finally:
    listener.stop()
    simulation_app.close()

print(f"\n✅ Test Complete!")
print(f"   Total Captures: {capture_count}")
print(f"   Output Directory: {output_dir}")
