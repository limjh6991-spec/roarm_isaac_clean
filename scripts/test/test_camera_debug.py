#!/usr/bin/env python3
"""
카메라 디버그 테스트
- 현재 카메라가 무엇을 보고 있는지 이미지 저장
- 다양한 카메라 회전 각도 테스트
- 큐브가 보이는 최적 카메라 설정 찾기

사용법:
    /home/roarm_m3/isaacsim/python.sh scripts/test/test_camera_debug.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import os

# Isaac Sim imports
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf, UsdLux, PhysxSchema
import omni.replicator.core as rep

# Project imports
import sys
sys.path.insert(0, "/home/roarm_m3/roarm_isaac_clean")
from envs.scene.scene_builder import create_dynamic_cuboid, create_physics_material


def detect_red_in_image(rgb_data):
    """이미지에서 빨간색 픽셀 비율 계산"""
    if rgb_data is None or len(rgb_data.shape) != 3:
        return 0.0, None
    
    rgb = rgb_data[:, :, :3]
    
    # 빨간색 검출 (R > 150, G < 100, B < 100)
    red_mask = (rgb[:, :, 0] > 150) & (rgb[:, :, 1] < 100) & (rgb[:, :, 2] < 100)
    red_ratio = np.sum(red_mask) / (rgb.shape[0] * rgb.shape[1])
    
    # 빨간색 중심 계산
    if np.sum(red_mask) > 0:
        y_coords, x_coords = np.where(red_mask)
        center = (np.mean(x_coords), np.mean(y_coords))
    else:
        center = None
    
    return red_ratio * 100, center


def save_image(rgb_data, filename, output_dir):
    """이미지 저장"""
    try:
        from PIL import Image
        if rgb_data is not None and len(rgb_data.shape) == 3:
            img = Image.fromarray(rgb_data[:, :, :3].astype(np.uint8))
            filepath = os.path.join(output_dir, filename)
            img.save(filepath)
            print(f"   💾 Saved: {filename}")
            return True
    except Exception as e:
        print(f"   ⚠️ Save failed: {e}")
    return False


def setup_scene():
    """씬 구성"""
    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    
    # Ground
    world.scene.add_default_ground_plane()
    
    # Light
    light_prim = stage.DefinePrim("/World/DomeLight", "DomeLight")
    light = UsdLux.DomeLight(light_prim)
    light.GetIntensityAttr().Set(1500.0)
    
    # Robot
    usd_path = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/usd/roarm_m3_with_camera_correct.usd"
    add_reference_to_stage(usd_path=usd_path, prim_path="/World/RoArm")
    
    # Physics material
    create_physics_material(stage)
    
    # Cube - 로봇에서 가까운 위치로 변경 (테스트용)
    cube_pos = (0.20, 0.0, 0.05)  # x=20cm (더 가깝게)
    create_dynamic_cuboid(
        stage=stage,
        prim_path="/World/Cube",
        position=cube_pos,
        size=0.05,  # 5cm cube (더 크게)
        color=(0.9, 0.1, 0.1),  # Bright red
        mass=0.1
    )
    print(f"✅ Cube at {cube_pos}")
    
    # Disable USD drives
    joint_names = ['base_link_to_link1', 'link1_to_link2', 'link2_to_link3',
                   'link3_to_link4', 'link4_to_link5', 'link5_to_gripper_link']
    for joint_name in joint_names:
        joint_path = f"/World/RoArm/joints/{joint_name}"
        joint_prim = stage.GetPrimAtPath(joint_path)
        if joint_prim.IsValid():
            drive_api = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
            if drive_api:
                drive_api.GetStiffnessAttr().Set(0.0)
                drive_api.GetDampingAttr().Set(0.0)
    
    # Disable gravity for robot links
    link_names = ['base_link', 'link1', 'link2', 'link3', 'link4', 
                  'link5', 'gripper_link', 'camera_link']
    for link_name in link_names:
        link_path = f"/World/RoArm/{link_name}"
        link_prim = stage.GetPrimAtPath(link_path)
        if link_prim.IsValid():
            physx_api = PhysxSchema.PhysxRigidBodyAPI.Apply(link_prim)
            physx_api.CreateDisableGravityAttr(True)
    
    world.reset()
    
    # Robot articulation
    robot = SingleArticulation(prim_path="/World/RoArm", name="roarm")
    world.scene.add(robot)
    world.reset()
    
    print(f"✅ Robot DOF: {robot.num_dof}")
    
    return world, robot, stage


def create_camera_with_rotation(stage, rotation_xyz, translate_z=0.06):
    """특정 회전 각도로 카메라 생성"""
    camera_parent_path = "/World/RoArm/gripper_link"
    camera_prim_path = f"{camera_parent_path}/hand_camera"
    
    # Remove existing camera
    existing = stage.GetPrimAtPath(camera_prim_path)
    if existing.IsValid():
        stage.RemovePrim(camera_prim_path)
    
    # Create new camera
    camera_prim = stage.DefinePrim(camera_prim_path, "Camera")
    camera = UsdGeom.Camera(camera_prim)
    
    camera.GetFocalLengthAttr().Set(18.0)
    camera.GetHorizontalApertureAttr().Set(20.955)
    camera.GetVerticalApertureAttr().Set(20.955)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10.0))
    
    # Apply transforms
    xformable = UsdGeom.Xformable(camera_prim)
    xformable.ClearXformOpOrder()
    
    # Translate
    translate_op = xformable.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(0.0, 0.0, translate_z))
    
    # Rotate
    rotate_op = xformable.AddRotateXYZOp()
    rotate_op.Set(Gf.Vec3f(*rotation_xyz))
    
    # Render product
    render_product = rep.create.render_product(camera_prim_path, (128, 128))  # 더 큰 해상도
    rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annot.attach([render_product])
    
    return rgb_annot, camera_prim_path


def main():
    print("\n" + "="*70)
    print("📷 카메라 디버그 테스트")
    print("="*70)
    
    # Output directory
    output_dir = "/home/roarm_m3/roarm_isaac_clean/resources/camera_debug"
    os.makedirs(output_dir, exist_ok=True)
    
    # Setup
    world, robot, stage = setup_scene()
    
    # 올바른 home position (link2~gripper 수평)
    # 참고: 큐브가 x=0.20에 있고 로봇이 그 방향을 향해야 함
    home_position = np.array([
        0.0,    # j0: base - 정면
        1.2,    # j1: shoulder - 앞으로 많이 기울임
        -1.2,   # j2: elbow - 위로 구부림 (수평 유지)
        0.0,    # j3: wrist1 
        0.0,    # j4: wrist2
        0.06    # j5: gripper open
    ], dtype=np.float32)
    
    # Set position
    robot.set_joint_positions(home_position)
    robot.set_joint_velocities(np.zeros(6))
    
    # Stabilize
    for _ in range(100):
        robot.set_joint_positions(home_position)
        robot.set_joint_velocities(np.zeros(6))
        world.step(render=True)
    
    print(f"\n📐 Robot Position: {np.round(home_position, 2)}")
    
    # ==================== 카메라 회전 테스트 ====================
    print("\n" + "="*70)
    print("🔄 카메라 회전 각도 테스트")
    print("="*70)
    
    # 테스트할 회전 조합
    # USD Camera는 기본적으로 -Z 방향을 봄
    # X 회전: pitch (위/아래)
    # Y 회전: yaw (좌/우)
    # Z 회전: roll
    
    rotation_tests = [
        # (name, (rotX, rotY, rotZ), description)
        ("default", (0, 0, 0), "기본 (-Z 방향)"),
        ("look_down_90", (-90, 0, 0), "X=-90 (아래)"),
        ("look_down_180Y", (-90, 180, 0), "X=-90, Y=180"),
        ("look_forward", (0, 180, 0), "Y=180 (앞)"),
        ("tilt_forward", (-45, 0, 0), "X=-45 (45도 아래)"),
        ("tilt_forward_flip", (-45, 180, 0), "X=-45, Y=180"),
        ("look_back", (0, 0, 0), "기본"),
        ("z_offset_down", (-90, 0, 180), "X=-90, Z=180"),
        ("all_180", (180, 0, 0), "X=180 (위/아래 반전)"),
        ("pitch_down_yaw", (-90, 90, 0), "X=-90, Y=90"),
    ]
    
    best_result = None
    best_red_ratio = 0
    
    for name, rotation, desc in rotation_tests:
        print(f"\n--- {name}: {rotation} ({desc}) ---")
        
        # Create camera with this rotation
        rgb_annot, camera_path = create_camera_with_rotation(stage, rotation)
        
        # Wait for rendering
        for _ in range(20):
            world.step(render=True)
        rep.orchestrator.step(rt_subframes=8)
        
        # Get image
        rgb_data = rgb_annot.get_data()
        red_ratio, center = detect_red_in_image(rgb_data)
        
        status = "🟢" if red_ratio > 1.0 else ("🟡" if red_ratio > 0.1 else "⚫")
        print(f"   {status} Red: {red_ratio:.2f}%", end="")
        if center:
            print(f" @ ({center[0]:.1f}, {center[1]:.1f})")
        else:
            print()
        
        # Save image
        filename = f"{name}_r{rotation[0]:.0f}_{rotation[1]:.0f}_{rotation[2]:.0f}_red{red_ratio:.1f}.png"
        save_image(rgb_data, filename, output_dir)
        
        # Track best
        if red_ratio > best_red_ratio:
            best_red_ratio = red_ratio
            best_result = (name, rotation, red_ratio, center)
    
    # ==================== 위치 오프셋 테스트 ====================
    print("\n" + "="*70)
    print("📍 카메라 위치 오프셋 테스트 (X=-90, Y=180)")
    print("="*70)
    
    base_rotation = (-90, 180, 0)
    
    # 그리퍼 앞쪽으로 오프셋 테스트
    offset_tests = [
        ("front_5cm", (0.05, 0, 0.06)),
        ("front_10cm", (0.10, 0, 0.06)),
        ("down_10cm", (0, 0, 0.10)),
        ("front_down", (0.05, 0, 0.10)),
    ]
    
    for name, offset in offset_tests:
        print(f"\n--- Offset {name}: {offset} ---")
        
        # Create camera with offset
        camera_parent_path = "/World/RoArm/gripper_link"
        camera_prim_path = f"{camera_parent_path}/hand_camera"
        
        existing = stage.GetPrimAtPath(camera_prim_path)
        if existing.IsValid():
            stage.RemovePrim(camera_prim_path)
        
        camera_prim = stage.DefinePrim(camera_prim_path, "Camera")
        camera = UsdGeom.Camera(camera_prim)
        camera.GetFocalLengthAttr().Set(18.0)
        camera.GetHorizontalApertureAttr().Set(20.955)
        camera.GetVerticalApertureAttr().Set(20.955)
        camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10.0))
        
        xformable = UsdGeom.Xformable(camera_prim)
        xformable.ClearXformOpOrder()
        translate_op = xformable.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(*offset))
        rotate_op = xformable.AddRotateXYZOp()
        rotate_op.Set(Gf.Vec3f(*base_rotation))
        
        render_product = rep.create.render_product(camera_prim_path, (128, 128))
        rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb_annot.attach([render_product])
        
        for _ in range(20):
            world.step(render=True)
        rep.orchestrator.step(rt_subframes=8)
        
        rgb_data = rgb_annot.get_data()
        red_ratio, center = detect_red_in_image(rgb_data)
        
        status = "🟢" if red_ratio > 1.0 else ("🟡" if red_ratio > 0.1 else "⚫")
        print(f"   {status} Red: {red_ratio:.2f}%")
        
        filename = f"offset_{name}_red{red_ratio:.1f}.png"
        save_image(rgb_data, filename, output_dir)
        
        if red_ratio > best_red_ratio:
            best_red_ratio = red_ratio
            best_result = (f"offset_{name}", base_rotation, red_ratio, center)
    
    # ==================== 결과 요약 ====================
    print("\n" + "="*70)
    print("📊 결과 요약")
    print("="*70)
    
    if best_result and best_red_ratio > 0:
        print(f"\n✅ 최적 설정 발견!")
        print(f"   Name: {best_result[0]}")
        print(f"   Rotation: {best_result[1]}")
        print(f"   Red ratio: {best_result[2]:.2f}%")
    else:
        print("\n❌ 큐브가 보이는 설정을 찾지 못했습니다.")
        print("   가능한 원인:")
        print("   1. 카메라가 그리퍼 링크의 반대 방향을 보고 있음")
        print("   2. 로봇 자세가 큐브 방향이 아님")
        print("   3. 큐브가 카메라 시야 밖에 있음")
    
    print(f"\n📁 이미지 저장 위치: {output_dir}")
    print("   → 이미지를 확인하여 카메라가 무엇을 보고 있는지 파악하세요")
    
    # Keep running
    print("\n🎮 GUI 확인 중... (Ctrl+C로 종료)")
    try:
        while True:
            world.step(render=True)
    except KeyboardInterrupt:
        print("\n⏹️ 종료")
    
    world.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
