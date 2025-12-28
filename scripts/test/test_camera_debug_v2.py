#!/usr/bin/env python3
"""
카메라 위치 디버그 v2
- 고정 씬 카메라로 큐브 확인
- 그리퍼 링크 좌표계 분석
- 카메라 시야 계산

사용법:
    /home/roarm_m3/isaacsim/python.sh scripts/test/test_camera_debug_v2.py
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
from pxr import UsdGeom, UsdPhysics, Gf, UsdLux, PhysxSchema, Usd
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
    red_mask = (rgb[:, :, 0] > 150) & (rgb[:, :, 1] < 100) & (rgb[:, :, 2] < 100)
    red_ratio = np.sum(red_mask) / (rgb.shape[0] * rgb.shape[1])
    
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


def get_world_transform(stage, prim_path):
    """Get world transform of a prim"""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None, None
    
    xformable = UsdGeom.Xformable(prim)
    world_matrix = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    
    translation = world_matrix.ExtractTranslation()
    # Rotation matrix (3x3)
    rot_matrix = Gf.Matrix3d(
        world_matrix[0][0], world_matrix[0][1], world_matrix[0][2],
        world_matrix[1][0], world_matrix[1][1], world_matrix[1][2],
        world_matrix[2][0], world_matrix[2][1], world_matrix[2][2]
    )
    
    return np.array([translation[0], translation[1], translation[2]]), rot_matrix


def setup_scene():
    """씬 구성"""
    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    
    world.scene.add_default_ground_plane()
    
    light_prim = stage.DefinePrim("/World/DomeLight", "DomeLight")
    light = UsdLux.DomeLight(light_prim)
    light.GetIntensityAttr().Set(1500.0)
    
    usd_path = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/usd/roarm_m3_with_camera_correct.usd"
    add_reference_to_stage(usd_path=usd_path, prim_path="/World/RoArm")
    
    create_physics_material(stage)
    
    # 큐브 - 로봇 앞 20cm
    cube_pos = (0.20, 0.0, 0.05)
    create_dynamic_cuboid(
        stage=stage,
        prim_path="/World/Cube",
        position=cube_pos,
        size=0.05,
        color=(0.9, 0.1, 0.1),
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
    
    # Disable gravity
    link_names = ['base_link', 'link1', 'link2', 'link3', 'link4', 
                  'link5', 'gripper_link', 'camera_link']
    for link_name in link_names:
        link_path = f"/World/RoArm/{link_name}"
        link_prim = stage.GetPrimAtPath(link_path)
        if link_prim.IsValid():
            physx_api = PhysxSchema.PhysxRigidBodyAPI.Apply(link_prim)
            physx_api.CreateDisableGravityAttr(True)
    
    world.reset()
    
    robot = SingleArticulation(prim_path="/World/RoArm", name="roarm")
    world.scene.add(robot)
    world.reset()
    
    print(f"✅ Robot DOF: {robot.num_dof}")
    
    return world, robot, stage


def create_fixed_camera(stage, position, look_at, name):
    """고정 씬 카메라 생성"""
    camera_path = f"/World/{name}"
    camera_prim = stage.DefinePrim(camera_path, "Camera")
    camera = UsdGeom.Camera(camera_prim)
    
    camera.GetFocalLengthAttr().Set(35.0)
    camera.GetHorizontalApertureAttr().Set(20.955)
    camera.GetVerticalApertureAttr().Set(20.955)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))
    
    # Calculate look-at transform
    pos = Gf.Vec3d(*position)
    target = Gf.Vec3d(*look_at)
    up = Gf.Vec3d(0, 0, 1)
    
    direction = target - pos
    direction = direction.GetNormalized()
    
    xformable = UsdGeom.Xformable(camera_prim)
    xformable.ClearXformOpOrder()
    
    # Set position
    translate_op = xformable.AddTranslateOp()
    translate_op.Set(pos)
    
    # Calculate rotation to look at target
    # Camera looks -Z by default, so we need to rotate
    from_vec = Gf.Vec3d(0, 0, -1)  # Default camera direction
    
    # Simple approach: use euler angles
    # yaw = atan2(dir.y, dir.x)
    # pitch = atan2(-dir.z, sqrt(dir.x^2 + dir.y^2))
    import math
    yaw = math.degrees(math.atan2(direction[1], direction[0]))
    pitch = math.degrees(math.atan2(-direction[2], math.sqrt(direction[0]**2 + direction[1]**2)))
    
    rotate_op = xformable.AddRotateXYZOp()
    rotate_op.Set(Gf.Vec3f(pitch + 90, 0, yaw + 90))  # Adjust for camera convention
    
    render_product = rep.create.render_product(camera_path, (256, 256))
    rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annot.attach([render_product])
    
    return rgb_annot, camera_path


def main():
    print("\n" + "="*70)
    print("📷 카메라 디버그 v2 - 고정 카메라 + 좌표계 분석")
    print("="*70)
    
    output_dir = "/home/roarm_m3/roarm_isaac_clean/resources/camera_debug_v2"
    os.makedirs(output_dir, exist_ok=True)
    
    world, robot, stage = setup_scene()
    
    # 수평 자세 (link2~gripper가 수평)
    home_position = np.array([
        0.0,    # j0: base
        1.0,    # j1: shoulder - 앞으로 기울임
        -1.0,   # j2: elbow
        0.0,    # j3: wrist1 
        0.0,    # j4: wrist2
        0.06    # j5: gripper
    ], dtype=np.float32)
    
    robot.set_joint_positions(home_position)
    robot.set_joint_velocities(np.zeros(6))
    
    for _ in range(100):
        robot.set_joint_positions(home_position)
        robot.set_joint_velocities(np.zeros(6))
        world.step(render=True)
    
    print(f"\n📐 Robot Position: {home_position}")
    
    # ==================== 좌표계 분석 ====================
    print("\n" + "="*70)
    print("🔍 그리퍼 좌표계 분석")
    print("="*70)
    
    gripper_pos, gripper_rot = get_world_transform(stage, "/World/RoArm/gripper_link")
    cube_pos, _ = get_world_transform(stage, "/World/Cube")
    
    if gripper_pos is not None:
        print(f"\n📍 Gripper World Position: {np.round(gripper_pos, 3)}")
        print(f"📍 Cube World Position: {np.round(cube_pos, 3)}")
        
        # 그리퍼에서 큐브까지 벡터
        direction_to_cube = cube_pos - gripper_pos
        distance = np.linalg.norm(direction_to_cube)
        direction_normalized = direction_to_cube / distance
        
        print(f"\n📐 Direction to Cube: {np.round(direction_normalized, 3)}")
        print(f"📐 Distance: {distance:.3f}m")
        
        # 그리퍼 로컬 축 (월드 좌표계로 변환)
        if gripper_rot:
            local_x = np.array([gripper_rot[0][0], gripper_rot[1][0], gripper_rot[2][0]])
            local_y = np.array([gripper_rot[0][1], gripper_rot[1][1], gripper_rot[2][1]])
            local_z = np.array([gripper_rot[0][2], gripper_rot[1][2], gripper_rot[2][2]])
            
            print(f"\n🔸 Gripper Local X (World): {np.round(local_x, 3)}")
            print(f"🔸 Gripper Local Y (World): {np.round(local_y, 3)}")
            print(f"🔸 Gripper Local Z (World): {np.round(local_z, 3)}")
            
            # 각 축과 큐브 방향의 각도
            dot_x = np.dot(local_x, direction_normalized)
            dot_y = np.dot(local_y, direction_normalized)
            dot_z = np.dot(local_z, direction_normalized)
            
            print(f"\n📐 Dot product with cube direction:")
            print(f"   X: {dot_x:.3f} (1.0 = same direction)")
            print(f"   Y: {dot_y:.3f}")
            print(f"   Z: {dot_z:.3f}")
    
    # ==================== 고정 씬 카메라 테스트 ====================
    print("\n" + "="*70)
    print("📷 고정 씬 카메라 테스트")
    print("="*70)
    
    # 위에서 내려다보는 카메라
    rgb_annot_top, _ = create_fixed_camera(
        stage, 
        position=(0.2, 0.0, 0.5),  # 큐브 위 50cm
        look_at=(0.2, 0.0, 0.0),  # 큐브 방향
        name="TopCamera"
    )
    
    for _ in range(20):
        world.step(render=True)
    rep.orchestrator.step(rt_subframes=8)
    
    rgb_data = rgb_annot_top.get_data()
    red_ratio, center = detect_red_in_image(rgb_data)
    print(f"\n🔝 Top Camera: Red {red_ratio:.2f}%")
    save_image(rgb_data, "top_camera.png", output_dir)
    
    # 측면에서 보는 카메라
    rgb_annot_side, _ = create_fixed_camera(
        stage,
        position=(0.5, 0.3, 0.2),
        look_at=(0.15, 0.0, 0.1),
        name="SideCamera"
    )
    
    for _ in range(20):
        world.step(render=True)
    rep.orchestrator.step(rt_subframes=8)
    
    rgb_data = rgb_annot_side.get_data()
    red_ratio, center = detect_red_in_image(rgb_data)
    print(f"📐 Side Camera: Red {red_ratio:.2f}%")
    save_image(rgb_data, "side_camera.png", output_dir)
    
    # ==================== 그리퍼 앞쪽 카메라 시도 ====================
    print("\n" + "="*70)
    print("📷 그리퍼 기준 카메라 (큐브 방향 직접 계산)")
    print("="*70)
    
    if gripper_pos is not None and cube_pos is not None:
        # 그리퍼에서 약간 뒤로 물러난 위치에서 큐브를 바라봄
        camera_pos = gripper_pos + np.array([0, 0, 0.1])  # 10cm 위
        
        rgb_annot_gripper, _ = create_fixed_camera(
            stage,
            position=tuple(camera_pos),
            look_at=tuple(cube_pos),
            name="GripperViewCamera"
        )
        
        for _ in range(20):
            world.step(render=True)
        rep.orchestrator.step(rt_subframes=8)
        
        rgb_data = rgb_annot_gripper.get_data()
        red_ratio, center = detect_red_in_image(rgb_data)
        print(f"🤖 Gripper View Camera: Red {red_ratio:.2f}%")
        save_image(rgb_data, "gripper_view_camera.png", output_dir)
    
    # ==================== 결과 ====================
    print("\n" + "="*70)
    print("📊 결과")
    print("="*70)
    print(f"\n📁 이미지 저장 위치: {output_dir}")
    print("\n💡 다음 단계:")
    print("   1. top_camera.png에서 큐브와 로봇 배치 확인")
    print("   2. side_camera.png에서 로봇 자세 확인") 
    print("   3. gripper_view_camera.png에서 그리퍼 시점 확인")
    print("   4. 좌표계 분석 결과로 카메라 회전 계산")
    
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
