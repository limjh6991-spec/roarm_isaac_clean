#!/usr/bin/env python3
"""
카메라 큐브 탐색 테스트
로봇팔의 각 링크를 움직이면서 카메라가 큐브를 찾는 위치 확인

Home Position:
- base_link, link1: 지표와 수직
- link2 ~ gripper: 지표와 수평

사용법:
    /home/roarm_m3/isaacsim/python.sh scripts/test/test_camera_cube_search.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import os
from datetime import datetime

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
    
    return red_ratio * 100, center  # 퍼센트로 반환


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
    
    # Cube - 로봇에서 32cm 앞
    cube_pos = (0.32, 0.0, 0.05)
    create_dynamic_cuboid(
        stage=stage,
        prim_path="/World/Cube",
        position=cube_pos,
        size=0.04,
        color=(0.8, 0.2, 0.2),  # Red
        mass=0.1
    )
    
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
    
    # Controller gains
    try:
        controller = robot.get_articulation_controller()
        kp = np.array([10000.0, 50000.0, 40000.0, 20000.0, 15000.0, 8000.0], dtype=np.float32)
        kd = np.array([1000.0, 5000.0, 4000.0, 2000.0, 1500.0, 800.0], dtype=np.float32)
        controller.set_gains(kp, kd)
    except Exception as e:
        print(f"⚠️ Controller setup: {e}")
    
    return world, robot, stage


def setup_camera(stage):
    """Hand-Eye 카메라 설정 (그리퍼 상단, 아래 방향)"""
    camera_parent_path = "/World/RoArm/gripper_link"
    camera_prim_path = f"{camera_parent_path}/hand_camera"
    
    camera_prim = stage.DefinePrim(camera_prim_path, "Camera")
    camera = UsdGeom.Camera(camera_prim)
    
    camera.GetFocalLengthAttr().Set(18.0)
    camera.GetHorizontalApertureAttr().Set(20.955)
    camera.GetVerticalApertureAttr().Set(20.955)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10.0))
    
    # 그리퍼 상단에서 아래를 내려다봄
    xformable = UsdGeom.Xformable(camera_prim)
    xformable.ClearXformOpOrder()
    translate_op = xformable.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(0.0, 0.0, 0.06))  # 6cm above gripper
    rotate_xyz_op = xformable.AddRotateXYZOp()
    rotate_xyz_op.Set(Gf.Vec3f(-90.0, 0.0, 180.0))  # Look down
    
    # Render product
    render_product = rep.create.render_product(camera_prim_path, (84, 84))
    rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annot.attach([render_product])
    
    print(f"✅ Camera setup at {camera_prim_path}")
    return rgb_annot


def save_image(rgb_data, filename, output_dir):
    """이미지 저장"""
    try:
        from PIL import Image
        if rgb_data is not None and len(rgb_data.shape) == 3:
            img = Image.fromarray(rgb_data[:, :, :3].astype(np.uint8))
            filepath = os.path.join(output_dir, filename)
            img.save(filepath)
            return True
    except:
        pass
    return False


def main():
    print("\n" + "="*70)
    print("🔍 카메라 큐브 탐색 테스트")
    print("="*70)
    
    # Output directory
    output_dir = "/home/roarm_m3/roarm_isaac_clean/resources/camera_search_test"
    os.makedirs(output_dir, exist_ok=True)
    
    # Setup
    world, robot, stage = setup_scene()
    rgb_annot = setup_camera(stage)
    
    # Home position (지표 수평)
    # joint0: base rotation (0 = 정면)
    # joint1: shoulder (0 = 수직) 
    # joint2: elbow - 수평이 되려면 조정 필요
    # joint3: wrist1
    # joint4: wrist2
    # joint5: gripper
    
    # 수평 자세: link2~gripper가 지표와 수평
    # shoulder를 앞으로 기울이고(+), elbow를 위로(-) 조정
    home_position = np.array([
        0.0,    # joint0: base - 정면
        0.8,    # joint1: shoulder - 앞으로 기울임 (~45도)
        -0.8,   # joint2: elbow - 위로 구부림
        0.0,    # joint3: wrist1 
        0.0,    # joint4: wrist2
        0.06    # joint5: gripper open
    ], dtype=np.float32)
    
    print(f"\n📐 Home Position: {home_position}")
    print("   - base(j0)=0, shoulder(j1)=0.8, elbow(j2)=-0.8")
    print("   - 목표: link2~gripper가 지표와 수평, 카메라가 아래를 봄\n")
    
    # Set initial position
    robot.set_joint_positions(home_position)
    robot.set_joint_velocities(np.zeros(6))
    
    # Stabilize
    for _ in range(100):
        robot.set_joint_positions(home_position)
        robot.set_joint_velocities(np.zeros(6))
        world.step(render=True)
    
    # Initialize replicator
    rep.orchestrator.step(rt_subframes=4)
    
    # Check initial view
    rgb_data = rgb_annot.get_data()
    red_ratio, center = detect_red_in_image(rgb_data)
    print(f"🏠 Home position - Red: {red_ratio:.2f}%, Center: {center}")
    save_image(rgb_data, "00_home.png", output_dir)
    
    # ==================== 탐색 테스트 ====================
    print("\n" + "="*70)
    print("🔎 각 관절을 움직이며 큐브 탐색")
    print("="*70)
    
    best_positions = []
    test_count = 0
    
    # 테스트할 관절 범위
    joint_ranges = [
        ("j0_base", 0, np.linspace(-0.5, 0.5, 5)),      # base rotation
        ("j1_shoulder", 1, np.linspace(0.3, 1.2, 5)),   # shoulder
        ("j2_elbow", 2, np.linspace(-1.2, -0.3, 5)),    # elbow
        ("j3_wrist1", 3, np.linspace(-0.5, 0.5, 5)),    # wrist1
    ]
    
    for joint_name, joint_idx, values in joint_ranges:
        print(f"\n--- {joint_name} (joint {joint_idx}) 테스트 ---")
        
        for val in values:
            test_pos = home_position.copy()
            test_pos[joint_idx] = val
            
            # Move robot
            robot.set_joint_positions(test_pos)
            robot.set_joint_velocities(np.zeros(6))
            
            for _ in range(30):
                robot.set_joint_positions(test_pos)
                world.step(render=True)
            
            rep.orchestrator.step(rt_subframes=4)
            
            # Check camera
            rgb_data = rgb_annot.get_data()
            red_ratio, center = detect_red_in_image(rgb_data)
            
            status = "🟢" if red_ratio > 1.0 else ("🟡" if red_ratio > 0.1 else "⚫")
            print(f"  {status} {joint_name}={val:.2f} → Red: {red_ratio:.2f}%", end="")
            if center:
                print(f" @ ({center[0]:.1f}, {center[1]:.1f})")
            else:
                print()
            
            # Save if cube visible
            if red_ratio > 0.5:
                test_count += 1
                filename = f"{test_count:02d}_{joint_name}_{val:.2f}_red{red_ratio:.1f}.png"
                save_image(rgb_data, filename, output_dir)
                best_positions.append({
                    "joint": joint_name,
                    "value": val,
                    "red_ratio": red_ratio,
                    "center": center,
                    "positions": test_pos.copy()
                })
    
    # ==================== 복합 테스트 ====================
    print("\n" + "="*70)
    print("🔎 복합 관절 조합 테스트 (shoulder + elbow)")
    print("="*70)
    
    for shoulder in np.linspace(0.5, 1.5, 5):
        for elbow in np.linspace(-1.5, -0.5, 5):
            test_pos = home_position.copy()
            test_pos[1] = shoulder
            test_pos[2] = elbow
            
            robot.set_joint_positions(test_pos)
            robot.set_joint_velocities(np.zeros(6))
            
            for _ in range(20):
                robot.set_joint_positions(test_pos)
                world.step(render=True)
            
            rep.orchestrator.step(rt_subframes=4)
            
            rgb_data = rgb_annot.get_data()
            red_ratio, center = detect_red_in_image(rgb_data)
            
            if red_ratio > 1.0:
                test_count += 1
                print(f"  🟢 shoulder={shoulder:.2f}, elbow={elbow:.2f} → Red: {red_ratio:.2f}%")
                filename = f"{test_count:02d}_combo_s{shoulder:.2f}_e{elbow:.2f}_red{red_ratio:.1f}.png"
                save_image(rgb_data, filename, output_dir)
                best_positions.append({
                    "joint": "combo",
                    "shoulder": shoulder,
                    "elbow": elbow,
                    "red_ratio": red_ratio,
                    "center": center,
                    "positions": test_pos.copy()
                })
    
    # ==================== 결과 요약 ====================
    print("\n" + "="*70)
    print("📊 결과 요약")
    print("="*70)
    
    if best_positions:
        print(f"\n✅ 큐브가 보이는 위치: {len(best_positions)}개 발견!")
        
        # Sort by red ratio
        best_positions.sort(key=lambda x: x["red_ratio"], reverse=True)
        
        print("\nTop 5 positions:")
        for i, pos in enumerate(best_positions[:5]):
            if "combo" in pos.get("joint", ""):
                print(f"  {i+1}. shoulder={pos['shoulder']:.2f}, elbow={pos['elbow']:.2f}")
            else:
                print(f"  {i+1}. {pos['joint']}={pos['value']:.2f}")
            print(f"      Red: {pos['red_ratio']:.2f}%, Center: {pos['center']}")
            print(f"      Full: {np.round(pos['positions'], 2)}")
        
        # Best position for home
        best = best_positions[0]
        print(f"\n🏆 최적 home position:")
        print(f"   {np.round(best['positions'], 3)}")
        print(f"   Red ratio: {best['red_ratio']:.2f}%")
        
    else:
        print("\n❌ 큐브가 보이는 위치를 찾지 못했습니다.")
        print("   카메라 위치/각도를 조정해야 합니다.")
    
    print(f"\n📁 이미지 저장됨: {output_dir}")
    print(f"   총 {test_count}개 이미지")
    
    # Keep running for GUI inspection
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
