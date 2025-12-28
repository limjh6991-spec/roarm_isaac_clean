#!/usr/bin/env python
"""
RoArm-M3 + D405 Camera URDF Test with IsaacLab (v3)

이 스크립트는 IsaacLab을 사용하여 RoArm-M3 로봇에 D405 카메라를 통합한
URDF를 로드하고 테스트합니다.

Features:
- IsaacLab Articulation API 사용
- URDF Converter를 통한 USD 변환
- 'ㄱ'자 포즈 설정
- Ground plane에 안정적으로 배치

작성일: 2025-11-02
버전: v3 (전체 재작성)
"""

import argparse
import os

# AppLauncher는 다른 import보다 먼저 실행되어야 함
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.app import AppLauncher

# Argument parser
parser = argparse.ArgumentParser(
    description="RoArm-M3 + D405 Camera URDF Test with IsaacLab"
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch Omniverse App
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows after AppLauncher."""

import torch
import numpy as np

import omni.isaac.lab.sim as sim_utils
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.assets import Articulation, ArticulationCfg
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.actuators import ImplicitActuatorCfg
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.sim import SimulationCfg, SimulationContext
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.sim.converters import UrdfConverter, UrdfConverterCfg

# ==============================================================================
# Configuration
# ==============================================================================

# 경로 설정
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "../.."))
URDF_PATH = os.path.join(PROJECT_DIR, "assets/roarm_m3/urdf/roarm_m3_with_d405.urdf")
USD_DIR = os.path.join(PROJECT_DIR, "assets/roarm_m3/usd")
USD_FILE = "roarm_m3_with_d405.usd"

print("=" * 80)
print("🎥 RoArm-M3 + D405 Camera URDF Test (IsaacLab v3)")
print("=" * 80)
print(f"\n📁 Project Directory: {PROJECT_DIR}")
print(f"📁 URDF Path: {URDF_PATH}")
print(f"📁 USD Output Directory: {USD_DIR}")

# URDF 파일 확인
if not os.path.exists(URDF_PATH):
    raise FileNotFoundError(f"URDF file not found: {URDF_PATH}")

# USD 디렉토리 생성
os.makedirs(USD_DIR, exist_ok=True)

# ==============================================================================
# URDF to USD Conversion
# ==============================================================================

def convert_urdf_to_usd():
    """Convert URDF to USD using IsaacLab UrdfConverter."""
    print("\n📦 Converting URDF to USD...")
    
    # URDF Converter configuration
    converter_cfg = UrdfConverterCfg(
        asset_path=URDF_PATH,
        usd_dir=USD_DIR,
        usd_file_name=USD_FILE,
        fix_base=False,  # floating base (not fixed to ground)
        merge_fixed_joints=True,
        collision_from_visuals=False,
        collider_type="convex_hull",
        self_collision=False,
        force_usd_conversion=True,  # Always regenerate USD
        make_instanceable=True,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            drive_type="force",
            target_type="position",
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=100.0,
                damping=10.0,
            ),
        ),
    )
    
    # Run the converter
    converter = UrdfConverter(converter_cfg)
    
    print(f"✅ USD file generated: {converter.usd_path}")
    return converter.usd_path

# ==============================================================================
# Robot Configuration
# ==============================================================================

def get_roarm_m3_cfg() -> ArticulationCfg:
    """
    Get RoArm-M3 robot configuration for IsaacLab.
    
    Returns:
        ArticulationCfg: Robot configuration with URDF import and actuators
    """
    # URDF file path
    project_dir = Path(__file__).resolve().parents[2]
    urdf_path = project_dir / "assets" / "roarm_m3" / "urdf" / "roarm_m3_with_d405.urdf"
    
    # USD output directory
    usd_dir = project_dir / "assets" / "roarm_m3" / "usd"
    usd_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"\n{'='*80}")
    print("STEP 1: URDF to USD Conversion")
    print(f"{'='*80}\n")
    print("📦 Converting URDF to USD...")
    
    # Convert URDF to USD with fix_base=True
    usd_path = str(usd_dir / "roarm_m3_with_d405.usd")
    UrdfConverter(str(urdf_path), usd_path, fix_base=True)
    
    print(f"✅ USD file generated: {usd_path}")
    
    # Create ArticulationCfg
    cfg = ArticulationCfg(# ==============================================================================
# Main Execution
# ==============================================================================

def main():
    """Main function"""
    
    # 1. URDF to USD 변환
    print("\n" + "=" * 80)
    print("STEP 1: URDF to USD Conversion")
    print("=" * 80)
    usd_path = convert_urdf_to_usd()
    
    # 2. Simulation 설정
    print("\n" + "=" * 80)
    print("STEP 2: Simulation Setup")
    print("=" * 80)
    print("🌍 Initializing Simulation Context...")
    
    sim_cfg = SimulationCfg(
        dt=1.0 / 60.0,  # 60 Hz
        device="cuda:0" if torch.cuda.is_available() else "cpu",
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=0.5,
            dynamic_friction=0.5,
            restitution=0.0,
        ),
    )
    sim = SimulationContext(sim_cfg)
    
    # 카메라 뷰 설정 (로봇을 정면에서 볼 수 있도록)
    sim.set_camera_view(eye=[2.0, 2.0, 1.5], target=[0.0, 0.0, 0.5])
    
    print(f"✅ Simulation initialized on {sim.device}")
    
    # 3. Scene 구성
    print("\n" + "=" * 80)
    print("STEP 3: Scene Setup")
    print("=" * 80)
    
    # 3-1. Ground plane 생성
    print("📐 Creating ground plane...")
    ground_cfg = sim_utils.GroundPlaneCfg(
        color=(0.15, 0.15, 0.15),
        size=(100.0, 100.0),
    )
    ground_cfg.func("/World/ground", ground_cfg)
    print("✅ Ground plane created")
    
    # 3-2. Light 생성
    print("💡 Creating light...")
    light_cfg = sim_utils.DomeLightCfg(
        intensity=3000.0,
        color=(0.75, 0.75, 0.75),
    )
    light_cfg.func("/World/light", light_cfg)
    print("✅ Light created")
    
    # 3-3. Robot 생성
    print("🤖 Creating robot...")
    robot_cfg = get_roarm_m3_cfg(usd_path)
    robot = Articulation(cfg=robot_cfg)
    print(f"✅ Robot created at {robot.cfg.prim_path}")
    
    # 4. Simulation 초기화
    print("\n" + "=" * 80)
    print("STEP 4: Initialize Scene")
    print("=" * 80)
    print("🔄 Resetting simulation...")
    sim.reset()
    print("✅ Simulation reset complete")
    
    # Robot 초기 업데이트
    robot.update(sim.cfg.dt)
    
    # 5. Robot 상태 확인
    print("\n" + "=" * 80)
    print("STEP 5: Robot State")
    print("=" * 80)
    print(f"📊 Robot Information:")
    print(f"   Prim Path: {robot.cfg.prim_path}")
    print(f"   Number of Bodies: {robot.num_bodies}")
    print(f"   Number of DOF: {robot.num_joints}")
    print(f"   Body Names: {robot.body_names}")
    print(f"   Joint Names: {robot.joint_names}")
    
    print(f"\n🤖 Initial Robot State:")
    root_pos = robot.data.root_pos_w[0].cpu().numpy()
    root_quat = robot.data.root_quat_w[0].cpu().numpy()
    print(f"   Root Position: [{root_pos[0]:.3f}, {root_pos[1]:.3f}, {root_pos[2]:.3f}]")
    print(f"   Root Orientation (wxyz): [{root_quat[0]:.3f}, {root_quat[1]:.3f}, {root_quat[2]:.3f}, {root_quat[3]:.3f}]")
    
    print(f"\n   Joint Positions:")
    joint_pos = robot.data.joint_pos[0].cpu().numpy()
    for i, name in enumerate(robot.joint_names):
        rad_val = joint_pos[i]
        deg_val = np.degrees(rad_val)
        print(f"      {name:12s}: {rad_val:7.3f} rad ({deg_val:7.1f}°)")
    
    # 6. Simulation Loop
    print("\n" + "=" * 80)
    print("STEP 6: Running Simulation")
    print("=" * 80)
    print("▶️  Simulation loop started")
    print("   Press Ctrl+C to stop")
    
    sim_dt = sim.cfg.dt
    count = 0
    reset_interval = 500
    
    try:
        while simulation_app.is_running():
            # 주기적 리셋 (500 프레임마다)
            if count % reset_interval == 0 and count > 0:
                print(f"\n[Frame {count}] 🔄 Resetting robot...")
                
                # Root 상태 리셋
                root_state = robot.data.default_root_state.clone()
                robot.write_root_pose_to_sim(root_state[:, :7])
                robot.write_root_velocity_to_sim(root_state[:, 7:])
                
                # Joint 상태 리셋
                joint_pos = robot.data.default_joint_pos.clone()
                joint_vel = robot.data.default_joint_vel.clone()
                robot.write_joint_state_to_sim(joint_pos, joint_vel)
                
                # 내부 버퍼 리셋
                robot.reset()
                
                print(f"   ✅ Reset complete")
            
            # 현재 위치 유지 (PD 제어)
            target_pos = robot.data.default_joint_pos.clone()
            robot.set_joint_position_target(target_pos)
            robot.write_data_to_sim()
            
            # Simulation step
            sim.step()
            
            # Robot 상태 업데이트
            robot.update(sim_dt)
            
            # 주기적 상태 출력
            if count % 100 == 0:
                root_pos = robot.data.root_pos_w[0].cpu().numpy()
                print(f"   [Frame {count:5d}] Root Z: {root_pos[2]:.3f}m")
            
            count += 1
            
    except KeyboardInterrupt:
        print(f"\n\n⚠️  Simulation interrupted by user")
    
    # 7. 종료
    print("\n" + "=" * 80)
    print("STEP 7: Cleanup")
    print("=" * 80)
    print(f"📊 Statistics:")
    print(f"   Total Frames: {count}")
    print(f"   Simulation Time: {count * sim_dt:.2f}s")
    print(f"   Average FPS: {count / (count * sim_dt):.1f}")
    
    print("\n✅ Simulation Complete!")
    print("=" * 80)
    
    # Cleanup
    simulation_app.close()

if __name__ == "__main__":
    main()
