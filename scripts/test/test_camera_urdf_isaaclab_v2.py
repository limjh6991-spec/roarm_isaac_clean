#!/usr/bin/env python
"""
RoArm-M3 + D405 Camera URDF Test with IsaacLab

이 스크립트는 IsaacLab을 사용하여 RoArm-M3 로봇에 D405 카메라를 통합한
URDF를 로드하고 테스트합니다.

Features:
- IsaacLab Articulation API 사용
- URDF Converter를 통한 USD 변환
- 'ㄱ'자 포즈 설정
- Ground plane에 안정적으로 배치

작성일: 2025-11-02
"""

import argparse
import os
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

"""Rest everything follows."""

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
print("🎥 RoArm-M3 + D405 Camera URDF Test (IsaacLab)")
print("=" * 80)
print(f"\n📁 Project Directory: {PROJECT_DIR}")
print(f"📁 URDF Path: {URDF_PATH}")
print(f"📁 USD Output Directory: {USD_DIR}")

# ==============================================================================
# URDF to USD Conversion
# ==============================================================================

def convert_urdf_to_usd():
    """Convert URDF to USD using IsaacLab UrdfConverter."""
    print("[INFO] Converting URDF to USD...")
    
    # URDF Converter configuration
    converter_cfg = UrdfConverterCfg(
        asset_path=URDF_PATH,
        usd_dir=USD_DIR,
        usd_file_name=USD_FILE,
        fix_base=False,  # floating base
        merge_fixed_joints=True,
        collision_from_visuals=False,
        collider_type="convex_hull",
        self_collision=False,
        force_usd_conversion=True,  # Force conversion
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
    
    print(f"[INFO] USD file generated: {converter.usd_path}")
    return converter.usd_path

# ==============================================================================
# Robot Configuration
# ==============================================================================

def get_roarm_m3_cfg(usd_path: str) -> ArticulationCfg:
    """RoArm-M3 Articulation 설정 생성"""
    return ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=usd_path,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=5.0,
                max_linear_velocity=100.0,
                max_angular_velocity=100.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=1,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.05),  # 바닥에서 5cm 위 (베이스 높이)
            joint_pos={
                "joint1": 0.0,       # 정면 향함
                "joint2": -1.57,     # -90° (수평)
                "joint3": 1.57,      # +90° (수직) - 'ㄱ'자 형태
                "joint4": 0.0,       # 중립
                "joint5": 0.0,       # 중립
                "gripper": 0.01,     # 약간 열림
            },
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=["joint[1-5]"],
                effort_limit=10.0,
                velocity_limit=2.0,
                stiffness=100.0,
                damping=10.0,
            ),
            "gripper": ImplicitActuatorCfg(
                joint_names_expr=["gripper"],
                effort_limit=5.0,
                velocity_limit=1.0,
                stiffness=50.0,
                damping=5.0,
            ),
        },
    )

# ==============================================================================
# Main Execution
# ==============================================================================

def main():
    """Main function"""
    
    # 1. URDF to USD 변환
    usd_path = convert_urdf_to_usd()
    
    # 2. Simulation 설정
    print(f"\n🌍 Initializing Simulation...")
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
    
    # 카메라 뷰 설정
    sim.set_camera_view([2.0, 2.0, 2.0], [0.0, 0.0, 0.0])
    
    # 3. Ground plane 생성
    print(f"   Creating ground plane...")
    ground_cfg = sim_utils.GroundPlaneCfg(
        color=(0.1, 0.1, 0.1),
        size=(100.0, 100.0),
    )
    ground_cfg.func("/World/ground", ground_cfg)
    
    # 4. Robot 생성
    print(f"   Creating robot...")
    robot_cfg = get_roarm_m3_cfg(usd_path)
    robot = Articulation(cfg=robot_cfg)
    
    # 5. Simulation 초기화
    print(f"   Resetting simulation...")
    sim.reset()
    
    # 6. Robot 업데이트
    robot.update(sim.cfg.dt)
    
    print(f"\n✅ Setup Complete!")
    print(f"   Robot Path: {robot.cfg.prim_path}")
    print(f"   Number of DOF: {robot.num_joints}")
    print(f"   Joint Names: {robot.joint_names}")
    
    # Joint 초기 상태 출력
    print(f"\n🤖 Initial Robot State:")
    print(f"   Root Position: {robot.data.root_pos_w[0].cpu().numpy()}")
    print(f"   Root Orientation: {robot.data.root_quat_w[0].cpu().numpy()}")
    print(f"   Joint Positions:")
    for i, (name, pos) in enumerate(zip(robot.joint_names, robot.data.joint_pos[0].cpu().numpy())):
        print(f"      {name}: {pos:.3f} rad ({np.degrees(pos):.1f}°)")
    
    # ==========================
    # Simulation Loop
    # ==========================
    print(f"\n▶️  Running Simulation...")
    print(f"   Press Ctrl+C to exit")
    
    sim_dt = sim.cfg.dt
    count = 0
    
    try:
        while simulation_app.is_running():
            # Reset 주기 (500 프레임마다)
            if count % 500 == 0 and count > 0:
                print(f"\n[Frame {count}] Resetting robot...")
                
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
            
            # 액추에이터 명령 설정 (현재 위치 유지)
            target_pos = robot.data.default_joint_pos.clone()
            robot.set_joint_position_target(target_pos)
            
            # Simulation에 명령 적용
            robot.write_data_to_sim()
            
            # Simulation step
            sim.step()
            
            # Robot 상태 업데이트
            robot.update(sim_dt)
            
            # 주기적으로 상태 출력
            if count % 100 == 0:
                print(f"   Frame {count}")
            
            count += 1
            
    except KeyboardInterrupt:
        print(f"\n\n⚠️  Simulation interrupted by user")
    
    print(f"\n✅ Simulation Complete!")
    print(f"   Total Frames: {count}")
    
    # Cleanup
    simulation_app.close()

if __name__ == "__main__":
    main()
