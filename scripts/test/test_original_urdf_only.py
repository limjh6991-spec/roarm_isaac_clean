#!/usr/bin/env python
"""
원본 RoArm-M3 URDF만 로드하는 테스트 스크립트 (카메라 없이)
비교를 위해 roarm_m3.generated.urdf 파일을 로드합니다.
"""

import argparse
import os
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.app import AppLauncher

# Argument parser
parser = argparse.ArgumentParser(description="Load original RoArm-M3 URDF without camera")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch app
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

# 경로 설정
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "../.."))
URDF_PATH = os.path.join(PROJECT_DIR, "assets/roarm_m3/urdf/roarm_m3.generated.urdf")
USD_DIR = os.path.join(PROJECT_DIR, "assets/roarm_m3/usd")

print("=" * 80)
print("🤖 Original RoArm-M3 URDF Test (No Camera)")
print("=" * 80)
print(f"\n📁 URDF Path: {URDF_PATH}")

# URDF 확인
if not os.path.exists(URDF_PATH):
    raise FileNotFoundError(f"URDF file not found: {URDF_PATH}")

os.makedirs(USD_DIR, exist_ok=True)

def convert_urdf_to_usd():
    """URDF to USD 변환"""
    print("\n📦 Converting URDF to USD...")
    
    converter_cfg = UrdfConverterCfg(
        asset_path=URDF_PATH,
        usd_dir=USD_DIR,
        usd_file_name="roarm_m3_original.usd",
        fix_base=False,
        merge_fixed_joints=True,
        collision_from_visuals=False,
        collider_type="convex_hull",
        self_collision=False,
        force_usd_conversion=True,
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
    
    converter = UrdfConverter(converter_cfg)
    print(f"✅ USD file: {converter.usd_path}")
    return converter.usd_path

def get_robot_cfg(usd_path: str) -> ArticulationCfg:
    """로봇 설정"""
    return ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=usd_path,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=1,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.1),
            joint_pos={
                "base_to_link1": 0.0,
                "link1_to_link2": -0.9,
                "link2_to_link3": 1.5,
                "link3_to_link4": 0.0,
                "link4_to_link5": 0.0,
                "link5_to_gripper_link": 0.0,
                "gripper_link_to_left_link": 0.01,
            },
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=["base_to_link1", "link1_to_link2", "link2_to_link3",
                                  "link3_to_link4", "link4_to_link5", "link5_to_gripper_link"],
                effort_limit=10.0,
                velocity_limit=2.0,
                stiffness=100.0,
                damping=10.0,
            ),
            "gripper": ImplicitActuatorCfg(
                joint_names_expr=["gripper_link_to_left_link"],
                effort_limit=5.0,
                velocity_limit=1.0,
                stiffness=50.0,
                damping=5.0,
            ),
        },
    )

def main():
    # 1. USD 변환
    usd_path = convert_urdf_to_usd()
    
    # 2. Simulation 설정
    print("\n🌍 Initializing Simulation...")
    sim_cfg = SimulationCfg(
        dt=1.0 / 60.0,
        device="cuda:0" if torch.cuda.is_available() else "cpu",
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=0.5,
            dynamic_friction=0.5,
            restitution=0.0,
        ),
    )
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[1.5, 1.5, 1.0], target=[0.0, 0.0, 0.3])
    
    # 3. Scene 구성
    print("📐 Creating ground plane...")
    ground_cfg = sim_utils.GroundPlaneCfg(color=(0.15, 0.15, 0.15), size=(100.0, 100.0))
    ground_cfg.func("/World/ground", ground_cfg)
    
    print("💡 Creating light...")
    light_cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    light_cfg.func("/World/light", light_cfg)
    
    print("🤖 Creating robot...")
    robot_cfg = get_robot_cfg(usd_path)
    robot = Articulation(cfg=robot_cfg)
    
    # 4. 초기화
    print("🔄 Resetting...")
    sim.reset()
    robot.update(sim.cfg.dt)
    
    # 5. 상태 출력
    print("\n📊 Robot State:")
    print(f"   Bodies: {robot.num_bodies}")
    print(f"   DOF: {robot.num_joints}")
    print(f"   Joints: {robot.joint_names}")
    
    root_pos = robot.data.root_pos_w[0].cpu().numpy()
    print(f"\n   Root Position: [{root_pos[0]:.3f}, {root_pos[1]:.3f}, {root_pos[2]:.3f}]")
    
    joint_pos = robot.data.joint_pos[0].cpu().numpy()
    print(f"\n   Joint Positions:")
    for i, name in enumerate(robot.joint_names):
        print(f"      {name}: {joint_pos[i]:.3f} rad ({np.degrees(joint_pos[i]):.1f}°)")
    
    # 6. Simulation loop
    print("\n▶️  Running Simulation (Press Ctrl+C to stop)")
    
    count = 0
    try:
        while simulation_app.is_running():
            # 위치 유지
            target_pos = robot.data.default_joint_pos.clone()
            robot.set_joint_position_target(target_pos)
            robot.write_data_to_sim()
            
            sim.step()
            robot.update(sim.cfg.dt)
            
            if count % 100 == 0:
                root_pos = robot.data.root_pos_w[0].cpu().numpy()
                print(f"   [Frame {count:5d}] Root Z: {root_pos[2]:.3f}m")
            
            count += 1
            
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted")
    
    print("\n✅ Complete!")
    simulation_app.close()

if __name__ == "__main__":
    main()
