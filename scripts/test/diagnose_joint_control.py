#!/usr/bin/env python3
"""
로봇 Joint 제어 진단 스크립트
- 초기 자세가 제대로 설정되는지 확인
- set_joint_position_target이 작동하는지 확인
"""

import sys
import os
import numpy as np

# IsaacLab 임포트
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.app import AppLauncher

# Argument parser
app_launcher = AppLauncher(headless=False)
simulation_app = app_launcher.app

# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.assets import Articulation, ArticulationCfg
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.actuators import ImplicitActuatorCfg
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.sim import SimulationCfg, SimulationContext
import omni.isaac.lab.sim as sim_utils
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR


def main():
    # Simulation 설정
    sim_cfg = SimulationCfg(
        device="cuda:0",
        dt=1.0/60.0,  # 60 FPS
        render_interval=1,
        gravity=(0.0, 0.0, -9.81),
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
    )
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[0.5, 0.5, 0.5], target=[0.0, 0.0, 0.2])
    
    # Ground plane
    ground_cfg = sim_utils.GroundPlaneCfg(
        color=(0.15, 0.15, 0.15),
        size=(10.0, 10.0),
    )
    ground_cfg.func("/World/ground", ground_cfg)
    
    # Light
    light_cfg = sim_utils.DomeLightCfg(
        intensity=3000.0,
        color=(0.75, 0.75, 0.75),
    )
    light_cfg.func("/World/light", light_cfg)
    
    # RoArm-M3 사용 (원본 URDF - 카메라 없음)
    urdf_path = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3.generated.urdf"
    
    robot_cfg = ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UrdfFileCfg(
            asset_path=urdf_path,
            fix_base=True,
            merge_fixed_joints=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            joint_pos={
                "base_link_to_link1": 0.0,
                "link1_to_link2": 0.0,
                "link2_to_link3": -0.9,  # 'ㄱ'자
                "link3_to_link4": 0.0,
                "link4_to_link5": 0.0,
                "link5_to_gripper_link": 0.0,
            },
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                stiffness=2000.0,  # 높은 stiffness로 테스트
                damping=200.0,
            ),
        },
    )
    
    robot = Articulation(cfg=robot_cfg)
    
    print("\n" + "=" * 80)
    print("로봇 Joint 제어 진단")
    print("=" * 80)
    
    # Reset
    sim.reset()
    robot.reset()
    
    print(f"\n📊 로봇 정보:")
    print(f"   DOF: {robot.num_joints}")
    print(f"   Joint names: {robot.joint_names}")
    
    # 초기 상태 확인
    joint_pos = robot.data.joint_pos[0].cpu().numpy()
    print(f"\n🎯 설정한 초기 자세:")
    print(f"   Joint 0 (Base):     0.0")
    print(f"   Joint 1 (Shoulder): 0.0")
    print(f"   Joint 2 (Elbow):   -0.9 rad (목표)")
    print(f"   Joint 3 (Wrist1):   0.0")
    print(f"   Joint 4 (Wrist2):   0.0")
    print(f"   Joint 5 (Gripper):  0.0")
    
    print(f"\n📍 실제 초기 자세:")
    for i, (name, pos) in enumerate(zip(robot.joint_names, joint_pos)):
        print(f"   Joint {i} ({name:25s}): {pos:7.3f} rad ({np.rad2deg(pos):7.1f}°)")
    
    # 100 프레임 시뮬레이션 (자세 안정화)
    print(f"\n⏳ 100 프레임 안정화 중...")
    for frame in range(100):
        sim.step()
        robot.update(dt=sim_cfg.dt)
        
        if frame % 20 == 0:
            joint_pos = robot.data.joint_pos[0].cpu().numpy()
            root_z = robot.data.root_pos_w[0, 2].item()
            print(f"   Frame {frame:3d}: Root Z={root_z:.3f}m, Joint[2]={joint_pos[2]:7.3f} rad")
    
    # 최종 자세
    joint_pos_final = robot.data.joint_pos[0].cpu().numpy()
    print(f"\n✅ 100 프레임 후 자세:")
    for i, (name, pos) in enumerate(zip(robot.joint_names, joint_pos_final)):
        diff = pos - joint_pos[i]
        print(f"   Joint {i} ({name:25s}): {pos:7.3f} rad (변화: {diff:+7.3f})")
    
    # Joint 2 분석
    target = -0.9
    actual = joint_pos_final[2]
    error = actual - target
    print(f"\n🔍 Joint 2 (Elbow) 분석:")
    print(f"   목표:   {target:7.3f} rad ({np.rad2deg(target):7.1f}°)")
    print(f"   실제:   {actual:7.3f} rad ({np.rad2deg(actual):7.1f}°)")
    print(f"   오차:   {error:7.3f} rad ({np.rad2deg(error):7.1f}°)")
    
    if abs(error) > 0.1:
        print(f"   ❌ 초기 자세가 제대로 설정되지 않음!")
        print(f"   → init_state가 무시되거나, 중력에 의해 자세가 무너짐")
    else:
        print(f"   ✅ 초기 자세 정상")
    
    # Cleanup
    print(f"\n🧹 종료 중...")
    simulation_app.close()


if __name__ == "__main__":
    main()
