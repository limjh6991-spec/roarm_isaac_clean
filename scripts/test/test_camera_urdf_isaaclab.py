#!/usr/bin/env python3
"""
RoArm-M3 + D405 Camera URDF Test (Isaac Sim 5.0 Compatible)
IsaacLab API 사용
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
print("🎥 RoArm-M3 + D405 Camera URDF Test (IsaacLab)")
print("=" * 80)

import numpy as np
import torch

# IsaacLab imports
import omni.isaac.lab.sim as sim_utils
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.sim.converters import UrdfConverter, UrdfConverterCfg
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.sensors import Camera, CameraCfg
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.assets import Articulation, ArticulationCfg
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.utils import configclass

@configclass
class RoArmCameraSceneCfg(InteractiveSceneCfg):
    """Configuration for RoArm-M3 + D405 scene"""
    
    # Ground plane
    ground = sim_utils.GroundPlaneCfg()
    
    # Robot
    robot: ArticulationCfg = ArticulationCfg(
        prim_path="/World/robot",
        spawn=sim_utils.UrdfFileCfg(
            asset_path="/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3_with_d405.urdf",
            fix_base=False,
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            joint_pos={
                "world_to_base": 0.0,
                "joint1": 0.0,
                "joint2": 0.5,
                "joint3": -0.5,
                "joint4": 0.0,
                "joint5": 0.0,
                "gripper_link_to_left_link": 0.01,
            },
        ),
    )
    
    # Camera (attached to robot)
    camera: CameraCfg = CameraCfg(
        prim_path="/World/robot/camera_link",
        update_period=0.0,
        height=256,
        width=256,
        data_types=["rgb", "distance_to_camera"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            focus_distance=400.0,
            horizontal_aperture=20.955,
        ),
    )


def main():
    """Main function"""
    
    # Create simulation context
    sim_cfg = sim_utils.SimulationCfg(dt=1/60.0)
    sim = sim_utils.SimulationContext(sim_cfg)
    
    # Set camera (optional)
    sim.set_camera_view(eye=[2.0, 2.0, 2.0], target=[0.0, 0.0, 0.5])
    
    # Create scene
    print("\n🌍 Creating Scene...")
    scene_cfg = RoArmCameraSceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    
    # Play the simulator
    print("\n▶️  Starting Simulation...")
    sim.reset()
    
    # Get handles
    robot = scene["robot"]
    camera = scene["camera"]
    
    print(f"\n✅ Scene Created:")
    print(f"   Robot: {robot.num_instances} instances")
    print(f"   Robot DOF: {robot.num_dof}")
    print(f"   Camera: {camera.image_shape}")
    print(f"   Camera Device: {camera.device}")
    
    # Print camera frames
    print(f"\n📸 Camera Frames:")
    print(f"   Camera Link: {camera.cfg.prim_path}")
    
    # Simulation loop
    print("\n▶️  Running Simulation...")
    print("   Press Ctrl+C or close window to exit")
    
    frame_count = 0
    
    try:
        while simulation_app.is_running():
            # Step simulation
            sim.step()
            
            # Update scene
            scene.update(dt=sim_cfg.dt)
            
            # Update camera
            camera.update(dt=sim_cfg.dt)
            
            frame_count += 1
            
            # Print info every 100 frames
            if frame_count % 100 == 0:
                print(f"\n   Frame {frame_count}")
                
                # Robot state
                joint_pos = robot.data.joint_pos[0]
                joint_names = robot.data.joint_names
                print(f"   Joint Positions:")
                for name, pos in zip(joint_names, joint_pos):
                    print(f"      {name}: {pos:.3f}")
                
                # Camera state
                if "rgb" in camera.data.output:
                    rgb = camera.data.output["rgb"]
                    depth = camera.data.output["distance_to_camera"]
                    print(f"   Camera RGB: {rgb.shape}, dtype: {rgb.dtype}")
                    print(f"   Camera Depth: {depth.shape}, dtype: {depth.dtype}")
                    print(f"   RGB range: [{rgb.min():.1f}, {rgb.max():.1f}]")
                    print(f"   Depth range: [{depth.min():.3f}, {depth.max():.3f}]")
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    
    finally:
        simulation_app.close()
    
    print(f"\n✅ Test Complete!")
    print(f"   Total Frames: {frame_count}")


if __name__ == "__main__":
    main()
