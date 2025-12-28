#!/usr/bin/env python3
"""
Simple Vision Environment Test (Standalone)

기존 test_roarm_with_camera_isaaclab.py를 기반으로
Vision observation을 추가한 간단한 테스트

목표:
1. Camera에서 RGB-D 획득
2. 84x84로 resize 및 normalize
3. 이미지 저장 및 확인

작성일: 2025-11-02
"""

import argparse
from pathlib import Path
import numpy as np

# AppLauncher must be first
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.app import AppLauncher

parser = argparse.ArgumentParser(description="Vision Environment Simple Test")
parser.add_argument("--num_envs", type=int, default=1)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch
from PIL import Image
from datetime import datetime
import cv2

import omni.isaac.lab.sim as sim_utils
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.assets import Articulation, ArticulationCfg
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.actuators import ImplicitActuatorCfg
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.sim import SimulationCfg, SimulationContext
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.sim.converters import UrdfConverter, UrdfConverterCfg
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.sensors import Camera, CameraCfg

# Paths
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parents[1]
URDF_PATH = PROJECT_DIR / "assets" / "roarm_m3" / "urdf" / "roarm_m3_with_camera_correct.urdf"
USD_DIR = PROJECT_DIR / "assets" / "roarm_m3" / "usd"
USD_FILE = "roarm_m3_with_camera_correct.usd"
OUTPUT_DIR = PROJECT_DIR / "output" / "vision_env_simple" / datetime.now().strftime("%Y%m%d_%H%M%S")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

print("=" * 80)
print("🎥 Vision Environment Simple Test")
print("=" * 80)
print(f"📁 Output: {OUTPUT_DIR}")


def preprocess_rgbd(rgb, depth):
    """
    RGB-D 전처리: resize to 84x84 and normalize
    
    Args:
        rgb: (H, W, 3) numpy array [0, 255]
        depth: (H, W) numpy array [m]
    
    Returns:
        rgbd: (4, 84, 84) numpy array [0, 1]
    """
    # Resize to 84x84
    rgb_resized = cv2.resize(rgb, (84, 84))
    depth_resized = cv2.resize(depth, (84, 84))
    
    # Normalize RGB
    rgb_norm = rgb_resized.astype(np.float32) / 255.0  # [0, 1]
    
    # Normalize Depth (0.07m ~ 2m)
    depth_norm = np.clip(depth_resized, 0.07, 2.0)
    depth_norm = (depth_norm - 0.07) / 1.93  # [0, 1]
    
    # Stack: (H, W, 4) → (4, H, W)
    rgbd = np.concatenate([rgb_norm, depth_norm[:, :, None]], axis=-1)
    rgbd = np.transpose(rgbd, (2, 0, 1))  # (4, 84, 84)
    
    return rgbd


def save_observation(rgbd, step_idx):
    """Save RGB-D observation as images"""
    # Extract RGB and Depth
    rgb = (rgbd[:3].transpose(1, 2, 0) * 255).astype(np.uint8)  # (3, 84, 84) → (84, 84, 3)
    depth = (rgbd[3] * 255).astype(np.uint8)  # (84, 84)
    
    # Save
    rgb_path = OUTPUT_DIR / f"step_{step_idx:03d}_rgb.png"
    depth_path = OUTPUT_DIR / f"step_{step_idx:03d}_depth.png"
    
    Image.fromarray(rgb).save(rgb_path)
    Image.fromarray(depth).save(depth_path)
    
    print(f"  📸 Saved: {rgb_path.name}, {depth_path.name}")


def setup_robot_cfg():
    """Robot configuration (simplified)"""
    # Use existing USD file directly (skip conversion)
    usd_path = USD_DIR / USD_FILE
    
    if not usd_path.exists():
        print(f"⚠️  USD file not found: {usd_path}")
        print("   Converting URDF to USD...")
        converter_cfg = UrdfConverterCfg(
            asset_path=str(URDF_PATH),
            usd_dir=str(USD_DIR),
            usd_file_name=USD_FILE,
            fix_base=True,
            merge_fixed_joints=False,
            force_usd_conversion=True,
            make_instanceable=True,
            joint_drive=UrdfConverterCfg.JointDriveCfg(
                gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                    stiffness=400.0,
                    damping=40.0,
                ),
            ),
        )
        converter = UrdfConverter(converter_cfg)
        usd_path = converter.usd_path
    else:
        print(f"✅ Using existing USD: {usd_path}")
    
    cfg = ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UsdFileCfg(usd_path=usd_path),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            joint_pos={
                "base_link_to_link1": 0.0,
                "link1_to_link2": 0.0,
                "link2_to_link3": -0.9,
                "link3_to_link4": 0.0,
                "link4_to_link5": 0.0,
                "link5_to_gripper_link": 0.5,
            },
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                stiffness={"base_link_to_link1": 400.0, "link1_to_link2": 5000.0, 
                          "link2_to_link3": 3000.0, "link3_to_link4": 2500.0,
                          "link4_to_link5": 400.0, "link5_to_gripper_link": 250.0},
                damping={"base_link_to_link1": 8.0, "link1_to_link2": 100.0,
                        "link2_to_link3": 80.0, "link3_to_link4": 60.0,
                        "link4_to_link5": 8.0, "link5_to_gripper_link": 5.0},
            ),
        },
    )
    
    return cfg


def setup_camera_cfg():
    """Camera configuration"""
    return CameraCfg(
        prim_path="/World/Robot/gripper_link/camera_link/Camera",
        update_period=0.0,
        height=480,
        width=640,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=1.88,
            focus_distance=400.0,
            horizontal_aperture=3.6,
            clipping_range=(0.07, 10.0),
        ),
    )


def main():
    """Main test function"""
    
    # 1. Setup simulation
    print("\n🌍 Setting up simulation...")
    sim_cfg = SimulationCfg(dt=1.0/60.0)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[1.5, 1.5, 1.0], target=[0.0, 0.0, 0.3])
    
    # 2. Create scene
    print("📐 Creating scene...")
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/ground", ground_cfg)
    
    light_cfg = sim_utils.DomeLightCfg(intensity=3000.0)
    light_cfg.func("/World/light", light_cfg)
    
    # 3. Create robot
    print("🤖 Creating robot...")
    robot_cfg = setup_robot_cfg()
    robot = Articulation(cfg=robot_cfg)
    
    # 4. Create camera
    print("📷 Creating camera...")
    camera_cfg = setup_camera_cfg()
    camera = Camera(cfg=camera_cfg)
    
    # 5. Initialize
    print("🔄 Initializing...")
    sim.reset()
    robot.reset()
    camera.reset()
    
    # Set initial pose
    desired_pos = torch.tensor([[0.0, 0.0, -0.9, 0.0, 0.0, 0.5]], device=sim.device)
    robot.set_joint_position_target(desired_pos)
    robot.write_data_to_sim()
    
    # Stabilize
    print("⏳ Stabilizing for 3 seconds...")
    for _ in range(180):
        robot.set_joint_position_target(desired_pos)
        robot.write_data_to_sim()
        sim.step()
        robot.update(dt=sim_cfg.dt)
    
    print("✅ Initialization complete\n")
    
    # 6. Test vision observation
    print("=" * 80)
    print("🎥 Testing Vision Observation")
    print("=" * 80)
    
    num_samples = 5
    
    for i in range(num_samples):
        print(f"\n📷 Sample {i+1}/{num_samples}")
        
        # Update camera
        camera.update(dt=0.0)
        
        # Get RGB (shape: [1, H, W, 4] - batch, height, width, RGBA)
        rgba_data = camera.data.output["rgb"][0].cpu().numpy()
        rgb = (rgba_data[:, :, :3] * 255).astype(np.uint8)
        
        # Get Depth (shape: [1, H, W, 1])
        depth_data = camera.data.output["distance_to_image_plane"][0].cpu().numpy()
        depth = depth_data[:, :, 0]
        
        print(f"  RGB shape: {rgb.shape}, range: [{rgb.min()}, {rgb.max()}]")
        print(f"  Depth shape: {depth.shape}, range: [{depth.min():.3f}m, {depth.max():.3f}m]")
        
        # Preprocess
        rgbd = preprocess_rgbd(rgb, depth)
        print(f"  RGBD shape: {rgbd.shape}, range: [{rgbd.min():.3f}, {rgbd.max():.3f}]")
        
        # Save
        save_observation(rgbd, i)
        
        # Random action
        if i < num_samples - 1:
            action = torch.randn((1, 6), device=sim.device) * 0.1
            target_pos = robot.data.joint_pos + action
            robot.set_joint_position_target(target_pos)
            robot.write_data_to_sim()
            
            # Execute for 1 second
            for _ in range(60):
                sim.step()
                robot.update(dt=sim_cfg.dt)
    
    # Summary
    print("\n" + "=" * 80)
    print("✅ Test Complete")
    print("=" * 80)
    print(f"📁 Output directory: {OUTPUT_DIR}")
    print(f"📊 Saved {num_samples * 2} images (RGB + Depth)")
    print("\n🔍 Check images:")
    print(f"   ls -lh {OUTPUT_DIR}")
    
    # Cleanup
    simulation_app.close()


if __name__ == "__main__":
    main()
