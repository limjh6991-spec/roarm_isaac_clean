#!/usr/bin/env python3
"""
Quick Vision Test - RGB-D 전처리만 테스트

기존 성공한 test_roarm_with_camera_isaaclab.py에
Vision 전처리만 추가

작성일: 2025-11-02
"""

import argparse
from pathlib import Path
import numpy as np

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
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

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.sensors import Camera, CameraCfg

# ==============================================================================
# Paths & Config
# ==============================================================================

PROJECT_DIR = Path(__file__).parents[2]
USD_PATH = PROJECT_DIR / "assets" / "roarm_m3" / "usd" / "roarm_m3_with_camera_correct.usd"
OUTPUT_DIR = PROJECT_DIR / "output" / "vision_test_quick" / datetime.now().strftime("%Y%m%d_%H%M%S")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

print("=" * 80)
print("🎥 Quick Vision Test")
print("=" * 80)
print(f"📁 USD: {USD_PATH}")
print(f"📁 Output: {OUTPUT_DIR}")

# ==============================================================================
# Vision Preprocessing
# ==============================================================================

def preprocess_rgbd(rgb, depth):
    """
    RGB-D 전처리: 84x84 resize + normalize
    
    Args:
        rgb: (H, W, 3) [0, 255]
        depth: (H, W) [meters]
    
    Returns:
        rgbd: (4, 84, 84) [0, 1]
    """
    # Resize
    rgb_84 = cv2.resize(rgb, (84, 84))
    depth_84 = cv2.resize(depth, (84, 84))
    
    # Normalize
    rgb_norm = rgb_84.astype(np.float32) / 255.0
    depth_norm = np.clip(depth_84, 0.07, 2.0)
    depth_norm = (depth_norm - 0.07) / 1.93
    
    # Stack (H,W,4) → (4,H,W)
    rgbd = np.concatenate([rgb_norm, depth_norm[:, :, None]], axis=-1)
    rgbd = rgbd.transpose(2, 0, 1)
    
    return rgbd


def save_obs(rgbd, idx):
    """Save observation"""
    rgb = (rgbd[:3].transpose(1, 2, 0) * 255).astype(np.uint8)
    depth = (rgbd[3] * 255).astype(np.uint8)
    
    Image.fromarray(rgb).save(OUTPUT_DIR / f"sample_{idx:02d}_rgb.png")
    Image.fromarray(depth).save(OUTPUT_DIR / f"sample_{idx:02d}_depth.png")
    
    print(f"  ✅ Saved sample {idx}")


# ==============================================================================
# Main
# ==============================================================================

def main():
    # 1. Simulation
    print("\n🌍 Setup...")
    sim_cfg = SimulationCfg(dt=1.0/60.0)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[1.5, 1.5, 1.0], target=[0.0, 0.0, 0.3])
    
    # 2. Scene
    print("📐 Scene...")
    sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=3000.0).func("/World/light", sim_utils.DomeLightCfg(intensity=3000.0))
    
    # 3. Robot (직접 USD 사용)
    print("🤖 Robot...")
    robot_cfg = ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UsdFileCfg(usd_path=str(USD_PATH)),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            joint_pos={"base_link_to_link1": 0.0, "link1_to_link2": 0.0, 
                       "link2_to_link3": -0.9, "link3_to_link4": 0.0,
                       "link4_to_link5": 0.0, "link5_to_gripper_link": 0.5},
        ),
        actuators={"arm": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            stiffness={"base_link_to_link1": 400.0, "link1_to_link2": 5000.0,
                      "link2_to_link3": 3000.0, "link3_to_link4": 2500.0,
                      "link4_to_link5": 400.0, "link5_to_gripper_link": 250.0},
            damping={"base_link_to_link1": 8.0, "link1_to_link2": 100.0,
                    "link2_to_link3": 80.0, "link3_to_link4": 60.0,
                    "link4_to_link5": 8.0, "link5_to_gripper_link": 5.0},
        )},
    )
    robot = Articulation(cfg=robot_cfg)
    
    # 4. Camera
    print("📷 Camera...")
    camera_cfg = CameraCfg(
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
    camera = Camera(cfg=camera_cfg)
    
    # 5. Initialize
    print("🔄 Initialize...")
    sim.reset()
    robot.reset()
    camera.reset()
    
    # Stabilize
    target = torch.tensor([[0.0, 0.0, -0.9, 0.0, 0.0, 0.5]], device=sim.device)
    robot.set_joint_position_target(target)
    robot.write_data_to_sim()
    
    for _ in range(100):
        robot.set_joint_position_target(target)
        robot.write_data_to_sim()
        sim.step()
        robot.update(dt=sim_cfg.dt)
    
    print("✅ Ready\n")
    
    # 6. Test Vision
    print("=" * 80)
    print("🎥 Vision Test")
    print("=" * 80)
    
    for i in range(5):
        print(f"\n📷 Sample {i+1}/5")
        
        # Get camera data
        camera.update(dt=0.0)
        
        rgba = camera.data.output["rgb"][0].cpu().numpy()
        rgb = (rgba[:, :, :3] * 255).astype(np.uint8)
        
        depth_data = camera.data.output["distance_to_image_plane"][0].cpu().numpy()
        depth = depth_data[:, :, 0]
        
        print(f"  RGB: {rgb.shape}, [{rgb.min()}, {rgb.max()}]")
        print(f"  Depth: {depth.shape}, [{depth.min():.3f}, {depth.max():.3f}]m")
        
        # Preprocess
        rgbd = preprocess_rgbd(rgb, depth)
        print(f"  RGBD: {rgbd.shape}, [{rgbd.min():.3f}, {rgbd.max():.3f}]")
        
        # Save
        save_obs(rgbd, i)
        
        # Random motion
        if i < 4:
            action = torch.randn((1, 6), device=sim.device) * 0.05
            target = robot.data.joint_pos + action
            robot.set_joint_position_target(target)
            robot.write_data_to_sim()
            
            for _ in range(60):
                sim.step()
                robot.update(dt=sim_cfg.dt)
    
    print("\n" + "=" * 80)
    print("✅ Test Complete!")
    print("=" * 80)
    print(f"📁 {OUTPUT_DIR}")
    print(f"📊 10 images saved (5 RGB + 5 Depth)")
    
    simulation_app.close()


if __name__ == "__main__":
    main()
