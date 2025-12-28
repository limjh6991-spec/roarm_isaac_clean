#!/usr/bin/env python3
"""
Vision Environment Test for Isaac Sim 5.1
Minimal test to verify VRL components work correctly

작성일: 2025-12-27
"""

import argparse
from pathlib import Path
import numpy as np

# Isaac Sim 5.1 native initialization (MUST BE FIRST!)
from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Vision Environment Test 5.1")
parser.add_argument("--headless", action="store_true", help="Run in headless mode")
args_cli = parser.parse_args()

# Initialize simulation app
simulation_app = SimulationApp({
    "headless": args_cli.headless,
    "width": 1280,
    "height": 720,
})

"""After SimulationApp, we can import Isaac modules"""

import torch
import cv2

# Isaac Sim 5.1 Core API
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.sensors.camera import Camera

# USD utilities
from pxr import Gf, UsdGeom
import omni.usd

print("=" * 80)
print("🎥 Vision Environment Test (Isaac Sim 5.1)")
print("=" * 80)


def create_test_scene():
    """Create a minimal test scene with robot, cube, and camera"""
    
    # Create World
    world = World(stage_units_in_meters=1.0)
    
    # Add ground plane
    world.scene.add_ground_plane()
    
    # Robot USD path
    robot_usd = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/usd/roarm_m3_with_camera_correct.usd"
    
    # Add robot
    print("📦 Loading robot...")
    robot = world.scene.add(
        SingleArticulation(
            prim_path="/World/RoArm",
            name="roarm",
            usd_path=robot_usd,
            translation=np.array([0.0, 0.0, 0.0]),
        )
    )
    
    # Add target cube
    print("📦 Adding cube...")
    cube = world.scene.add(
        DynamicCuboid(
            prim_path="/World/Cube",
            name="cube",
            size=0.05,
            color=np.array([1.0, 0.0, 0.0]),  # Red
            translation=np.array([0.3, 0.0, 0.45]),
        )
    )
    
    # Add target marker (green)
    target = world.scene.add(
        VisualCuboid(
            prim_path="/World/Target",
            name="target",
            size=0.06,
            color=np.array([0.0, 1.0, 0.0]),  # Green
            translation=np.array([-0.3, 0.0, 0.45]),
        )
    )
    
    # Add camera
    print("📷 Adding camera...")
    camera = Camera(
        prim_path="/World/Camera",
        name="vision_camera",
        translation=np.array([1.0, 0.0, 0.8]),
        orientation=np.array([0.0, 0.0, 0.707, 0.707]),  # Look at origin
        resolution=(256, 256),
    )
    
    # Reset world
    world.reset()
    
    return world, robot, cube, camera


def test_camera(camera, world, steps=5):
    """Test camera capture"""
    print("\n📷 Testing camera...")
    
    # Initialize camera
    camera.initialize()
    
    for i in range(steps):
        world.step(render=True)
        
        # Get RGB
        rgb = camera.get_rgba()
        if rgb is not None:
            print(f"   Step {i}: RGB shape={rgb.shape}, range=[{rgb.min():.2f}, {rgb.max():.2f}]")
        else:
            print(f"   Step {i}: RGB is None (warming up)")
    
    return rgb


def test_robot(robot, world, steps=20):
    """Test robot joint control"""
    print("\n🤖 Testing robot control...")
    
    # Get joint info
    num_dof = robot.num_dof
    print(f"   Number of DOFs: {num_dof}")
    
    # Get current positions
    joint_positions = robot.get_joint_positions()
    print(f"   Initial joint positions: {joint_positions}")
    
    # Apply small random actions
    for i in range(steps):
        # Random delta
        delta = (np.random.rand(num_dof) - 0.5) * 0.1
        
        # Get current and compute target
        current = robot.get_joint_positions()
        target = current + delta
        
        # Apply
        robot.set_joint_position_targets(target)
        
        # Step
        world.step(render=True)
        
        if i % 5 == 0:
            new_pos = robot.get_joint_positions()
            print(f"   Step {i}: joints={new_pos[:3]}...")


def test_observation(camera, world):
    """Test getting observation (RGB-D style)"""
    print("\n🔍 Testing observation...")
    
    # Step to ensure camera is updated
    for _ in range(5):
        world.step(render=True)
    
    # Get RGB
    rgb = camera.get_rgba()
    if rgb is None:
        print("   ❌ Failed to get RGB")
        return None
    
    # Remove alpha, normalize
    rgb = rgb[:, :, :3]  # (H, W, 3)
    rgb_norm = rgb.astype(np.float32) / 255.0
    
    # Resize to 84x84
    rgb_resized = cv2.resize(rgb_norm, (84, 84))
    
    # For depth, we'll use a placeholder (proper depth requires depth camera setup)
    depth_placeholder = np.zeros((84, 84), dtype=np.float32)
    
    # Combine to RGBD: (4, 84, 84)
    rgbd = np.stack([
        rgb_resized[:, :, 0],
        rgb_resized[:, :, 1],
        rgb_resized[:, :, 2],
        depth_placeholder
    ], axis=0)
    
    print(f"   ✅ Observation shape: {rgbd.shape}")
    print(f"   ✅ Observation range: [{rgbd.min():.3f}, {rgbd.max():.3f}]")
    
    return rgbd


def main():
    """Main test function"""
    
    print("\n1️⃣ Creating scene...")
    world, robot, cube, camera = create_test_scene()
    print("   ✅ Scene created")
    
    print("\n2️⃣ Warming up simulation...")
    for _ in range(10):
        world.step(render=True)
    print("   ✅ Simulation ready")
    
    print("\n3️⃣ Testing components...")
    
    # Test camera
    rgb = test_camera(camera, world)
    
    # Test robot
    test_robot(robot, world)
    
    # Test observation
    obs = test_observation(camera, world)
    
    print("\n" + "=" * 80)
    print("✅ All tests completed!")
    print("=" * 80)
    
    # Summary
    print("\n📊 Summary:")
    print(f"   Robot DOFs: {robot.num_dof}")
    if obs is not None:
        print(f"   Observation: {obs.shape} float32")
    print(f"   Camera: 256x256 RGBA")
    
    print("\n💡 Next steps:")
    print("   1. Update simple_vision_env.py with correct Isaac Sim 5.1 imports")
    print("   2. Add proper depth camera support")
    print("   3. Run full VRL training")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n🧹 Closing simulation...")
        simulation_app.close()
