#!/usr/bin/env python3
"""
Collect Training Data for Cube Detection
Generates (image, cube_position) pairs from Isaac Sim

Usage:
    /home/roarm_m3/isaacsim/python.sh scripts/train/collect_detection_data.py
"""

import sys
import os
import warnings
import time

warnings.filterwarnings("ignore")
os.environ["GYM_IGNORE_DEPRECATION_WARNINGS"] = "1"

# Initialize Isaac Sim
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

print("="*80)
print("📷 Cube Detection Data Collection")
print("="*80)

sys.path.append(os.path.join(os.path.dirname(__file__), "../.."))

import numpy as np
from pathlib import Path
from datetime import datetime
import pickle

# Isaac Sim imports
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import UsdGeom, UsdLux, Gf
import omni.usd
import omni.replicator.core as rep

print("✅ Imports successful")


class DataCollector:
    def __init__(self, num_samples=10000):
        self.num_samples = num_samples
        self.data = []
        
        # Paths
        self.usd_path = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/usd/roarm_m3_with_camera_correct.usd"
        self.output_dir = Path("data/cube_detection")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Cube position ranges (robot workspace)
        self.cube_x_range = (0.15, 0.30)
        self.cube_y_range = (-0.15, 0.15)
        self.cube_z_range = (0.02, 0.10)
        
        # Initialize world
        self.world = World(stage_units_in_meters=1.0)
        self._setup_scene()
        
    def _setup_scene(self):
        """Setup minimal scene for data collection"""
        stage = omni.usd.get_context().get_stage()
        
        # Ground
        self.world.scene.add_default_ground_plane()
        
        # Lighting
        light_prim = stage.DefinePrim("/World/DomeLight", "DomeLight")
        light = UsdLux.DomeLight(light_prim)
        light.GetIntensityAttr().Set(1500.0)
        
        # Robot
        add_reference_to_stage(usd_path=self.usd_path, prim_path="/World/RoArm")
        
        # Cube
        from envs.scene.scene_builder import create_dynamic_cuboid
        self.cube_prim = create_dynamic_cuboid(
            stage=stage,
            prim_path="/World/Cube",
            position=(0.25, 0.0, 0.05),
            size=0.04,
            color=(0.8, 0.2, 0.2),
            mass=0.1
        )
        
        self.world.reset()
        
        # Robot articulation
        self.robot = SingleArticulation(prim_path="/World/RoArm", name="roarm")
        self.world.scene.add(self.robot)
        self.world.reset()
        
        self.dof_count = self.robot.num_dof
        print(f"🔧 Robot DOF: {self.dof_count}")
        
        # Camera - positioned VERY CLOSE to cube working area
        # Goal: Make cube occupy 20%+ of image pixels
        # Cube range: X=[0.15,0.30], Y=[-0.15,0.15], Z=[0.02,0.10]
        
        # Create camera using Replicator (Isaac Sim 5.1 proper way)
        self.camera = rep.create.camera(
            position=(0.30, 0.12, 0.20),     # Very close to cube
            look_at=(0.22, 0.0, 0.05),       # Look at cube center
            clipping_range=(0.01, 5.0),      # Near 1cm, Far 5m
            focal_length=50                   # Longer focal = zoom effect
        )
        
        # IMPORTANT: Get the camera prim path for render product
        # In Isaac Sim 5.1, we need to use the prim path string
        camera_prim_path = "/Replicator/Camera_Xform/Camera"
        
        self.render_product = rep.create.render_product(camera_prim_path, (224, 224))
        
        # RGB annotator
        self.rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        self.rgb_annot.attach([self.render_product])
        
        # Depth annotator for RGB-D
        self.depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
        self.depth_annot.attach([self.render_product])
        
        print("✅ Scene setup complete (RGB-D camera)")
    
    def _randomize_cube_position(self):
        """Randomize cube position"""
        x = np.random.uniform(*self.cube_x_range)
        y = np.random.uniform(*self.cube_y_range)
        z = np.random.uniform(*self.cube_z_range)
        
        stage = omni.usd.get_context().get_stage()
        cube_prim = stage.GetPrimAtPath("/World/Cube")
        
        if cube_prim.IsValid():
            xformable = UsdGeom.Xformable(cube_prim)
            xformable.ClearXformOpOrder()
            xformable.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        
        return np.array([x, y, z])
    
    def _randomize_robot_pose(self):
        """Keep robot in retracted position to not occlude cube"""
        retracted_positions = np.zeros(self.dof_count)
        # Retract arm upward and back to clear camera view
        retracted_positions[0] = 0.0    # Base centered
        retracted_positions[1] = -0.5   # Shoulder back
        retracted_positions[2] = 0.5    # Elbow up
        retracted_positions[3] = 0.0    # Wrist1
        retracted_positions[4] = 0.0    # Wrist2
        retracted_positions[5] = 0.06   # Gripper open
        
        self.robot.set_joint_positions(retracted_positions)
        self.robot.set_joint_velocities(np.zeros(self.dof_count))
    
    def _get_image(self):
        """Capture RGB-D image (4 channels)"""
        rgb_data = self.rgb_annot.get_data()
        depth_data = self.depth_annot.get_data()
        
        if rgb_data is not None and len(rgb_data.shape) == 3:
            rgb = rgb_data[:, :, :3].copy()  # [H, W, 3]
            
            # Process depth
            if depth_data is not None and len(depth_data.shape) >= 2:
                depth = depth_data.copy()
                if len(depth.shape) == 3:
                    depth = depth[:, :, 0]  # Take first channel
                
                # Normalize depth to 0-255 range (clip at 2m max)
                depth = np.clip(depth, 0, 2.0)  # 2m max distance
                depth = (depth / 2.0 * 255).astype(np.uint8)
                depth = depth[:, :, np.newaxis]  # [H, W, 1]
            else:
                # Fallback: constant depth
                depth = np.full((rgb.shape[0], rgb.shape[1], 1), 128, dtype=np.uint8)
            
            # Combine RGB + D -> RGBD [H, W, 4]
            rgbd = np.concatenate([rgb, depth], axis=2)
            return rgbd
        
        return None
    
    def collect(self):
        """Main data collection loop"""
        print(f"\n📊 Collecting {self.num_samples} samples...")
        
        start_time = time.time()
        
        for i in range(self.num_samples):
            # Randomize scene
            cube_pos = self._randomize_cube_position()
            self._randomize_robot_pose()
            
            # Step simulation to stabilize
            for _ in range(10):
                self.world.step(render=False)
            
            # Render and capture
            self.world.step(render=True)
            
            image = self._get_image()
            
            if image is not None:
                self.data.append({
                    'image': image,
                    'cube_pos': cube_pos.astype(np.float32)
                })
            
            # Progress
            if (i + 1) % 500 == 0:
                elapsed = time.time() - start_time
                rate = (i + 1) / elapsed
                remaining = (self.num_samples - i - 1) / rate
                print(f"   Progress: {i+1}/{self.num_samples} ({100*(i+1)/self.num_samples:.1f}%) - "
                      f"Rate: {rate:.1f} samples/s - ETA: {remaining:.0f}s")
        
        elapsed = time.time() - start_time
        print(f"\n✅ Collected {len(self.data)} samples in {elapsed:.1f}s")
        
    def save(self):
        """Save collected data"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = self.output_dir / f"cube_detection_data_{timestamp}.pkl"
        
        # Split images and labels
        images = np.stack([d['image'] for d in self.data])
        positions = np.stack([d['cube_pos'] for d in self.data])
        
        data = {
            'images': images,
            'positions': positions,
            'num_samples': len(self.data)
        }
        
        with open(filepath, 'wb') as f:
            pickle.dump(data, f)
        
        print(f"✅ Saved to {filepath}")
        print(f"   Images shape: {images.shape}")
        print(f"   Positions shape: {positions.shape}")
        
        return filepath


def main():
    collector = DataCollector(num_samples=10000)
    collector.collect()
    filepath = collector.save()
    
    print("\n" + "="*80)
    print("🎉 Data collection complete!")
    print(f"   Output: {filepath}")
    print("="*80)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        simulation_app.close()
