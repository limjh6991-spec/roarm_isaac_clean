#!/usr/bin/env python3
"""
RoArm-M3 Pick and Place Environment with Vision Support
기존 roarm_pick_place_env.py를 확장하여 vision mode 추가

사용법:
    # Vector mode (기존)
    env = RoArmVisionEnv(obs_mode="vector")
    
    # Vision mode (새로 추가)
    env = RoArmVisionEnv(obs_mode="vision")
"""

import numpy as np
import sys
import os
from typing import Dict, Tuple, Optional
import cv2

# 프로젝트 루트 추가
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# Isaac Sim imports (AppLauncher 이후에 import)
# 이 파일은 AppLauncher가 실행된 후에만 import 가능!

class RoArmVisionEnv:
    """
    Vision-capable wrapper for RoArmPickPlaceEnv
    
    Features:
    - obs_mode="vector": 기존 24-dim observation (기본)
    - obs_mode="vision": RGB-D (4, 84, 84) observation
    
    Vision mode:
    - Camera: D405 on gripper
    - Resolution: 256x256 → 84x84
    - Format: (4, 84, 84) [R, G, B, D] in [0, 1]
    """
    
    def __init__(self, obs_mode="vector", **kwargs):
        """
        Args:
            obs_mode: "vector" (24-dim) or "vision" (4,84,84 RGB-D)
            **kwargs: Additional arguments passed to RoArmPickPlaceEnv
        """
        self.obs_mode = obs_mode
        
        # Lazy import RoArmPickPlaceEnv by reading file directly
        # This avoids import at module level
        import sys
        from pathlib import Path
        
        env_file = Path(__file__).parent / "roarm_pick_place_env.py"
        namespace = {"__name__": "envs.roarm_pick_place_env"}
        
        # Execute file directly to get RoArmPickPlaceEnv class
        with open(env_file, 'r') as f:
            code = f.read()
        exec(code, namespace)
        
        RoArmPickPlaceEnv = namespace['RoArmPickPlaceEnv']
        
        # Initialize base environment
        super().__init__(**kwargs)
        self._base_env = RoArmPickPlaceEnv(**kwargs)
    
    def _setup_camera(self):
        """Setup D405 camera on gripper"""
        print("\n📷 Setting up camera...")
        
        # Get robot prim path from base_env
        robot_prim_path = self.base_env.robot.prim_path
        
        # Camera configuration
        camera_cfg = self.CameraCfg(
            prim_path=f"{robot_prim_path}/gripper_link/camera_link/Camera",
            update_period=0.0,  # Update every frame
            height=256,
            width=256,
            data_types=["rgb", "distance_to_image_plane"],
            spawn=self.sim_utils.PinholeCameraCfg(
                focal_length=1.88,
                focus_distance=400.0,
                horizontal_aperture=3.6,
                clipping_range=(0.07, 10.0),
            ),
        )
        
        self.camera = self.Camera(camera_cfg)
        
        # Initialize camera
        self.base_env.world.reset()
        
        print("✅ Camera initialized")
    
    def _get_vision_observation(self) -> np.ndarray:
        """Get RGB-D observation (4, 84, 84)"""
        # Update camera
        self.camera.update(self.base_env.world.get_physics_dt())
        
        # Get RGB
        rgb_data = self.camera.data.output["rgb"][0].cpu().numpy()  # (256, 256, 3)
        rgb_resized = cv2.resize(rgb_data, (84, 84))
        rgb_norm = rgb_resized.astype(np.float32) / 255.0  # [0, 1]
        
        # Get Depth
        depth_data = self.camera.data.output["distance_to_image_plane"][0].cpu().numpy()  # (256, 256)
        depth_resized = cv2.resize(depth_data, (84, 84))
        depth_clipped = np.clip(depth_resized, 0.07, 2.0)
        depth_norm = ((depth_clipped - 0.07) / 1.93).astype(np.float32)  # [0, 1]
        
        # Stack: (H, W, 4) → (4, H, W)
        rgbd = np.concatenate([rgb_norm, depth_norm[:, :, None]], axis=-1)
        rgbd = np.transpose(rgbd, (2, 0, 1))  # (4, 84, 84)
        
        return rgbd
    
    def reset(self, seed=None, options=None):
        """Reset environment"""
        obs = self.base_env.reset(seed=seed, options=options)
        
        if self.obs_mode == "vision":
            # Get vision observation
            obs = self._get_vision_observation()
        
        return obs, {}
    
    def step(self, action):
        """Execute one step"""
        # Base environment step
        obs, reward, terminated, truncated, info = self.base_env.step(action)
        
        if self.obs_mode == "vision":
            # Replace observation with vision
            obs = self._get_vision_observation()
        
        return obs, reward, terminated, truncated, info
    
    def render(self):
        """Render (handled by Isaac Sim)"""
        return self.base_env.render()
    
    def close(self):
        """Close environment"""
        self.base_env.close()
    
    # Delegate other methods to base_env
    def __getattr__(self, name):
        """Delegate unknown attributes to base environment"""
        return getattr(self.base_env, name)


# Factory function for easy creation
def create_roarm_vision_env(obs_mode="vector", **kwargs):
    """
    Factory function to create RoArm environment
    
    Args:
        obs_mode: "vector" or "vision"
        **kwargs: Environment config parameters
    
    Returns:
        RoArmVisionEnv instance
    """
    return RoArmVisionEnv(obs_mode=obs_mode, **kwargs)


if __name__ == "__main__":
    print("이 모듈은 AppLauncher가 실행된 후에만 import 가능합니다.")
    print("사용 예시:")
    print("  # TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.app import AppLauncher")
    print("  app_launcher = AppLauncher(args)")
    print("  simulation_app = app_launcher.app")
    print("  ")
    print("  from envs.roarm_vision_wrapper import create_roarm_vision_env")
    print("  env = create_roarm_vision_env(obs_mode='vision')")
