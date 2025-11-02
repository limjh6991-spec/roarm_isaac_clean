#!/usr/bin/env python3
"""
Simple Vision Environment for SAC Training

test_vision_quick.py 기반 - 검증된 코드 활용
- Camera: 작동 확인 ✅
- 전처리: RGB-D (4, 84, 84) ✅
- 로봇: USD 로드 성공 ✅

Gymnasium API 구현:
- observation_space: Box(0, 1, (4, 84, 84))
- action_space: Box(-1, 1, (7,))
- step(), reset(), render(), close()
"""

import argparse
from pathlib import Path
import numpy as np
from typing import Tuple, Dict, Any

import gymnasium as gym
from gymnasium import spaces

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch
import cv2

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.sensors import Camera, CameraCfg


class SimpleVisionEnv(gym.Env):
    """
    Simple Vision Environment for SAC
    
    Based on test_vision_quick.py (verified working)
    """
    
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}
    
    def __init__(self, render_mode=None):
        super().__init__()
        
        # Paths - 절대 경로 사용
        self.usd_path = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/usd/roarm_m3_with_camera_correct.usd"
        
        # Observation space: RGB-D (4, 84, 84)
        self.observation_space = spaces.Box(
            low=0.0,
            high=1.0,
            shape=(4, 84, 84),
            dtype=np.float32
        )
        
        # Action space: 7 joints (6 arm + 1 gripper)
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(7,),
            dtype=np.float32
        )
        
        self.render_mode = render_mode
        
        # Initialize simulation
        self._setup_simulation()
        
        # Episode tracking
        self.episode_step = 0
        self.max_episode_steps = 200
        
        print("✅ SimpleVisionEnv initialized")
        print(f"   Observation: {self.observation_space.shape}")
        print(f"   Action: {self.action_space.shape}")
    
    def _setup_simulation(self):
        """Setup Isaac Lab simulation"""
        # Simulation config (simple)
        sim_cfg = SimulationCfg(dt=1/60.0)
        self.sim = SimulationContext(sim_cfg)
        self.sim.set_camera_view(eye=[1.5, 1.5, 1.0], target=[0.0, 0.0, 0.5])
        
        # Ground and light
        sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
        sim_utils.DomeLightCfg(intensity=3000.0).func("/World/light", sim_utils.DomeLightCfg(intensity=3000.0))
        
        # Robot
        robot_cfg = ArticulationCfg(
            prim_path="/World/RoArm",
            spawn=sim_utils.UsdFileCfg(usd_path=str(self.usd_path)),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0.0, 0.0, 0.0),
                rot=(1.0, 0.0, 0.0, 0.0),
                joint_pos={
                    ".*": 0.0,
                },
            ),
            actuators={
                "all_joints": ImplicitActuatorCfg(
                    joint_names_expr=[".*_to_.*"],  # 실제 조인트 이름 패턴
                    stiffness=50.0,
                    damping=5.0,
                ),
            },
        )
        self.robot = Articulation(robot_cfg)
        
        # Camera
        camera_cfg = CameraCfg(
            prim_path="/World/RoArm/gripper_link/camera_link/Camera",
            update_period=0.0,
            height=256,
            width=256,
            data_types=["rgb", "distance_to_image_plane"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=1.88,
                focus_distance=400.0,
                horizontal_aperture=3.6,
                clipping_range=(0.07, 10.0),
            ),
        )
        self.camera = Camera(camera_cfg)
        
        # Play simulation
        self.sim.reset()
        
        # Home position
        self.home_joint_pos = torch.zeros(self.robot.num_joints, device=self.sim.device)
        self.home_joint_pos[1] = 0.5  # joint2
        self.home_joint_pos[2] = -0.5  # joint3
    
    def _get_observation(self) -> np.ndarray:
        """Get RGB-D observation (4, 84, 84)"""
        # Update camera
        self.camera.update(self.sim.get_physics_dt())
        
        # RGB
        rgb_data = self.camera.data.output["rgb"][0].cpu().numpy()  # (H, W, 3)
        rgb_resized = cv2.resize(rgb_data, (84, 84))
        rgb_norm = rgb_resized.astype(np.float32) / 255.0  # [0, 1]
        
        # Depth
        depth_data = self.camera.data.output["distance_to_image_plane"][0].cpu().numpy()  # (H, W)
        depth_resized = cv2.resize(depth_data, (84, 84))
        depth_clipped = np.clip(depth_resized, 0.1, 2.0)
        depth_norm = ((depth_clipped - 0.1) / 1.9).astype(np.float32)  # [0, 1]
        
        # RGBD: (H, W, 4) → (4, H, W)
        rgbd = np.concatenate([rgb_norm, depth_norm[:, :, None]], axis=-1)
        rgbd = np.transpose(rgbd, (2, 0, 1))  # (4, 84, 84)
        
        return rgbd
    
    def _compute_reward(self) -> float:
        """Simple reward: penalize large movements"""
        # Get joint velocities
        joint_vel = self.robot.data.joint_vel[0].cpu().numpy()
        
        # Penalty for large movements
        motion_penalty = -0.01 * np.sum(np.abs(joint_vel))
        
        # Small survival bonus
        alive_bonus = 0.1
        
        reward = alive_bonus + motion_penalty
        
        return float(reward)
    
    def reset(self, seed=None, options=None) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Reset environment"""
        super().reset(seed=seed)
        
        # Reset episode counter
        self.episode_step = 0
        
        # Reset robot to home
        self.robot.write_joint_state_to_sim(
            self.home_joint_pos.unsqueeze(0),
            torch.zeros_like(self.home_joint_pos).unsqueeze(0)
        )
        
        # Step simulation to apply reset
        for _ in range(10):
            self.sim.step()
        
        # Get observation
        obs = self._get_observation()
        
        info = {
            "episode_step": self.episode_step,
        }
        
        return obs, info
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        """Execute one step"""
        self.episode_step += 1
        
        # Convert action to joint targets
        action = np.clip(action, -1.0, 1.0)
        action_torch = torch.tensor(action, dtype=torch.float32, device=self.sim.device)
        
        # Current joint positions
        current_joint_pos = self.robot.data.joint_pos[0]
        
        # Delta control: ±0.05 rad per step
        delta = action_torch * 0.05
        target_joint_pos = current_joint_pos + delta
        
        # Apply joint limits
        target_joint_pos = torch.clamp(
            target_joint_pos,
            self.robot.data.soft_joint_pos_limits[0, :, 0],
            self.robot.data.soft_joint_pos_limits[0, :, 1]
        )
        
        # Set targets
        self.robot.set_joint_position_target(target_joint_pos.unsqueeze(0))
        
        # Step simulation
        self.sim.step()
        
        # Get observation
        obs = self._get_observation()
        
        # Compute reward
        reward = self._compute_reward()
        
        # Check termination
        terminated = False  # Simple env: no failure condition
        truncated = self.episode_step >= self.max_episode_steps
        
        info = {
            "episode_step": self.episode_step,
        }
        
        return obs, reward, terminated, truncated, info
    
    def render(self):
        """Render (handled by Isaac Sim viewer)"""
        pass
    
    def close(self):
        """Close environment"""
        if hasattr(self, 'sim'):
            self.sim.stop()
        simulation_app.close()


# Test function
def test_env():
    """Test environment"""
    print("=" * 80)
    print("🧪 Testing SimpleVisionEnv")
    print("=" * 80)
    
    env = SimpleVisionEnv()
    
    print("\n1. Reset environment...")
    obs, info = env.reset()
    print(f"✅ Observation shape: {obs.shape}")
    print(f"   Observation dtype: {obs.dtype}")
    print(f"   Observation range: [{obs.min():.3f}, {obs.max():.3f}]")
    
    print("\n2. Running random policy (20 steps)...")
    total_reward = 0.0
    
    for step in range(20):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        
        if step % 5 == 0:
            print(f"   Step {step:3d}: reward={reward:7.3f}")
        
        if terminated or truncated:
            print(f"   Episode ended at step {step}")
            break
    
    print(f"\n✅ Test completed")
    print(f"   Total reward: {total_reward:.3f}")
    print(f"   Final obs shape: {obs.shape}")
    
    env.close()


if __name__ == "__main__":
    test_env()
