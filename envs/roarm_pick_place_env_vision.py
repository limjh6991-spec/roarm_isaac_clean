#!/usr/bin/env python3
"""
RoArm-M3 Pick and Place Environment with Vision (RGB-D)
- Observation: RGB-D 이미지 (4, 84, 84)
- Reward: Dense reward with vision guidance
- Camera: D405 mounted on gripper
"""

import numpy as np
import torch
from typing import Optional, Tuple
import gymnasium as gym
from gymnasium import spaces

from omni.isaac.lab.envs import ManagerBasedRLEnv, ManagerBasedRLEnvCfg
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.sensors import Camera
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.utils.math import quat_to_euler_xyz

import cv2


class RoArmPickPlaceVisionEnv(ManagerBasedRLEnv):
    """
    Vision-based Pick and Place Environment
    
    Observation Space:
        - Type: Image (RGB-D)
        - Shape: (4, 84, 84)
        - Channels: [R, G, B, D]
        - Range: [0, 1]
    
    Action Space:
        - Type: Continuous
        - Shape: (7,)
        - Range: [-1, 1]
        - Mapping: [joint1, joint2, joint3, joint4, joint5, joint6, gripper]
    
    Reward:
        - Reach object: distance-based
        - Grasp object: contact-based
        - Lift object: height-based
        - Place object: goal distance-based
        - Vision guidance: object in view bonus
    """
    
    def __init__(self, cfg: ManagerBasedRLEnvCfg, render_mode: Optional[str] = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        
        # Robot
        self.robot: Articulation = self.scene["robot"]
        self.robot_dof = self.robot.num_dof
        
        # Camera
        self.camera: Camera = self.scene["camera"]
        
        # Object tracking
        self.object_pos = None
        self.object_in_view = False
        
        # Episode tracking
        self.episode_length = 0
        self.max_episode_length = cfg.episode_length_s * cfg.sim.dt
        
        # Task state
        self.task_phase = 0  # 0: reach, 1: grasp, 2: lift, 3: place
        
        # Define observation and action spaces
        self.observation_space = spaces.Box(
            low=0.0,
            high=1.0,
            shape=(4, 84, 84),  # RGB-D
            dtype=np.float32
        )
        
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(self.robot_dof,),
            dtype=np.float32
        )
        
        print(f"✅ RoArmPickPlaceVisionEnv initialized")
        print(f"   Observation: {self.observation_space.shape}")
        print(f"   Action: {self.action_space.shape}")
    
    def _get_observations(self) -> dict:
        """Get RGB-D observation from camera"""
        # Update camera
        self.camera.update(self.sim.get_physics_dt())
        
        # Get RGB (H, W, 4) → (H, W, 3)
        rgba = self.camera.data.rgb
        if isinstance(rgba, torch.Tensor):
            rgba = rgba.cpu().numpy()
        rgb = rgba[0, :, :, :3]  # Remove batch dim and alpha
        
        # Get Depth (H, W)
        depth = self.camera.data.depth
        if isinstance(depth, torch.Tensor):
            depth = depth.cpu().numpy()
        depth = depth[0, :, :]  # Remove batch dim
        
        # Resize to 84x84
        rgb_resized = cv2.resize(rgb, (84, 84))
        depth_resized = cv2.resize(depth, (84, 84))
        
        # Normalize
        rgb_norm = rgb_resized / 255.0  # [0, 1]
        depth_norm = np.clip(depth_resized, 0.1, 2.0)  # 10cm ~ 2m
        depth_norm = (depth_norm - 0.1) / 1.9  # [0, 1]
        
        # Stack: (H, W, 4) → (4, H, W)
        rgbd = np.concatenate([rgb_norm, depth_norm[:, :, None]], axis=-1)
        rgbd = np.transpose(rgbd, (2, 0, 1))  # (4, 84, 84)
        
        return {
            "policy": torch.tensor(rgbd, dtype=torch.float32, device=self.device)
        }
    
    def _get_rewards(self) -> torch.Tensor:
        """Compute dense reward with vision guidance"""
        # Get robot state
        joint_pos = self.robot.data.joint_pos
        gripper_pos = self._get_gripper_position()
        
        # Phase-based reward
        reward = 0.0
        
        # Phase 0: Reach object
        if self.task_phase == 0:
            dist_to_object = torch.norm(gripper_pos - self.object_pos)
            reach_reward = -dist_to_object * 10.0
            reward += reach_reward
            
            # Vision guidance: object in view
            if self._is_object_in_view():
                reward += 0.1
                self.object_in_view = True
            
            # Transition to grasp
            if dist_to_object < 0.05:
                self.task_phase = 1
                reward += 10.0
        
        # Phase 1: Grasp object
        elif self.task_phase == 1:
            grasp_success = self._check_grasp()
            if grasp_success:
                reward += 20.0
                self.task_phase = 2
            else:
                reward -= 0.1
        
        # Phase 2: Lift object
        elif self.task_phase == 2:
            object_height = self.object_pos[2]
            lift_reward = object_height * 50.0
            reward += lift_reward
            
            # Transition to place
            if object_height > 0.2:
                self.task_phase = 3
                reward += 30.0
        
        # Phase 3: Place object at goal
        elif self.task_phase == 3:
            dist_to_goal = torch.norm(self.object_pos - self.goal_pos)
            place_reward = -dist_to_goal * 20.0
            reward += place_reward
            
            # Success
            if dist_to_goal < 0.05:
                reward += 100.0
        
        # Penalty for excessive motion
        joint_vel = self.robot.data.joint_vel
        motion_penalty = -0.001 * torch.norm(joint_vel)
        reward += motion_penalty
        
        return torch.tensor([reward], device=self.device)
    
    def _get_dones(self) -> Tuple[torch.Tensor, torch.Tensor]:
        """Check termination conditions"""
        # Time limit
        time_out = self.episode_length >= self.max_episode_length
        
        # Success: object at goal
        dist_to_goal = torch.norm(self.object_pos - self.goal_pos)
        success = dist_to_goal < 0.05 and self.task_phase == 3
        
        # Failure: object fell
        object_fell = self.object_pos[2] < 0.05
        
        terminated = success or object_fell
        truncated = time_out
        
        return (
            torch.tensor([terminated], device=self.device),
            torch.tensor([truncated], device=self.device)
        )
    
    def _reset_idx(self, env_ids: torch.Tensor):
        """Reset environment"""
        super()._reset_idx(env_ids)
        
        # Reset robot to home position
        home_positions = torch.tensor(
            [0.0, 0.5, -0.5, 0.0, 0.0, 0.0, 0.01],
            device=self.device
        )
        self.robot.set_joint_position_target(home_positions)
        
        # Reset object position (random)
        object_x = np.random.uniform(0.2, 0.4)
        object_y = np.random.uniform(-0.2, 0.2)
        object_z = 0.1
        self.object_pos = torch.tensor(
            [object_x, object_y, object_z],
            device=self.device
        )
        
        # Reset goal position (random)
        goal_x = np.random.uniform(0.2, 0.4)
        goal_y = np.random.uniform(-0.2, 0.2)
        goal_z = 0.1
        self.goal_pos = torch.tensor(
            [goal_x, goal_y, goal_z],
            device=self.device
        )
        
        # Reset task state
        self.task_phase = 0
        self.episode_length = 0
        self.object_in_view = False
    
    def _apply_action(self, actions: torch.Tensor):
        """Apply action to robot"""
        # Clip actions to [-1, 1]
        actions = torch.clamp(actions, -1.0, 1.0)
        
        # Map to joint position targets
        joint_pos = self.robot.data.joint_pos
        delta = actions * 0.05  # 5cm per step
        target_pos = joint_pos + delta
        
        # Apply limits
        target_pos = torch.clamp(
            target_pos,
            self.robot.data.joint_limits[:, 0],
            self.robot.data.joint_limits[:, 1]
        )
        
        self.robot.set_joint_position_target(target_pos)
    
    def _get_gripper_position(self) -> torch.Tensor:
        """Get gripper position in world frame"""
        # Get gripper link index
        gripper_link_idx = self.robot.find_bodies("gripper_link")[0][0]
        
        # Get position
        gripper_pos = self.robot.data.body_pos_w[0, gripper_link_idx, :]
        
        return gripper_pos
    
    def _is_object_in_view(self) -> bool:
        """Check if object is visible in camera view"""
        # Simple heuristic: object within 0.5m of gripper
        # (Real implementation would use image segmentation)
        gripper_pos = self._get_gripper_position()
        dist = torch.norm(gripper_pos - self.object_pos)
        return dist < 0.5
    
    def _check_grasp(self) -> bool:
        """Check if object is grasped"""
        # Simple heuristic: gripper close to object and gripper closed
        gripper_pos = self._get_gripper_position()
        dist = torch.norm(gripper_pos - self.object_pos)
        
        gripper_joint_pos = self.robot.data.joint_pos[0, -1]  # Last joint
        gripper_closed = gripper_joint_pos < 0.005
        
        return dist < 0.03 and gripper_closed
    
    def step(self, action: torch.Tensor) -> Tuple[dict, torch.Tensor, torch.Tensor, torch.Tensor, dict]:
        """Execute one environment step"""
        # Apply action
        self._apply_action(action)
        
        # Step simulation
        self.sim_context.step(render=self.render_mode is not None)
        
        # Update episode length
        self.episode_length += 1
        
        # Get observation
        obs = self._get_observations()
        
        # Get reward
        reward = self._get_rewards()
        
        # Check done
        terminated, truncated = self._get_dones()
        
        # Info
        info = {
            "task_phase": self.task_phase,
            "object_in_view": self.object_in_view,
            "episode_length": self.episode_length
        }
        
        return obs, reward, terminated, truncated, info


# Environment Configuration
class RoArmPickPlaceVisionEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for RoArm Pick Place Vision Environment"""
    
    # Simulation
    sim = SimulationContext.Config(
        dt=1/60.0,  # 60 FPS
        substeps=1,
        gravity=(0.0, 0.0, -9.81),
    )
    
    # Episode
    episode_length_s = 10.0  # 10 seconds
    
    # Scene
    scene = {
        "robot": SceneEntityCfg(
            prim_path="/World/roarm_m3_with_d405",
            spawn_cfg={
                "urdf_path": "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3_with_d405.urdf",
            }
        ),
        "camera": SceneEntityCfg(
            prim_path="/World/roarm_m3_with_d405/camera_link/Camera",
            spawn_cfg={
                "resolution": (256, 256),
                "frequency": 20,  # 20 FPS
            }
        )
    }
    
    # Viewer
    viewer = {
        "eye": (1.5, 1.5, 1.0),
        "lookat": (0.0, 0.0, 0.5),
    }


# Factory function for registration
def create_roarm_vision_env(cfg: dict, render_mode: Optional[str] = None, **kwargs):
    """Create RoArm Vision Environment"""
    env_cfg = RoArmPickPlaceVisionEnvCfg()
    return RoArmPickPlaceVisionEnv(env_cfg, render_mode=render_mode, **kwargs)
