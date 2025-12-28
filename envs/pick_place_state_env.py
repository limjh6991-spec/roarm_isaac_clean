#!/usr/bin/env python3
"""
State-based Pick & Place Environment
Uses direct state (joint positions, EE pos, cube pos) instead of vision

This environment is more reliable for initial learning.
"""

import sys
import os
import warnings
from typing import Optional, Dict, Tuple
import numpy as np

warnings.filterwarnings("ignore")
os.environ["GYM_IGNORE_DEPRECATION_WARNINGS"] = "1"

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import gymnasium as gym
from gymnasium import spaces

# Isaac Sim imports
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import UsdGeom, UsdLux, Gf
import omni.usd

from envs.scene.scene_builder import create_dynamic_cuboid


class PickPlaceStateEnv(gym.Env):
    """
    State-based Pick & Place Environment
    
    Observation: [joint_pos (6), ee_pos (3), cube_pos (3), gripper (1)] = 13D
    Action: [joint_delta (6), gripper (1)] = 7D
    """
    
    metadata = {"render_modes": ["human", "rgb_array"]}
    
    def __init__(self, render_mode: Optional[str] = None, headless: bool = True):
        super().__init__()
        
        # Paths
        self.usd_path = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/usd/roarm_m3_with_camera_correct.usd"
        
        # State: joint_pos (6) + ee_pos (3) + cube_pos (3) + gripper (1) = 13D
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(13,),
            dtype=np.float32
        )
        
        # Action: joint_delta (6) + gripper (1) = 7D
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(7,),
            dtype=np.float32
        )
        
        self.render_mode = render_mode
        self.headless = headless
        
        # Episode tracking
        self.episode_step = 0
        self.max_episode_steps = 200
        
        # Positions
        self.cube_initial_pos = np.array([0.25, 0.0, 0.05])
        self.target_pos = np.array([0.25, 0.15, 0.10])  # Target to place
        
        # Gripper state
        self.gripper_width = 0.06
        self.gripper_target = 0.06
        self.is_grasped = False
        self.grasp_threshold = 0.03
        
        # Stage tracking
        self._stage_reached = False
        self._stage_grasped = False
        self._stage_lifted = False
        
        # Initialize world
        self.world = World(stage_units_in_meters=1.0)
        self._setup_scene()
        
        # Joint limits
        self.joint_lower = np.array([-2.0, -1.0, -2.0, -1.5, -1.5, 0.0])
        self.joint_upper = np.array([2.0, 1.5, 1.0, 1.5, 1.5, 0.06])
        
        self._prev_dist_ee_cube = None
        self._prev_dist_cube_target = None
        
        print("✅ PickPlaceStateEnv initialized")
        print(f"   Observation: 13D (state-based)")
        print(f"   Action: 7D (joint delta + gripper)")
    
    def _setup_scene(self):
        """Setup minimal scene"""
        stage = omni.usd.get_context().get_stage()
        
        # Ground
        self.world.scene.add_default_ground_plane()
        
        # Light
        light_prim = stage.DefinePrim("/World/DomeLight", "DomeLight")
        light = UsdLux.DomeLight(light_prim)
        light.GetIntensityAttr().Set(1500.0)
        
        # Robot
        add_reference_to_stage(usd_path=self.usd_path, prim_path="/World/RoArm")
        
        # Cube
        self.cube_prim = create_dynamic_cuboid(
            stage=stage,
            prim_path="/World/Cube",
            position=tuple(self.cube_initial_pos),
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
    
    def reset(self, seed=None, options=None) -> Tuple[np.ndarray, Dict]:
        super().reset(seed=seed)
        
        self.episode_step = 0
        self.is_grasped = False
        self._stage_reached = False
        self._stage_grasped = False
        self._stage_lifted = False
        self._prev_dist_ee_cube = None
        self._prev_dist_cube_target = None
        
        # Reset robot
        home_positions = np.array([0.0, 0.3, -0.5, 0.0, 0.0, 0.06])
        self.robot.set_joint_positions(home_positions)
        self.robot.set_joint_velocities(np.zeros(self.dof_count))
        
        # Reset cube
        stage = omni.usd.get_context().get_stage()
        cube_prim = stage.GetPrimAtPath("/World/Cube")
        if cube_prim.IsValid():
            xformable = UsdGeom.Xformable(cube_prim)
            xformable.ClearXformOpOrder()
            
            # Random position with small variation
            offset = np.random.uniform(-0.03, 0.03, 2)
            pos = self.cube_initial_pos.copy()
            pos[0] += offset[0]
            pos[1] += offset[1]
            
            xformable.AddTranslateOp().Set(Gf.Vec3d(*pos))
        
        # Gripper
        self.gripper_width = 0.06
        self.gripper_target = 0.06
        
        # Step to stabilize
        for _ in range(20):
            self.world.step(render=False)
        
        obs = self._get_observation()
        
        return obs, {"cube_pos": self._get_cube_pos()}
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        self.episode_step += 1
        
        # Process action
        joint_delta = action[:6] * 0.05  # Scale down for smooth motion
        gripper_action = action[6]
        
        # Current positions
        current_positions = self.robot.get_joint_positions()[:6]
        new_positions = current_positions + joint_delta
        
        # Clip to limits
        new_positions = np.clip(new_positions, self.joint_lower, self.joint_upper)
        
        # Gripper control: -1 = close, +1 = open
        self.gripper_target = 0.03 + 0.03 * gripper_action  # [0, 0.06]
        self.gripper_width = self.gripper_target
        
        # Apply action
        full_positions = np.concatenate([new_positions, [self.gripper_target]])
        self.robot.set_joint_positions(full_positions)
        
        # Step simulation
        for _ in range(4):
            self.world.step(render=False)
        
        # Get observation and reward
        obs = self._get_observation()
        reward, terminated, info = self._compute_reward()
        
        truncated = self.episode_step >= self.max_episode_steps
        
        return obs, reward, terminated, truncated, info
    
    def _get_observation(self) -> np.ndarray:
        """Get state-based observation"""
        joint_pos = self.robot.get_joint_positions()[:6]
        ee_pos = self._get_end_effector_pos()
        cube_pos = self._get_cube_pos()
        gripper = np.array([self.gripper_width])
        
        # Normalize
        joint_pos_norm = (joint_pos - self.joint_lower) / (self.joint_upper - self.joint_lower) * 2 - 1
        ee_pos_norm = ee_pos / 0.5  # Normalize to ~[-1, 1]
        cube_pos_norm = cube_pos / 0.5
        gripper_norm = gripper / 0.06 * 2 - 1
        
        return np.concatenate([joint_pos_norm, ee_pos_norm, cube_pos_norm, gripper_norm]).astype(np.float32)
    
    def _get_end_effector_pos(self) -> np.ndarray:
        """Get EE position from robot FK"""
        stage = omni.usd.get_context().get_stage()
        ee_prim = stage.GetPrimAtPath("/World/RoArm/link5")
        if ee_prim.IsValid():
            xformable = UsdGeom.Xformable(ee_prim)
            world_tf = xformable.ComputeLocalToWorldTransform(0)
            translation = world_tf.ExtractTranslation()
            return np.array([translation[0], translation[1], translation[2]])
        return np.array([0.2, 0.0, 0.1])
    
    def _get_cube_pos(self) -> np.ndarray:
        """Get cube position"""
        stage = omni.usd.get_context().get_stage()
        cube_prim = stage.GetPrimAtPath("/World/Cube")
        if cube_prim.IsValid():
            xformable = UsdGeom.Xformable(cube_prim)
            world_tf = xformable.ComputeLocalToWorldTransform(0)
            translation = world_tf.ExtractTranslation()
            return np.array([translation[0], translation[1], translation[2]])
        return self.cube_initial_pos.copy()
    
    def _check_grasp(self) -> bool:
        """Check if cube is grasped"""
        ee_pos = self._get_end_effector_pos()
        cube_pos = self._get_cube_pos()
        dist = np.linalg.norm(ee_pos - cube_pos)
        return dist < self.grasp_threshold and self.gripper_width < 0.03
    
    def _compute_reward(self) -> Tuple[float, bool, Dict]:
        """Multi-stage reward for pick & place"""
        ee_pos = self._get_end_effector_pos()
        cube_pos = self._get_cube_pos()
        
        dist_ee_cube = np.linalg.norm(ee_pos - cube_pos)
        dist_cube_target = np.linalg.norm(cube_pos - self.target_pos)
        cube_height = cube_pos[2]
        
        # Initialize tracking
        if self._prev_dist_ee_cube is None:
            self._prev_dist_ee_cube = dist_ee_cube
            self._prev_dist_cube_target = dist_cube_target
        
        reward = 0.0
        done = False
        info = {"stage": "REACH", "dist_ee_cube": dist_ee_cube}
        
        # Check grasp
        self.is_grasped = self._check_grasp()
        
        # Stage 1: REACH
        if not self.is_grasped:
            # Distance improvement
            dist_improvement = self._prev_dist_ee_cube - dist_ee_cube
            reward += 50.0 * dist_improvement
            
            # Proximity bonus
            reward += np.exp(-10.0 * dist_ee_cube) * 1.0
            
            # Stage bonus
            if dist_ee_cube < 0.08 and not self._stage_reached:
                reward += 20.0
                self._stage_reached = True
                print(f"📍 REACHED at {dist_ee_cube:.3f}m")
        
        # Stage 2: GRASP
        if dist_ee_cube < self.grasp_threshold:
            close_bonus = (0.06 - self.gripper_width) / 0.06 * 5.0
            reward += close_bonus
            
            if self.is_grasped and not self._stage_grasped:
                reward += 100.0
                self._stage_grasped = True
                print("🤏 GRASPED!")
        
        # Stage 3: LIFT
        if self.is_grasped:
            info["stage"] = "LIFT"
            
            if cube_height > 0.08:
                reward += 30.0 * (cube_height - 0.05)
                
                if cube_height > 0.12 and not self._stage_lifted:
                    reward += 50.0
                    self._stage_lifted = True
                    print(f"⬆️ LIFTED to {cube_height:.3f}m")
        
        # Stage 4: PLACE
        if self._stage_lifted:
            info["stage"] = "PLACE"
            dist_improvement = self._prev_dist_cube_target - dist_cube_target
            reward += 30.0 * dist_improvement
            
            if dist_cube_target < 0.05:
                reward += 200.0
                done = True
                print("🎯 SUCCESS!")
        
        # Penalties
        reward -= 0.05  # Time penalty
        
        self._prev_dist_ee_cube = dist_ee_cube
        self._prev_dist_cube_target = dist_cube_target
        
        info["stage_reached"] = self._stage_reached
        info["stage_grasped"] = self._stage_grasped
        info["stage_lifted"] = self._stage_lifted
        info["is_grasped"] = self.is_grasped
        
        return reward, done, info
    
    def close(self):
        if hasattr(self, 'world'):
            self.world.stop()
        simulation_app.close()


# Test
if __name__ == "__main__":
    print("="*80)
    print("🧪 Testing PickPlaceStateEnv")
    print("="*80)
    
    env = PickPlaceStateEnv()
    
    obs, info = env.reset()
    print(f"\nInitial observation shape: {obs.shape}")
    print(f"Observation: {obs}")
    
    for step in range(50):
        action = env.action_space.sample() * 0.5
        obs, reward, done, truncated, info = env.step(action)
        
        if step % 10 == 0:
            print(f"Step {step}: reward={reward:.3f}, stage={info['stage']}")
        
        if done:
            print(f"Done at step {step}!")
            break
    
    print("\n✅ Test complete!")
    env.close()
