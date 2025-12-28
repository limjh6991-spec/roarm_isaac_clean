#!/usr/bin/env python3
"""
Pick & Place Reward Functions - Modular 3-Phase Strategy

Phase 1: SEARCH - Find cube in camera view
Phase 2: APPROACH - Move EE toward cube
Phase 3: GRASP - Close gripper on cube
Phase 4: LIFT & PLACE - Move cube to target
"""

import numpy as np
from typing import Dict, Tuple, Optional
from dataclasses import dataclass, field


@dataclass
class RewardConfig:
    """Reward function configuration"""
    # Phase 1: Search
    search_detection_bonus: float = 10.0
    search_time_penalty: float = -0.01
    red_pixel_threshold: float = 0.005  # 0.5% of image
    
    # Phase 2: Approach
    approach_distance_scale: float = 50.0
    approach_proximity_scale: float = 1.0
    approach_reached_bonus: float = 20.0
    reached_threshold: float = 0.15  # meters
    
    # Phase 3: Grasp
    grasp_success_bonus: float = 100.0
    grasp_maintain_bonus: float = 2.0
    grasp_close_bonus: float = 5.0
    grasp_distance_threshold: float = 0.05  # meters
    gripper_close_threshold: float = 0.04
    
    # Phase 4: Lift & Place
    lift_bonus: float = 50.0
    lift_height_threshold: float = 0.05  # meters
    place_distance_scale: float = 30.0
    place_success_bonus: float = 200.0
    place_threshold: float = 0.05  # meters


class PickPlaceReward:
    """
    Modular reward function for Pick & Place with Vision RL
    
    Supports both:
    - Physics-only reward (traditional)
    - Vision-based reward (Active Vision RL)
    """
    
    def __init__(self, config: Optional[RewardConfig] = None, use_vision_reward: bool = False):
        self.config = config or RewardConfig()
        self.use_vision_reward = use_vision_reward
        
        # State tracking
        self._prev_dist_ee_cube: Optional[float] = None
        self._prev_dist_cube_target: Optional[float] = None
        self._stage_reached: bool = False
        self._stage_grasped: bool = False
        self._stage_lifted: bool = False
        
    def reset(self):
        """Reset reward state for new episode"""
        self._prev_dist_ee_cube = None
        self._prev_dist_cube_target = None
        self._stage_reached = False
        self._stage_grasped = False
        self._stage_lifted = False
    
    def compute(
        self,
        ee_pos: np.ndarray,
        cube_pos: np.ndarray,
        target_pos: np.ndarray,
        is_grasped: bool,
        gripper_width: float,
        image: Optional[np.ndarray] = None
    ) -> Tuple[float, bool, Dict]:
        """
        Compute reward for current state
        
        Args:
            ee_pos: End effector position [x, y, z]
            cube_pos: Cube position [x, y, z]
            target_pos: Target position [x, y, z]
            is_grasped: Whether cube is grasped
            gripper_width: Current gripper width
            image: Optional RGB image for vision reward
            
        Returns:
            reward: Total reward
            terminated: Whether episode should end
            info: Additional info dict
        """
        dist_ee_to_cube = np.linalg.norm(ee_pos - cube_pos)
        dist_cube_to_target = np.linalg.norm(cube_pos - target_pos)
        cube_height = cube_pos[2]
        
        # Initialize tracking on first call
        if self._prev_dist_ee_cube is None:
            self._prev_dist_ee_cube = dist_ee_to_cube
            self._prev_dist_cube_target = dist_cube_to_target
        
        reward = 0.0
        terminated = False
        info = {}
        
        # ================================================================
        # Phase 1: SEARCH (Vision-based, optional)
        # ================================================================
        if self.use_vision_reward and image is not None:
            search_reward, cube_visible = self._compute_search_reward(image)
            reward += search_reward
            info['cube_visible'] = cube_visible
        else:
            cube_visible = True  # Assume visible in physics-only mode
        
        # ================================================================
        # Phase 2: APPROACH (EE → Cube)
        # ================================================================
        if not is_grasped:
            approach_reward = self._compute_approach_reward(
                dist_ee_to_cube, cube_visible
            )
            reward += approach_reward
        
        self._prev_dist_ee_cube = dist_ee_to_cube
        
        # ================================================================
        # Phase 3: GRASP
        # ================================================================
        grasp_reward, newly_grasped = self._compute_grasp_reward(
            is_grasped, dist_ee_to_cube, gripper_width
        )
        reward += grasp_reward
        info['newly_grasped'] = newly_grasped
        
        # ================================================================
        # Phase 4: LIFT & PLACE
        # ================================================================
        if is_grasped:
            lift_place_reward, terminated = self._compute_lift_place_reward(
                cube_height, dist_cube_to_target
            )
            reward += lift_place_reward
        
        self._prev_dist_cube_target = dist_cube_to_target
        
        # Build info dict
        info.update({
            'dist_ee_to_cube': dist_ee_to_cube,
            'dist_cube_to_target': dist_cube_to_target,
            'cube_height': cube_height,
            'is_grasped': is_grasped,
            'stage_reached': self._stage_reached,
            'stage_grasped': self._stage_grasped,
            'stage_lifted': self._stage_lifted,
        })
        
        return reward, terminated, info
    
    def _compute_search_reward(self, image: np.ndarray) -> Tuple[float, bool]:
        """
        Phase 1: Search reward based on cube visibility in image
        
        Args:
            image: RGB image [H, W, 3] or [3, H, W]
            
        Returns:
            reward: Search reward
            cube_visible: Whether cube is visible
        """
        # Handle CHW vs HWC format
        if image.shape[0] == 3:  # CHW
            image = np.transpose(image, (1, 2, 0))
        
        # Detect red pixels (cube is red: high R, low G/B)
        red_mask = (image[:, :, 0] > 150) & (image[:, :, 1] < 100) & (image[:, :, 2] < 100)
        red_ratio = np.sum(red_mask) / (image.shape[0] * image.shape[1])
        
        cube_visible = red_ratio > self.config.red_pixel_threshold
        
        if cube_visible:
            reward = self.config.search_detection_bonus
            print(f"👁️ Cube detected! ({red_ratio*100:.2f}% red pixels)")
        else:
            reward = self.config.search_time_penalty
        
        return reward, cube_visible
    
    def _compute_approach_reward(
        self,
        dist_ee_to_cube: float,
        cube_visible: bool = True
    ) -> float:
        """
        Phase 2: Approach reward for moving EE toward cube
        """
        reward = 0.0
        
        # Only give approach reward if cube is visible (or in physics-only mode)
        if not cube_visible:
            return reward
        
        # Distance improvement reward
        if self._prev_dist_ee_cube is not None:
            dist_improvement = self._prev_dist_ee_cube - dist_ee_to_cube
            reward += self.config.approach_distance_scale * dist_improvement
        
        # Proximity bonus
        reward += np.exp(-10.0 * dist_ee_to_cube) * self.config.approach_proximity_scale
        
        # Reached bonus
        if dist_ee_to_cube < self.config.reached_threshold and not self._stage_reached:
            reward += self.config.approach_reached_bonus
            self._stage_reached = True
            print(f"📍 REACHED cube at {dist_ee_to_cube:.3f}m")
        
        return reward
    
    def _compute_grasp_reward(
        self,
        is_grasped: bool,
        dist_ee_to_cube: float,
        gripper_width: float
    ) -> Tuple[float, bool]:
        """
        Phase 3: Grasp reward
        """
        reward = 0.0
        newly_grasped = False
        
        # First grasp bonus
        if is_grasped and not self._stage_grasped:
            reward += self.config.grasp_success_bonus
            self._stage_grasped = True
            newly_grasped = True
            print(f"🤏 GRASPED cube!")
        
        # Maintain grasp bonus
        if is_grasped:
            reward += self.config.grasp_maintain_bonus
        
        # Gripper shaping: encourage closing near cube
        if dist_ee_to_cube < self.config.grasp_distance_threshold:
            if gripper_width < self.config.gripper_close_threshold:
                reward += self.config.grasp_close_bonus
        
        return reward, newly_grasped
    
    def _compute_lift_place_reward(
        self,
        cube_height: float,
        dist_cube_to_target: float
    ) -> Tuple[float, bool]:
        """
        Phase 4: Lift and Place reward
        """
        reward = 0.0
        terminated = False
        
        # Lift bonus
        if cube_height > self.config.lift_height_threshold:
            if not self._stage_lifted:
                reward += self.config.lift_bonus
                self._stage_lifted = True
                print(f"⬆️ LIFTED cube to {cube_height:.3f}m!")
            
            # Move toward target reward
            if self._prev_dist_cube_target is not None:
                target_improvement = self._prev_dist_cube_target - dist_cube_to_target
                reward += self.config.place_distance_scale * target_improvement
        
        # Success condition
        if dist_cube_to_target < self.config.place_threshold and cube_height > self.config.lift_height_threshold:
            reward += self.config.place_success_bonus
            terminated = True
            print(f"🎯 SUCCESS! Cube placed at target!")
        
        return reward, terminated


# Test
if __name__ == "__main__":
    import numpy as np
    
    # Create reward function
    config = RewardConfig()
    reward_fn = PickPlaceReward(config, use_vision_reward=False)
    
    # Test compute
    ee_pos = np.array([0.15, 0.0, 0.3])
    cube_pos = np.array([0.25, 0.0, 0.02])
    target_pos = np.array([0.25, 0.2, 0.15])
    
    reward, terminated, info = reward_fn.compute(
        ee_pos, cube_pos, target_pos,
        is_grasped=False,
        gripper_width=0.06
    )
    
    print(f"Reward: {reward:.2f}")
    print(f"Info: {info}")
