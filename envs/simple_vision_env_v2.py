#!/usr/bin/env python3
"""
Simple Vision Environment for SAC Training
Isaac Sim 5.1 Compatible Version - Fixed Physics

Gymnasium API:
- observation_space: Box(0, 1, (4, 84, 84)) - RGB-D
- action_space: Box(-1, 1, (6,)) - 6 arm joints
"""

import numpy as np
from typing import Tuple, Dict, Any, Optional
import gymnasium as gym
from gymnasium import spaces
import cv2

# Note: SimulationApp must be initialized BEFORE importing Isaac modules
# This should be done in the training script (train_vision_sac.py)

class SimpleVisionEnv(gym.Env):
    """
    Simple Vision-Based Pick and Place Environment
    
    Uses Isaac Sim 5.1 isaacsim.* API
    Physics fixed for proper ground collision
    """
    
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}
    
    def __init__(self, render_mode: Optional[str] = None, headless: bool = True):
        super().__init__()
        
        # Import Isaac Sim 5.1 modules (must be after SimulationApp init)
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation  # 5.1 API
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.sensors.camera import Camera
        import isaacsim.core.utils.prims as prim_utils
        from pxr import UsdPhysics, Gf
        
        self._prim_utils = prim_utils
        self._SingleArticulation = SingleArticulation
        self._Camera = Camera
        
        # Paths
        self.usd_path = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/usd/roarm_m3_with_camera_correct.usd"
        
        # Observation space: RGB-D (4 channels, 84x84)
        self.observation_space = spaces.Box(
            low=0.0,
            high=1.0,
            shape=(4, 84, 84),
            dtype=np.float32
        )
        
        # Action space: 6 joints (RoArm-M3: 6DOF arm)
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(6,),
            dtype=np.float32
        )
        
        self.render_mode = render_mode
        self.headless = headless
        
        # Episode tracking
        self.episode_step = 0
        self.max_episode_steps = 200
        self.target_pos = np.array([0.25, 0.0, 0.05])  # Target on table
        self.cube_pos = np.array([0.15, 0.1, 0.05])  # Cube position
        
        # Initialize world with physics
        self.world = World(stage_units_in_meters=1.0)
        
        # Setup scene
        self._setup_scene()
        
        print("✅ SimpleVisionEnv initialized (Isaac Sim 5.1 - Fixed Physics)")
        print(f"   Observation: {self.observation_space.shape}")
        print(f"   Action: {self.action_space.shape}")
    
    def _setup_scene(self):
        """Setup Isaac Sim 5.1 scene with proper physics"""
        import omni.usd
        from pxr import UsdGeom, UsdPhysics, Gf, Sdf, UsdLux, PhysxSchema
        from isaacsim.core.utils.stage import add_reference_to_stage
        
        stage = omni.usd.get_context().get_stage()
        
        # ========== 1. Add proper ground plane with collision ==========
        # Use world's default ground which has proper collision
        self.world.scene.add_default_ground_plane()
        print("✅ Ground plane with collision added")
        
        # ========== 2. Add lighting ==========
        light_path = "/World/DomeLight"
        light_prim = stage.DefinePrim(light_path, "DomeLight")
        light = UsdLux.DomeLight(light_prim)
        light.GetIntensityAttr().Set(1500.0)
        print("✅ Lighting added")
        
        # ========== 3. Add robot at proper height ==========
        # Robot base is at z=0.0701 in URDF (world_to_base_link joint)
        # So we place robot at origin, it will sit correctly on ground
        robot_prim = add_reference_to_stage(
            usd_path=self.usd_path,
            prim_path="/World/RoArm"
        )
        
        print("✅ Robot USD loaded at ground level")
        
        # ========== CRITICAL FIX: Disable USD drives BEFORE world.reset() ==========
        try:
            from pxr import PhysxSchema, UsdPhysics
            
            # Disable USD joint drives (they have stiffness that pulls joints to 0)
            joint_names_for_disable = ['base_link_to_link1', 'link1_to_link2', 'link2_to_link3', 
                           'link3_to_link4', 'link4_to_link5', 'link5_to_gripper_link']
            
            for joint_name in joint_names_for_disable:
                joint_path = f"/World/RoArm/joints/{joint_name}"
                joint_prim = stage.GetPrimAtPath(joint_path)
                if joint_prim.IsValid():
                    drive_api = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
                    if drive_api:
                        drive_api.GetStiffnessAttr().Set(0.0)
                        drive_api.GetDampingAttr().Set(0.0)
            
            print("✅ USD drives disabled (stiffness=0)")
            
            # Disable gravity for robot links
            link_names_for_gravity = ['base_link', 'link1', 'link2', 'link3', 'link4', 
                                     'link5', 'gripper_link', 'camera_link']
            
            for link_name in link_names_for_gravity:
                link_path = f"/World/RoArm/{link_name}"
                link_prim = stage.GetPrimAtPath(link_path)
                if link_prim.IsValid():
                    physx_api = PhysxSchema.PhysxRigidBodyAPI.Apply(link_prim)
                    physx_api.CreateDisableGravityAttr(True)
            
            print("✅ Gravity disabled for robot links")
        except Exception as e:
            print(f"⚠️ Failed to disable drives/gravity: {e}")
        
        # ========== 4. Add target marker (visual only, no physics) ==========
        target_path = "/World/Target"
        target_geom = UsdGeom.Cylinder.Define(stage, target_path)
        target_geom.GetRadiusAttr().Set(0.03)
        target_geom.GetHeightAttr().Set(0.005)
        target_xform = UsdGeom.Xformable(target_geom.GetPrim())
        target_xform.AddTranslateOp().Set(Gf.Vec3d(
            self.target_pos[0], 
            self.target_pos[1], 
            0.002  # Just above ground
        ))
        # Green color
        target_geom.GetDisplayColorAttr().Set([(0.0, 1.0, 0.0)])
        print("✅ Target marker added")
        
        # ========== 5. Initialize world to load assets ==========
        self.world.reset()
        
        # ========== 6. Create robot articulation ==========
        try:
            self.robot = self._SingleArticulation(
                prim_path="/World/RoArm",
                name="roarm"
            )
            self.world.scene.add(self.robot)
            self.world.reset()
            
            # Get robot info
            dof_count = self.robot.num_dof
            print(f"🔧 Robot DOF: {dof_count}")
            print(f"🔧 Joint names: {self.robot.dof_names}")
            
            # Note: USD drives and gravity already disabled BEFORE world.reset()
            
            # Set initial home position and SAVE it
            self.home_position = np.zeros(dof_count)
            self.home_position[1] = 0.5   # Shoulder up
            self.home_position[2] = -0.8  # Elbow bent
            self.home_position[3] = 0.3   # Wrist slightly bent
            self.robot.set_joint_positions(self.home_position)
            self.robot.set_joint_velocities(np.zeros(dof_count))
            print(f"🔧 Home position set: {self.home_position}")
            
            # CRITICAL: Must step simulation BEFORE setting gains
            self.world.step(render=False)
            
            # ========== FIX: Set gains via ArticulationController AFTER world.step() ==========
            try:
                controller = self.robot.get_articulation_controller()
                
                # High stiffness for position control
                kp = np.array([10000.0, 50000.0, 40000.0, 20000.0, 15000.0, 10000.0], dtype=np.float32)
                kd = np.array([1000.0, 5000.0, 4000.0, 2000.0, 1500.0, 1000.0], dtype=np.float32)
                
                controller.set_gains(kp, kd)
                print(f"✅ ArticulationController gains set: kp_max={kp.max()}, kd_max={kd.max()}")
                
                self._controller = controller
            except Exception as e:
                print(f"⚠️ Failed to set controller gains: {e}")
                self._controller = None
            
            # Wait for physics to stabilize
            for _ in range(100):
                self.world.step(render=False)
            
            # Get end effector position
            try:
                ee_pos = self._get_end_effector_pos()
                print(f"🔧 End effector initial pos: {ee_pos}")
            except:
                pass
            
            # Adjust action space if needed
            if dof_count != self.action_space.shape[0]:
                print(f"⚠️ Adjusting action space from {self.action_space.shape[0]} to {dof_count}")
                self.action_space = spaces.Box(
                    low=-1.0,
                    high=1.0,
                    shape=(dof_count,),
                    dtype=np.float32
                )
        except Exception as e:
            print(f"⚠️ Robot articulation setup failed: {e}")
            import traceback
            traceback.print_exc()
            self.robot = None
        
        # ========== 7. Setup camera ==========
        try:
            import omni.replicator.core as rep
            
            self.camera_rep = rep.create.camera(
                position=(0.5, 0.3, 0.4),
                look_at=(0.15, 0.0, 0.1)
            )
            self.render_product = rep.create.render_product(self.camera_rep, (84, 84))
            
            # RGB annotator
            self.rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
            self.rgb_annot.attach([self.render_product])
            
            print("✅ Camera setup complete")
        except Exception as e:
            print(f"⚠️ Camera setup failed: {e}")
            self.camera_rep = None
        
        print("✅ Scene setup complete")
    
    def _get_end_effector_pos(self) -> np.ndarray:
        """Get end effector position (gripper_link)"""
        if self.robot is None:
            return np.array([0.0, 0.0, 0.0])
        
        try:
            # Get the gripper link world pose
            import omni.usd
            from pxr import UsdGeom, Gf
            
            stage = omni.usd.get_context().get_stage()
            gripper_prim = stage.GetPrimAtPath("/World/RoArm/gripper_link")
            
            if gripper_prim.IsValid():
                xformable = UsdGeom.Xformable(gripper_prim)
                world_transform = xformable.ComputeLocalToWorldTransform(0)
                translation = world_transform.ExtractTranslation()
                return np.array([translation[0], translation[1], translation[2]])
        except Exception as e:
            pass
        
        return np.array([0.15, 0.0, 0.2])  # Default position
    
    def _get_observation(self) -> np.ndarray:
        """Get RGB-D observation from camera"""
        try:
            if self.camera_rep is not None:
                # Get RGB from replicator
                rgb_data = self.rgb_annot.get_data()
                if rgb_data is not None and len(rgb_data.shape) == 3:
                    rgb = rgb_data[:, :, :3]  # Remove alpha
                    rgb_norm = rgb.astype(np.float32) / 255.0
                    rgb_chw = np.transpose(rgb_norm, (2, 0, 1))  # HWC -> CHW
                    
                    # Fake depth for now
                    depth_chw = np.full((1, 84, 84), 0.5, dtype=np.float32)
                    
                    observation = np.concatenate([rgb_chw, depth_chw], axis=0)
                    return observation
        except Exception as e:
            pass
        
        return np.zeros((4, 84, 84), dtype=np.float32)
    
    def _compute_reward(self) -> Tuple[float, bool, Dict]:
        """
        Compute reward with dense multi-stage shaping for effective learning.
        
        v2.0: Enhanced reward to address non-learning issue identified at 400K steps.
        Key improvements:
        1. Dense distance-based shaping (not just -distance)
        2. Progressive stage bonuses (one-time triggers)
        3. Movement diversity bonus
        4. Time penalty to encourage faster solutions
        """
        # Get end effector position
        ee_pos = self._get_end_effector_pos()
        
        # Distance to target
        distance = np.linalg.norm(ee_pos - self.target_pos)
        
        # Initialize stage tracking on first call
        if not hasattr(self, '_stage_triggered'):
            self._stage_triggered = {
                '15cm': False, '10cm': False, '7cm': False,
                '5cm': False, '3cm': False, '2cm': False
            }
            self._prev_distance = distance
            self._prev_action = None
        
        reward = 0.0
        
        # ═══════════════════════════════════════════════════════════
        # 1. Dense Distance Reward (improvement-based, not absolute)
        # ═══════════════════════════════════════════════════════════
        if self._prev_distance is not None:
            distance_improvement = self._prev_distance - distance
            # Amplified improvement reward (100x multiplier)
            distance_reward = 100.0 * distance_improvement
            reward += distance_reward
        self._prev_distance = distance
        
        # ═══════════════════════════════════════════════════════════
        # 2. Exponential Proximity Bonus (always active, encourages approach)
        # ═══════════════════════════════════════════════════════════
        # exp(-10*d): 0.3m->0.05, 0.1m->0.37, 0.05m->0.61, 0.02m->0.82
        proximity_bonus = np.exp(-10.0 * distance) * 2.0
        reward += proximity_bonus
        
        # ═══════════════════════════════════════════════════════════
        # 3. Progressive Stage Bonuses (one-time, prevents oscillation gaming)
        # ═══════════════════════════════════════════════════════════
        stage_bonus = 0.0
        
        if distance < 0.15 and not self._stage_triggered['15cm']:
            stage_bonus += 5.0
            self._stage_triggered['15cm'] = True
        
        if distance < 0.10 and not self._stage_triggered['10cm']:
            stage_bonus += 10.0
            self._stage_triggered['10cm'] = True
        
        if distance < 0.07 and not self._stage_triggered['7cm']:
            stage_bonus += 15.0
            self._stage_triggered['7cm'] = True
        
        if distance < 0.05 and not self._stage_triggered['5cm']:
            stage_bonus += 25.0
            self._stage_triggered['5cm'] = True
        
        if distance < 0.03 and not self._stage_triggered['3cm']:
            stage_bonus += 40.0
            self._stage_triggered['3cm'] = True
        
        if distance < 0.02 and not self._stage_triggered['2cm']:
            stage_bonus += 50.0
            self._stage_triggered['2cm'] = True
        
        reward += stage_bonus
        
        # ═══════════════════════════════════════════════════════════
        # 4. Time Penalty (encourages faster solutions)
        # ═══════════════════════════════════════════════════════════
        time_penalty = -0.1  # Small per-step penalty
        reward += time_penalty
        
        # ═══════════════════════════════════════════════════════════
        # 5. Success Detection & Big Reward
        # ═══════════════════════════════════════════════════════════
        success = distance < 0.02  # 2cm threshold
        done = success or (self.episode_step >= self.max_episode_steps)
        
        if success:
            # Huge success bonus + time bonus (faster = better)
            time_bonus = max(0, (self.max_episode_steps - self.episode_step) * 0.5)
            reward += 200.0 + time_bonus
            print(f"🎯 SUCCESS! Distance: {distance:.4f}, Steps: {self.episode_step}, TimeBonus: {time_bonus:.1f}")
        
        info = {
            "distance": distance,
            "success": success,
            "episode_step": self.episode_step,
            "ee_pos": ee_pos.tolist(),
            "target_pos": self.target_pos.tolist(),
            "stage_bonus": stage_bonus,
            "proximity_bonus": proximity_bonus,
        }
        
        return reward, done, info
    
    def reset(self, seed: Optional[int] = None, options: Optional[Dict] = None) -> Tuple[np.ndarray, Dict]:
        """Reset environment"""
        super().reset(seed=seed)
        
        # Reset world
        self.world.reset()
        
        # Reset robot to home position
        if self.robot is not None:
            try:
                # Home position: arm raised and stable
                joint_positions = np.zeros(self.robot.num_dof)
                joint_positions[1] = 0.5   # Shoulder up
                joint_positions[2] = -0.8  # Elbow bent
                joint_positions[3] = 0.3   # Wrist slightly bent
                
                # Set position (PD drive will maintain it)
                self.robot.set_joint_positions(joint_positions)
            except Exception as e:
                print(f"⚠️ Robot reset failed: {e}")
        
        # Reset episode counter
        self.episode_step = 0
        
        # Reset stage tracking for new episode
        if hasattr(self, '_stage_triggered'):
            self._stage_triggered = {
                '15cm': False, '10cm': False, '7cm': False,
                '5cm': False, '3cm': False, '2cm': False
            }
        self._prev_distance = None
        
        # More stabilization steps - set position each step!
        for _ in range(100):
            if self.robot is not None:
                self.robot.set_joint_positions(joint_positions)
                self.robot.set_joint_velocities(np.zeros(self.robot.num_dof))
            self.world.step(render=not self.headless)
        
        # Get initial observation
        observation = self._get_observation()
        
        # Get info
        ee_pos = self._get_end_effector_pos()
        info = {
            "ee_pos": ee_pos.tolist(),
            "target_pos": self.target_pos.tolist()
        }
        
        return observation, info
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """Execute action by directly setting joint positions"""
        if self.robot is not None:
            try:
                # Get current positions
                current_positions = self.robot.get_joint_positions()
                
                # Apply action as delta (scale from [-1,1])
                action_scale = 0.02  # Small steps for smooth motion
                new_positions = current_positions + action * action_scale
                
                # Clip to joint limits
                new_positions = np.clip(new_positions, -1.57, 1.57)
                
                # Set positions BEFORE physics step
                self.robot.set_joint_positions(new_positions)
                self.robot.set_joint_velocities(np.zeros(self.robot.num_dof))
            except Exception as e:
                new_positions = self.home_position if hasattr(self, 'home_position') else np.zeros(6)
        
        # Step simulation with position enforcement
        for _ in range(4):
            self.world.step(render=not self.headless)
            # Enforce position AFTER each physics step (critical for stability)
            if self.robot is not None:
                self.robot.set_joint_positions(new_positions)
                self.robot.set_joint_velocities(np.zeros(self.robot.num_dof))
        
        # Update episode counter
        self.episode_step += 1
        
        # Get observation and reward
        observation = self._get_observation()
        reward, terminated, info = self._compute_reward()
        
        truncated = False
        
        return observation, reward, terminated, truncated, info
    
    def render(self):
        """Render is handled by Isaac Sim"""
        if self.render_mode == "rgb_array":
            return self._get_observation()[:3]
        return None
    
    def close(self):
        """Close environment"""
        if hasattr(self, 'world'):
            self.world.stop()
        print("✅ Environment closed")


# Test standalone
if __name__ == "__main__":
    from isaacsim import SimulationApp
    
    simulation_app = SimulationApp({"headless": False})
    
    env = SimpleVisionEnv(render_mode="human", headless=False)
    
    print("\n🧪 Testing environment...", flush=True)
    
    obs, info = env.reset()
    print(f"✅ Reset - Observation shape: {obs.shape}", flush=True)
    print(f"   EE pos: {info.get('ee_pos')}", flush=True)
    print(f"   Target: {info.get('target_pos')}", flush=True)
    
    for i in range(20):
        action = env.action_space.sample() * 0.5  # Smaller random actions
        obs, reward, terminated, truncated, info = env.step(action)
        print(f"✅ Step {i+1}: reward={reward:.3f}, dist={info['distance']:.3f}", flush=True)
        
        if terminated:
            print("🎯 Episode terminated!", flush=True)
            break
    
    env.close()
    simulation_app.close()
    print("\n🎉 Test complete!", flush=True)
