#!/usr/bin/env python3
"""
Mobile Manipulation VRL Environment
Husky (4 wheels) + RoArm-M3 (6 arm joints) = 10 DOF

4-Phase State Machine:
1. SEARCH: Rotate arm to find cube with hand-eye camera
2. NAVIGATE: Move Husky towards cube
3. APPROACH: Fine positioning
4. GRASP: Arm control to grab cube

Isaac Sim 5.1 Compatible
"""

import numpy as np
from typing import Tuple, Dict, Any, Optional
import gymnasium as gym
from gymnasium import spaces
import cv2

# Note: SimulationApp must be initialized BEFORE importing Isaac modules


class MobileManipulationEnv(gym.Env):
    """
    Mobile Manipulation Environment for VRL
    
    Observation: Hand-eye camera RGB-D (4, 84, 84)
    Action: 10 DOF (4 wheels + 6 arm joints)
    """
    
    # Phase constants
    SEARCH = 0
    NAVIGATE = 1
    APPROACH = 2
    GRASP = 3
    
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}
    
    def __init__(self, render_mode: Optional[str] = None, headless: bool = True):
        super().__init__()
        
        # Import Isaac Sim 5.1 modules
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.sensors.camera import Camera
        
        self._World = World
        self._SingleArticulation = SingleArticulation
        self._add_reference_to_stage = add_reference_to_stage
        self._Camera = Camera
        
        # Paths
        self.robot_usd = "/home/roarm_m3/roarm_isaac_clean/assets/mobile_manipulator/husky_roarm_complete.usd"
        
        # Observation: Hand-eye camera RGB-D
        self.observation_space = spaces.Box(
            low=0.0,
            high=1.0,
            shape=(4, 84, 84),
            dtype=np.float32
        )
        
        # Action: 10 DOF (4 wheels + 6 arm)
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(10,),
            dtype=np.float32
        )
        
        self.render_mode = render_mode
        self.headless = headless
        
        # Episode tracking
        self.episode_step = 0
        self.max_episode_steps = 500  # Longer episodes for mobile manipulation
        
        # Phase tracking
        self.phase = self.SEARCH
        self.cube_detected = False
        self.cube_center = None
        self.cube_distance = None
        
        # Initialize world
        self.world = self._World(stage_units_in_meters=1.0, physics_dt=1.0/60.0)
        
        # Setup scene
        self._setup_scene()
        
        print("✅ MobileManipulationEnv initialized")
        print(f"   Observation: {self.observation_space.shape}")
        print(f"   Action: {self.action_space.shape} (4 wheels + 6 arm)")
    
    def _setup_scene(self):
        """Setup scene with robot, cube, and hand-eye camera"""
        import omni.usd
        from pxr import UsdGeom, UsdPhysics, Gf, UsdLux, PhysxSchema
        
        stage = omni.usd.get_context().get_stage()
        
        # ========== 1. Ground plane ==========
        self.world.scene.add_default_ground_plane()
        print("✅ Ground plane added")
        
        # ========== 2. Lighting ==========
        light_prim = stage.DefinePrim("/World/DomeLight", "DomeLight")
        light = UsdLux.DomeLight(light_prim)
        light.GetIntensityAttr().Set(1500.0)
        print("✅ Lighting added")
        
        # ========== 3. Load mobile manipulator (10 DOF) ==========
        self._add_reference_to_stage(
            usd_path=self.robot_usd,
            prim_path="/World/Robot"
        )
        print("✅ Mobile manipulator USD loaded")
        
        self.world.reset()
        
        # Create articulation
        try:
            self.robot = self._SingleArticulation(prim_path="/World/Robot", name="robot")
            self.world.scene.add(self.robot)
            self.world.reset()
            
            print(f"🔧 Robot DOF: {self.robot.num_dof}")
            print(f"🔧 Joints: {self.robot.dof_names}")
            
            # Home position (arm raised)
            self.home_position = np.zeros(self.robot.num_dof)
            if self.robot.num_dof >= 10:
                # Wheels at 0
                self.home_position[0:4] = 0.0
                # Arm in search position
                self.home_position[4] = 0.0   # base rotation
                self.home_position[5] = 0.3   # shoulder up
                self.home_position[6] = 0.5   # elbow
                self.home_position[7] = 0.2   # wrist
                self.home_position[8] = 0.0   # wrist roll
                self.home_position[9] = 0.5   # gripper open
            
            self.robot.set_joint_positions(self.home_position)
            
        except Exception as e:
            print(f"⚠️ Robot setup failed: {e}")
            self.robot = None
        
        # ========== 4. Add red cube (target object) ==========
        self._create_cube(position=(3.0, 0.5, 0.025))  # 3m away, slightly offset
        
        # ========== 5. Setup hand-eye camera ==========
        self._setup_hand_eye_camera()
        
        # Stabilize physics
        for _ in range(50):
            self.world.step(render=False)
        
        print("✅ Scene setup complete")
    
    def _create_cube(self, position=(3.0, 0.0, 0.025)):
        """Create red cube as target object"""
        import omni.usd
        from pxr import UsdGeom, UsdPhysics, Gf
        
        stage = omni.usd.get_context().get_stage()
        
        cube_path = "/World/Cube"
        cube_geom = UsdGeom.Cube.Define(stage, cube_path)
        cube_geom.GetSizeAttr().Set(0.05)  # 5cm cube
        
        # Position
        xform = UsdGeom.Xformable(cube_geom.GetPrim())
        xform.AddTranslateOp().Set(Gf.Vec3d(*position))
        
        # Red color
        cube_geom.GetDisplayColorAttr().Set([(0.9, 0.1, 0.1)])
        
        # Physics
        UsdPhysics.RigidBodyAPI.Apply(cube_geom.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube_geom.GetPrim())
        UsdPhysics.MassAPI.Apply(cube_geom.GetPrim()).GetMassAttr().Set(0.05)
        
        self.cube_initial_pos = np.array(position)
        print(f"✅ Red cube created at {position}")
    
    def _setup_hand_eye_camera(self):
        """Setup hand-eye camera attached to gripper_link using USD Camera"""
        try:
            import omni.usd
            from pxr import UsdGeom, Gf, Sdf
            
            stage = omni.usd.get_context().get_stage()
            
            # Create camera as child of gripper_link
            camera_path = "/World/Robot/gripper_link/HandEyeCamera"
            
            # Define camera prim
            camera_prim = stage.DefinePrim(camera_path, "Camera")
            camera = UsdGeom.Camera(camera_prim)
            
            # Set camera properties
            camera.GetFocalLengthAttr().Set(18.0)  # Wide angle for close objects
            camera.GetHorizontalApertureAttr().Set(20.955)
            camera.GetVerticalApertureAttr().Set(20.955)
            camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10.0))
            
            # Set local transform (relative to gripper_link)
            # Camera looking forward from gripper
            xformable = UsdGeom.Xformable(camera_prim)
            xformable.ClearXformOpOrder()
            
            # Position: slightly in front and above gripper center
            xformable.AddTranslateOp().Set(Gf.Vec3d(0.05, 0.0, 0.03))
            
            # Rotation: camera looks forward (along X axis of gripper)
            # Rotate -90 degrees around Y to look forward, then tilt down slightly
            xformable.AddRotateXYZOp().Set(Gf.Vec3f(-10.0, -90.0, 0.0))  # Degrees
            
            self.camera_prim_path = camera_path
            self.use_usd_camera = True
            
            # Setup Replicator for image capture
            import omni.replicator.core as rep
            
            self.render_product = rep.create.render_product(camera_path, (84, 84))
            
            # RGB annotator
            self.rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
            self.rgb_annot.attach([self.render_product])
            
            # Depth annotator
            self.depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
            self.depth_annot.attach([self.render_product])
            
            print(f"✅ Hand-eye camera attached to {camera_path}")
            print("   → Camera moves with gripper automatically!")
            
        except Exception as e:
            print(f"⚠️ USD Camera setup failed: {e}")
            import traceback
            traceback.print_exc()
            self.use_usd_camera = False
    
    def _get_camera_image(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get RGB and depth from hand-eye camera"""
        # Step for rendering (camera position updates automatically since attached to gripper)
        self.world.step(render=True)
        
        rgb = np.zeros((84, 84, 3), dtype=np.uint8)
        depth = np.zeros((84, 84), dtype=np.float32)
        
        if hasattr(self, 'use_usd_camera') and self.use_usd_camera:
            try:
                # Get RGB
                rgb_data = self.rgb_annot.get_data()
                if rgb_data is not None and len(rgb_data.shape) >= 3:
                    rgb = rgb_data[:, :, :3].astype(np.uint8)
                
                # Get Depth
                depth_data = self.depth_annot.get_data()
                if depth_data is not None and depth_data.size > 0:
                    depth = depth_data.astype(np.float32)
            except:
                pass
        
        return rgb, depth
    
    def _detect_cube_in_image(self, rgb: np.ndarray, depth: np.ndarray) -> Tuple[bool, Optional[Tuple], Optional[float]]:
        """
        Detect red cube in camera image
        
        Returns:
            detected: bool - whether cube is visible
            center: (x, y) - center pixel coordinates (0-84)
            distance: float - estimated distance from depth
        """
        # Convert to float for processing
        rgb_float = rgb.astype(np.float32)
        
        # Auto-detect scale: if max > 1.0, assume 0-255 scale
        if rgb_float.max() > 1.0:
            # Normalize to 0-1
            rgb_float = rgb_float / 255.0
        
        # Red detection (high R, low G, low B) - now using 0-1 scale
        r_channel = rgb_float[:, :, 0]
        g_channel = rgb_float[:, :, 1]
        b_channel = rgb_float[:, :, 2]
        
        # Red mask: R > 0.6 and R > G*1.3 and R > B*1.3 (relaxed threshold)
        red_mask = (r_channel > 0.5) & (r_channel > g_channel * 1.2) & (r_channel > b_channel * 1.2)
        
        red_pixels = np.sum(red_mask)
        
        if red_pixels > 30:  # Minimum 30 pixels (relaxed for small cube at distance)
            # Find center of red region
            y_coords, x_coords = np.where(red_mask)
            center_x = np.mean(x_coords)
            center_y = np.mean(y_coords)
            
            # Get distance from depth at center
            depth_value = depth[int(center_y), int(center_x)] if depth.size > 0 else 1.0
            
            return True, (center_x, center_y), float(depth_value)
        
        return False, None, None
    
    def _get_observation(self) -> np.ndarray:
        """Get RGB-D observation from hand-eye camera"""
        rgb, depth = self._get_camera_image()
        
        # Detect cube
        self.cube_detected, self.cube_center, self.cube_distance = self._detect_cube_in_image(rgb, depth)
        
        # Normalize RGB to [0, 1]
        rgb_norm = rgb.astype(np.float32) / 255.0
        
        # Normalize depth to [0, 1] (assume max 10m)
        depth_norm = np.clip(depth / 10.0, 0.0, 1.0).astype(np.float32)
        
        # Stack to (4, 84, 84)
        if len(rgb_norm.shape) == 3:
            rgb_chw = np.transpose(rgb_norm, (2, 0, 1))  # HWC -> CHW
        else:
            rgb_chw = np.zeros((3, 84, 84), dtype=np.float32)
        
        if len(depth_norm.shape) == 2:
            depth_chw = depth_norm[np.newaxis, :, :]
        else:
            depth_chw = np.zeros((1, 84, 84), dtype=np.float32)
        
        observation = np.concatenate([rgb_chw, depth_chw], axis=0)
        return observation.astype(np.float32)
    
    def _compute_reward(self) -> Tuple[float, bool, Dict]:
        """Compute reward based on current phase"""
        reward = 0.0
        done = False
        info = {"phase": self.phase, "cube_detected": self.cube_detected}
        
        if self.phase == self.SEARCH:
            reward, done, info = self._reward_search()
        elif self.phase == self.NAVIGATE:
            reward, done, info = self._reward_navigate()
        elif self.phase == self.APPROACH:
            reward, done, info = self._reward_approach()
        elif self.phase == self.GRASP:
            reward, done, info = self._reward_grasp()
        
        # Time penalty
        reward -= 0.01
        
        return reward, done, info
    
    def _reward_search(self) -> Tuple[float, bool, Dict]:
        """Search phase reward: find cube with camera"""
        reward = 0.0
        done = False
        info = {"phase": "SEARCH", "cube_detected": self.cube_detected}
        
        if self.cube_detected:
            # Cube found! Big bonus
            reward += 20.0
            
            # Center bonus (cube centered in view)
            if self.cube_center is not None:
                center_x, center_y = self.cube_center
                offset = np.sqrt((center_x - 42)**2 + (center_y - 42)**2) / 42
                reward += (1.0 - offset) * 10.0
            
            # Transition to NAVIGATE
            self.phase = self.NAVIGATE
            info["phase_transition"] = "SEARCH -> NAVIGATE"
            print(f"🔍 CUBE FOUND! Transitioning to NAVIGATE")
        else:
            # Small exploration reward
            reward += 0.1
        
        return reward, done, info
    
    def _reward_navigate(self) -> Tuple[float, bool, Dict]:
        """Navigate phase reward: move towards cube"""
        reward = 0.0
        done = False
        info = {"phase": "NAVIGATE", "cube_detected": self.cube_detected}
        
        if not self.cube_detected:
            # Lost sight of cube - penalty
            reward -= 2.0
            # Go back to SEARCH
            self.phase = self.SEARCH
            info["phase_transition"] = "NAVIGATE -> SEARCH (lost cube)"
        else:
            # Keep cube in view bonus
            reward += 1.0
            
            # Distance-based reward
            if self.cube_distance is not None:
                info["cube_distance"] = self.cube_distance
                
                # Approaching bonus
                if not hasattr(self, '_prev_cube_distance'):
                    self._prev_cube_distance = self.cube_distance
                
                approach_reward = (self._prev_cube_distance - self.cube_distance) * 50.0
                reward += approach_reward
                self._prev_cube_distance = self.cube_distance
                
                # Close enough - transition to APPROACH
                if self.cube_distance < 0.8:  # 80cm
                    reward += 30.0
                    self.phase = self.APPROACH
                    info["phase_transition"] = "NAVIGATE -> APPROACH"
                    print(f"🚗 Close enough! Transitioning to APPROACH")
        
        return reward, done, info
    
    def _reward_approach(self) -> Tuple[float, bool, Dict]:
        """Approach phase reward: fine positioning"""
        reward = 0.0
        done = False
        info = {"phase": "APPROACH", "cube_detected": self.cube_detected}
        
        if self.cube_distance is not None:
            # Optimal distance for grasping: 25-30cm
            optimal_dist = 0.25
            dist_error = abs(self.cube_distance - optimal_dist)
            reward += (1.0 - min(dist_error * 5, 1.0)) * 5.0
            
            # Ready to grasp
            if 0.2 < self.cube_distance < 0.35:
                reward += 40.0
                self.phase = self.GRASP
                info["phase_transition"] = "APPROACH -> GRASP"
                print(f"📍 In position! Transitioning to GRASP")
        
        return reward, done, info
    
    def _reward_grasp(self) -> Tuple[float, bool, Dict]:
        """Grasp phase reward: grab the cube"""
        reward = 0.0
        done = False
        info = {"phase": "GRASP", "cube_detected": self.cube_detected}
        
        # Check cube height (if lifted)
        try:
            import omni.usd
            from pxr import UsdGeom
            
            stage = omni.usd.get_context().get_stage()
            cube_prim = stage.GetPrimAtPath("/World/Cube")
            if cube_prim.IsValid():
                xform = UsdGeom.Xformable(cube_prim)
                world_transform = xform.ComputeLocalToWorldTransform(0)
                cube_z = world_transform.ExtractTranslation()[2]
                
                if cube_z > self.cube_initial_pos[2] + 0.05:
                    # Cube lifted! SUCCESS!
                    reward += 200.0
                    done = True
                    info["success"] = True
                    print(f"🎉 SUCCESS! Cube lifted!")
        except:
            pass
        
        # Reward for centering cube
        if self.cube_detected and self.cube_center is not None:
            center_x, center_y = self.cube_center
            offset = np.sqrt((center_x - 42)**2 + (center_y - 42)**2) / 42
            reward += (1.0 - offset) * 2.0
        
        return reward, done, info
    
    def reset(self, seed: Optional[int] = None, options: Optional[Dict] = None) -> Tuple[np.ndarray, Dict]:
        """Reset environment"""
        super().reset(seed=seed)
        
        self.world.reset()
        
        # Reset phase
        self.phase = self.SEARCH
        self.episode_step = 0
        self.cube_detected = False
        self._prev_cube_distance = None
        
        # Reset robot
        if self.robot is not None:
            self.robot.set_joint_positions(self.home_position)
            self.robot.set_joint_velocities(np.zeros(self.robot.num_dof))
        
        # Randomize cube position
        if seed is not None:
            np.random.seed(seed)
        
        cube_x = np.random.uniform(2.0, 4.0)
        cube_y = np.random.uniform(-1.0, 1.0)
        self._move_cube([cube_x, cube_y, 0.025])
        
        # Stabilize
        for _ in range(20):
            self.world.step(render=not self.headless)
        
        observation = self._get_observation()
        info = {"phase": self.phase}
        
        return observation, info
    
    def _move_cube(self, position):
        """Move cube to new position"""
        try:
            import omni.usd
            from pxr import UsdGeom, Gf
            
            stage = omni.usd.get_context().get_stage()
            cube_prim = stage.GetPrimAtPath("/World/Cube")
            if cube_prim.IsValid():
                xform = UsdGeom.Xformable(cube_prim)
                # Clear existing ops and set new position
                xform.ClearXformOpOrder()
                xform.AddTranslateOp().Set(Gf.Vec3d(*position))
                self.cube_initial_pos = np.array(position)
        except:
            pass
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """Execute action"""
        self.episode_step += 1
        
        if self.robot is not None:
            # Get current positions
            current_pos = self.robot.get_joint_positions()
            
            # Apply action based on phase
            action = np.clip(action, -1.0, 1.0)
            
            if self.phase == self.SEARCH:
                # Only rotate arm base for searching
                new_pos = current_pos.copy()
                new_pos[4] += action[4] * 0.05  # Arm base rotation
            elif self.phase == self.NAVIGATE:
                # Wheels + keep arm stable
                new_pos = current_pos.copy()
                new_pos[0:4] += action[0:4] * 0.1  # Wheels
            elif self.phase in [self.APPROACH, self.GRASP]:
                # Full control
                new_pos = current_pos + action * 0.03
            else:
                new_pos = current_pos
            
            # Apply position
            self.robot.set_joint_positions(new_pos)
        
        # Step simulation
        for _ in range(4):
            self.world.step(render=not self.headless)
        
        # Get observation and reward
        observation = self._get_observation()
        reward, done, info = self._compute_reward()
        
        truncated = self.episode_step >= self.max_episode_steps
        
        return observation, reward, done, truncated, info
    
    def render(self):
        """Render is handled by Isaac Sim"""
        pass
    
    def close(self):
        """Close environment"""
        if hasattr(self, 'world'):
            self.world.stop()
        print("✅ Environment closed")


# Test standalone
if __name__ == "__main__":
    from isaacsim import SimulationApp
    
    simulation_app = SimulationApp({"headless": False})
    
    env = MobileManipulationEnv(render_mode="human", headless=False)
    
    print("\n🧪 Testing Mobile Manipulation Environment...")
    
    obs, info = env.reset()
    print(f"✅ Reset - Observation: {obs.shape}, Phase: {info['phase']}")
    
    for i in range(100):
        action = env.action_space.sample() * 0.3
        obs, reward, done, truncated, info = env.step(action)
        
        if i % 20 == 0:
            print(f"Step {i}: phase={info.get('phase')}, reward={reward:.2f}, cube_detected={info.get('cube_detected')}")
        
        if done:
            print("🎯 Episode done!")
            break
    
    env.close()
    simulation_app.close()
    print("\n🎉 Test complete!")
