#!/usr/bin/env python3
"""
Pick & Place Vision Environment for SAC Training
Isaac Sim 5.1 Compatible - End-to-End VRL with CnnPolicy

Gymnasium API:
- observation_space: Box(-1, 1, (3, 84, 84)) - RGB (Isaac Lab style)
- action_space: Box(-1, 1, (7,)) - 6 arm joints + 1 gripper

Modular Architecture:
- Reward: envs/rewards/pick_place_reward.py
- Physics: envs/utils/physics_utils.py
"""

import numpy as np
from typing import Tuple, Dict, Any, Optional
import gymnasium as gym
from gymnasium import spaces

# Modular components
from envs.rewards.pick_place_reward import PickPlaceReward, RewardConfig
from envs.utils.physics_utils import PhysicsQueryInterface, detect_cube_in_image

# Note: SimulationApp must be initialized BEFORE importing Isaac modules


class PickPlaceVisionEnv(gym.Env):
    """
    Vision-Based Pick and Place Environment with Gripper Control
    
    Extends SimpleVisionEnv to add:
    - Dynamic cube with physics
    - Gripper control (7th action dimension)
    - Grasp detection
    - Multi-stage rewards (reach, grasp, lift, place)
    """
    
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}
    
    def __init__(self, render_mode: Optional[str] = None, headless: bool = True):
        super().__init__()
        
        # Import Isaac Sim 5.1 modules
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.sensors.camera import Camera
        import isaacsim.core.utils.prims as prim_utils
        from pxr import UsdPhysics, Gf
        
        self._prim_utils = prim_utils
        self._SingleArticulation = SingleArticulation
        self._Camera = Camera
        
        # Paths
        self.usd_path = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/usd/roarm_m3_with_camera_correct.usd"
        
        # Observation space: RGB only (3 channels, 84x84)
        # Isaac Lab 방식: End-to-End CNN Policy에서 RGB 직접 학습
        self.observation_space = spaces.Box(
            low=-1.0,  # Mean subtraction can give negative values
            high=1.0,
            shape=(3, 84, 84),
            dtype=np.float32
        )
        
        # Action space: 6 arm joints + 1 gripper = 7
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(7,),  # Extended for gripper
            dtype=np.float32
        )
        
        self.render_mode = render_mode
        self.headless = headless
        
        # Episode tracking
        self.episode_step = 0
        self.max_episode_steps = 300  # Longer for pick & place
        
        # Positions - cube far in front of robot for camera visibility test
        self.cube_initial_pos = np.array([1.0, 0.0, 0.02])  # 1m in front of robot
        self.target_pos = np.array([0.8, 0.2, 0.10])  # Target position (to the side, lifted)
        
        # Gripper state
        self.gripper_width = 0.06  # Max open width
        self.gripper_target = 0.06  # Current target
        self.is_grasped = False
        self.grasp_threshold = 0.03  # 3cm distance for grasp
        
        # Initialize modular reward function (Vision reward enabled for Active Vision RL)
        reward_config = RewardConfig()
        self.reward_fn = PickPlaceReward(reward_config, use_vision_reward=True)
        
        # Legacy stage tracking (for backward compatibility)
        self._stage_reached = False
        self._stage_grasped = False
        self._stage_lifted = False
        
        # Initialize world
        self.world = World(stage_units_in_meters=1.0)
        
        # Setup scene with cube
        self._setup_scene()
        
        print("✅ PickPlaceVisionEnv initialized (Isaac Sim 5.1)")
        print(f"   Observation: {self.observation_space.shape}")
        print(f"   Action: {self.action_space.shape} (6 arm + 1 gripper)")
    
    def _setup_scene(self):
        """Setup scene with robot, cube, and target"""
        import omni.usd
        from pxr import UsdGeom, UsdPhysics, Gf, Sdf, UsdLux, PhysxSchema
        from isaacsim.core.utils.stage import add_reference_to_stage
        from envs.scene.scene_builder import create_dynamic_cuboid, create_physics_material
        
        stage = omni.usd.get_context().get_stage()
        
        # 1. Ground plane
        self.world.scene.add_default_ground_plane()
        print("✅ Ground plane added")
        
        # 2. Lighting
        light_path = "/World/DomeLight"
        light_prim = stage.DefinePrim(light_path, "DomeLight")
        light = UsdLux.DomeLight(light_prim)
        light.GetIntensityAttr().Set(1500.0)
        print("✅ Lighting added")
        
        # 3. Robot
        robot_prim = add_reference_to_stage(
            usd_path=self.usd_path,
            prim_path="/World/RoArm"
        )
        print("✅ Robot loaded")
        
        # Disable USD drives
        self._disable_usd_drives(stage)
        
        # 4. Create physics material
        create_physics_material(stage)
        print("✅ Physics material created")
        
        # 5. Dynamic Cube (the object to pick)
        self.cube_prim = create_dynamic_cuboid(
            stage=stage,
            prim_path="/World/Cube",
            position=tuple(self.cube_initial_pos),
            size=0.04,  # 4cm cube
            color=(0.8, 0.2, 0.2),  # Red
            mass=0.1
        )
        print("✅ Dynamic cube added")
        
        # 6. Target marker (visual only)
        target_path = "/World/Target"
        target_geom = UsdGeom.Sphere.Define(stage, target_path)
        target_geom.GetRadiusAttr().Set(0.02)
        target_xform = UsdGeom.Xformable(target_geom.GetPrim())
        target_xform.AddTranslateOp().Set(Gf.Vec3d(*self.target_pos))
        target_geom.GetDisplayColorAttr().Set([(0.0, 1.0, 0.0)])  # Green
        print("✅ Target marker added")
        
        # Initialize world
        self.world.reset()
        
        # 7. Setup robot articulation
        try:
            self.robot = self._SingleArticulation(
                prim_path="/World/RoArm",
                name="roarm"
            )
            self.world.scene.add(self.robot)
            self.world.reset()
            
            dof_count = self.robot.num_dof
            print(f"🔧 Robot DOF: {dof_count}")
            
            # Home position - robot standing upright, gripper pointing forward/down
            # base_link vertical, link1 vertical, link2~gripper horizontal forward
            self.home_position = np.zeros(dof_count)
            self.home_position[0] = 0.0    # Base rotation: forward
            self.home_position[1] = -0.3   # Shoulder: slightly back (upright)
            self.home_position[2] = 0.8    # Elbow: bent forward
            self.home_position[3] = 0.5    # Wrist1: tilt down
            self.home_position[4] = 0.0    # Wrist2: neutral
            self.home_position[5] = 0.06   # Gripper open
            
            self.robot.set_joint_positions(self.home_position)
            self.robot.set_joint_velocities(np.zeros(dof_count))
            
            self.world.step(render=False)
            
            # Set controller gains
            try:
                controller = self.robot.get_articulation_controller()
                kp = np.array([10000.0, 50000.0, 40000.0, 20000.0, 15000.0, 8000.0], dtype=np.float32)
                kd = np.array([1000.0, 5000.0, 4000.0, 2000.0, 1500.0, 800.0], dtype=np.float32)
                controller.set_gains(kp, kd)
                self._controller = controller
            except Exception as e:
                print(f"⚠️ Controller gains error: {e}")
                self._controller = None
            
            # Stabilize
            for _ in range(100):
                self.world.step(render=False)
            
            # Initialize dynamic control for physics position queries
            try:
                from omni.isaac.dynamic_control import _dynamic_control
                self._dc = _dynamic_control.acquire_dynamic_control_interface()
                print("✅ Dynamic Control interface acquired")
            except Exception as e:
                print(f"⚠️ Dynamic Control init failed: {e}")
                self._dc = None
            
        except Exception as e:
            print(f"⚠️ Robot setup failed: {e}")
            import traceback
            traceback.print_exc()
            self.robot = None
            self._dc = None
        
        # 8. Hand-Eye Camera setup (mounted on top of gripper, looking down)
        # Camera is placed above gripper_link with downward view for better cube visibility
        try:
            import omni.replicator.core as rep
            from pxr import UsdGeom, Gf
            import omni.usd
            
            stage = omni.usd.get_context().get_stage()
            
            # Mount camera on gripper_link (top position, looking down at workspace)
            camera_parent_path = "/World/RoArm/gripper_link"
            camera_prim_path = f"{camera_parent_path}/hand_camera"
            
            # Add camera prim
            camera_prim = stage.DefinePrim(camera_prim_path, "Camera")
            camera = UsdGeom.Camera(camera_prim)
            
            # Set camera properties
            camera.GetFocalLengthAttr().Set(18.0)  # Wider FOV for close-range
            camera.GetHorizontalApertureAttr().Set(20.955)
            camera.GetVerticalApertureAttr().Set(20.955)
            camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10.0))
            
            # Position camera on TOP of gripper, looking DOWN
            # Offset: Z +0.05m (above gripper), looking down at workspace
            xformable = UsdGeom.Xformable(camera_prim)
            xformable.ClearXformOpOrder()
            
            # Translate: up from gripper link
            translate_op = xformable.AddTranslateOp()
            translate_op.Set(Gf.Vec3d(0.0, 0.0, 0.06))  # 6cm above gripper
            
            # Rotate: X -90° to look down (USD camera looks -Z by default)
            # Then Y 180° to flip orientation
            rotate_xyz_op = xformable.AddRotateXYZOp()
            rotate_xyz_op.Set(Gf.Vec3f(-90.0, 0.0, 180.0))  # Look down at workspace
            
            # Create render product
            self.render_product = rep.create.render_product(camera_prim_path, (84, 84))
            
            self.rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
            self.rgb_annot.attach([self.render_product])
            
            self.camera_rep = camera_prim  # Store reference
            self._camera_prim_path = camera_prim_path
            
            print(f"✅ Hand-Eye Camera setup at {camera_prim_path} (top-down view)")
        except Exception as e:
            print(f"⚠️ Camera setup failed: {e}")
            import traceback
            traceback.print_exc()
            self.camera_rep = None
        
        print("✅ Scene setup complete")
    
    def _disable_usd_drives(self, stage):
        """Disable USD joint drives"""
        from pxr import PhysxSchema, UsdPhysics
        
        joint_names = ['base_link_to_link1', 'link1_to_link2', 'link2_to_link3',
                      'link3_to_link4', 'link4_to_link5', 'link5_to_gripper_link']
        
        for joint_name in joint_names:
            joint_path = f"/World/RoArm/joints/{joint_name}"
            joint_prim = stage.GetPrimAtPath(joint_path)
            if joint_prim.IsValid():
                drive_api = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
                if drive_api:
                    drive_api.GetStiffnessAttr().Set(0.0)
                    drive_api.GetDampingAttr().Set(0.0)
        
        # Disable gravity for robot links
        link_names = ['base_link', 'link1', 'link2', 'link3', 'link4',
                     'link5', 'gripper_link', 'camera_link']
        
        for link_name in link_names:
            link_path = f"/World/RoArm/{link_name}"
            link_prim = stage.GetPrimAtPath(link_path)
            if link_prim.IsValid():
                physx_api = PhysxSchema.PhysxRigidBodyAPI.Apply(link_prim)
                physx_api.CreateDisableGravityAttr(True)
    
    def _get_end_effector_pos(self) -> np.ndarray:
        """Get gripper position from physics simulation using dynamic_control"""
        if self.robot is None:
            return np.array([0.0, 0.0, 0.0])
        
        # Use dynamic_control API for real-time physics position
        if hasattr(self, '_dc') and self._dc is not None:
            try:
                rigid_body = self._dc.get_rigid_body("/World/RoArm/gripper_link")
                if rigid_body:
                    pose = self._dc.get_rigid_body_pose(rigid_body)
                    return np.array([pose.p.x, pose.p.y, pose.p.z])
            except:
                pass
        
        # Fallback: USD API (may not reflect physics state)
        try:
            from pxr import UsdGeom, Usd
            import omni.usd
            
            stage = omni.usd.get_context().get_stage()
            gripper_prim = stage.GetPrimAtPath("/World/RoArm/gripper_link")
            
            if gripper_prim.IsValid():
                xformable = UsdGeom.Xformable(gripper_prim)
                world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                translation = world_transform.ExtractTranslation()
                return np.array([translation[0], translation[1], translation[2]])
        except:
            pass
        
        return np.array([0.15, 0.0, 0.2])
    
    def _get_cube_pos(self) -> np.ndarray:
        """Get cube world position from physics simulation"""
        # Use dynamic_control API for real-time physics position
        if hasattr(self, '_dc') and self._dc is not None:
            try:
                rigid_body = self._dc.get_rigid_body("/World/Cube")
                if rigid_body:
                    pose = self._dc.get_rigid_body_pose(rigid_body)
                    return np.array([pose.p.x, pose.p.y, pose.p.z])
            except:
                pass
        
        # Fallback: USD API
        try:
            from pxr import UsdGeom, Usd
            import omni.usd
            
            stage = omni.usd.get_context().get_stage()
            cube_prim = stage.GetPrimAtPath("/World/Cube")
            
            if cube_prim.IsValid():
                xformable = UsdGeom.Xformable(cube_prim)
                world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                translation = world_transform.ExtractTranslation()
                return np.array([translation[0], translation[1], translation[2]])
        except:
            pass
        
        return self.cube_initial_pos.copy()
    
    def _get_observation(self) -> np.ndarray:
        """Get RGB observation with Isaac Lab style normalization"""
        try:
            if self.camera_rep is not None:
                rgb_data = self.rgb_annot.get_data()
                if rgb_data is not None and len(rgb_data.shape) == 3:
                    rgb = rgb_data[:, :, :3]
                    # Isaac Lab 방식: /255 정규화 + 평균 빼기
                    rgb_norm = rgb.astype(np.float32) / 255.0
                    rgb_norm -= np.mean(rgb_norm, axis=(0, 1), keepdims=True)
                    rgb_chw = np.transpose(rgb_norm, (2, 0, 1))  # HWC -> CHW
                    return rgb_chw
        except:
            pass
        
        return np.zeros((3, 84, 84), dtype=np.float32)
    
    def _check_grasp(self) -> bool:
        """Check if cube is grasped"""
        ee_pos = self._get_end_effector_pos()
        cube_pos = self._get_cube_pos()
        
        dist = np.linalg.norm(ee_pos - cube_pos)
        gripper_closed = self.gripper_width < 0.04  # Gripper mostly closed
        
        return dist < self.grasp_threshold and gripper_closed
    
    def _compute_reward(self) -> Tuple[float, bool, Dict]:
        """
        Multi-stage reward for Pick & Place
        
        Delegates to modular PickPlaceReward class (envs/rewards/pick_place_reward.py)
        
        Stages:
        1. Reach: EE approaches cube
        2. Grasp: Gripper closes on cube
        3. Lift: Cube lifted from table
        4. Place: Cube near target
        """
        ee_pos = self._get_end_effector_pos()
        cube_pos = self._get_cube_pos()
        
        # Update grasp state
        was_grasped = self.is_grasped
        self.is_grasped = self._check_grasp()
        
        # Get current image for vision reward (optional)
        image = self._get_raw_image() if self.reward_fn.use_vision_reward else None
        
        # Delegate to modular reward function
        reward, terminated, info = self.reward_fn.compute(
            ee_pos=ee_pos,
            cube_pos=cube_pos,
            target_pos=self.target_pos,
            is_grasped=self.is_grasped,
            gripper_width=self.gripper_width,
            image=image
        )
        
        # Sync stage tracking for backward compatibility
        self._stage_reached = self.reward_fn._stage_reached
        self._stage_grasped = self.reward_fn._stage_grasped
        self._stage_lifted = self.reward_fn._stage_lifted
        
        # Time penalty
        reward -= 0.05
        
        # Termination conditions
        done = terminated or (self.episode_step >= self.max_episode_steps)
        
        # Failure check: cube fell off table
        cube_height = cube_pos[2]
        if cube_height < -0.1:
            done = True
            reward -= 50.0
        
        # Add success flag to info
        info['success'] = terminated
        
        return reward, done, info
    
    def _get_raw_image(self) -> Optional[np.ndarray]:
        """Get raw RGB image without normalization (for vision reward)"""
        try:
            if self.camera_rep is not None:
                rgb_data = self.rgb_annot.get_data()
                if rgb_data is not None and len(rgb_data.shape) == 3:
                    return rgb_data[:, :, :3]
        except:
            pass
        return None
    
    def reset(self, seed: Optional[int] = None, options: Optional[Dict] = None) -> Tuple[np.ndarray, Dict]:
        """Reset environment"""
        super().reset(seed=seed)
        
        self.world.reset()
        
        # Reset robot
        if self.robot is not None:
            self.robot.set_joint_positions(self.home_position)
            self.robot.set_joint_velocities(np.zeros(self.robot.num_dof))
        
        # Reset cube position
        try:
            import omni.usd
            from pxr import UsdGeom, Gf
            
            stage = omni.usd.get_context().get_stage()
            cube_prim = stage.GetPrimAtPath("/World/Cube")
            if cube_prim.IsValid():
                xformable = UsdGeom.Xformable(cube_prim)
                xformable.ClearXformOpOrder()
                xformable.AddTranslateOp().Set(Gf.Vec3d(*self.cube_initial_pos))
        except Exception as e:
            print(f"⚠️ Cube reset failed: {e}")
        
        # Reset state
        self.episode_step = 0
        self.gripper_width = 0.06
        self.gripper_target = 0.06
        self.is_grasped = False
        self._stage_reached = False
        self._stage_grasped = False
        self._stage_lifted = False
        self._prev_dist_ee_cube = None
        self._prev_dist_cube_target = None
        
        # Reset modular reward function
        self.reward_fn.reset()
        
        # Stabilize
        for _ in range(100):
            if self.robot is not None:
                self.robot.set_joint_positions(self.home_position)
                self.robot.set_joint_velocities(np.zeros(self.robot.num_dof))
            self.world.step(render=not self.headless)
        
        # Ensure camera rendering is initialized (required for first frame)
        try:
            import omni.replicator.core as rep
            rep.orchestrator.step(rt_subframes=4)
        except:
            pass
        
        observation = self._get_observation()
        
        ee_pos = self._get_end_effector_pos()
        cube_pos = self._get_cube_pos()
        
        info = {
            "ee_pos": ee_pos.tolist(),
            "cube_pos": cube_pos.tolist(),
            "target_pos": self.target_pos.tolist()
        }
        
        return observation, info
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """Execute action: 6 arm joints + 1 gripper"""
        if self.robot is not None:
            try:
                current_positions = self.robot.get_joint_positions()
                
                # Arm joints (first 6 actions -> first 5 joints)
                arm_action = action[:6]
                action_scale = 0.02
                new_arm_positions = current_positions[:5] + arm_action[:5] * action_scale
                new_arm_positions = np.clip(new_arm_positions, -1.57, 1.57)
                
                # Gripper control (7th action -> 6th joint)
                gripper_action = action[6] if len(action) > 6 else 0.0
                # Map [-1, 1] to [0.0, 0.06]: -1=closed, +1=open
                self.gripper_target = 0.03 + 0.03 * gripper_action
                self.gripper_width = self.gripper_target  # Instant for now
                
                # Combine
                new_positions = np.concatenate([new_arm_positions, [self.gripper_width]])
                
                self.robot.set_joint_positions(new_positions)
                self.robot.set_joint_velocities(np.zeros(self.robot.num_dof))
                
            except Exception as e:
                new_positions = self.home_position
        
        # Step simulation
        for _ in range(4):
            self.world.step(render=not self.headless)
            if self.robot is not None:
                self.robot.set_joint_positions(new_positions)
                self.robot.set_joint_velocities(np.zeros(self.robot.num_dof))
        
        self.episode_step += 1
        
        observation = self._get_observation()
        reward, terminated, info = self._compute_reward()
        
        return observation, reward, terminated, False, info
    
    def render(self):
        """Render"""
        if self.render_mode == "rgb_array":
            return self._get_observation()[:3]
        return None
    
    def close(self):
        """Close environment"""
        if hasattr(self, 'world'):
            self.world.stop()
        print("✅ PickPlaceVisionEnv closed")


# Test
if __name__ == "__main__":
    from isaacsim import SimulationApp
    
    simulation_app = SimulationApp({"headless": False})
    
    env = PickPlaceVisionEnv(render_mode="human", headless=False)
    
    print("\n🧪 Testing Pick & Place environment...")
    
    obs, info = env.reset()
    print(f"✅ Reset - Obs shape: {obs.shape}")
    print(f"   EE: {info.get('ee_pos')}")
    print(f"   Cube: {info.get('cube_pos')}")
    
    for i in range(50):
        # Random action with occasional gripper close
        action = env.action_space.sample() * 0.5
        if i > 30:
            action[6] = -0.8  # Close gripper
        
        obs, reward, done, trunc, info = env.step(action)
        print(f"Step {i+1}: reward={reward:.2f}, grasped={info['is_grasped']}")
        
        if done:
            print("Episode done!")
            break
    
    env.close()
    simulation_app.close()
