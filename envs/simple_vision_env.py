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

# ✅ FIX: AppLauncher 초기화는 이 모듈을 import하는 메인 스크립트에서 수행
# (train_vision_sac.py에서 SimulationApp을 먼저 초기화함)

"""Rest everything follows."""

import torch
import cv2

# Isaac Sim 5.1 API
from isaacsim import SimulationApp

# Isaac Core API (5.1 namespace)
from isaacsim.core.api import World
from isaacsim.core.api.articulations import Articulation
from isaacsim.sensors.camera import Camera as IsaacCamera

# Note: DynamicCuboid, VisualCuboid, rotations moved or replaced in 5.1
# Using USD prims directly instead


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
        
        # Create table (visual only)
        table_cfg = sim_utils.CuboidCfg(
            size=(0.8, 0.8, 0.4),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.5, 0.5)),
        )
        table_cfg.func("/World/Table", table_cfg, translation=(0.0, 0.0, 0.2))
        
        # Create cube (pick object) as RigidObject
        cube_cfg = RigidObjectCfg(
            prim_path="/World/Cube",
            spawn=sim_utils.CuboidCfg(
                size=(0.05, 0.05, 0.05),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    kinematic_enabled=False,
                    disable_gravity=False,
                ),
                mass_props=sim_utils.MassPropertiesCfg(mass=0.05),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=(0.3, 0.0, 0.45),
                rot=(1.0, 0.0, 0.0, 0.0),
            ),
        )
        self.cube = RigidObject(cube_cfg)
        
        # Create target (goal marker - visual only)
        target_cfg = sim_utils.CuboidCfg(
            size=(0.06, 0.06, 0.001),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
        )
        target_cfg.func("/World/Target", target_cfg, translation=(-0.3, 0.0, 0.45))
        
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
        
        # Cube and target positions
        self.cube_initial_pos = torch.tensor([0.3, 0.0, 0.45], device=self.sim.device)
        self.target_pos = torch.tensor([-0.3, 0.0, 0.45], device=self.sim.device)
    
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
    
    def _get_cube_position(self) -> torch.Tensor:
        """Get current cube position"""
        # Get cube position from RigidObject
        return self.cube.data.root_pos_w[0]
    
    def _get_gripper_position(self) -> torch.Tensor:
        """Get gripper position (end-effector)"""
        # Assume last link is gripper
        gripper_link_idx = self.robot.num_bodies - 1
        gripper_pos = self.robot.data.body_pos_w[0, gripper_link_idx, :]
        return gripper_pos
    
    def _compute_reward(self) -> float:
        """Pick and Place reward"""
        # Get positions
        gripper_pos = self._get_gripper_position()
        cube_pos = self._get_cube_position()
        
        # Distance from gripper to cube
        dist_to_cube = torch.norm(gripper_pos - cube_pos).item()
        
        # Distance from cube to target
        dist_cube_to_target = torch.norm(cube_pos - self.target_pos).item()
        
        # Reward components
        reward = 0.0
        
        # 1. Reach reward: encourage approaching cube
        reach_reward = -dist_to_cube * 2.0  # Scale: closer = higher reward
        reward += reach_reward
        
        # 2. Grasp bonus: if gripper is very close to cube
        if dist_to_cube < 0.05:  # 5cm
            reward += 5.0
        
        # 3. Lift bonus: if cube is lifted
        if cube_pos[2] > self.cube_initial_pos[2] + 0.05:  # 5cm above initial
            reward += 10.0
        
        # 4. Place reward: if cube is close to target
        if dist_cube_to_target < 0.1:  # 10cm
            reward += 20.0
        
        # 5. Success bonus: if cube is at target
        if dist_cube_to_target < 0.05:  # 5cm
            reward += 100.0
        
        # 6. Small time penalty to encourage efficiency
        reward -= 0.01
        
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
        
        # Reset cube position (with randomization)
        if seed is not None:
            np.random.seed(seed)
        
        # Random cube position (within ±10cm from initial)
        cube_offset = (np.random.rand(2) - 0.5) * 0.2  # ±10cm in x, y
        new_cube_pos = self.cube_initial_pos.clone()
        new_cube_pos[0] += cube_offset[0]
        new_cube_pos[1] += cube_offset[1]
        
        # Set cube position using RigidObject
        self.cube.write_root_pose_to_sim(
            new_cube_pos.unsqueeze(0),
            torch.tensor([1.0, 0.0, 0.0, 0.0], device=self.sim.device).unsqueeze(0)
        )
        
        # Step simulation to apply reset
        for _ in range(10):
            self.sim.step()
        
        # Get observation
        obs = self._get_observation()
        
        # Get initial distances for info
        gripper_pos = self._get_gripper_position()
        cube_pos = self._get_cube_position()
        dist_to_cube = torch.norm(gripper_pos - cube_pos).item()
        dist_cube_to_target = torch.norm(cube_pos - self.target_pos).item()
        
        info = {
            "episode_step": self.episode_step,
            "dist_to_cube": dist_to_cube,
            "dist_cube_to_target": dist_cube_to_target,
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
        
        # Get current state for info
        gripper_pos = self._get_gripper_position()
        cube_pos = self._get_cube_position()
        dist_to_cube = torch.norm(gripper_pos - cube_pos).item()
        dist_cube_to_target = torch.norm(cube_pos - self.target_pos).item()
        
        # Check termination
        # Success: cube reached target
        terminated = dist_cube_to_target < 0.05  # 5cm
        
        # Truncation: max steps reached
        truncated = self.episode_step >= self.max_episode_steps
        
        info = {
            "episode_step": self.episode_step,
            "dist_to_cube": dist_to_cube,
            "dist_cube_to_target": dist_cube_to_target,
            "is_success": terminated,
        }
        
        return obs, reward, terminated, truncated, info
    
    def render(self):
        """Render (handled by Isaac Sim viewer)"""
        pass
    
    def close(self):
        """Close environment"""
        if hasattr(self, 'sim'):
            self.sim.stop()
        # SimulationApp은 메인 스크립트에서 관리


# Test function
def test_env():
    """Test environment (requires SimulationApp initialized)"""
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
    # AppLauncher 초기화 (standalone 실행 시)
    # TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.app import AppLauncher
    import argparse
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_envs", type=int, default=1)
    AppLauncher.add_app_launcher_args(parser)
    args_cli = parser.parse_args()
    args_cli.enable_cameras = True
    
    app_launcher = AppLauncher(args_cli)
    simulation_app = app_launcher.app
    
    try:
        test_env()
    finally:
        simulation_app.close()
