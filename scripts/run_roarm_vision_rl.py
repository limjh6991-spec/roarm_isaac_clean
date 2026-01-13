#!/usr/bin/env python3
"""
RoArm-M3 Vision RL Environment (Isaac Lab)

RGB-D 카메라 관측 + Pick & Place 보상 함수를 포함한 Vision RL 환경.

실행:
    docker exec isaac-sim-5.1 /isaac-sim/python.sh /workspace/scripts/run_roarm_vision_rl.py --num_envs 1 --headless
"""

import argparse

from isaaclab.app import AppLauncher

# =========== 1. Argument Parsing ===========
parser = argparse.ArgumentParser(description="RoArm-M3 Vision RL Environment")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments")

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = True  # Enable cameras for vision RL

# =========== 2. Launch Omniverse App ===========
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# =========== 3. Imports (AFTER app launch) ===========
import torch
import numpy as np
from collections.abc import Sequence

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.sensors import Camera, CameraCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.utils import configclass


# =========== 4. Robot Configuration ===========
ROARM_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/workspace/assets/roarm_m3/usd/roarm_m3_with_d405.usd",
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        joint_pos={".*": 0.0},
    ),
    actuators={
        "all_joints": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            stiffness=100.0,
            damping=10.0,
        ),
    },
)


# =========== 5. Environment Configuration ===========
@configclass
class RoArmVisionEnvCfg(DirectRLEnvCfg):
    """RoArm-M3 Vision RL Environment Configuration"""
    
    decimation = 2
    episode_length_s = 10.0
    action_scale = 0.5
    action_space = 7  # 6 arm joints + 1 gripper
    observation_space = 4 * 84 * 84  # RGBD flattened
    state_space = 0
    
    sim: SimulationCfg = SimulationCfg(dt=1/60, render_interval=2)
    robot_cfg: ArticulationCfg = ROARM_CFG.replace(prim_path="/World/envs/env_.*/Robot")
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=1, env_spacing=2.0, replicate_physics=True
    )
    
    # Reward scales
    dist_reward_scale = 2.0
    grasp_bonus = 5.0
    lift_bonus = 10.0
    success_bonus = 100.0
    time_penalty = 0.01


# =========== 6. Vision RL Environment ===========
class RoArmVisionRLEnv(DirectRLEnv):
    """RoArm-M3 Vision RL Environment with RGB-D observations"""
    
    cfg: RoArmVisionEnvCfg
    
    def __init__(self, cfg: RoArmVisionEnvCfg, render_mode=None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        
        self.action_scale = cfg.action_scale
        
        # Cube position (target object)
        self.cube_pos = torch.zeros(self.num_envs, 3, device=self.device)
        self.cube_pos[:, 0] = 0.3  # x
        self.cube_pos[:, 2] = 0.45  # z
        
        # Target position (goal)
        self.target_pos = torch.zeros(self.num_envs, 3, device=self.device)
        self.target_pos[:, 0] = -0.3
        self.target_pos[:, 2] = 0.45
        
        print(f"✅ RoArmVisionRLEnv initialized")
        print(f"   Num envs: {self.num_envs}")
        print(f"   Robot joints: {self.robot.num_joints}")
        print(f"   Camera: {self.camera.cfg.height}x{self.camera.cfg.width} RGBD")
    
    def _setup_scene(self):
        """Setup scene with robot, camera, cube, and target"""
        # Robot
        self.robot = Articulation(self.cfg.robot_cfg)
        
        # Ground plane
        spawn_ground_plane("/World/ground", GroundPlaneCfg())
        
        # Table
        table_cfg = sim_utils.CuboidCfg(
            size=(0.8, 0.8, 0.4),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.5, 0.5)),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        )
        table_cfg.func("/World/Table", table_cfg, translation=(0.0, 0.0, 0.2))
        
        # Cube (pick object)
        cube_cfg = sim_utils.CuboidCfg(
            size=(0.05, 0.05, 0.05),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.05),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
        )
        cube_cfg.func("/World/Cube", cube_cfg, translation=(0.3, 0.0, 0.45))
        
        # Target marker (visual only)
        target_cfg = sim_utils.CuboidCfg(
            size=(0.06, 0.06, 0.002),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
        )
        target_cfg.func("/World/Target", target_cfg, translation=(-0.3, 0.0, 0.42))
        
        # Hand-eye Camera
        camera_cfg = CameraCfg(
            prim_path="/World/envs/env_.*/Robot/gripper_link/camera",
            update_period=0.0,
            height=84,
            width=84,
            data_types=["rgb", "distance_to_image_plane"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=2.0,
                focus_distance=400.0,
                horizontal_aperture=3.0,
                clipping_range=(0.01, 2.0),
            ),
            offset=CameraCfg.OffsetCfg(
                pos=(0.0, 0.0, 0.05),
                rot=(0.5, -0.5, 0.5, -0.5),
                convention="ros",
            ),
        )
        self.camera = Camera(camera_cfg)
        
        # Clone environments
        self.scene.clone_environments(copy_from_source=False)
        
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[])
        
        # Register assets
        self.scene.articulations["robot"] = self.robot
        self.scene.sensors["camera"] = self.camera
        
        # Light
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.9, 0.9, 0.9))
        light_cfg.func("/World/Light", light_cfg)
    
    def _pre_physics_step(self, actions: torch.Tensor):
        self.actions = self.action_scale * actions.clone()
    
    def _apply_action(self):
        if self.actions.shape[1] < self.robot.num_joints:
            padding = torch.zeros(self.num_envs, self.robot.num_joints - self.actions.shape[1], device=self.device)
            full_actions = torch.cat([self.actions, padding], dim=1)
        else:
            full_actions = self.actions[:, :self.robot.num_joints]
        self.robot.set_joint_position_target(full_actions)
    
    def _get_observations(self):
        """Get RGB-D observation from camera"""
        # Update camera
        self.camera.update(self.cfg.sim.dt)
        
        # Get RGB (N, H, W, 3)
        rgb = self.camera.data.output["rgb"]
        rgb_norm = rgb.float() / 255.0  # [0, 1]
        
        # Get Depth (N, H, W, 1)
        depth = self.camera.data.output["distance_to_image_plane"]
        depth_clipped = torch.clamp(depth, 0.01, 2.0)
        depth_norm = (depth_clipped - 0.01) / 1.99  # [0, 1]
        
        # Stack to (N, 4, H, W)
        rgbd = torch.cat([
            rgb_norm.permute(0, 3, 1, 2),
            depth_norm.permute(0, 3, 1, 2)
        ], dim=1)
        
        # Flatten for observation
        obs_flat = rgbd.flatten(start_dim=1)
        
        return {"policy": obs_flat}
    
    def _get_rewards(self):
        """Pick and Place reward function"""
        # Gripper position (last body)
        gripper_pos = self.robot.data.body_pos_w[:, -1]
        
        # Distance to cube
        dist_to_cube = torch.norm(gripper_pos - self.cube_pos, dim=1)
        
        # Distance from cube to target
        dist_cube_to_target = torch.norm(self.cube_pos - self.target_pos, dim=1)
        
        # Dense reward: approach cube
        reward = -dist_to_cube * self.cfg.dist_reward_scale
        
        # Grasp bonus (close to cube)
        grasp_mask = dist_to_cube < 0.05
        reward += grasp_mask.float() * self.cfg.grasp_bonus
        
        # Lift bonus (cube lifted)
        lifted_mask = self.cube_pos[:, 2] > 0.5
        reward += lifted_mask.float() * self.cfg.lift_bonus
        
        # Success bonus (at target)
        success_mask = dist_cube_to_target < 0.05
        reward += success_mask.float() * self.cfg.success_bonus
        
        # Time penalty
        reward -= self.cfg.time_penalty
        
        return reward
    
    def _get_dones(self):
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        
        # Success termination
        dist_cube_to_target = torch.norm(self.cube_pos - self.target_pos, dim=1)
        terminated = dist_cube_to_target < 0.05
        
        return terminated, time_out
    
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)
        
        # Reset robot
        joint_pos = self.robot.data.default_joint_pos[env_ids]
        joint_vel = self.robot.data.default_joint_vel[env_ids]
        default_root_state = self.robot.data.default_root_state[env_ids]
        default_root_state[:, :3] += self.scene.env_origins[env_ids]
        
        self.robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)
        
        # Randomize cube position
        num_reset = len(env_ids)
        self.cube_pos[env_ids, 0] = 0.3 + (torch.rand(num_reset, device=self.device) - 0.5) * 0.2
        self.cube_pos[env_ids, 1] = (torch.rand(num_reset, device=self.device) - 0.5) * 0.2


# =========== 7. Main ===========
def main():
    print("=" * 60)
    print("🤖 RoArm-M3 Vision RL Environment")
    print("=" * 60)
    
    env_cfg = RoArmVisionEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.sim.device = args_cli.device
    env = RoArmVisionRLEnv(cfg=env_cfg)
    
    count = 0
    total_reward = 0.0
    
    while simulation_app.is_running():
        with torch.inference_mode():
            if count % 300 == 0:
                count = 0
                total_reward = 0.0
                env.reset()
                print("-" * 60)
                print("[INFO] Resetting environment...")
            
            # Random actions
            actions = torch.randn(args_cli.num_envs, 7, device=env.device)
            obs, rew, terminated, truncated, info = env.step(actions)
            total_reward += rew.sum().item()
            
            if count % 50 == 0:
                print(f"Step {count:4d}: reward={rew.mean().item():+.3f}, obs_shape={obs['policy'].shape}")
            
            count += 1
            
            if count >= 600:  # Run for 600 steps then exit
                break
    
    print(f"\n✅ Total reward: {total_reward:.2f}")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
