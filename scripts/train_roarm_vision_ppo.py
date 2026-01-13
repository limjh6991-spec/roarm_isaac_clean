#!/usr/bin/env python3
"""
RoArm-M3 Vision RL Training Script

Isaac Lab + Stable-Baselines3 PPO를 사용한 Vision RL 훈련.

실행:
    docker exec isaac-sim-5.1 /isaac-sim/python.sh /workspace/scripts/train_roarm_vision_ppo.py --num_envs 8 --headless
"""

import argparse
import os
import sys
from datetime import datetime
from pathlib import Path

from isaaclab.app import AppLauncher

# =========== Argument Parsing ===========
parser = argparse.ArgumentParser(description="Train RoArm-M3 Vision RL with PPO")
parser.add_argument("--num_envs", type=int, default=4, help="Number of parallel environments")
parser.add_argument("--max_iterations", type=int, default=1000, help="Max training iterations")
parser.add_argument("--seed", type=int, default=42, help="Random seed")
parser.add_argument("--checkpoint", type=str, default=None, help="Resume from checkpoint")
parser.add_argument("--log_dir", type=str, default="logs/roarm_vision_ppo", help="Log directory")

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = True  # Enable for vision

# =========== Launch App ===========
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# =========== Imports (after app launch) ===========
import torch
import numpy as np
from collections.abc import Sequence

import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.vec_env import VecMonitor

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.sensors import Camera, CameraCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.utils import configclass

from isaaclab_rl.sb3 import Sb3VecEnvWrapper


# =========== Robot Config ===========
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


# =========== Environment Config ===========
@configclass
class RoArmVisionEnvCfg(DirectRLEnvCfg):
    decimation = 2
    episode_length_s = 10.0
    action_scale = 0.5
    action_space = 7
    observation_space = 4 * 84 * 84  # RGBD
    state_space = 0
    
    sim: SimulationCfg = SimulationCfg(dt=1/60, render_interval=2)
    robot_cfg: ArticulationCfg = ROARM_CFG.replace(prim_path="/World/envs/env_.*/Robot")
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=4, env_spacing=2.0, replicate_physics=True
    )
    
    dist_reward_scale = 2.0
    grasp_bonus = 5.0
    lift_bonus = 10.0
    success_bonus = 100.0


# =========== Vision RL Environment ===========
class RoArmVisionRLEnv(DirectRLEnv):
    cfg: RoArmVisionEnvCfg
    
    def __init__(self, cfg: RoArmVisionEnvCfg, render_mode=None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self.action_scale = cfg.action_scale
        self.cube_pos = torch.zeros(self.num_envs, 3, device=self.device)
        self.cube_pos[:, 0] = 0.3
        self.cube_pos[:, 2] = 0.45
        self.target_pos = torch.zeros(self.num_envs, 3, device=self.device)
        self.target_pos[:, 0] = -0.3
        self.target_pos[:, 2] = 0.45
    
    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot_cfg)
        spawn_ground_plane("/World/ground", GroundPlaneCfg())
        
        # Table
        table_cfg = sim_utils.CuboidCfg(
            size=(0.8, 0.8, 0.4),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.5, 0.5)),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        )
        table_cfg.func("/World/Table", table_cfg, translation=(0.0, 0.0, 0.2))
        
        # Cube
        cube_cfg = sim_utils.CuboidCfg(
            size=(0.05, 0.05, 0.05),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.05),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
        )
        cube_cfg.func("/World/Cube", cube_cfg, translation=(0.3, 0.0, 0.45))
        
        # Camera
        camera_cfg = CameraCfg(
            prim_path="/World/envs/env_.*/Robot/gripper_link/camera",
            update_period=0.0, height=84, width=84,
            data_types=["rgb", "distance_to_image_plane"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=2.0, horizontal_aperture=3.0, clipping_range=(0.01, 2.0)
            ),
            offset=CameraCfg.OffsetCfg(pos=(0.0, 0.0, 0.05), rot=(0.5, -0.5, 0.5, -0.5), convention="ros"),
        )
        self.camera = Camera(camera_cfg)
        
        self.scene.clone_environments(copy_from_source=False)
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[])
        
        self.scene.articulations["robot"] = self.robot
        self.scene.sensors["camera"] = self.camera
        
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.9, 0.9, 0.9))
        light_cfg.func("/World/Light", light_cfg)
    
    def _pre_physics_step(self, actions: torch.Tensor):
        self.actions = self.action_scale * actions.clone()
    
    def _apply_action(self):
        full_actions = self.actions[:, :self.robot.num_joints] if self.actions.shape[1] >= self.robot.num_joints else torch.cat([self.actions, torch.zeros(self.num_envs, self.robot.num_joints - self.actions.shape[1], device=self.device)], dim=1)
        self.robot.set_joint_position_target(full_actions)
    
    def _get_observations(self):
        self.camera.update(self.cfg.sim.dt)
        rgb = self.camera.data.output["rgb"].float() / 255.0
        depth = torch.clamp(self.camera.data.output["distance_to_image_plane"], 0.01, 2.0)
        depth_norm = (depth - 0.01) / 1.99
        rgbd = torch.cat([rgb.permute(0, 3, 1, 2), depth_norm.permute(0, 3, 1, 2)], dim=1)
        return {"policy": rgbd.flatten(start_dim=1)}
    
    def _get_rewards(self):
        gripper_pos = self.robot.data.body_pos_w[:, -1]
        dist_to_cube = torch.norm(gripper_pos - self.cube_pos, dim=1)
        reward = -dist_to_cube * self.cfg.dist_reward_scale
        reward += (dist_to_cube < 0.05).float() * self.cfg.grasp_bonus
        reward += (self.cube_pos[:, 2] > 0.5).float() * self.cfg.lift_bonus
        dist_to_target = torch.norm(self.cube_pos - self.target_pos, dim=1)
        reward += (dist_to_target < 0.05).float() * self.cfg.success_bonus
        return reward
    
    def _get_dones(self):
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        terminated = torch.norm(self.cube_pos - self.target_pos, dim=1) < 0.05
        return terminated, time_out
    
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)
        
        joint_pos = self.robot.data.default_joint_pos[env_ids]
        joint_vel = self.robot.data.default_joint_vel[env_ids]
        default_root_state = self.robot.data.default_root_state[env_ids]
        default_root_state[:, :3] += self.scene.env_origins[env_ids]
        
        self.robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)
        
        num_reset = len(env_ids)
        self.cube_pos[env_ids, 0] = 0.3 + (torch.rand(num_reset, device=self.device) - 0.5) * 0.2
        self.cube_pos[env_ids, 1] = (torch.rand(num_reset, device=self.device) - 0.5) * 0.2


# =========== Main Training ===========
def main():
    print("=" * 60)
    print("🤖 RoArm-M3 Vision RL Training (PPO)")
    print("=" * 60)
    
    # Setup logging
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = os.path.join(args_cli.log_dir, run_id)
    os.makedirs(log_dir, exist_ok=True)
    print(f"[INFO] Log directory: {log_dir}")
    
    # Create environment
    env_cfg = RoArmVisionEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed
    env = RoArmVisionRLEnv(cfg=env_cfg)
    
    print(f"✅ Environment created: {args_cli.num_envs} envs")
    
    # Wrap for SB3
    env = Sb3VecEnvWrapper(env)
    env = VecMonitor(env)
    
    # Create PPO agent
    # Using CNN policy for vision observations
    policy_kwargs = dict(
        features_extractor_kwargs=dict(features_dim=256),
    )
    
    agent = PPO(
        "CnnPolicy",
        env,
        learning_rate=3e-4,
        n_steps=128,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        verbose=1,
        tensorboard_log=log_dir,
        seed=args_cli.seed,
    )
    
    if args_cli.checkpoint:
        agent = PPO.load(args_cli.checkpoint, env=env)
        print(f"[INFO] Loaded checkpoint: {args_cli.checkpoint}")
    
    print(f"✅ PPO agent created")
    
    # Callbacks
    checkpoint_cb = CheckpointCallback(
        save_freq=1000,
        save_path=log_dir,
        name_prefix="roarm_ppo",
        verbose=1
    )
    
    # Train
    total_timesteps = args_cli.max_iterations * 128 * args_cli.num_envs
    print(f"[INFO] Training for {total_timesteps} timesteps ({args_cli.max_iterations} iterations)")
    
    try:
        agent.learn(
            total_timesteps=total_timesteps,
            callback=checkpoint_cb,
            progress_bar=True,
        )
    except KeyboardInterrupt:
        print("\n[INFO] Training interrupted")
    
    # Save final model
    final_path = os.path.join(log_dir, "roarm_ppo_final")
    agent.save(final_path)
    print(f"✅ Model saved: {final_path}.zip")
    
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
