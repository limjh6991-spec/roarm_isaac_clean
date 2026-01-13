#!/usr/bin/env python3
"""
RoArm-M3 Isaac Lab RL Environment Runner

Cartpole 튜토리얼 패턴을 따른 RoArm 환경 실행 스크립트.
AppLauncher를 사용하여 올바른 초기화 순서 보장.

실행:
    docker exec isaac-sim-5.1 /isaac-sim/python.sh /workspace/scripts/run_roarm_isaac_lab.py --num_envs 1 --headless
"""

import argparse

from isaaclab.app import AppLauncher

# =========== 1. Argument Parsing ===========
parser = argparse.ArgumentParser(description="RoArm-M3 Isaac Lab RL Environment")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments")

# AppLauncher args (--headless, --device, etc.)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# =========== 2. Launch Omniverse App (BEFORE any other imports!) ===========
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# =========== 3. Rest of imports (AFTER app launch) ===========
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
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
class RoArmEnvCfg(DirectRLEnvCfg):
    """RoArm-M3 Environment Configuration"""
    
    decimation = 2
    episode_length_s = 10.0
    action_scale = 0.5
    action_space = 7
    observation_space = 16  # joint_pos(8) + joint_vel(8)
    state_space = 0
    
    sim: SimulationCfg = SimulationCfg(dt=1/60, render_interval=2)
    robot_cfg: ArticulationCfg = ROARM_CFG.replace(prim_path="/World/envs/env_.*/Robot")
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=1, env_spacing=2.0, replicate_physics=True
    )


# =========== 6. Environment Class ===========
class RoArmDirectEnv(DirectRLEnv):
    """RoArm-M3 Direct RL Environment"""
    
    cfg: RoArmEnvCfg
    
    def __init__(self, cfg: RoArmEnvCfg, render_mode=None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self.action_scale = cfg.action_scale
        self.joint_pos = self.robot.data.joint_pos
        self.joint_vel = self.robot.data.joint_vel
        print(f"✅ RoArmDirectEnv initialized: {self.num_envs} envs, {self.robot.num_joints} joints")
    
    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot_cfg)
        spawn_ground_plane("/World/ground", GroundPlaneCfg())
        self.scene.clone_environments(copy_from_source=False)
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[])
        self.scene.articulations["robot"] = self.robot
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
        self.joint_pos = self.robot.data.joint_pos
        self.joint_vel = self.robot.data.joint_vel
        obs = torch.cat([self.joint_pos, self.joint_vel], dim=-1)
        return {"policy": obs}
    
    def _get_rewards(self):
        return -0.1 * torch.sum(torch.square(self.joint_pos), dim=-1)
    
    def _get_dones(self):
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        terminated = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        return terminated, time_out
    
    def _reset_idx(self, env_ids):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)
        
        joint_pos = self.robot.data.default_joint_pos[env_ids]
        joint_vel = self.robot.data.default_joint_vel[env_ids]
        default_root_state = self.robot.data.default_root_state[env_ids]
        default_root_state[:, :3] += self.scene.env_origins[env_ids]
        
        self.joint_pos[env_ids] = joint_pos
        self.joint_vel[env_ids] = joint_vel
        
        self.robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)


# =========== 7. Main Function ===========
def main():
    print("=" * 60)
    print("🤖 RoArm-M3 Isaac Lab Environment")
    print("=" * 60)
    
    # Create environment
    env_cfg = RoArmEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.sim.device = args_cli.device
    env = RoArmDirectEnv(cfg=env_cfg)
    
    # Run loop
    count = 0
    while simulation_app.is_running():
        with torch.inference_mode():
            if count % 300 == 0:
                count = 0
                env.reset()
                print("-" * 60)
                print("[INFO]: Resetting environment...")
            
            # Random actions
            actions = torch.randn(args_cli.num_envs, 7, device=env.device)
            obs, rew, terminated, truncated, info = env.step(actions)
            
            # Print first joint position
            if count % 50 == 0:
                print(f"[Env 0]: Joint 1 pos: {obs['policy'][0][0].item():.4f}")
            
            count += 1
    
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
