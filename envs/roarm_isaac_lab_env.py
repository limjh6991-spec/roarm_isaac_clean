#!/usr/bin/env python3
"""
RoArm-M3 Isaac Lab 환경 (공식 예제 스타일)

Isaac Lab 2.3.0 (v0.47.2) + Isaac Sim 5.1 호환
CartpoleEnv 예제 패턴을 따릅니다.
"""

from __future__ import annotations

# 1. SimulationApp 먼저 초기화 (필수!)
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import torch
from collections.abc import Sequence

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.utils import configclass


# RoArm-M3 Robot Configuration
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


@configclass
class RoArmEnvCfg(DirectRLEnvCfg):
    """RoArm-M3 Environment Configuration (Cartpole style)"""
    
    # env
    decimation = 2
    episode_length_s = 10.0
    action_scale = 0.5
    action_space = 7  # 6 arm joints + 1 gripper
    observation_space = 16  # joint_pos(8) + joint_vel(8)
    state_space = 0
    
    # simulation
    sim: SimulationCfg = SimulationCfg(dt=1/60, render_interval=decimation)
    
    # robot
    robot_cfg: ArticulationCfg = ROARM_CFG.replace(prim_path="/World/envs/env_.*/Robot")
    
    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=1, env_spacing=2.0, replicate_physics=True
    )
    
    # reward scales
    rew_scale_joint = -0.1


class RoArmIsaacLabEnv(DirectRLEnv):
    """RoArm-M3 Isaac Lab Environment (Cartpole pattern)"""
    
    cfg: RoArmEnvCfg
    
    def __init__(self, cfg: RoArmEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        
        self.action_scale = self.cfg.action_scale
        self.joint_pos = self.robot.data.joint_pos
        self.joint_vel = self.robot.data.joint_vel
        
        print(f"✅ RoArmIsaacLabEnv initialized")
        print(f"   Num envs: {self.num_envs}")
        print(f"   Robot joints: {self.robot.num_joints}")
    
    def _setup_scene(self):
        """Setup scene - following Cartpole example pattern"""
        # Create robot articulation
        self.robot = Articulation(self.cfg.robot_cfg)
        
        # Add ground plane using native Isaac Sim API
        import omni.isaac.core.utils.prims as prim_utils
        from pxr import UsdPhysics, Gf
        
        stage = prim_utils.get_current_stage()
        ground_path = "/World/ground"
        if not stage.GetPrimAtPath(ground_path):
            prim_utils.create_prim(ground_path, "Plane")
            ground_prim = stage.GetPrimAtPath(ground_path)
            # Add collision
            UsdPhysics.CollisionAPI.Apply(ground_prim)
        
        # Clone environments
        self.scene.clone_environments(copy_from_source=False)
        
        # Filter collisions for CPU simulation
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[])
        
        # Add articulation to scene
        self.scene.articulations["robot"] = self.robot
        
        # Add lights
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.9, 0.9, 0.9))
        light_cfg.func("/World/Light", light_cfg)
        
        print(f"✅ Scene setup complete")
    
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        """Store actions before physics step"""
        self.actions = self.action_scale * actions.clone()
    
    def _apply_action(self) -> None:
        """Apply actions to robot joints"""
        # Pad actions to match robot DOF if needed
        if self.actions.shape[1] < self.robot.num_joints:
            padding = torch.zeros(
                self.num_envs, 
                self.robot.num_joints - self.actions.shape[1], 
                device=self.device
            )
            full_actions = torch.cat([self.actions, padding], dim=1)
        else:
            full_actions = self.actions[:, :self.robot.num_joints]
        
        self.robot.set_joint_position_target(full_actions)
    
    def _get_observations(self) -> dict:
        """Get observations"""
        self.joint_pos = self.robot.data.joint_pos
        self.joint_vel = self.robot.data.joint_vel
        
        obs = torch.cat([
            self.joint_pos,
            self.joint_vel,
        ], dim=-1)
        
        return {"policy": obs}
    
    def _get_rewards(self) -> torch.Tensor:
        """Compute rewards"""
        # Simple reward: penalize joint deviation from zero
        reward = self.cfg.rew_scale_joint * torch.sum(torch.square(self.joint_pos), dim=-1)
        return reward
    
    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        """Check termination conditions"""
        self.joint_pos = self.robot.data.joint_pos
        self.joint_vel = self.robot.data.joint_vel
        
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        terminated = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        
        return terminated, time_out
    
    def _reset_idx(self, env_ids: Sequence[int] | None):
        """Reset environments"""
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)
        
        # Reset joints to default
        joint_pos = self.robot.data.default_joint_pos[env_ids]
        joint_vel = self.robot.data.default_joint_vel[env_ids]
        
        default_root_state = self.robot.data.default_root_state[env_ids]
        default_root_state[:, :3] += self.scene.env_origins[env_ids]
        
        self.joint_pos[env_ids] = joint_pos
        self.joint_vel[env_ids] = joint_vel
        
        self.robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)


def main():
    """Test"""
    print("=" * 60)
    print("🤖 RoArm Isaac Lab Environment Test")
    print("=" * 60)
    
    cfg = RoArmEnvCfg()
    env = RoArmIsaacLabEnv(cfg)
    
    print("\n🧪 Running test...")
    
    obs, info = env.reset()
    print(f"Observation shape: {obs['policy'].shape}")
    
    for step in range(50):
        action = torch.rand(env.num_envs, 7, device=env.device) * 2 - 1
        obs, reward, terminated, truncated, info = env.step(action)
        
        if step % 10 == 0:
            print(f"Step {step}: reward = {reward.mean().item():.4f}")
    
    print("\n✅ Test completed!")
    
    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
