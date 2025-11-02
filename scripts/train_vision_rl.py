#!/usr/bin/env python3
"""
RoArm-M3 Vision RL Training Script
Stable Baselines3 + CnnPolicy
"""

import sys
import os
from pathlib import Path
import numpy as np
import torch

# Isaac Sim 초기화
from isaacsim import SimulationApp
simulation_app = SimulationApp({
    "headless": False,
    "width": 1280,
    "height": 720,
})

print("=" * 80)
print("🤖 RoArm-M3 Vision RL Training")
print("=" * 80)

# Environment import
sys.path.append(str(Path(__file__).parent.parent))
from envs.roarm_pick_place_env_vision import RoArmPickPlaceVisionEnv, RoArmPickPlaceVisionEnvCfg

# Stable Baselines3
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor

# Gym wrapper for Isaac Sim env
class IsaacSimGymWrapper(gym.Env):
    """Gym wrapper for Isaac Sim environment"""
    
    def __init__(self, env_cfg):
        self.env = RoArmPickPlaceVisionEnv(env_cfg)
        self.observation_space = self.env.observation_space
        self.action_space = self.env.action_space
        
    def reset(self, **kwargs):
        obs_dict = self.env.reset()
        obs = obs_dict["policy"].cpu().numpy()
        return obs, {}
    
    def step(self, action):
        action_tensor = torch.tensor(action, device=self.env.device)
        obs_dict, reward, terminated, truncated, info = self.env.step(action_tensor)
        
        obs = obs_dict["policy"].cpu().numpy()
        reward = reward.cpu().item()
        terminated = terminated.cpu().item()
        truncated = truncated.cpu().item()
        
        return obs, reward, terminated, truncated, info
    
    def render(self):
        pass
    
    def close(self):
        self.env.close()


def make_env():
    """Environment factory"""
    def _init():
        env_cfg = RoArmPickPlaceVisionEnvCfg()
        env = IsaacSimGymWrapper(env_cfg)
        env = Monitor(env)
        return env
    return _init


def main():
    # Config
    log_dir = Path("/home/roarm_m3/roarm_isaac_clean/logs/vision_rl")
    log_dir.mkdir(parents=True, exist_ok=True)
    
    model_dir = log_dir / "models"
    model_dir.mkdir(exist_ok=True)
    
    tensorboard_dir = log_dir / "tensorboard"
    tensorboard_dir.mkdir(exist_ok=True)
    
    print(f"\n📁 Directories:")
    print(f"   Log: {log_dir}")
    print(f"   Models: {model_dir}")
    print(f"   TensorBoard: {tensorboard_dir}")
    
    # Create environment
    print(f"\n🌍 Creating Environment...")
    env = DummyVecEnv([make_env()])
    
    print(f"   Observation: {env.observation_space.shape}")
    print(f"   Action: {env.action_space.shape}")
    
    # Policy config
    policy_kwargs = dict(
        features_extractor_class=NatureCNN,
        features_extractor_kwargs=dict(features_dim=512),
        net_arch=[256, 256],
        activation_fn=torch.nn.ReLU,
    )
    
    # Create model
    print(f"\n🤖 Creating PPO Model with CnnPolicy...")
    model = PPO(
        policy="CnnPolicy",
        env=env,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        vf_coef=0.5,
        max_grad_norm=0.5,
        policy_kwargs=policy_kwargs,
        verbose=1,
        tensorboard_log=str(tensorboard_dir),
        device="cuda" if torch.cuda.is_available() else "cpu",
    )
    
    print(f"   Device: {model.device}")
    print(f"   Policy: CnnPolicy")
    print(f"   Learning Rate: 3e-4")
    
    # Callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=10000,
        save_path=str(model_dir),
        name_prefix="roarm_vision_ppo",
    )
    
    print(f"\n▶️  Starting Training...")
    print(f"   Total Steps: 100,000")
    print(f"   Checkpoint: Every 10,000 steps")
    print(f"   TensorBoard: tensorboard --logdir {tensorboard_dir}")
    
    try:
        model.learn(
            total_timesteps=100000,
            callback=checkpoint_callback,
            progress_bar=True,
        )
        
        # Save final model
        final_model_path = model_dir / "roarm_vision_ppo_final.zip"
        model.save(str(final_model_path))
        
        print(f"\n✅ Training Complete!")
        print(f"   Final Model: {final_model_path}")
        
    except KeyboardInterrupt:
        print(f"\n⚠️  Training Interrupted!")
        
    finally:
        env.close()
        simulation_app.close()


# Nature CNN for vision (same as DQN paper)
import torch.nn as nn
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor


class NatureCNN(BaseFeaturesExtractor):
    """
    Nature CNN from DQN paper (Mnih et al., 2015)
    3 conv layers + flatten + 2 FC layers
    """
    
    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 512):
        super().__init__(observation_space, features_dim)
        
        n_input_channels = observation_space.shape[0]  # 4 (RGB-D)
        
        self.cnn = nn.Sequential(
            nn.Conv2d(n_input_channels, 32, kernel_size=8, stride=4, padding=0),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2, padding=0),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1, padding=0),
            nn.ReLU(),
            nn.Flatten(),
        )
        
        # Compute shape by doing one forward pass
        with torch.no_grad():
            sample_input = torch.zeros(1, *observation_space.shape)
            n_flatten = self.cnn(sample_input).shape[1]
        
        self.linear = nn.Sequential(
            nn.Linear(n_flatten, features_dim),
            nn.ReLU(),
        )
    
    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        return self.linear(self.cnn(observations))


if __name__ == "__main__":
    import gymnasium as gym  # Import here to avoid circular dependency
    main()
