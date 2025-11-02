#!/usr/bin/env python3
"""
Vision Environment Test Script

목표:
1. RoArmPickPlaceVisionEnv 로드
2. Random policy로 rollout
3. RGB-D 이미지 확인
4. gym.check_env() 통과

작성일: 2025-11-02
"""

import argparse
import numpy as np
from pathlib import Path

# AppLauncher must be first
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Vision Environment Test")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments")
parser.add_argument("--headless", action="store_true", help="Run in headless mode")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch
import gymnasium as gym
from PIL import Image

# Import environment
import sys
sys.path.insert(0, str(Path(__file__).parents[2]))
from envs.roarm_pick_place_env_vision import RoArmPickPlaceVisionEnv

print("=" * 80)
print("🎥 Vision Environment Test")
print("=" * 80)


def test_random_policy(env, num_episodes=3, max_steps=100):
    """
    Test environment with random policy
    
    Args:
        env: Vision environment
        num_episodes: Number of test episodes
        max_steps: Max steps per episode
    """
    print(f"\n📊 Testing with random policy...")
    print(f"   Episodes: {num_episodes}")
    print(f"   Max steps per episode: {max_steps}")
    
    episode_rewards = []
    episode_lengths = []
    
    for ep in range(num_episodes):
        print(f"\n{'=' * 60}")
        print(f"Episode {ep + 1}/{num_episodes}")
        print(f"{'=' * 60}")
        
        obs, info = env.reset()
        episode_reward = 0
        step_count = 0
        
        print(f"   Initial observation shape: {obs.shape}")
        print(f"   Observation range: [{obs.min():.3f}, {obs.max():.3f}]")
        
        # Save initial observation
        save_observation(obs, ep, 0)
        
        for step in range(max_steps):
            # Random action
            action = env.action_space.sample()
            
            # Step
            obs, reward, terminated, truncated, info = env.step(action)
            
            episode_reward += reward
            step_count += 1
            
            # Print progress
            if step % 10 == 0:
                print(f"   Step {step:3d}: Reward={reward:7.3f}, "
                      f"Cumulative={episode_reward:7.3f}")
            
            # Save observation every 20 steps
            if step % 20 == 0:
                save_observation(obs, ep, step)
            
            if terminated or truncated:
                print(f"   Episode finished at step {step_count}")
                print(f"   Reason: {'terminated' if terminated else 'truncated'}")
                break
        
        episode_rewards.append(episode_reward)
        episode_lengths.append(step_count)
        
        print(f"\n   Episode Summary:")
        print(f"   Total Reward: {episode_reward:.3f}")
        print(f"   Steps: {step_count}")
    
    # Final statistics
    print(f"\n{'=' * 80}")
    print("📊 Test Results")
    print(f"{'=' * 80}")
    print(f"   Average Reward: {np.mean(episode_rewards):.3f} ± {np.std(episode_rewards):.3f}")
    print(f"   Average Length: {np.mean(episode_lengths):.1f} ± {np.std(episode_lengths):.1f}")
    print(f"   Min Reward: {np.min(episode_rewards):.3f}")
    print(f"   Max Reward: {np.max(episode_rewards):.3f}")


def save_observation(obs, episode, step):
    """
    Save RGB-D observation as image
    
    Args:
        obs: Observation tensor (4, 84, 84)
        episode: Episode number
        step: Step number
    """
    output_dir = Path(__file__).parents[2] / "output" / "vision_env_test"
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Convert to numpy if tensor
    if isinstance(obs, torch.Tensor):
        obs = obs.cpu().numpy()
    
    # Extract RGB and Depth
    rgb = (obs[:3] * 255).astype(np.uint8).transpose(1, 2, 0)  # (3, 84, 84) → (84, 84, 3)
    depth = (obs[3] * 255).astype(np.uint8)  # (84, 84)
    
    # Save images
    rgb_path = output_dir / f"ep{episode:02d}_step{step:03d}_rgb.png"
    depth_path = output_dir / f"ep{episode:02d}_step{step:03d}_depth.png"
    
    Image.fromarray(rgb).save(rgb_path)
    Image.fromarray(depth).save(depth_path)


def test_observation_space(env):
    """Test observation space properties"""
    print(f"\n{'=' * 80}")
    print("🔍 Observation Space Test")
    print(f"{'=' * 80}")
    
    obs_space = env.observation_space
    print(f"   Type: {type(obs_space)}")
    print(f"   Shape: {obs_space.shape}")
    print(f"   Dtype: {obs_space.dtype}")
    print(f"   Low: {obs_space.low.min()}")
    print(f"   High: {obs_space.high.max()}")
    
    # Sample observation
    sample_obs = obs_space.sample()
    print(f"\n   Sample Observation:")
    print(f"   Shape: {sample_obs.shape}")
    print(f"   Range: [{sample_obs.min():.3f}, {sample_obs.max():.3f}]")
    print(f"   Mean: {sample_obs.mean():.3f}")
    print(f"   Std: {sample_obs.std():.3f}")


def test_action_space(env):
    """Test action space properties"""
    print(f"\n{'=' * 80}")
    print("🎮 Action Space Test")
    print(f"{'=' * 80}")
    
    action_space = env.action_space
    print(f"   Type: {type(action_space)}")
    print(f"   Shape: {action_space.shape}")
    print(f"   Dtype: {action_space.dtype}")
    print(f"   Low: {action_space.low}")
    print(f"   High: {action_space.high}")
    
    # Sample action
    sample_action = action_space.sample()
    print(f"\n   Sample Action:")
    print(f"   {sample_action}")


def main():
    """Main test function"""
    
    print("\n📦 Loading Vision Environment...")
    
    try:
        # Create environment
        # TODO: Load from config
        env = RoArmPickPlaceVisionEnv()
        
        print("✅ Environment loaded successfully")
        
        # Test spaces
        test_observation_space(env)
        test_action_space(env)
        
        # Test random policy
        test_random_policy(env, num_episodes=3, max_steps=100)
        
        print(f"\n{'=' * 80}")
        print("✅ All Tests Passed")
        print(f"{'=' * 80}")
        print(f"\n📁 Output images saved to:")
        print(f"   {Path(__file__).parents[2] / 'output' / 'vision_env_test'}")
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        print("\n🧹 Closing simulation...")
        simulation_app.close()


if __name__ == "__main__":
    main()
