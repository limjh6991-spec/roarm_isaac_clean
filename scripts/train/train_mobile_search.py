#!/usr/bin/env python3
"""
Train Mobile Manipulation - Search Phase
Uses PPO for exploration-focused learning

Usage:
    /home/roarm_m3/isaacsim/python.sh scripts/train/train_mobile_search.py
"""

import sys
import os
import argparse
import warnings

warnings.filterwarnings("ignore", message=".*Gym has been unmaintained.*")

# SimulationApp first
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

print("=" * 80)
print("🔍 Mobile Manipulation VRL - Search Phase Training")
print("=" * 80)

sys.path.append(os.path.join(os.path.dirname(__file__), "../.."))

from datetime import datetime
from pathlib import Path
import numpy as np

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback

from envs.mobile_manipulation_env import MobileManipulationEnv
from models.cnn_extractor import NatureCNN

print("✅ All imports successful")


def train_search(total_timesteps=100000):
    """Train Search phase with PPO"""
    
    # Output directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(f"logs/mobile_manipulation/search_{timestamp}")
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"\n📁 Output: {output_dir}")
    
    # Create environment
    print("\n1. Creating environment...")
    env = MobileManipulationEnv(headless=True)
    print("✅ Environment created")
    
    # Policy with NatureCNN
    print("\n2. Setting up PPO with NatureCNN...")
    policy_kwargs = dict(
        features_extractor_class=NatureCNN,
        features_extractor_kwargs=dict(features_dim=512),
        net_arch=[256, 256],
    )
    
    model = PPO(
        "CnnPolicy",
        env,
        policy_kwargs=policy_kwargs,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,  # Exploration bonus
        tensorboard_log=str(output_dir / "tensorboard"),
        verbose=1,
        device="cuda",
    )
    print("✅ PPO model created")
    
    # Callbacks
    print("\n3. Setting up callbacks...")
    checkpoint_cb = CheckpointCallback(
        save_freq=25000,
        save_path=str(output_dir / "checkpoints"),
        name_prefix="ppo_search",
    )
    
    eval_cb = EvalCallback(
        env,
        best_model_save_path=str(output_dir / "best_model"),
        log_path=str(output_dir / "eval"),
        eval_freq=25000,
        n_eval_episodes=5,
        deterministic=True,
    )
    
    callbacks = [checkpoint_cb, eval_cb]
    print("✅ Callbacks configured")
    
    # Train
    print(f"\n4. Starting training ({total_timesteps:,} steps)...")
    print(f"   TensorBoard: tensorboard --logdir {output_dir / 'tensorboard'}")
    
    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=callbacks,
            progress_bar=True,
        )
        
        # Save final
        model.save(str(output_dir / "final_model" / "ppo_search_final"))
        print(f"\n✅ Training complete! Model saved to {output_dir}")
        
    except KeyboardInterrupt:
        model.save(str(output_dir / "interrupted_model"))
        print("\n⚠️ Training interrupted, model saved")
    
    finally:
        env.close()
    
    return output_dir


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--timesteps", type=int, default=100000)
    args = parser.parse_args()
    
    try:
        train_search(total_timesteps=args.timesteps)
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        simulation_app.close()
        print("✅ SimulationApp closed")
