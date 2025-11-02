#!/usr/bin/env python3
"""
Train SAC with Vision (RGB-D)
Based on: docs/VISION_SAC_GUIDE.md

Usage:
    /home/roarm_m3/isaacsim/python.sh scripts/train/train_vision_sac.py
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), "../.."))

import numpy as np
from datetime import datetime
from pathlib import Path

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor

from envs.roarm_pick_place_env_vision import RoArmPickPlaceVisionEnv, RoArmPickPlaceVisionEnvCfg
from models.cnn_extractor import NatureCNN


def create_env(render=False):
    """Create vision environment"""
    cfg = RoArmPickPlaceVisionEnvCfg()
    env = RoArmPickPlaceVisionEnv(cfg, render_mode="human" if render else None)
    env = Monitor(env)
    return env


def train_sac():
    """Train SAC on vision task"""
    print("=" * 80)
    print("🚀 Training SAC on RoArm Vision Task")
    print("=" * 80)
    
    # Create output directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(f"output/train_vision_sac/{timestamp}")
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"\n📁 Output directory: {output_dir}")
    
    # Create environments
    print("\n1. Creating environments...")
    train_env = create_env(render=False)
    eval_env = create_env(render=False)
    print("✅ Environments created")
    
    # Policy kwargs with NatureCNN
    print("\n2. Configuring SAC policy...")
    policy_kwargs = dict(
        features_extractor_class=NatureCNN,
        features_extractor_kwargs=dict(features_dim=512),
        net_arch=[256, 256],  # Actor-Critic network
    )
    print("✅ Policy configured with NatureCNN (512 features)")
    
    # SAC model
    print("\n3. Creating SAC model...")
    model = SAC(
        "CnnPolicy",
        train_env,
        policy_kwargs=policy_kwargs,
        buffer_size=100_000,  # 100K transitions
        batch_size=256,
        learning_rate=3e-4,
        tau=0.005,  # Soft update coefficient
        gamma=0.99,  # Discount factor
        train_freq=1,  # Train every step
        gradient_steps=1,  # 1 gradient step per env step
        learning_starts=10_000,  # Start training after 10K steps
        tensorboard_log=str(output_dir / "tensorboard"),
        verbose=1,
        device="cuda",  # Use GPU
    )
    print("✅ SAC model created")
    print(f"   Buffer size: 100K")
    print(f"   Batch size: 256")
    print(f"   Learning rate: 3e-4")
    print(f"   Device: cuda")
    
    # Callbacks
    print("\n4. Setting up callbacks...")
    checkpoint_callback = CheckpointCallback(
        save_freq=10_000,  # Save every 10K steps
        save_path=str(output_dir / "checkpoints"),
        name_prefix="sac_vision",
        save_replay_buffer=True,
        save_vecnormalize=True,
    )
    
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=str(output_dir / "best_model"),
        log_path=str(output_dir / "eval"),
        eval_freq=10_000,  # Evaluate every 10K steps
        n_eval_episodes=10,
        deterministic=True,
        render=False,
    )
    
    callbacks = [checkpoint_callback, eval_callback]
    print("✅ Callbacks configured")
    print(f"   Checkpoint: every 10K steps")
    print(f"   Evaluation: every 10K steps (10 episodes)")
    
    # Training
    print("\n5. Starting training...")
    print(f"   Total timesteps: 500K")
    print(f"   Estimated time: 5-10 hours")
    print(f"   TensorBoard: tensorboard --logdir {output_dir / 'tensorboard'}")
    print("")
    
    try:
        model.learn(
            total_timesteps=500_000,  # 500K steps
            callback=callbacks,
            log_interval=10,
            progress_bar=True,
        )
        
        print("\n✅ Training completed!")
        
        # Save final model
        final_model_path = output_dir / "final_model" / "sac_vision_final"
        model.save(str(final_model_path))
        print(f"✅ Final model saved: {final_model_path}")
        
    except KeyboardInterrupt:
        print("\n⚠️  Training interrupted by user")
        
        # Save interrupted model
        interrupted_model_path = output_dir / "interrupted_model" / "sac_vision_interrupted"
        model.save(str(interrupted_model_path))
        print(f"✅ Interrupted model saved: {interrupted_model_path}")
    
    finally:
        # Close environments
        train_env.close()
        eval_env.close()
        print("\n✅ Environments closed")
    
    print("\n" + "=" * 80)
    print("🎉 Training session completed!")
    print("=" * 80)
    print(f"\n📊 Results:")
    print(f"   Output: {output_dir}")
    print(f"   TensorBoard: tensorboard --logdir {output_dir / 'tensorboard'}")
    print(f"   Best model: {output_dir / 'best_model'}")
    print(f"   Checkpoints: {output_dir / 'checkpoints'}")


if __name__ == "__main__":
    train_sac()
