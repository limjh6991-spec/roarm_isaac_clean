#!/usr/bin/env python3
"""
Train SAC with Vision (RGB-D)
Based on: docs/VISION_SAC_GUIDE.md

Usage:
    /home/roarm_m3/isaacsim/python.sh scripts/train/train_vision_sac.py
"""

import sys
import os
import argparse
import warnings

# 🔧 FIX: Suppress gym deprecation warning
warnings.filterwarnings("ignore", message=".*Gym has been unmaintained.*")
os.environ["GYM_IGNORE_DEPRECATION_WARNINGS"] = "1"

# 🔧 FIX: stdout 리다이렉트 제거 (Isaac Sim 로깅 충돌 방지)
sys.stdout = sys.__stdout__
sys.stderr = sys.__stderr__

# ✅ SimulationApp 먼저 초기화 (다른 import 전에)
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

print("="*80, flush=True)
print("🚀 SAC Vision Training - SimulationApp 초기화 완료!", flush=True)
print("="*80, flush=True)

sys.path.append(os.path.join(os.path.dirname(__file__), "../.."))

print("\n🔧 Importing dependencies...", flush=True)

import numpy as np
from datetime import datetime
from pathlib import Path

print("✅ Basic imports successful", flush=True)

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor

print("✅ Stable-baselines3 imported", flush=True)

from envs.simple_vision_env_v2 import SimpleVisionEnv  # CPU annotator version
from models.cnn_extractor import NatureCNN

print("✅ Environment and model imported", flush=True)


def create_env(render=False):
    """Create vision environment"""
    env = SimpleVisionEnv()
    # Note: Monitor wrapper removed due to gym/gymnasium compatibility issue
    # Stable-baselines3 2.7.0 should work with gymnasium directly
    return env


def train_sac(total_timesteps=50000):
    """Train SAC on vision task"""
    print("=" * 80)
    print(f"🚀 Training SAC on RoArm Vision Task ({total_timesteps:,} timesteps)")
    print("=" * 80)
    
    # Create output directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(f"logs/sac_training/{timestamp}")
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"\n📁 Output directory: {output_dir}")
    
    # Create environments
    print("\n1. Creating environments...")
    train_env = create_env(render=False)
    # Note: Using same environment for eval to avoid World name conflicts
    eval_env = train_env  
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
    # 🔧 디스크 관리 정책 (2024-12-27):
    #    - Replay buffer 저장 비활성화 (각 22GB로 디스크 폭발 방지)
    #    - 체크포인트 간격 50K 스텝 (500K 학습 시 ~10개 체크포인트)
    print("\n4. Setting up callbacks...")
    checkpoint_callback = CheckpointCallback(
        save_freq=50_000,  # Save every 50K steps (디스크 절약)
        save_path=str(output_dir / "checkpoints"),
        name_prefix="sac_vision",
        save_replay_buffer=False,  # ⚠️ 비활성화: 각 22GB로 디스크 폭발 위험
        save_vecnormalize=True,
    )
    
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=str(output_dir / "best_model"),
        log_path=str(output_dir / "eval"),
        eval_freq=50_000,  # Evaluate every 50K steps
        n_eval_episodes=10,
        deterministic=True,
        render=False,
    )
    
    callbacks = [checkpoint_callback, eval_callback]
    print("✅ Callbacks configured (Disk-optimized)")
    print(f"   Checkpoint: every 50K steps (no replay buffer)")
    print(f"   Evaluation: every 50K steps (10 episodes)")
    
    # Training
    print("\n5. Starting training...")
    print(f"   Total timesteps: {total_timesteps:,}")
    print(f"   TensorBoard: tensorboard --logdir {output_dir / 'tensorboard'}")
    print("")
    
    try:
        model.learn(
            total_timesteps=total_timesteps,
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
        # Close environments (train_env only since eval_env is the same)
        train_env.close()
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
    print("\n" + "=" * 80, flush=True)
    print("🎯 Main script starting...", flush=True)
    print("=" * 80, flush=True)
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", help="Run in headless mode (ignored, always headless)")
    parser.add_argument("--total_timesteps", type=int, default=50000, help="Total training timesteps")
    args = parser.parse_args()
    
    print(f"\n📊 Arguments parsed: total_timesteps={args.total_timesteps}", flush=True)
    
    try:
        print("\n🚀 Starting SAC training...", flush=True)
        train_sac(total_timesteps=args.total_timesteps)
    except Exception as e:
        print(f"\n❌ Training error: {e}", flush=True)
        import traceback
        traceback.print_exc()
    finally:
        # SimulationApp 정리
        simulation_app.close()
        print("\n✅ SimulationApp closed", flush=True)
