#!/usr/bin/env python3
"""
Train SAC for Pick & Place with Gripper Control
Based on successful Reaching model, extended for grasping

Usage:
    /home/roarm_m3/isaacsim/python.sh scripts/train/train_pick_place_sac.py
"""

import sys
import os
import argparse
import warnings

# Suppress warnings
warnings.filterwarnings("ignore", message=".*Gym has been unmaintained.*")
os.environ["GYM_IGNORE_DEPRECATION_WARNINGS"] = "1"

sys.stdout = sys.__stdout__
sys.stderr = sys.__stderr__

# Initialize SimulationApp first
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

print("="*80)
print("🚀 Pick & Place SAC Training - SimulationApp Initialized!")
print("="*80)

sys.path.append(os.path.join(os.path.dirname(__file__), "../.."))

from datetime import datetime
from pathlib import Path

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback

from envs.pick_place_vision_env import PickPlaceVisionEnv
from models.cnn_extractor import NatureCNN

print("✅ All imports successful")


def create_env():
    """Create Pick & Place environment"""
    env = PickPlaceVisionEnv()
    return env


def train_sac(total_timesteps=100000):
    """Train SAC on Pick & Place task"""
    print("=" * 80)
    print(f"🚀 Training SAC on Pick & Place Task ({total_timesteps:,} timesteps)")
    print("=" * 80)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(f"logs/pick_place_training/{timestamp}")
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"\n📁 Output directory: {output_dir}")
    
    # Create environment
    print("\n1. Creating Pick & Place environment...")
    train_env = create_env()
    eval_env = train_env  # Same env for eval to avoid conflicts
    print("✅ Environment created (7D action space: 6 arm + 1 gripper)")
    
    # Policy configuration
    print("\n2. Configuring SAC policy...")
    policy_kwargs = dict(
        features_extractor_class=NatureCNN,
        features_extractor_kwargs=dict(features_dim=512),
        net_arch=[256, 256],
    )
    print("✅ Policy configured with NatureCNN")
    
    # SAC model with high exploration (proven effective in Reaching)
    print("\n3. Creating SAC model...")
    model = SAC(
        "CnnPolicy",
        train_env,
        policy_kwargs=policy_kwargs,
        buffer_size=100_000,
        batch_size=256,
        learning_rate=3e-4,
        tau=0.005,
        gamma=0.99,
        train_freq=1,
        gradient_steps=1,
        learning_starts=5_000,  # Proven: start early
        ent_coef=0.2,  # Proven: high exploration
        target_entropy="auto",
        tensorboard_log=str(output_dir / "tensorboard"),
        verbose=1,
        device="cuda",
    )
    print("✅ SAC model created")
    print(f"   Action space: 7D (6 arm + 1 gripper)")
    print(f"   Entropy coef: 0.2 (high exploration)")
    print(f"   Learning starts: 5K steps")
    
    # Callbacks
    print("\n4. Setting up callbacks...")
    checkpoint_callback = CheckpointCallback(
        save_freq=25_000,  # More frequent for complex task
        save_path=str(output_dir / "checkpoints"),
        name_prefix="pick_place",
        save_replay_buffer=False,
        save_vecnormalize=True,
    )
    
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=str(output_dir / "best_model"),
        log_path=str(output_dir / "eval"),
        eval_freq=25_000,
        n_eval_episodes=5,
        deterministic=True,
        render=False,
    )
    
    callbacks = [checkpoint_callback, eval_callback]
    print("✅ Callbacks configured")
    print(f"   Checkpoint: every 25K steps")
    print(f"   Evaluation: every 25K steps (5 episodes)")
    
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
        
        final_model_path = output_dir / "final_model" / "pick_place_final"
        model.save(str(final_model_path))
        print(f"✅ Final model saved: {final_model_path}")
        
    except KeyboardInterrupt:
        print("\n⚠️ Training interrupted")
        interrupted_path = output_dir / "interrupted_model" / "pick_place_interrupted"
        model.save(str(interrupted_path))
        print(f"✅ Interrupted model saved: {interrupted_path}")
    
    finally:
        train_env.close()
        print("✅ Environment closed")
    
    print("\n" + "=" * 80)
    print("🎉 Training session completed!")
    print("=" * 80)
    print(f"\n📊 Results:")
    print(f"   Output: {output_dir}")
    print(f"   TensorBoard: tensorboard --logdir {output_dir / 'tensorboard'}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--total_timesteps", type=int, default=100000,
                       help="Total training timesteps (default: 100K)")
    args = parser.parse_args()
    
    try:
        train_sac(total_timesteps=args.total_timesteps)
    except Exception as e:
        print(f"\n❌ Training error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        simulation_app.close()
        print("\n✅ SimulationApp closed")
