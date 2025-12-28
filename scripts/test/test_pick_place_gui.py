#!/usr/bin/env python3
"""
Visualize Pick & Place Model in GUI Mode
Shows the trained robot performing pick and place task

Usage:
    /home/roarm_m3/isaacsim/python.sh scripts/test/test_pick_place_gui.py
"""

import sys
import os
import warnings

warnings.filterwarnings("ignore", message=".*Gym has been unmaintained.*")
os.environ["GYM_IGNORE_DEPRECATION_WARNINGS"] = "1"

# GUI mode (not headless)
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

print("="*80)
print("🎮 Pick & Place Visualization - GUI Mode")
print("="*80)

sys.path.append(os.path.join(os.path.dirname(__file__), "../.."))

import numpy as np
from pathlib import Path
import time

from stable_baselines3 import SAC
from envs.pick_place_vision_env import PickPlaceVisionEnv

print("✅ Imports successful")


def find_best_model():
    """Find the best/latest model from training"""
    # Check for v3 training models first
    v3_dirs = sorted(Path("logs/pick_place_training").glob("20251227_17*"))
    
    for training_dir in reversed(v3_dirs):
        best_model = training_dir / "best_model" / "best_model.zip"
        if best_model.exists():
            return str(best_model)
        
        # Check checkpoints
        checkpoints = sorted(training_dir.glob("checkpoints/*.zip"))
        if checkpoints:
            return str(checkpoints[-1])
    
    return None


def test_model(model_path=None, num_episodes=5):
    """Test model in GUI mode"""
    
    # Create environment in GUI mode
    print("\n1. Creating environment (GUI mode)...")
    env = PickPlaceVisionEnv(render_mode="human", headless=False)
    print("✅ Environment created")
    
    # Load model if provided
    model = None
    if model_path and Path(model_path).exists():
        print(f"\n2. Loading model: {model_path}")
        model = SAC.load(model_path)
        print("✅ Model loaded")
    else:
        print("\n2. No model loaded - using random actions")
    
    print(f"\n3. Running {num_episodes} episodes...")
    print("="*60)
    
    for episode in range(num_episodes):
        obs, info = env.reset()
        print(f"\n📍 Episode {episode + 1}")
        print(f"   Cube: {info.get('cube_pos')}")
        print(f"   Target: {info.get('target_pos')}")
        
        total_reward = 0
        reached = False
        grasped = False
        lifted = False
        
        for step in range(300):
            if model:
                action, _ = model.predict(obs, deterministic=True)
            else:
                # Random action with bias toward closing gripper
                action = env.action_space.sample() * 0.3
                if step > 100:
                    action[6] = -0.8  # Close gripper
            
            obs, reward, done, truncated, info = env.step(action)
            total_reward += reward
            
            # Track milestones
            if info.get('stage_reached') and not reached:
                print(f"   📍 REACHED at step {step}")
                reached = True
            if info.get('stage_grasped') and not grasped:
                print(f"   🤏 GRASPED at step {step}")
                grasped = True
            if info.get('stage_lifted') and not lifted:
                print(f"   ⬆️ LIFTED at step {step}")
                lifted = True
            if info.get('success'):
                print(f"   🎯 SUCCESS at step {step}!")
            
            # Small delay for visualization
            time.sleep(0.01)
            
            if done:
                break
        
        print(f"   Total reward: {total_reward:.1f}")
        print(f"   Steps: {step + 1}")
    
    print("\n" + "="*60)
    print("🎉 Visualization complete!")
    
    env.close()


if __name__ == "__main__":
    try:
        # Find best model
        model_path = find_best_model()
        if model_path:
            print(f"Found model: {model_path}")
        else:
            print("No trained model found, using random actions")
        
        test_model(model_path=model_path, num_episodes=3)
        
    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        simulation_app.close()
        print("✅ Closed")
