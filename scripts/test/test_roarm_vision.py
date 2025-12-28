#!/usr/bin/env python3
"""
Test RoArm Vision Environment
기존 roarm_pick_place_env.py + Vision 기능 테스트
"""

import argparse
import numpy as np
from pathlib import Path

# AppLauncher must be first
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.app import AppLauncher

parser = argparse.ArgumentParser(description="Test RoArm Vision Environment")
parser.add_argument("--obs_mode", type=str, default="vision", choices=["vector", "vision"],
                    help="Observation mode: vector or vision")
parser.add_argument("--num_steps", type=int, default=50, help="Number of test steps")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch Isaac Sim
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import sys
sys.path.insert(0, str(Path(__file__).parents[2]))

# NOTE: import create_roarm_vision_env later, AFTER AppLauncher!
from PIL import Image
from datetime import datetime

print("=" * 80)
print("🧪 Testing RoArm Vision Environment")
print("=" * 80)

# Output directory
output_dir = Path(__file__).parents[2] / "output" / "roarm_vision_test" / datetime.now().strftime("%Y%m%d_%H%M%S")
output_dir.mkdir(parents=True, exist_ok=True)

def main():
    # Import environment factory AFTER AppLauncher initialized
    from envs.roarm_vision_wrapper import create_roarm_vision_env
    
    # Create environment
    print(f"\n1. Creating environment (mode={args_cli.obs_mode})...")
    env = create_roarm_vision_env(obs_mode=args_cli.obs_mode)
    
    print(f"✅ Environment created")
    print(f"   Observation space: {env.observation_space.shape}")
    print(f"   Action space: {env.action_space.shape}")
    
    # Reset
    print(f"\n2. Resetting environment...")
    obs, info = env.reset()
    print(f"✅ Reset successful")
    print(f"   Observation shape: {obs.shape}")
    print(f"   Observation dtype: {obs.dtype}")
    
    if args_cli.obs_mode == "vision":
        print(f"   Observation range: [{obs.min():.3f}, {obs.max():.3f}]")
        
        # Save first observation
        rgb = (obs[:3].transpose(1, 2, 0) * 255).astype(np.uint8)
        depth = (obs[3] * 255).astype(np.uint8)
        Image.fromarray(rgb).save(output_dir / "step_000_rgb.png")
        Image.fromarray(depth).save(output_dir / "step_000_depth.png")
        print(f"   💾 Saved: step_000_rgb.png, step_000_depth.png")
    else:
        print(f"   Vector observation: {obs[:5]}...")  # First 5 values
    
    # Run steps
    print(f"\n3. Running {args_cli.num_steps} steps with random policy...")
    total_reward = 0.0
    
    for step in range(args_cli.num_steps):
        # Random action
        action = env.action_space.sample()
        
        # Step
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        
        # Log every 10 steps
        if step % 10 == 0:
            print(f"   Step {step:3d}: reward={reward:7.3f}, total={total_reward:7.2f}")
            
            # Save vision observation
            if args_cli.obs_mode == "vision":
                rgb = (obs[:3].transpose(1, 2, 0) * 255).astype(np.uint8)
                depth = (obs[3] * 255).astype(np.uint8)
                Image.fromarray(rgb).save(output_dir / f"step_{step:03d}_rgb.png")
                Image.fromarray(depth).save(output_dir / f"step_{step:03d}_depth.png")
        
        # Check done
        if terminated or truncated:
            print(f"\n   Episode ended at step {step}")
            print(f"   Reason: {'terminated' if terminated else 'truncated'}")
            break
    
    print(f"\n✅ Test completed")
    print(f"   Total reward: {total_reward:.2f}")
    print(f"   Final observation shape: {obs.shape}")
    
    if args_cli.obs_mode == "vision":
        print(f"   💾 Saved images: {output_dir}")
    
    # Close
    print(f"\n4. Closing environment...")
    env.close()
    print("✅ Environment closed")
    
    print("\n" + "=" * 80)
    print("✅ All tests passed!")
    print("=" * 80)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        simulation_app.close()
        sys.exit(1)
    
    simulation_app.close()
