#!/usr/bin/env python3
"""
Vision Environment Pre-flight Check

최종 확인 사항:
1. Environment 초기화
2. Observation space 검증
3. Action space 검증
4. 1 Episode 실행
5. 이미지 전처리 확인

작성일: 2025-11-02
"""

import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parents[2]))

print("=" * 80)
print("🔍 Vision RL Pre-flight Check")
print("=" * 80)
print()

# Enable cameras before importing
import sys
sys.argv.extend(["--enable_cameras"])

# 1. Environment import
print("1️⃣ Importing environment...")
try:
    from envs.simple_vision_env import SimpleVisionEnv
    print("   ✅ SimpleVisionEnv imported")
except Exception as e:
    print(f"   ❌ Failed: {e}")
    sys.exit(1)

# 2. CNN Extractor import
print("\n2️⃣ Importing CNN extractor...")
try:
    from models.cnn_extractor import NatureCNN
    print("   ✅ NatureCNN imported")
except Exception as e:
    print(f"   ❌ Failed: {e}")
    sys.exit(1)

# 3. Create environment
print("\n3️⃣ Creating environment...")
try:
    env = SimpleVisionEnv(render_mode=None)
    print(f"   ✅ Environment created")
    print(f"   📊 Observation space: {env.observation_space.shape}")
    print(f"   🎮 Action space: {env.action_space.shape}")
except Exception as e:
    print(f"   ❌ Failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# 4. Reset
print("\n4️⃣ Testing reset...")
try:
    obs, info = env.reset()
    print(f"   ✅ Reset successful")
    print(f"   📷 Observation shape: {obs.shape}")
    print(f"   📏 Observation range: [{obs.min():.3f}, {obs.max():.3f}]")
except Exception as e:
    print(f"   ❌ Failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# 5. One step
print("\n5️⃣ Testing step...")
try:
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    print(f"   ✅ Step successful")
    print(f"   🎁 Reward: {reward:.3f}")
    print(f"   🏁 Done: {terminated or truncated}")
except Exception as e:
    print(f"   ❌ Failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# 6. Test CNN
print("\n6️⃣ Testing CNN extractor...")
try:
    import torch
    from gymnasium import spaces
    
    obs_space = spaces.Box(0, 1, shape=(4, 84, 84), dtype='float32')
    cnn = NatureCNN(obs_space, features_dim=512)
    
    batch = torch.randn(8, 4, 84, 84)
    features = cnn(batch)
    
    print(f"   ✅ CNN test successful")
    print(f"   📊 Input: {batch.shape}")
    print(f"   📊 Output: {features.shape}")
except Exception as e:
    print(f"   ❌ Failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# 7. Test gym.check_env
print("\n7️⃣ Running gym.check_env...")
try:
    from gymnasium.utils.env_checker import check_env
    check_env(env, skip_render_check=True)
    print("   ✅ gym.check_env passed")
except Exception as e:
    print(f"   ⚠️  Warning: {e}")

# Cleanup
print("\n8️⃣ Cleanup...")
env.close()
print("   ✅ Environment closed")

print()
print("=" * 80)
print("✅ Pre-flight check completed!")
print("=" * 80)
print()
print("📋 Next steps:")
print("   1. Test: bash scripts/launch_vision_rl.sh --test")
print("   2. Quick train: bash scripts/launch_vision_rl.sh --quick")
print("   3. Full train: bash scripts/launch_vision_rl.sh --train")
print()
