# Vision-Based SAC 구현 가이드

**날짜**: 2025-11-02  
**알고리즘**: SAC (Soft Actor-Critic)  
**목표**: Sample-efficient vision-based Pick & Place

---

## 🎯 왜 SAC인가?

### PPO vs SAC 비교

| 특성 | PPO | SAC |
|------|-----|-----|
| **Sample Efficiency** | ⭐⭐ (2-5M steps) | ⭐⭐⭐⭐⭐ (500K steps) |
| **학습 속도** | 느림 (10-20 hours) | 빠름 (5-10 hours) |
| **메모리** | 적음 | 많음 (Replay Buffer) |
| **안정성** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ |
| **구현 난이도** | ⭐⭐ | ⭐⭐⭐ |
| **Vision 특화** | ⭐⭐ | ⭐⭐⭐⭐⭐ |

### SAC 장점
1. **Off-Policy Learning**: Replay buffer로 데이터 재사용
2. **Entropy Regularization**: 자동 exploration
3. **Sample Efficiency**: Vision RL에서 검증됨
4. **Continuous Action**: 로봇 제어에 적합

---

## 📋 구현 단계

### Phase 1: Environment 완성 (3일)
**파일**: `envs/roarm_pick_place_env_vision.py`

**체크리스트**:
- [ ] Vision observation (4, 84, 84)
- [ ] Reward function (dense + sparse)
- [ ] Reset logic (randomization)
- [ ] Contact detection (grasp check)
- [ ] gym.check_env() 통과

---

### Phase 2: CNN Feature Extractor (2일)
**파일**: `models/cnn_extractor.py`

```python
import torch
import torch.nn as nn
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

class NatureCNN(BaseFeaturesExtractor):
    """
    CNN Feature Extractor for Vision RL
    
    Architecture:
        Input: (4, 84, 84) - RGBD
        Conv1: 32 filters (8×8, stride 4) → (32, 20, 20)
        Conv2: 64 filters (4×4, stride 2) → (64, 9, 9)
        Conv3: 64 filters (3×3, stride 1) → (64, 7, 7)
        Flatten: 3136
        Linear: 512
    """
    
    def __init__(self, observation_space, features_dim=512):
        super().__init__(observation_space, features_dim)
        
        n_input_channels = observation_space.shape[0]  # 4 for RGBD
        
        self.cnn = nn.Sequential(
            nn.Conv2d(n_input_channels, 32, kernel_size=8, stride=4, padding=0),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2, padding=0),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1, padding=0),
            nn.ReLU(),
            nn.Flatten(),
        )
        
        # Compute output shape
        with torch.no_grad():
            sample_input = torch.zeros(1, *observation_space.shape)
            n_flatten = self.cnn(sample_input).shape[1]
        
        self.linear = nn.Sequential(
            nn.Linear(n_flatten, features_dim),
            nn.ReLU(),
        )
    
    def forward(self, observations):
        return self.linear(self.cnn(observations))
```

**테스트**:
```bash
python -c "
import torch
from models.cnn_extractor import NatureCNN
from gymnasium import spaces

obs_space = spaces.Box(0, 1, shape=(4, 84, 84))
cnn = NatureCNN(obs_space, features_dim=512)

sample = torch.randn(8, 4, 84, 84)  # Batch of 8
features = cnn(sample)
print(f'Input: {sample.shape}')
print(f'Output: {features.shape}')  # (8, 512)
"
```

---

### Phase 3: SAC Training Script (2일)
**파일**: `scripts/train/train_vision_sac.py`

```python
#!/usr/bin/env python3
"""
Vision-based SAC Training

SAC 특징:
- Off-policy learning (replay buffer)
- Entropy regularization (automatic exploration)
- Sample efficient (500K steps)
"""

import argparse
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=4)
parser.add_argument("--total_timesteps", type=int, default=500_000)
parser.add_argument("--save_freq", type=int, default=10_000)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import (
    CheckpointCallback,
    EvalCallback,
    CallbackList
)
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

# Custom imports
import sys
sys.path.insert(0, str(Path(__file__).parents[2]))
from envs.roarm_pick_place_env_vision import RoArmPickPlaceVisionEnv
from models.cnn_extractor import NatureCNN

# Paths
PROJECT_DIR = Path(__file__).parents[2]
LOG_DIR = PROJECT_DIR / "logs" / "vision_sac"
CHECKPOINT_DIR = PROJECT_DIR / "checkpoints" / "vision_sac"
LOG_DIR.mkdir(parents=True, exist_ok=True)
CHECKPOINT_DIR.mkdir(parents=True, exist_ok=True)

print("=" * 80)
print("🎮 Vision-based SAC Training")
print("=" * 80)
print(f"Total timesteps: {args.total_timesteps:,}")
print(f"Num envs: {args.num_envs}")
print(f"Log dir: {LOG_DIR}")


def make_env():
    """Create environment"""
    env = RoArmPickPlaceVisionEnv()
    env = Monitor(env)
    return env


def main():
    # 1. Create environment
    print("\n📦 Creating environment...")
    env = DummyVecEnv([make_env for _ in range(args.num_envs)])
    env = VecNormalize(env, norm_obs=False, norm_reward=True, clip_reward=10.0)
    
    print(f"✅ Environment created")
    print(f"   Observation space: {env.observation_space.shape}")
    print(f"   Action space: {env.action_space.shape}")
    
    # 2. Create SAC model
    print("\n🧠 Creating SAC model...")
    model = SAC(
        "CnnPolicy",
        env,
        policy_kwargs=dict(
            features_extractor_class=NatureCNN,
            features_extractor_kwargs=dict(features_dim=512),
            net_arch=dict(pi=[256, 256], qf=[256, 256]),
        ),
        # SAC hyperparameters
        buffer_size=100_000,
        learning_rate=3e-4,
        batch_size=256,
        tau=0.005,
        gamma=0.99,
        train_freq=1,
        gradient_steps=1,
        ent_coef='auto',  # Automatic entropy tuning
        target_update_interval=1,
        target_entropy='auto',
        # Logging
        verbose=1,
        tensorboard_log=str(LOG_DIR),
        device='cuda' if torch.cuda.is_available() else 'cpu',
    )
    
    print(f"✅ SAC model created")
    print(f"   Device: {model.device}")
    print(f"   Buffer size: {model.buffer_size:,}")
    print(f"   Batch size: {model.batch_size}")
    
    # 3. Callbacks
    print("\n📊 Setting up callbacks...")
    checkpoint_callback = CheckpointCallback(
        save_freq=args.save_freq,
        save_path=str(CHECKPOINT_DIR),
        name_prefix="sac_vision",
        save_replay_buffer=True,
        save_vecnormalize=True,
    )
    
    eval_env = DummyVecEnv([make_env])
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=str(CHECKPOINT_DIR / "best"),
        log_path=str(LOG_DIR / "eval"),
        eval_freq=10_000,
        n_eval_episodes=10,
        deterministic=True,
    )
    
    callbacks = CallbackList([checkpoint_callback, eval_callback])
    
    # 4. Train
    print("\n" + "=" * 80)
    print("🚀 Starting Training")
    print("=" * 80)
    print(f"Target: {args.total_timesteps:,} steps")
    print(f"Estimated time: ~5-10 hours (RTX 3090)")
    print(f"\n📈 Monitor training:")
    print(f"   tensorboard --logdir {LOG_DIR}")
    print()
    
    try:
        model.learn(
            total_timesteps=args.total_timesteps,
            callback=callbacks,
            log_interval=10,
            tb_log_name="sac_vision_run",
        )
        
        # Save final model
        final_path = CHECKPOINT_DIR / "sac_vision_final"
        model.save(final_path)
        env.save(CHECKPOINT_DIR / "vec_normalize.pkl")
        
        print("\n" + "=" * 80)
        print("✅ Training Complete!")
        print("=" * 80)
        print(f"📁 Final model: {final_path}.zip")
        print(f"📁 Best model: {CHECKPOINT_DIR / 'best' / 'best_model.zip'}")
        
    except KeyboardInterrupt:
        print("\n⏹️  Training interrupted by user")
        
        # Save interrupted model
        interrupted_path = CHECKPOINT_DIR / "sac_vision_interrupted"
        model.save(interrupted_path)
        print(f"💾 Model saved: {interrupted_path}.zip")
    
    finally:
        env.close()
        simulation_app.close()


if __name__ == "__main__":
    main()
```

**실행**:
```bash
cd /home/roarm_m3/roarm_isaac_clean
/home/roarm_m3/isaacsim/python.sh scripts/train/train_vision_sac.py \
    --num_envs 4 \
    --total_timesteps 500000 \
    --enable_cameras
```

---

### Phase 4: Evaluation Script (1일)
**파일**: `scripts/eval/eval_vision_sac.py`

```python
#!/usr/bin/env python3
"""
Evaluate trained SAC model
"""

import argparse
from pathlib import Path
import numpy as np

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--model_path", type=str, required=True)
parser.add_argument("--num_episodes", type=int, default=100)
parser.add_argument("--render", action="store_true")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

import sys
sys.path.insert(0, str(Path(__file__).parents[2]))
from envs.roarm_pick_place_env_vision import RoArmPickPlaceVisionEnv

print("=" * 80)
print("📊 SAC Model Evaluation")
print("=" * 80)
print(f"Model: {args.model_path}")
print(f"Episodes: {args.num_episodes}")


def main():
    # Load model
    print("\n📦 Loading model...")
    model = SAC.load(args.model_path)
    print("✅ Model loaded")
    
    # Create env
    env = DummyVecEnv([lambda: RoArmPickPlaceVisionEnv()])
    
    # Load normalization
    vec_normalize_path = Path(args.model_path).parent / "vec_normalize.pkl"
    if vec_normalize_path.exists():
        env = VecNormalize.load(vec_normalize_path, env)
        env.training = False
        env.norm_reward = False
        print("✅ VecNormalize loaded")
    
    # Evaluate
    print("\n🎮 Running evaluation...")
    episode_rewards = []
    success_count = 0
    
    for ep in range(args.num_episodes):
        obs = env.reset()
        episode_reward = 0
        done = False
        step_count = 0
        
        while not done and step_count < 1000:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            episode_reward += reward[0]
            step_count += 1
            
            if done:
                if info[0].get('is_success', False):
                    success_count += 1
        
        episode_rewards.append(episode_reward)
        
        if (ep + 1) % 10 == 0:
            print(f"  Episode {ep+1}/{args.num_episodes}: "
                  f"Reward={episode_reward:.2f}, "
                  f"Success Rate={success_count/(ep+1)*100:.1f}%")
    
    # Results
    print("\n" + "=" * 80)
    print("📊 Evaluation Results")
    print("=" * 80)
    print(f"Episodes: {args.num_episodes}")
    print(f"Success Rate: {success_count/args.num_episodes*100:.1f}%")
    print(f"Mean Reward: {np.mean(episode_rewards):.2f} ± {np.std(episode_rewards):.2f}")
    print(f"Min Reward: {np.min(episode_rewards):.2f}")
    print(f"Max Reward: {np.max(episode_rewards):.2f}")
    
    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
```

---

## 📊 학습 모니터링

### TensorBoard
```bash
# Terminal 1: Training
/home/roarm_m3/isaacsim/python.sh scripts/train/train_vision_sac.py --enable_cameras

# Terminal 2: Monitoring
tensorboard --logdir logs/vision_sac --port 6006
# http://localhost:6006
```

### 주요 Metrics
- **rollout/ep_rew_mean**: Episode reward (증가해야 함)
- **rollout/ep_len_mean**: Episode length
- **train/actor_loss**: Actor loss
- **train/critic_loss**: Critic loss
- **train/ent_coef**: Entropy coefficient (자동 조정)
- **train/learning_rate**: Learning rate

---

## ⚙️ Hyperparameter Tuning

### 기본 설정 (추천)
```python
SAC(
    buffer_size=100_000,      # Replay buffer
    learning_rate=3e-4,       # Adam LR
    batch_size=256,           # Batch size
    tau=0.005,                # Soft update
    gamma=0.99,               # Discount factor
    train_freq=1,             # Update every step
    gradient_steps=1,         # 1 gradient step per env step
    ent_coef='auto',          # Auto entropy tuning
)
```

### 실험할 변형
1. **Large Buffer** (더 많은 경험)
   ```python
   buffer_size=200_000
   ```

2. **Aggressive Update** (빠른 학습)
   ```python
   train_freq=4
   gradient_steps=4
   ```

3. **Larger Network** (표현력 증가)
   ```python
   net_arch=dict(pi=[512, 512], qf=[512, 512])
   ```

---

## 🎯 예상 성능

### 학습 곡선 (500K steps)
```
0-100K:   Exploration (reward ~-500)
100K-200K: 학습 시작 (reward ~-200)
200K-300K: 빠른 향상 (reward ~0)
300K-400K: 수렴 (reward ~50)
400K-500K: Fine-tuning (reward ~100)
```

### Success Rate
- **100K**: ~0%
- **200K**: ~20%
- **300K**: ~50%
- **400K**: ~65%
- **500K**: ~70-80% (목표)

---

## 🚀 다음 단계

### Immediate
1. ✅ Vision preprocessing (완료)
2. ⏳ Environment 완성
3. ⏳ CNN feature extractor
4. ⏳ SAC training script

### This Week
5. 첫 SAC 학습 (100K steps baseline)
6. Hyperparameter tuning
7. 500K steps 학습

### Next Week
8. Evaluation & visualization
9. Domain randomization (선택)
10. 문서화 & 결과 보고서

---

**Last Updated**: 2025-11-02  
**Status**: SAC 구현 시작  
**Next Action**: Environment 완성
