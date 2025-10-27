#!/usr/bin/env python3
"""
간단한 학습 실행 스크립트 (상태 체크용)
5000 timesteps만 실행하여 시스템 동작 확인
"""

import sys
from pathlib import Path

# stdout buffering 해결
sys.stdout = sys.stderr

from isaacsim import SimulationApp

# Isaac Sim 초기화 (headless=True for faster training)
simulation_app = SimulationApp({
    "headless": True,
    "width": 640,
    "height": 480,
})

print("✅ Isaac Sim 초기화 완료 (Quick Test)\n")

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
import gymnasium as gym
from gymnasium import spaces
import time
from datetime import datetime

# 환경 임포트
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from envs.roarm_pick_place_env import RoArmPickPlaceEnv, RoArmPickPlaceEnvCfg

# Callbacks 임포트
try:
    from scripts.rl.training_callbacks import TrainingProgressCallback, CurriculumCallback
    CALLBACKS_AVAILABLE = True
except ImportError:
    print("⚠️ Training callbacks not available")
    CALLBACKS_AVAILABLE = False


class GymWrapper(gym.Env):
    """Isaac Sim 환경을 Gymnasium 형식으로 래핑"""
    
    def __init__(self):
        super().__init__()
        
        # 환경 생성
        print("🔧 환경 생성 중 (Quick Test)...")
        cfg = RoArmPickPlaceEnvCfg()
        cfg.episode_length_s = 10.0
        cfg.curriculum_enabled = True
        cfg.curriculum_phase = 0
        
        self.env = RoArmPickPlaceEnv(cfg)
        
        self._max_episode_steps = self.env.max_steps
        
        # Observation space
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(self.env.observation_space_dim,),
            dtype=np.float32
        )
        
        # Action space
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(self.env.action_space_dim,),
            dtype=np.float32
        )
        
        self.episode_count = 0
        self.total_steps = 0
        self._elapsed_steps = 0
        
        print("✅ 환경 생성 완료\n")
    
    def reset(self, seed=None, options=None):
        if seed is not None:
            np.random.seed(seed)
            torch.manual_seed(seed)
        
        obs = self.env.reset()
        self.episode_count += 1
        self._elapsed_steps = 0
        
        if self.episode_count % 5 == 0:
            print(f"📊 Episode {self.episode_count} | Total steps: {self.total_steps}")
        
        return obs, {}
    
    def step(self, action):
        obs, reward, done, info = self.env.step(action)
        self.total_steps += 1
        self._elapsed_steps += 1
        
        terminated = done and self._elapsed_steps < self._max_episode_steps
        truncated = self._elapsed_steps >= self._max_episode_steps
        
        if truncated:
            info["TimeLimit.truncated"] = True
        
        return obs, reward, terminated, truncated, info


def quick_test(timesteps: int = 5000):
    """간단한 학습 테스트"""
    print("=" * 60)
    print("🧪 RoArm-M3 Quick Training Test")
    print("=" * 60)
    print(f"  테스트 timesteps: {timesteps:,}")
    print(f"  시작 시각: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    
    # 로그 디렉토리
    log_dir = Path("logs/quick_test")
    log_dir.mkdir(parents=True, exist_ok=True)
    
    # 환경 생성
    def make_env():
        env = GymWrapper()
        env = Monitor(env, str(log_dir / "monitor.csv"))
        return env
    
    print("🔧 환경 래핑 중...")
    env = DummyVecEnv([make_env])
    
    # VecNormalize
    print("🔧 VecNormalize 적용...")
    env = VecNormalize(
        env,
        norm_obs=True,
        norm_reward=True,
        clip_obs=10.0,
        clip_reward=10.0,
        gamma=0.99,
    )
    print()
    
    # 콜백
    callbacks = []
    
    # 진행 상황 로깅
    if CALLBACKS_AVAILABLE:
        progress_callback = TrainingProgressCallback(
            verbose=1,
            log_freq=5,  # 5 에피소드마다
        )
        callbacks.append(progress_callback)
        print("✅ 진행 상황 로깅 활성화 (5 에피소드마다)")
    
    # Curriculum (quick test에서는 비활성화)
    # if CALLBACKS_AVAILABLE:
    #     curriculum_callback = CurriculumCallback(...)
    #     callbacks.append(curriculum_callback)
    
    # 체크포인트
    checkpoint_callback = CheckpointCallback(
        save_freq=2000,
        save_path=str(log_dir / "checkpoints"),
        name_prefix="quick_test",
        save_vecnormalize=True,
    )
    callbacks.append(checkpoint_callback)
    print()
    
    # PPO 모델
    print("🤖 PPO 모델 생성...")
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        n_steps=512,  # Quick test: 작게
        batch_size=64,
        n_epochs=5,   # Quick test: 작게
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        clip_range_vf=1.0,
        ent_coef=0.01,
        vf_coef=0.5,
        max_grad_norm=0.5,
        target_kl=0.03,
        verbose=1,
        device="cuda" if torch.cuda.is_available() else "cpu",
        tensorboard_log=str(log_dir / "tensorboard"),
    )
    print(f"  Device: {model.device}")
    print(f"  n_steps: 512 (quick test)")
    print(f"  n_epochs: 5 (quick test)")
    print()
    
    # 학습 시작
    print("🎓 Quick 학습 시작...\n")
    start_time = time.time()
    
    try:
        model.learn(
            total_timesteps=timesteps,
            callback=callbacks,
            progress_bar=True,  # Quick test에서는 progress bar 표시
        )
    except KeyboardInterrupt:
        print("\n\n⚠️ 학습 중단 (Ctrl+C)")
    except Exception as e:
        print(f"\n\n❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
    
    # 완료
    elapsed_time = time.time() - start_time
    print("\n" + "=" * 60)
    print("✅ Quick Test 완료!")
    print("=" * 60)
    print(f"  총 시간: {elapsed_time / 60:.1f}분")
    print(f"  FPS: {timesteps / elapsed_time:.1f}")
    print(f"  종료 시각: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    # 모델 저장
    model_path = log_dir / "quick_test_model.zip"
    model.save(str(model_path))
    print(f"\n💾 모델 저장: {model_path}")
    
    vecnorm_path = log_dir / "vecnormalize.pkl"
    env.save(str(vecnorm_path))
    print(f"💾 VecNormalize 저장: {vecnorm_path}")
    
    return model


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Quick Training Test")
    parser.add_argument(
        "--timesteps",
        type=int,
        default=5000,
        help="테스트 timesteps (기본: 5000)"
    )
    
    args = parser.parse_args()
    
    try:
        quick_test(args.timesteps)
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n🔚 Isaac Sim 종료 중...")
        simulation_app.close()


if __name__ == "__main__":
    main()
