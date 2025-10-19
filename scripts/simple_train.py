#!/usr/bin/env python3
"""
RoArm-M3 간단한 RL 학습 스크립트
단일 환경 + PPO 알고리즘
"""

import sys
from pathlib import Path

# stdout buffering 해결
sys.stdout = sys.stderr

from isaacsim import SimulationApp

# Isaac Sim 초기화 (headless=True for faster training)
simulation_app = SimulationApp({
    "headless": True,  # GUI 없이 학습 (빠름)
    "width": 640,
    "height": 480,
})

print("✅ Isaac Sim 초기화 완료\n")

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor
import gymnasium as gym
from gymnasium import spaces
import time
from datetime import datetime

# 환경 임포트
sys.path.append(str(Path(__file__).parent.parent))
from envs.roarm_pick_place_env import RoArmPickPlaceEnv, RoArmPickPlaceEnvCfg


class GymWrapper(gym.Env):
    """Isaac Sim 환경을 Gymnasium 형식으로 래핑"""
    
    def __init__(self):
        super().__init__()
        
        # 환경 생성
        print("🔧 환경 생성 중...")
        cfg = RoArmPickPlaceEnvCfg()
        cfg.episode_length_s = 10.0
        self.env = RoArmPickPlaceEnv(cfg)
        
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
        print("✅ 환경 생성 완료\n")
    
    def reset(self, seed=None, options=None):
        if seed is not None:
            np.random.seed(seed)
        
        obs = self.env.reset()
        self.episode_count += 1
        
        # 주기적으로 진행 상황 출력
        if self.episode_count % 10 == 0:
            print(f"📊 Episode {self.episode_count} | Total steps: {self.total_steps}")
        
        return obs.astype(np.float32), {}
    
    def step(self, action):
        obs, reward, done, info = self.env.step(action)
        self.total_steps += 1
        
        # 1000 스텝마다 진행 상황 출력
        if self.total_steps % 1000 == 0:
            print(f"  ⏱️  {self.total_steps} steps completed...")
        
        terminated = done
        truncated = False
        
        return obs.astype(np.float32), reward, terminated, truncated, info
    
    def render(self):
        self.env.render()
    
    def close(self):
        self.env.close()


def train_simple_ppo(total_timesteps=50000):
    """간단한 PPO 학습"""
    
    print("=" * 70)
    print("🚀 RoArm-M3 Pick and Place - Easy Mode Training")
    print("=" * 70)
    print(f"  알고리즘: PPO")
    print(f"  총 스텝: {total_timesteps:,}")
    print(f"  시작 시간: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 70)
    print()
    
    # 로그 디렉토리
    log_dir = Path("logs/rl_training")
    log_dir.mkdir(parents=True, exist_ok=True)
    tensorboard_log = str(log_dir / "tensorboard")
    
    # 환경 생성
    env = GymWrapper()
    env = Monitor(env, str(log_dir / "monitor"))
    
    print("🧠 PPO 모델 생성 중...")
    # PPO 모델
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        verbose=1,
        tensorboard_log=tensorboard_log,
        device="cuda" if torch.cuda.is_available() else "cpu"
    )
    
    print(f"  Device: {model.device}")
    print(f"  Policy: MlpPolicy")
    print(f"  Learning rate: 3e-4")
    print("✅ 모델 생성 완료\n")
    
    # Callback 설정
    checkpoint_callback = CheckpointCallback(
        save_freq=5000,
        save_path=str(log_dir / "checkpoints"),
        name_prefix="roarm_ppo",
        save_replay_buffer=False,
        save_vecnormalize=False,
    )
    
    print("=" * 70)
    print("🎓 학습 시작!")
    print("=" * 70)
    print()
    
    start_time = time.time()
    
    try:
        # 학습 실행
        model.learn(
            total_timesteps=total_timesteps,
            callback=checkpoint_callback,
            progress_bar=False,  # Isaac Sim과 충돌 방지
        )
        
        elapsed = time.time() - start_time
        
        print()
        print("=" * 70)
        print("✅ 학습 완료!")
        print("=" * 70)
        print(f"  소요 시간: {elapsed/60:.1f}분")
        print(f"  평균 속도: {total_timesteps/elapsed:.1f} steps/sec")
        print()
        
        # 모델 저장
        model_path = log_dir / "final_model"
        model_path.mkdir(parents=True, exist_ok=True)
        final_model = model_path / "roarm_ppo_final.zip"
        model.save(final_model)
        print(f"💾 최종 모델 저장: {final_model}")
        
        return model
        
    except KeyboardInterrupt:
        print("\n⚠️  사용자 중단")
        elapsed = time.time() - start_time
        print(f"  학습 시간: {elapsed/60:.1f}분")
        
        # 중단 시에도 모델 저장
        interrupted_path = log_dir / "interrupted_model.zip"
        model.save(interrupted_path)
        print(f"💾 중단된 모델 저장: {interrupted_path}")
        
        return model
    
    except Exception as e:
        print(f"\n❌ 에러 발생: {e}")
        import traceback
        traceback.print_exc()
        raise
    
    finally:
        env.close()


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="RoArm-M3 간단한 RL 학습")
    parser.add_argument("--timesteps", type=int, default=50000,
                        help="총 학습 스텝 (기본: 50000)")
    
    args = parser.parse_args()
    
    try:
        model = train_simple_ppo(total_timesteps=args.timesteps)
        print("\n🎉 모든 작업 완료!")
        
    finally:
        simulation_app.close()
        print("👋 Simulation 종료")
