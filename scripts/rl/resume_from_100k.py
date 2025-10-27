#!/usr/bin/env python3
"""
v3.7.7 100K 모델에서 학습 재개
"""
import sys
from pathlib import Path

# stdout buffering 해제
sys.stdout = sys.stderr

from isaacsim import SimulationApp

# Isaac Sim 초기화 (headless)
simulation_app = SimulationApp({
    "headless": True,
    "width": 640,
    "height": 480,
})

print("✅ Isaac Sim 초기화 완료 (학습 재개)\n")

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
import gymnasium as gym
from gymnasium import spaces
from datetime import datetime

# 환경 임포트
sys.path.append(str(Path(__file__).parent.parent))
from envs.roarm_pick_place_env import RoArmPickPlaceEnv, RoArmPickPlaceEnvCfg


class GymWrapper(gym.Env):
    """Isaac Sim 환경을 Gymnasium 형식으로 래핑"""
    
    def __init__(self):
        super().__init__()
        cfg = RoArmPickPlaceEnvCfg()
        cfg.episode_length_s = 10.0
        self.env = RoArmPickPlaceEnv(cfg)
        
        obs_dim = 28  # RoArm-M3
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(8,), dtype=np.float32
        )
        
        self.episode_count = 0
        self.total_steps = 0
    
    def reset(self, seed=None, options=None):
        if seed is not None:
            np.random.seed(seed)
        obs = self.env.reset()
        self.episode_count += 1
        if self.episode_count % 10 == 0:
            print(f"📊 Episode {self.episode_count} | Total steps: {self.total_steps}")
        return obs, {}
    
    def step(self, action):
        obs, reward, done, info = self.env.step(action)
        self.total_steps += 1
        terminated = done
        truncated = False
        return obs, reward, terminated, truncated, info


def main():
    # 설정
    checkpoint_path = "logs/rl_training_curriculum/checkpoints/roarm_ppo_curriculum_100000_steps.zip"
    vecnorm_path = "logs/rl_training_curriculum/checkpoints/roarm_ppo_curriculum_vecnormalize_100000_steps.pkl"
    additional_timesteps = 200000  # 200K 추가 (총 300K)
    
    print("=" * 80)
    print("🔄 v3.7.7 학습 재개 (100K → 300K)")
    print("=" * 80)
    print(f"  기존 모델: {checkpoint_path}")
    print(f"  VecNormalize: {vecnorm_path}")
    print(f"  추가 학습: {additional_timesteps:,} steps")
    print(f"  목표: GRIP 5% 유지 + REACH 증가")
    print(f"  시작: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 80)
    print()
    
    # 환경 생성
    print("📦 환경 생성 중...")
    
    def make_env():
        env = GymWrapper()
        return Monitor(env, filename=None, allow_early_resets=True)
    
    venv = DummyVecEnv([make_env])
    
    # VecNormalize 로드
    print(f"✅ VecNormalize 로드: {vecnorm_path}")
    venv = VecNormalize.load(vecnorm_path, venv)
    venv.training = True
    venv.norm_reward = True
    
    # 모델 로드
    print(f"📂 모델 로드: {checkpoint_path}")
    model = PPO.load(checkpoint_path, env=venv, device="auto")
    
    print(f"  현재 timesteps: {model.num_timesteps:,}")
    print(f"  목표 timesteps: {model.num_timesteps + additional_timesteps:,}")
    print()
    
    # 체크포인트 콜백
    log_dir = Path("logs/rl_training_resume_v3.7.7")
    log_dir.mkdir(parents=True, exist_ok=True)
    
    checkpoint_callback = CheckpointCallback(
        save_freq=5000,
        save_path=str(log_dir / "checkpoints"),
        name_prefix="roarm_ppo_resumed",
        save_vecnormalize=True,
        verbose=1
    )
    
    # 학습 재개
    print(f"🚀 학습 시작! (reset_num_timesteps=False)")
    print(f"{'=' * 80}\n")
    
    start_time = datetime.now()
    
    try:
        model.learn(
            total_timesteps=additional_timesteps,
            callback=checkpoint_callback,
            log_interval=10,
            progress_bar=True,
            reset_num_timesteps=False  # 🔥 중요: 기존 timestep 카운트 유지
        )
    except KeyboardInterrupt:
        print("\n\n⚠️  사용자 중단 (Ctrl+C)")
    except Exception as e:
        print(f"\n\n❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
    
    end_time = datetime.now()
    duration = (end_time - start_time).total_seconds() / 60
    
    # 최종 모델 저장
    print(f"\n{'=' * 80}")
    print("✅ 학습 완료!")
    print(f"{'=' * 80}")
    print(f"  총 시간: {duration:.1f}분")
    print(f"  최종 timesteps: {model.num_timesteps:,}")
    print(f"  종료: {end_time.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"{'=' * 80}\n")
    
    # 최종 모델 저장
    final_dir = log_dir / "final_model"
    final_dir.mkdir(parents=True, exist_ok=True)
    
    model_path = final_dir / "roarm_ppo_resumed_final.zip"
    model.save(str(model_path))
    print(f"💾 최종 모델 저장: {model_path}")
    
    # VecNormalize 저장
    vecnorm_save_path = final_dir / "vecnormalize.pkl"
    venv.save(str(vecnorm_save_path))
    print(f"💾 VecNormalize 저장: {vecnorm_save_path}")
    print(f"   ⚠️  테스트 시 반드시 함께 로드하세요!\n")
    
    print("🔚 Isaac Sim 종료 중...")
    simulation_app.close()
    print("✅ 완료")


if __name__ == "__main__":
    main()
