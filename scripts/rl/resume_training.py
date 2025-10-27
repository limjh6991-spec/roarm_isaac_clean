#!/usr/bin/env python3
"""
RoArm-M3 Dense Reward RL 학습 재개 스크립트
마지막 체크포인트부터 계속 학습
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

print("✅ Isaac Sim 초기화 완료 (학습 재개)\n")

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
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
        
        print("🔧 환경 생성 중 (Dense Reward)...")
        cfg = RoArmPickPlaceEnvCfg()
        cfg.episode_length_s = 10.0
        self.env = RoArmPickPlaceEnv(cfg)
        
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf,
            shape=(self.env.observation_space_dim,), dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=-1.0, high=1.0,
            shape=(self.env.action_space_dim,), dtype=np.float32
        )
        
        self.episode_count = 0
        self.total_steps = 0
        print("✅ 환경 생성 완료\n")
    
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


def resume_training(checkpoint_path: str, additional_timesteps: int = 50000):
    """체크포인트에서 학습 재개"""
    print("=" * 60)
    print("🔄 학습 재개")
    print("=" * 60)
    print(f"  체크포인트: {checkpoint_path}")
    print(f"  추가 timesteps: {additional_timesteps:,}")
    print(f"  시작: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    
    # 환경 생성
    env = GymWrapper()
    
    # Monitor로 래핑
    log_dir = Path("logs/rl_training_dense")
    env = Monitor(env, str(log_dir / "monitor.monitor.csv"), allow_early_resets=True)
    
    print(f"📂 로그 디렉토리: {log_dir}\n")
    
    # 체크포인트 콜백
    checkpoint_callback = CheckpointCallback(
        save_freq=5000,
        save_path=str(log_dir / "checkpoints"),
        name_prefix="roarm_ppo",
        save_replay_buffer=False,
        save_vecnormalize=False,
    )
    
    # 모델 로드
    print(f"🤖 체크포인트에서 모델 로드 중...")
    model = PPO.load(checkpoint_path, env=env)
    print(f"  Device: {model.device}")
    print()
    
    # 학습 재개
    print("🎓 학습 재개...\n")
    start_time = time.time()
    
    try:
        model.learn(
            total_timesteps=additional_timesteps,
            callback=checkpoint_callback,
            progress_bar=False,
            reset_num_timesteps=False,  # 중요! 스텝 카운트 유지
        )
    except KeyboardInterrupt:
        print("\n\n⚠️ 학습 중단 (Ctrl+C)")
    
    # 완료
    elapsed_time = time.time() - start_time
    print("\n" + "=" * 60)
    print("✅ 학습 완료!")
    print("=" * 60)
    print(f"  총 시간: {elapsed_time / 60:.1f}분")
    print(f"  종료: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    # 최종 모델 저장
    final_model_dir = log_dir / "final_model"
    final_model_dir.mkdir(parents=True, exist_ok=True)
    model_path = final_model_dir / "roarm_ppo_dense_final.zip"
    model.save(str(model_path))
    print(f"\n💾 최종 모델 저장: {model_path}")
    
    return model


def find_latest_checkpoint():
    """최신 체크포인트 찾기"""
    checkpoint_dir = Path("logs/rl_training_dense/checkpoints")
    
    if not checkpoint_dir.exists():
        return None
    
    checkpoints = sorted(checkpoint_dir.glob("roarm_ppo_*_steps.zip"))
    
    if not checkpoints:
        return None
    
    # 가장 큰 스텝 수를 가진 체크포인트 찾기
    latest = None
    max_steps = 0
    
    for ckpt in checkpoints:
        try:
            steps = int(ckpt.stem.split("_")[-2])
            if steps > max_steps:
                max_steps = steps
                latest = ckpt
        except:
            continue
    
    return latest


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="RoArm-M3 Dense Reward RL Resume Training")
    parser.add_argument(
        "--checkpoint",
        type=str,
        default=None,
        help="체크포인트 경로 (자동 탐색: 최신 체크포인트)"
    )
    parser.add_argument(
        "--timesteps",
        type=int,
        default=50000,
        help="추가 학습 timesteps (기본: 50,000)"
    )
    
    args = parser.parse_args()
    
    # 체크포인트 찾기
    if args.checkpoint is None:
        print("🔍 최신 체크포인트 검색 중...\n")
        checkpoint_path = find_latest_checkpoint()
        
        if checkpoint_path is None:
            print("❌ 체크포인트를 찾을 수 없습니다.")
            print("\n💡 처음부터 학습하려면:")
            print("   PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/train_dense_reward.py")
            simulation_app.close()
            return
        
        print(f"✅ 최신 체크포인트 발견: {checkpoint_path.name}\n")
    else:
        checkpoint_path = Path(args.checkpoint)
        
        if not checkpoint_path.exists():
            print(f"❌ 체크포인트가 없습니다: {checkpoint_path}")
            simulation_app.close()
            return
    
    try:
        resume_training(str(checkpoint_path), args.timesteps)
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n🔚 Isaac Sim 종료 중...")
        simulation_app.close()


if __name__ == "__main__":
    main()
