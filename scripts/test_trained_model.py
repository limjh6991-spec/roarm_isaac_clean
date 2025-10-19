#!/usr/bin/env python3
"""
학습된 RL 모델을 시각적으로 테스트하는 스크립트
Isaac Sim GUI 모드로 실행
"""

import sys
from pathlib import Path
import argparse

# stdout buffering 해결
sys.stdout = sys.stderr

from isaacsim import SimulationApp

# GUI 모드로 Isaac Sim 초기화
simulation_app = SimulationApp({
    "headless": False,  # GUI 활성화
    "width": 1280,
    "height": 720,
})

print("✅ Isaac Sim GUI 모드 초기화 완료\n")

import numpy as np
import torch
from stable_baselines3 import PPO
import gymnasium as gym
from gymnasium import spaces
import time

# 환경 임포트
sys.path.append(str(Path(__file__).parent.parent))
from envs.roarm_pick_place_env import RoArmPickPlaceEnv, RoArmPickPlaceEnvCfg


class GymWrapper(gym.Env):
    """Isaac Sim 환경을 Gymnasium 형식으로 래핑"""
    
    def __init__(self):
        super().__init__()
        
        # 환경 생성
        print("🔧 환경 생성 중 (Curriculum 환경)...")
        cfg = RoArmPickPlaceEnvCfg()
        cfg.episode_length_s = 10.0
        
        # Curriculum 설정 (Phase 0)
        cfg.curriculum_enabled = True
        cfg.curriculum_phase = 0  # Easy Mode로 테스트
        
        self.env = RoArmPickPlaceEnv(cfg)
        
        # Observation space (25 dim - Shaped-Sparse)
        obs_dim = 25
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )
        
        # Action space (8 dim: 6 joints + 2 gripper)
        action_dim = 8
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(action_dim,), dtype=np.float32
        )
        
        print(f"  Observation space: {self.observation_space.shape}")
        print(f"  Action space: {self.action_space.shape}\n")
        
        self.episode_count = 0
    
    def reset(self, seed=None, options=None):
        """환경 리셋"""
        super().reset(seed=seed)
        obs = self.env.reset()
        self.episode_count += 1
        return obs.astype(np.float32), {}
    
    def step(self, action):
        """한 스텝 실행"""
        obs, reward, done, info = self.env.step(action)
        # Gymnasium 형식으로 변환: done을 terminated와 truncated로 분리
        terminated = done
        truncated = False
        return obs.astype(np.float32), float(reward), terminated, truncated, info
    
    def close(self):
        """환경 종료"""
        pass


def test_model(model_path, episodes=5, render_delay=0.05):
    """학습된 모델 테스트"""
    
    print(f"📂 모델 로딩: {model_path}")
    
    # 환경 생성
    env = GymWrapper()
    
    # 모델 로드
    model = PPO.load(model_path)
    print("✅ 모델 로드 완료\n")
    
    # 통계
    episode_rewards = []
    episode_lengths = []
    success_count = 0
    
    print(f"🎬 {episodes}개 에피소드 테스트 시작...\n")
    print("="*60)
    
    for episode in range(episodes):
        obs, _ = env.reset()
        episode_reward = 0
        episode_length = 0
        done = False
        
        print(f"\n📊 Episode {episode + 1}/{episodes}")
        print("-" * 60)
        
        while not done:
            # 모델로 행동 예측
            action, _states = model.predict(obs, deterministic=True)
            
            # 환경 스텝
            obs, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            
            episode_reward += reward
            episode_length += 1
            
            # GUI 업데이트를 위한 짧은 대기
            time.sleep(render_delay)
        
        # 에피소드 통계
        episode_rewards.append(episode_reward)
        episode_lengths.append(episode_length)
        
        # 성공 여부 (보상 > -1000이면 성공으로 간주)
        if episode_reward > -1000:
            success_count += 1
            success_mark = "✅"
        else:
            success_mark = "❌"
        
        print(f"\n{success_mark} 에피소드 완료:")
        print(f"  보상: {episode_reward:.2f}")
        print(f"  길이: {episode_length} steps")
        print("=" * 60)
    
    # 최종 통계
    print("\n" + "="*60)
    print("📊 테스트 결과 요약")
    print("="*60)
    print(f"총 에피소드: {episodes}")
    print(f"성공: {success_count}/{episodes} ({success_count/episodes*100:.1f}%)")
    print(f"\n보상 통계:")
    print(f"  평균: {np.mean(episode_rewards):.2f}")
    print(f"  최고: {np.max(episode_rewards):.2f}")
    print(f"  최저: {np.min(episode_rewards):.2f}")
    print(f"  표준편차: {np.std(episode_rewards):.2f}")
    print(f"\n길이 통계:")
    print(f"  평균: {np.mean(episode_lengths):.1f} steps")
    print(f"  최장: {np.max(episode_lengths)} steps")
    print(f"  최단: {np.min(episode_lengths)} steps")
    print("="*60)
    
    env.close()


def main():
    parser = argparse.ArgumentParser(description="학습된 RL 모델 테스트 (GUI 모드)")
    parser.add_argument(
        "--model",
        type=str,
        default="logs/rl_training/final_model/roarm_ppo_final.zip",
        help="모델 경로 (기본: 최종 모델)"
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=5,
        help="테스트할 에피소드 수 (기본: 5)"
    )
    parser.add_argument(
        "--render-delay",
        type=float,
        default=0.05,
        help="렌더링 딜레이 (초, 기본: 0.05)"
    )
    
    args = parser.parse_args()
    
    # 모델 경로 확인
    model_path = Path(args.model)
    if not model_path.exists():
        print(f"❌ 모델 파일을 찾을 수 없습니다: {model_path}")
        print("\n사용 가능한 모델:")
        
        # 체크포인트 확인
        checkpoint_dir = Path("logs/rl_training/checkpoints")
        if checkpoint_dir.exists():
            checkpoints = sorted(checkpoint_dir.glob("*.zip"))
            for cp in checkpoints:
                print(f"  - {cp}")
        
        simulation_app.close()
        sys.exit(1)
    
    try:
        test_model(str(model_path), args.episodes, args.render_delay)
    except KeyboardInterrupt:
        print("\n\n⏸️  사용자에 의해 중단되었습니다.")
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n🔚 Isaac Sim 종료 중...")
        simulation_app.close()
        print("✅ 종료 완료")


if __name__ == "__main__":
    main()
