#!/usr/bin/env python3
"""
학습된 RL 모델을 비디오로 녹화하는 스크립트
Isaac Sim의 Replicator를 사용하여 MP4 저장
"""

import sys
from pathlib import Path
import argparse

# stdout buffering 해결
sys.stdout = sys.stderr

from isaacsim import SimulationApp

# Headless 모드로 초기화 (비디오 녹화용)
simulation_app = SimulationApp({
    "headless": False,  # 렌더링 필요
    "width": 1280,
    "height": 720,
})

print("✅ Isaac Sim 초기화 완료\n")

import numpy as np
import torch
from stable_baselines3 import PPO
import gymnasium as gym
from gymnasium import spaces
import time
import omni.replicator.core as rep
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
        
        # Observation space (15 dim)
        obs_dim = 15
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
    
    def reset(self, seed=None, options=None):
        """환경 리셋"""
        super().reset(seed=seed)
        obs = self.env.reset()
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


def record_video(model_path, output_path, episodes=3, fps=30):
    """학습된 모델의 비디오 녹화"""
    
    print(f"📂 모델 로딩: {model_path}")
    
    # 환경 생성
    env = GymWrapper()
    
    # 모델 로드
    model = PPO.load(model_path)
    print("✅ 모델 로드 완료\n")
    
    # 비디오 녹화 설정
    print(f"🎥 비디오 녹화 설정...")
    print(f"  출력 경로: {output_path}")
    print(f"  FPS: {fps}")
    print(f"  에피소드: {episodes}\n")
    
    # Replicator로 카메라 및 녹화 설정
    camera = rep.create.camera(
        position=(0.5, 0.5, 0.5),
        look_at=(0, 0, 0)
    )
    render_product = rep.create.render_product(camera, (1280, 720))
    
    # 비디오 writer 설정
    writer = rep.WriterRegistry.get("BasicWriter")
    output_dir = Path(output_path).parent
    output_dir.mkdir(parents=True, exist_ok=True)
    
    writer.initialize(
        output_dir=str(output_dir),
        rgb=True
    )
    writer.attach([render_product])
    
    print("🎬 녹화 시작...\n")
    print("="*60)
    
    total_frames = 0
    
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
            total_frames += 1
            
            # 프레임 캡처
            rep.orchestrator.step()
            
            # 너무 빠르면 느리게
            time.sleep(1.0 / fps)
        
        print(f"\n에피소드 완료:")
        print(f"  보상: {episode_reward:.2f}")
        print(f"  길이: {episode_length} steps")
        print("=" * 60)
    
    print(f"\n✅ 녹화 완료!")
    print(f"  총 프레임: {total_frames}")
    print(f"  영상 길이: {total_frames / fps:.1f}초")
    print(f"  저장 위치: {output_dir}")
    
    env.close()


def main():
    parser = argparse.ArgumentParser(description="학습된 RL 모델 비디오 녹화")
    parser.add_argument(
        "--model",
        type=str,
        default="logs/rl_training/final_model/roarm_ppo_final.zip",
        help="모델 경로 (기본: 최종 모델)"
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="출력 비디오 경로 (기본: logs/videos/roarm_YYYYMMDD_HHMMSS.mp4)"
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=3,
        help="녹화할 에피소드 수 (기본: 3)"
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="비디오 FPS (기본: 30)"
    )
    
    args = parser.parse_args()
    
    # 출력 경로 설정
    if args.output is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = f"logs/videos/roarm_{timestamp}.mp4"
    else:
        output_path = args.output
    
    # 모델 경로 확인
    model_path = Path(args.model)
    if not model_path.exists():
        print(f"❌ 모델 파일을 찾을 수 없습니다: {model_path}")
        simulation_app.close()
        sys.exit(1)
    
    try:
        record_video(str(model_path), output_path, args.episodes, args.fps)
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
