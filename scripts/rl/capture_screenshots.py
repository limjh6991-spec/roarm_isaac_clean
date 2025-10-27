#!/usr/bin/env python3
"""
RoArm-M3 RL 학습 결과 - 스크린샷 캡처
GUI에서 여러 시점의 스크린샷을 저장합니다.
"""

import sys
from pathlib import Path

# stdout buffering 해결
sys.stdout = sys.stderr

from isaacsim import SimulationApp

# Isaac Sim 초기화 (GUI 모드)
simulation_app = SimulationApp({
    "headless": False,
    "width": 1280,
    "height": 720,
})

print("✅ Isaac Sim GUI 모드 초기화 완료\n")

import numpy as np
import torch
from stable_baselines3 import PPO
import gymnasium as gym
from gymnasium import spaces
from datetime import datetime
import time

# 환경 임포트
sys.path.append(str(Path(__file__).parent.parent))
from envs.roarm_pick_place_env import RoArmPickPlaceEnv, RoArmPickPlaceEnvCfg

# Isaac Sim 임포트
import omni
import carb


class GymWrapper(gym.Env):
    """Isaac Sim 환경을 Gymnasium 형식으로 래핑"""
    
    def __init__(self):
        super().__init__()
        
        print("🔧 환경 생성 중...")
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
    
    def reset(self, seed=None, options=None):
        if seed is not None:
            np.random.seed(seed)
        obs = self.env.reset()
        return obs, {}
    
    def step(self, action):
        obs, reward, done, info = self.env.step(action)
        # Isaac Sim 환경은 done만 반환, Gymnasium 형식으로 변환
        terminated = done
        truncated = False
        return obs, reward, terminated, truncated, info


def capture_screenshot(output_path: str):
    """스크린샷 캡처"""
    viewport_api = omni.kit.viewport.utility.get_active_viewport()
    if viewport_api:
        viewport_api.schedule_capture(output_path)
        print(f"  📸 캡처: {Path(output_path).name}")
        return True
    return False


def capture_episode(model_path: str, output_dir: str, num_screenshots: int = 10):
    """
    에피소드를 실행하면서 주요 시점의 스크린샷 캡처
    
    Args:
        model_path: 학습된 모델 경로
        output_dir: 스크린샷 저장 디렉토리
        num_screenshots: 캡처할 스크린샷 개수
    """
    print(f"📂 모델 로딩: {model_path}")
    
    # 환경 생성
    env = GymWrapper()
    
    # 모델 로드
    try:
        model = PPO.load(model_path, env=env)
        print("✅ 모델 로드 완료\n")
    except Exception as e:
        print(f"❌ 모델 로드 실패: {e}")
        return
    
    # 출력 디렉토리 생성
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    print(f"📸 스크린샷 캡처 설정...")
    print(f"  출력 경로: {output_path}")
    print(f"  캡처 개수: {num_screenshots}\n")
    
    print("🎬 에피소드 시작...")
    print("="*60)
    
    obs, _ = env.reset()
    episode_reward = 0
    episode_length = 0
    
    # 에피소드 길이
    max_steps = 600
    capture_interval = max_steps // num_screenshots
    
    # 시작 스크린샷
    time.sleep(0.5)  # GUI 렌더링 대기
    screenshot_path = output_path / f"screenshot_000_start.png"
    capture_screenshot(str(screenshot_path))
    
    captured_count = 1
    
    for step in range(max_steps):
        # 액션 예측
        action, _states = model.predict(obs, deterministic=True)
        
        # 스텝 실행
        obs, reward, terminated, truncated, info = env.step(action)
        episode_reward += reward
        episode_length += 1
        
        # 주기적으로 스크린샷 캡처
        if step % capture_interval == 0 and step > 0:
            time.sleep(0.1)  # 렌더링 대기
            screenshot_path = output_path / f"screenshot_{captured_count:03d}_step{step:04d}.png"
            capture_screenshot(str(screenshot_path))
            captured_count += 1
        
        # 진행 상황 출력
        if step % 100 == 0:
            print(f"  Step {step}/{max_steps} | Reward: {episode_reward:.2f}")
        
        if terminated or truncated:
            break
    
    # 종료 스크린샷
    time.sleep(0.5)
    screenshot_path = output_path / f"screenshot_{captured_count:03d}_end.png"
    capture_screenshot(str(screenshot_path))
    
    print("="*60)
    print(f"\n✅ 캡처 완료!")
    print(f"  총 스크린샷: {captured_count + 1}개")
    print(f"  에피소드 보상: {episode_reward:.2f}")
    print(f"  에피소드 길이: {episode_length} steps")
    print(f"\n📁 저장 위치: {output_path}")
    
    # 이미지 뷰어로 열기
    print(f"\n💡 스크린샷 확인:")
    print(f"  eog {output_path}/*.png")
    print(f"  # 또는")
    print(f"  xdg-open {output_path}")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="RoArm-M3 RL 스크린샷 캡처")
    parser.add_argument(
        "--model",
        type=str,
        default="logs/rl_training/final_model/roarm_ppo_final.zip",
        help="모델 경로 (기본: logs/rl_training/final_model/roarm_ppo_final.zip)"
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="스크린샷 저장 디렉토리 (기본: logs/screenshots/YYYYMMDD_HHMMSS)"
    )
    parser.add_argument(
        "--num-screenshots",
        type=int,
        default=10,
        help="캡처할 스크린샷 개수 (기본: 10)"
    )
    
    args = parser.parse_args()
    
    # 기본 출력 디렉토리
    if args.output_dir is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output_dir = f"logs/screenshots/roarm_{timestamp}"
    
    model_path = Path(args.model)
    if not model_path.exists():
        print(f"❌ 모델 파일이 없습니다: {model_path}")
        print("\n💡 사용 가능한 모델:")
        
        # 최종 모델 확인
        final_model = Path("logs/rl_training/final_model/roarm_ppo_final.zip")
        if final_model.exists():
            print(f"  - {final_model} (최종 모델)")
        
        # 체크포인트 확인
        checkpoints_dir = Path("logs/rl_training/checkpoints")
        if checkpoints_dir.exists():
            checkpoints = sorted(checkpoints_dir.glob("*.zip"), reverse=True)
            for ckpt in checkpoints[:5]:
                print(f"  - {ckpt}")
        
        simulation_app.close()
        return
    
    try:
        capture_episode(str(model_path), args.output_dir, args.num_screenshots)
    except KeyboardInterrupt:
        print("\n\n⚠️ 사용자에 의해 중단됨")
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n🔚 Isaac Sim 종료 중...")
        simulation_app.close()


if __name__ == "__main__":
    main()
