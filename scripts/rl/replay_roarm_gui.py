#!/usr/bin/env python3
"""
GUI 리플레이 스크립트 - 학습된 PPO 모델을 Isaac Sim GUI에서 재생
"""
from isaacsim import SimulationApp
# 🔸 GUI 모드로 Isaac Sim을 "가장 먼저" 초기화해야 얼지 않음
simulation_app = SimulationApp({"headless": False, "width": 1280, "height": 800})

import argparse
import numpy as np
import sys
import os
from pathlib import Path

# 프로젝트 루트를 PYTHONPATH에 추가
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))
sys.path.insert(0, str(project_root / "envs"))

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
from stable_baselines3.common.monitor import Monitor
import gymnasium as gym
from gymnasium import spaces

# 환경 import
from roarm_pick_place_env import RoArmPickPlaceEnv, RoArmPickPlaceEnvCfg

class GymWrapper(gym.Env):
    """Isaac Sim 환경을 Gymnasium 형식으로 래핑"""
    def __init__(self):
        super().__init__()
        cfg = RoArmPickPlaceEnvCfg()
        cfg.episode_length_s = 10.0
        self.env = RoArmPickPlaceEnv(cfg)
        
        obs_dim = 28  # RoArm-M3 observation dimension
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )
        
        # action: [delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw, gripper_cmd, gripper_scalar]
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(8,), dtype=np.float32
        )
    
    def reset(self, seed=None, options=None):
        if seed is not None:
            np.random.seed(seed)
        obs_dict_or_array = self.env.reset()
        
        # Handle both dict and array returns
        if isinstance(obs_dict_or_array, dict):
            obs = obs_dict_or_array["policy"]
        else:
            obs = obs_dict_or_array  # Already numpy array
        
        return obs, {}
    
    def step(self, action):
        result = self.env.step(action)
        
        # Handle both (obs_dict, reward, done, info) and (obs, reward, done, info)
        if isinstance(result[0], dict):
            obs = result[0]["policy"]
        else:
            obs = result[0]
        
        reward = result[1]
        done = result[2]
        info = result[3] if len(result) > 3 else {}
        
        terminated = done
        truncated = False
        return obs, reward, terminated, truncated, info
    
    def render(self):
        pass
    
    def close(self):
        pass

def make_env():
    """환경 생성"""
    return Monitor(GymWrapper())

def main():
    parser = argparse.ArgumentParser(description="학습된 PPO 모델 GUI 리플레이")
    parser.add_argument("--model", required=True, help="학습된 PPO 모델(.zip)")
    parser.add_argument("--vecnorm", default=None, help="VecNormalize 통계(.pkl) 경로 (있다면 필수로 로드)")
    parser.add_argument("--episodes", type=int, default=5, help="재생할 에피소드 수")
    parser.add_argument("--det", action="store_true", default=True, help="deterministic 실행 (기본: True)")
    parser.add_argument("--render-delay", type=float, default=0.01, help="렌더링 딜레이(초)")
    args = parser.parse_args()

    print(f"\n{'='*60}")
    print("🎬 RoArm-M3 PPO 모델 GUI 리플레이")
    print(f"{'='*60}")
    print(f"  모델: {args.model}")
    print(f"  VecNormalize: {args.vecnorm if args.vecnorm else 'None'}")
    print(f"  에피소드: {args.episodes}")
    print(f"  Deterministic: {args.det}")
    print(f"{'='*60}\n")

    # 1) 벡터 환경 + (선택) VecNormalize 통계 로드
    print("📦 환경 생성 중...")
    venv = DummyVecEnv([make_env])
    
    if args.vecnorm:
        if os.path.exists(args.vecnorm):
            print(f"✅ VecNormalize 통계 로드: {args.vecnorm}")
            venv = VecNormalize.load(args.vecnorm, venv)
            venv.training = False
            venv.norm_reward = False
        else:
            print(f"⚠️  VecNormalize 파일 없음: {args.vecnorm}")
    else:
        print("⚠️  VecNormalize 통계 없이 실행 (정책이 부정확할 수 있음)")

    # 2) 모델 로드
    print(f"📂 모델 로딩: {args.model}")
    model = PPO.load(args.model, device="auto")
    print("✅ 모델 로드 완료\n")

    print(f"🎬 {args.episodes}개 에피소드 재생 시작...\n")
    print(f"{'='*60}\n")

    obs = venv.reset()
    done_count = 0
    step_count = 0
    episode_rewards = []
    current_episode_reward = 0

    # 3) GUI 이벤트 루프를 SimulationApp로 돌려야 "기다리기"가 안 뜸
    while simulation_app.is_running() and done_count < args.episodes:
        action, _ = model.predict(obs, deterministic=args.det)
        obs, rewards, dones, infos = venv.step(action)
        
        current_episode_reward += rewards[0]
        step_count += 1

        # 🔸 중요: 프레임마다 GUI 이벤트/렌더를 펌프
        simulation_app.update()
        
        # 렌더링 딜레이 (너무 빠르면 확인 어려움)
        if args.render_delay > 0:
            import time
            time.sleep(args.render_delay)

        if dones[0]:
            done_count += 1
            episode_rewards.append(current_episode_reward)
            
            print(f"✅ 에피소드 {done_count}/{args.episodes} 완료")
            print(f"   보상: {current_episode_reward:.2f}")
            print(f"   스텝: {step_count}")
            
            # milestone 정보 출력
            if infos and len(infos) > 0 and 'episode' in infos[0]:
                ep_info = infos[0]['episode']
                if 'reach_rate' in ep_info:
                    print(f"   REACH: {ep_info.get('reach_rate', 0):.1%}")
                if 'grip_rate' in ep_info:
                    print(f"   GRIP: {ep_info.get('grip_rate', 0):.1%}")
                if 'lift_rate' in ep_info:
                    print(f"   LIFT: {ep_info.get('lift_rate', 0):.1%}")
            print()
            
            current_episode_reward = 0
            step_count = 0
            obs = venv.reset()

    print(f"\n{'='*60}")
    print("🏁 재생 완료!")
    print(f"{'='*60}")
    if episode_rewards:
        print(f"  평균 보상: {np.mean(episode_rewards):.2f}")
        print(f"  최대 보상: {np.max(episode_rewards):.2f}")
        print(f"  최소 보상: {np.min(episode_rewards):.2f}")
    print(f"{'='*60}\n")

    simulation_app.close()

if __name__ == "__main__":
    main()
