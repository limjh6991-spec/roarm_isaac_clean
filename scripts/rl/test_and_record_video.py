#!/usr/bin/env python3
"""
학습된 모델 테스트 및 동영상 녹화
"""

import sys
from pathlib import Path
import numpy as np
import time

# stdout buffering 해결
sys.stdout = sys.stderr

from isaacsim import SimulationApp

# Isaac Sim 초기화 (GUI 모드 + 녹화)
simulation_app = SimulationApp({
    "headless": False,  # GUI 모드
    "width": 1920,
    "height": 1080,
})

print("✅ Isaac Sim 초기화 완료 (Video Recording Mode)\n")

import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
import gymnasium as gym
from gymnasium import spaces

# 환경 임포트
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from envs.roarm_pick_place_env import RoArmPickPlaceEnv, RoArmPickPlaceEnvCfg

# 동영상 녹화 임포트
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.kit.window.movie_capture")
import omni.kit.app


class GymWrapper(gym.Env):
    """Isaac Sim 환경을 Gymnasium 형식으로 래핑"""
    
    def __init__(self):
        super().__init__()
        
        # 환경 생성
        print("🔧 환경 생성 중...")
        cfg = RoArmPickPlaceEnvCfg()
        cfg.episode_length_s = 10.0
        
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


def test_and_record(model_path: str, vecnorm_path: str, num_episodes: int = 5, output_path: str = "logs/demo_video.mp4"):
    """학습된 모델 테스트 및 동영상 녹화"""
    print("=" * 80)
    print("🎬 학습된 모델 테스트 및 동영상 녹화")
    print("=" * 80)
    print(f"  모델 경로: {model_path}")
    print(f"  VecNormalize: {vecnorm_path}")
    print(f"  에피소드 수: {num_episodes}")
    print(f"  출력 경로: {output_path}")
    print()
    
    # 환경 생성
    def make_env():
        env = GymWrapper()
        return env
    
    env = DummyVecEnv([make_env])
    
    # VecNormalize 로드 (학습 시와 동일한 정규화!)
    print("🔧 VecNormalize 로드 중...")
    env = VecNormalize.load(vecnorm_path, env)
    env.training = False  # 테스트 모드
    env.norm_reward = False  # 보상 정규화 해제
    print("✅ VecNormalize 로드 완료\n")
    
    # 모델 로드
    print("🤖 PPO 모델 로드 중...")
    model = PPO.load(model_path, env=env)
    print(f"✅ 모델 로드 완료 (device: {model.device})\n")
    
    # 동영상 녹화 시작
    print("🎥 동영상 녹화 시작...\n")
    
    # 통계
    episode_rewards = []
    episode_lengths = []
    success_count = 0
    reach_count = 0
    grip_count = 0
    
    # 에피소드 실행
    for episode in range(num_episodes):
        obs = env.reset()
        episode_reward = 0
        episode_length = 0
        done = False
        
        print(f"{'='*80}")
        print(f"🎮 Episode {episode + 1}/{num_episodes}")
        print(f"{'='*80}")
        
        while not done:
            # 모델 추론
            action, _states = model.predict(obs, deterministic=True)
            
            # 환경 스텝
            obs, reward, terminated, truncated, info = env.step(action)
            done = terminated[0] or truncated[0]
            
            episode_reward += reward[0]
            episode_length += 1
            
            # 프레임 렌더링 (동영상 녹화)
            simulation_app.update()
            
            # 정보 출력 (일부 스텝)
            if episode_length % 100 == 0:
                print(f"  Step {episode_length}: reward={reward[0]:.3f}")
        
        # 에피소드 통계
        episode_rewards.append(episode_reward)
        episode_lengths.append(episode_length)
        
        # info에서 성공 여부 확인
        env_info = info[0] if isinstance(info, list) else info
        if env_info.get("is_success", False):
            success_count += 1
        if env_info.get("reached_near_cube", False):
            reach_count += 1
        if env_info.get("reached_grasp", False):
            grip_count += 1
        
        print(f"\n  ✅ Episode {episode + 1} 완료:")
        print(f"     Total Reward: {episode_reward:.2f}")
        print(f"     Length: {episode_length} steps")
        print(f"     Success: {'✅' if env_info.get('is_success', False) else '❌'}")
        print(f"     REACH: {'✅' if env_info.get('reached_near_cube', False) else '❌'}")
        print(f"     GRIP: {'✅' if env_info.get('reached_grasp', False) else '❌'}")
        print()
        
        # 에피소드 간 대기
        time.sleep(1)
    
    # 최종 통계
    print("\n" + "=" * 80)
    print("📊 최종 테스트 결과")
    print("=" * 80)
    print(f"  에피소드 수: {num_episodes}")
    print(f"  평균 보상: {np.mean(episode_rewards):.2f} ± {np.std(episode_rewards):.2f}")
    print(f"  평균 길이: {np.mean(episode_lengths):.0f} steps")
    print(f"  성공률: {success_count}/{num_episodes} ({success_count/num_episodes*100:.1f}%)")
    print(f"  REACH 달성: {reach_count}/{num_episodes} ({reach_count/num_episodes*100:.1f}%)")
    print(f"  GRIP 달성: {grip_count}/{num_episodes} ({grip_count/num_episodes*100:.1f}%)")
    print("=" * 80)
    
    print(f"\n💾 동영상 저장: {output_path}")
    print("   (Isaac Sim 내장 Movie Capture 사용)")
    
    return {
        "mean_reward": np.mean(episode_rewards),
        "std_reward": np.std(episode_rewards),
        "mean_length": np.mean(episode_lengths),
        "success_rate": success_count / num_episodes,
        "reach_rate": reach_count / num_episodes,
        "grip_rate": grip_count / num_episodes,
    }


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Test trained model and record video")
    parser.add_argument(
        "--model",
        type=str,
        default="logs/rl_training_curriculum/final_model/roarm_ppo_dense_final.zip",
        help="모델 경로"
    )
    parser.add_argument(
        "--vecnorm",
        type=str,
        default="logs/rl_training_curriculum/final_model/vecnormalize.pkl",
        help="VecNormalize 경로"
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=5,
        help="테스트 에피소드 수 (기본: 5)"
    )
    parser.add_argument(
        "--output",
        type=str,
        default="logs/demo_video.mp4",
        help="출력 동영상 경로"
    )
    
    args = parser.parse_args()
    
    # 경로 확인
    model_path = Path(args.model)
    vecnorm_path = Path(args.vecnorm)
    
    if not model_path.exists():
        print(f"❌ 모델 파일을 찾을 수 없습니다: {model_path}")
        print("\n사용 가능한 모델:")
        checkpoints_dir = Path("logs/rl_training_curriculum/checkpoints")
        if checkpoints_dir.exists():
            for ckpt in sorted(checkpoints_dir.glob("*.zip")):
                print(f"  - {ckpt}")
        simulation_app.close()
        return
    
    if not vecnorm_path.exists():
        print(f"❌ VecNormalize 파일을 찾을 수 없습니다: {vecnorm_path}")
        simulation_app.close()
        return
    
    try:
        results = test_and_record(
            model_path=str(model_path),
            vecnorm_path=str(vecnorm_path),
            num_episodes=args.episodes,
            output_path=args.output,
        )
        
        print("\n✅ 테스트 및 녹화 완료!")
        
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n🔚 Isaac Sim 종료 중...")
        simulation_app.close()


if __name__ == "__main__":
    main()
