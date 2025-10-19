#!/usr/bin/env python3
"""
RoArm-M3 Pick and Place 강화학습 학습 스크립트
Stable-Baselines3 PPO 알고리즘 사용
"""

import sys
from pathlib import Path

# Isaac Sim 환경 설정
isaac_sim_path = Path.home() / "isaacsim"
sys.path.append(str(isaac_sim_path))

from isaacsim import SimulationApp

# Isaac Sim 초기화 (headless mode for training)
simulation_app = SimulationApp({
    "headless": False,  # GUI로 확인하려면 False
    "width": 1280,
    "height": 720,
})

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor
import gymnasium as gym
from gymnasium import spaces

# 환경 임포트
sys.path.append(str(Path(__file__).parent.parent))
from envs.roarm_pick_place_env import RoArmPickPlaceEnv, RoArmPickPlaceEnvCfg


class GymWrapper(gym.Env):
    """
    Isaac Sim 환경을 Gymnasium 형식으로 래핑
    Stable-Baselines3와 호환
    """
    
    def __init__(self):
        super().__init__()
        
        # 환경 생성
        cfg = RoArmPickPlaceEnvCfg()
        cfg.episode_length_s = 10.0  # 10초 에피소드
        self.env = RoArmPickPlaceEnv(cfg)
        
        # Observation space (15 dim)
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(self.env.observation_space_dim,),
            dtype=np.float32
        )
        
        # Action space (8 dim, normalized [-1, 1])
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(self.env.action_space_dim,),
            dtype=np.float32
        )
    
    def reset(self, seed=None, options=None):
        """환경 리셋"""
        if seed is not None:
            np.random.seed(seed)
        
        obs = self.env.reset()
        return obs.astype(np.float32), {}
    
    def step(self, action):
        """환경 스텝"""
        obs, reward, done, info = self.env.step(action)
        
        # Gymnasium API: (obs, reward, terminated, truncated, info)
        terminated = done
        truncated = False
        
        return obs.astype(np.float32), reward, terminated, truncated, info
    
    def render(self):
        """렌더링 (Isaac Sim에서 자동)"""
        self.env.render()
    
    def close(self):
        """환경 종료"""
        self.env.close()


def make_env():
    """환경 생성 함수"""
    env = GymWrapper()
    env = Monitor(env)
    return env


def train_ppo(
    total_timesteps: int = 50000,
    save_dir: str = "logs/rl_training",
    checkpoint_freq: int = 5000,
    eval_freq: int = 2000,
):
    """
    PPO 알고리즘으로 학습
    
    Args:
        total_timesteps: 총 학습 스텝 수
        save_dir: 모델 저장 디렉토리
        checkpoint_freq: 체크포인트 저장 주기
        eval_freq: 평가 주기
    """
    print("=" * 60)
    print("🚀 RoArm-M3 Pick and Place PPO Training")
    print("=" * 60)
    print(f"  Total timesteps: {total_timesteps:,}")
    print(f"  Save directory: {save_dir}")
    
    # 디렉토리 생성
    save_path = Path(save_dir)
    save_path.mkdir(parents=True, exist_ok=True)
    
    # 환경 생성
    print(f"\n🔧 환경 생성 중...")
    env = DummyVecEnv([make_env])
    
    # 평가용 환경
    eval_env = DummyVecEnv([make_env])
    
    # PPO 모델 생성
    print(f"\n🧠 PPO 모델 생성 중...")
    model = PPO(
        policy="MlpPolicy",
        env=env,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        vf_coef=0.5,
        max_grad_norm=0.5,
        verbose=1,
        tensorboard_log=str(save_path / "tensorboard"),
        device="cuda" if torch.cuda.is_available() else "cpu",
    )
    
    print(f"  ✅ Device: {model.device}")
    print(f"  ✅ Policy network:")
    print(f"      - Observation space: {env.observation_space.shape}")
    print(f"      - Action space: {env.action_space.shape}")
    
    # Callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=checkpoint_freq,
        save_path=str(save_path / "checkpoints"),
        name_prefix="roarm_ppo",
        save_replay_buffer=False,
        save_vecnormalize=False,
    )
    
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=str(save_path / "best_model"),
        log_path=str(save_path / "eval_logs"),
        eval_freq=eval_freq,
        deterministic=True,
        render=False,
        n_eval_episodes=5,
    )
    
    # 학습 시작
    print(f"\n📚 학습 시작...")
    print(f"{'='*60}\n")
    
    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=[checkpoint_callback, eval_callback],
            progress_bar=True,
        )
        
        # 최종 모델 저장
        final_model_path = save_path / "roarm_ppo_final.zip"
        model.save(str(final_model_path))
        print(f"\n✅ 학습 완료!")
        print(f"  최종 모델 저장: {final_model_path}")
        
    except KeyboardInterrupt:
        print(f"\n⚠️ 사용자 중단 - 현재까지 학습된 모델 저장 중...")
        interrupted_model_path = save_path / "roarm_ppo_interrupted.zip"
        model.save(str(interrupted_model_path))
        print(f"  저장 완료: {interrupted_model_path}")
    
    finally:
        env.close()
        eval_env.close()
    
    return model


def test_trained_model(model_path: str, num_episodes: int = 5):
    """
    학습된 모델 테스트
    
    Args:
        model_path: 모델 파일 경로 (.zip)
        num_episodes: 테스트 에피소드 수
    """
    print("=" * 60)
    print("🎮 학습된 모델 테스트")
    print("=" * 60)
    print(f"  Model: {model_path}")
    print(f"  Episodes: {num_episodes}")
    
    # 환경 생성
    env = make_env()
    
    # 모델 로드
    print(f"\n🔧 모델 로딩 중...")
    model = PPO.load(model_path)
    print(f"  ✅ 로드 완료")
    
    # 테스트
    success_count = 0
    total_rewards = []
    
    for episode in range(num_episodes):
        print(f"\n{'='*60}")
        print(f"Episode {episode + 1}/{num_episodes}")
        print(f"{'='*60}")
        
        obs, _ = env.reset()
        done = False
        episode_reward = 0
        step = 0
        
        while not done:
            # 모델로 액션 예측
            action, _states = model.predict(obs, deterministic=True)
            
            obs, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            
            episode_reward += reward
            step += 1
            
            if step % 50 == 0:
                print(f"  Step {step}: Distance={info['distance_to_target']:.3f}m")
        
        total_rewards.append(episode_reward)
        
        # Success 판정
        if info['distance_to_target'] < 0.05:
            success_count += 1
            print(f"  ✅ SUCCESS!")
        else:
            print(f"  ❌ Failed")
        
        print(f"  📊 Episode reward: {episode_reward:.2f}")
    
    # 통계
    print(f"\n{'='*60}")
    print(f"📊 테스트 결과 요약")
    print(f"{'='*60}")
    print(f"  Success rate: {success_count}/{num_episodes} ({success_count/num_episodes*100:.1f}%)")
    print(f"  Average reward: {np.mean(total_rewards):.2f} ± {np.std(total_rewards):.2f}")
    print(f"  Min/Max reward: {np.min(total_rewards):.2f} / {np.max(total_rewards):.2f}")
    
    env.close()


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="RoArm-M3 Pick and Place RL Training")
    parser.add_argument("--mode", choices=["train", "test"], default="train",
                        help="학습(train) 또는 테스트(test)")
    parser.add_argument("--timesteps", type=int, default=50000,
                        help="총 학습 스텝 수")
    parser.add_argument("--model", type=str, default=None,
                        help="테스트할 모델 경로 (.zip)")
    parser.add_argument("--episodes", type=int, default=5,
                        help="테스트 에피소드 수")
    
    args = parser.parse_args()
    
    try:
        if args.mode == "train":
            # 학습
            model = train_ppo(total_timesteps=args.timesteps)
            
        elif args.mode == "test":
            # 테스트
            if args.model is None:
                # 최신 best 모델 찾기
                best_model_path = Path("logs/rl_training/best_model/best_model.zip")
                if best_model_path.exists():
                    args.model = str(best_model_path)
                else:
                    print("❌ 모델 경로를 지정하세요: --model <path>")
                    sys.exit(1)
            
            test_trained_model(args.model, num_episodes=args.episodes)
    
    finally:
        simulation_app.close()
        print("\n👋 Simulation 종료")
