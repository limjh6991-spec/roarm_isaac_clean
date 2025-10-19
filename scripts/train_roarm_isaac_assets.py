#!/usr/bin/env python3
"""
RoArm M3 Pick and Place PPO 학습 - Isaac Assets 버전
"""

import argparse
import sys
from pathlib import Path

# Isaac Sim 환경 설정
isaac_sim_path = Path.home() / "isaacsim"
sys.path.append(str(isaac_sim_path))

# Isaac Sim 초기화 (최우선)
from isaacsim import SimulationApp

def parse_args():
    parser = argparse.ArgumentParser(description="RoArm Isaac Assets RL Training")
    parser.add_argument("--mode", type=str, default="train", choices=["train", "test", "demo"])
    parser.add_argument("--level", type=str, default="easy", choices=["easy", "medium", "hard", "mixed"])
    parser.add_argument("--timesteps", type=int, default=50000)
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--render", type=str, default="true", choices=["true", "false"])
    parser.add_argument("--warehouse", type=str, default="false", choices=["true", "false"])
    parser.add_argument("--model", type=str, default=None, help="학습된 모델 경로 (test mode)")
    return parser.parse_args()

args = parse_args()

# SimulationApp 생성
simulation_app = SimulationApp({
    "headless": args.render == "false",
    "width": 1280,
    "height": 720,
})

# 이제 나머지 imports
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.vec_env import DummyVecEnv
import gymnasium as gym
from gymnasium import spaces

# 프로젝트 경로
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.append(str(PROJECT_ROOT))

# 환경 Import
from envs.roarm_pickplace_isaac_assets import RoArmPickPlaceIsaacEnv


class GymWrapper(gym.Env):
    """Isaac Sim 환경 → Gymnasium API"""
    
    def __init__(self, curriculum_level="easy", use_warehouse=False):
        super().__init__()
        
        # Isaac Sim 환경 생성
        self.env = RoArmPickPlaceIsaacEnv(
            curriculum_level=curriculum_level,
            use_warehouse=use_warehouse,
            render=args.render == "true"
        )
        
        # Gymnasium spaces
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(self.env.observation_dim,),
            dtype=np.float32
        )
        
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(self.env.action_dim,),
            dtype=np.float32
        )
    
    def reset(self, seed=None, options=None):
        if seed is not None:
            np.random.seed(seed)
        obs = self.env.reset()
        return obs, {}
    
    def step(self, action):
        obs, reward, done, info = self.env.step(action)
        truncated = False  # Gymnasium API
        return obs, reward, done, truncated, info
    
    def close(self):
        self.env.close()


def train_ppo(curriculum_level="easy", use_warehouse=False, total_timesteps=50000):
    """PPO 학습"""
    print(f"\n🎓 PPO 학습 시작 (Level: {curriculum_level})")
    print(f"   Total timesteps: {total_timesteps:,}")
    print(f"   Device: {'cuda' if torch.cuda.is_available() else 'cpu'}")
    
    # 환경 생성
    def make_env():
        return GymWrapper(
            curriculum_level=curriculum_level,
            use_warehouse=use_warehouse
        )
    
    env = DummyVecEnv([make_env])
    
    # 로그 디렉토리
    log_dir = PROJECT_ROOT / "logs" / "rl_training_isaac" / f"level_{curriculum_level}"
    log_dir.mkdir(parents=True, exist_ok=True)
    
    checkpoint_dir = log_dir / "checkpoints"
    checkpoint_dir.mkdir(exist_ok=True)
    
    tensorboard_dir = log_dir / "tensorboard"
    tensorboard_dir.mkdir(exist_ok=True)
    
    best_model_dir = log_dir / "best_model"
    best_model_dir.mkdir(exist_ok=True)
    
    # PPO 모델
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
        tensorboard_log=str(tensorboard_dir),
        device="cuda" if torch.cuda.is_available() else "cpu",
    )
    
    # Callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=5000,
        save_path=str(checkpoint_dir),
        name_prefix="roarm_ppo"
    )
    
    eval_callback = EvalCallback(
        env,
        best_model_save_path=str(best_model_dir),
        log_path=str(log_dir),
        eval_freq=2000,
        deterministic=True,
        render=False
    )
    
    # 학습 시작
    print(f"\n📊 TensorBoard: tensorboard --logdir {tensorboard_dir}")
    print(f"📁 Checkpoints: {checkpoint_dir}")
    print(f"🏆 Best model: {best_model_dir}")
    print("\n🚀 학습 시작...\n")
    
    model.learn(
        total_timesteps=total_timesteps,
        callback=[checkpoint_callback, eval_callback],
        progress_bar=True
    )
    
    # 최종 모델 저장
    final_model_path = log_dir / "final_model.zip"
    model.save(str(final_model_path))
    print(f"\n✅ 학습 완료! 최종 모델: {final_model_path}")
    
    env.close()


def test_trained_model(model_path=None, num_episodes=10, curriculum_level="easy"):
    """학습된 모델 테스트"""
    print(f"\n🧪 모델 테스트 (Level: {curriculum_level})")
    
    # 모델 경로 자동 찾기
    if model_path is None:
        best_model_path = PROJECT_ROOT / "logs" / "rl_training_isaac" / f"level_{curriculum_level}" / "best_model" / "best_model.zip"
        if best_model_path.exists():
            model_path = str(best_model_path)
            print(f"   Best model 사용: {model_path}")
        else:
            print(f"❌ 모델을 찾을 수 없습니다: {best_model_path}")
            return
    
    # 환경 생성
    env = GymWrapper(curriculum_level=curriculum_level, use_warehouse=False)
    
    # 모델 로드
    model = PPO.load(model_path)
    
    # 테스트
    success_count = 0
    total_rewards = []
    
    for ep in range(num_episodes):
        obs, _ = env.reset()
        done = False
        episode_reward = 0.0
        steps = 0
        
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, truncated, info = env.step(action)
            episode_reward += reward
            steps += 1
            
            if done or truncated:
                if info.get("success", False):
                    success_count += 1
                    print(f"   Episode {ep + 1}: ✅ SUCCESS (reward={episode_reward:.2f}, steps={steps})")
                else:
                    print(f"   Episode {ep + 1}: ❌ FAIL (reward={episode_reward:.2f}, reason={info.get('reason', 'unknown')})")
                total_rewards.append(episode_reward)
                break
    
    # 통계
    success_rate = (success_count / num_episodes) * 100
    avg_reward = np.mean(total_rewards)
    
    print(f"\n📊 테스트 결과:")
    print(f"   Success Rate: {success_rate:.1f}% ({success_count}/{num_episodes})")
    print(f"   Avg Reward: {avg_reward:.2f}")
    print(f"   Min/Max Reward: {np.min(total_rewards):.2f} / {np.max(total_rewards):.2f}")
    
    env.close()


def demo_environment(curriculum_level="easy", use_warehouse=False, num_episodes=3):
    """환경 데모 (랜덤 액션)"""
    print(f"\n🎬 환경 데모 (Level: {curriculum_level})")
    
    env = GymWrapper(curriculum_level=curriculum_level, use_warehouse=use_warehouse)
    
    for ep in range(num_episodes):
        print(f"\n📍 Episode {ep + 1}/{num_episodes}")
        obs, _ = env.reset()
        done = False
        episode_reward = 0.0
        steps = 0
        
        while not done and steps < 200:  # 최대 200 스텝
            # 랜덤 액션
            action = env.action_space.sample()
            
            # 주기적으로 그리퍼 열고 닫기
            if steps % 50 == 0:
                action[6:8] = 1.0 if (steps // 50) % 2 == 0 else -1.0
            
            obs, reward, done, truncated, info = env.step(action)
            episode_reward += reward
            steps += 1
            
            if steps % 50 == 0:
                print(f"   Step {steps}: Reward={reward:.2f}, Total={episode_reward:.2f}")
        
        if done:
            print(f"   Episode 종료: {info}")
        print(f"   Total Reward: {episode_reward:.2f}")
    
    env.close()
    print("\n✅ 데모 완료!")


def main():
    """메인 함수"""
    print("=" * 60)
    print("🤖 RoArm M3 Pick & Place - Isaac Assets RL")
    print("=" * 60)
    
    use_warehouse = args.warehouse == "true"
    
    try:
        if args.mode == "train":
            train_ppo(
                curriculum_level=args.level,
                use_warehouse=use_warehouse,
                total_timesteps=args.timesteps
            )
        
        elif args.mode == "test":
            test_trained_model(
                model_path=args.model,
                num_episodes=args.episodes,
                curriculum_level=args.level
            )
        
        elif args.mode == "demo":
            demo_environment(
                curriculum_level=args.level,
                use_warehouse=use_warehouse,
                num_episodes=args.episodes
            )
    
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        simulation_app.close()
        print("\n✅ 프로그램 종료")


if __name__ == "__main__":
    main()
