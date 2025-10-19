#!/usr/bin/env python3
"""
RoArm-M3 Dense Reward RL 학습 스크립트
개선된 보상 함수로 더 나은 학습 성능
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

print("✅ Isaac Sim 초기화 완료 (Dense Reward 환경)\n")

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
import gymnasium as gym
from gymnasium import spaces
import time
from datetime import datetime

# 환경 임포트
sys.path.append(str(Path(__file__).parent.parent))
from envs.roarm_pick_place_env import RoArmPickPlaceEnv, RoArmPickPlaceEnvCfg
from scripts.early_warning_callback import EarlyWarningCallback


class GymWrapper(gym.Env):
    """Isaac Sim 환경을 Gymnasium 형식으로 래핑"""
    
    def __init__(self):
        super().__init__()
        
        # 환경 생성
        print("🔧 환경 생성 중 (Shaped-Sparse + Curriculum)...")
        cfg = RoArmPickPlaceEnvCfg()
        cfg.episode_length_s = 10.0
        
        # ═══════════════════════════════════════════════════════════
        # 📚 CURRICULUM LEARNING 활성화
        # ═══════════════════════════════════════════════════════════
        cfg.curriculum_enabled = True
        cfg.curriculum_phase = 0  # Phase 0: Easy Mode
        
        self.env = RoArmPickPlaceEnv(cfg)
        
        # Observation space (25 dim now!)
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
        print("✅ Shaped-Sparse + Curriculum 환경 생성 완료\n")
    
    def reset(self, seed=None, options=None):
        if seed is not None:
            np.random.seed(seed)
        
        obs = self.env.reset()
        self.episode_count += 1
        
        # 주기적으로 진행 상황 출력
        if self.episode_count % 10 == 0:
            print(f"📊 Episode {self.episode_count} | Total steps: {self.total_steps}")
        
        return obs, {}
    
    def step(self, action):
        obs, reward, done, info = self.env.step(action)
        self.total_steps += 1
        
        # Gymnasium 형식으로 변환
        terminated = done
        truncated = False
        
        return obs, reward, terminated, truncated, info


def train(timesteps: int = 50000):
    """RL 학습 실행 (Curriculum + Shaped-Sparse)"""
    print("=" * 60)
    print("🚀 RoArm-M3 Curriculum + Shaped-Sparse RL 학습 시작")
    print("=" * 60)
    print(f"  목표 timesteps: {timesteps:,}")
    print(f"  학습 시작: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    
    # 환경 생성
    env = GymWrapper()
    
    # Monitor로 래핑 (학습 통계 기록)
    log_dir = Path("logs/rl_training_curriculum")  # Curriculum 로그
    log_dir.mkdir(parents=True, exist_ok=True)
    
    env = Monitor(env, str(log_dir / "monitor.monitor.csv"))
    
    # ✅ VecNormalize 적용
    print("🔧 VecNormalize 적용 중...")
    env = DummyVecEnv([lambda: env])
    env = VecNormalize(
        env,
        norm_obs=True,          # 관찰 정규화 ✅
        norm_reward=True,       # 보상 정규화 ✅
        clip_obs=10.0,          # 관찰 클립
        clip_reward=10.0,       # 리턴 클립 ✅
        gamma=0.99,
    )
    print(f"  ✅ Observation normalization: ON")
    print(f"  ✅ Reward normalization: ON")
    print(f"  ✅ Clip reward: ±10.0")
    print()
    
    print(f"📂 로그 디렉토리: {log_dir}\n")
    
    # 콜백 설정
    checkpoint_callback = CheckpointCallback(
        save_freq=5000,
        save_path=str(log_dir / "checkpoints"),
        name_prefix="roarm_ppo_curriculum",
        save_replay_buffer=False,
        save_vecnormalize=True,  # ✅ VecNormalize 통계 저장
    )
    
    # ✅ 조기 경보 시스템
    early_warning_callback = EarlyWarningCallback(
        ev_threshold=0.05,       # EV < 0.05
        ev_consecutive=5,        # 5회 연속
        vl_consecutive=5,        # VL 증가 5회 연속
        verbose=1,
    )
    print("✅ 조기 경보 시스템 활성화")
    print("   - EV < 0.05 연속 5회 → 자동 중단")
    print("   - Value Loss 증가 연속 5회 → 자동 중단")
    print()
    
    # ═══════════════════════════════════════════════════════════
    # 🎯 Curriculum + Shaped-Sparse 학습 설정
    # ═══════════════════════════════════════════════════════════
    # 1. Phase 0: Easy Mode (큐브 10~15cm, 타겟 20~25cm)
    # 2. Shaped-Sparse 보상 (게이팅 + 1회성 이벤트)
    # 3. 탐색 강화 (ent_coef=0.01)
    # 4. Value Clipping + target_kl 유지
    # ═══════════════════════════════════════════════════════════
    
    print("🤖 PPO 모델 생성 중 (Curriculum + Shaped-Sparse)...")
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3e-4,      # 기본값
        n_steps=2048,
        batch_size=64,
        n_epochs=10,             # 기본값
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        clip_range_vf=1.0,       # ✅ Value Clipping
        ent_coef=0.01,           # ✅ 탐색 강화 (Shaped-Sparse용)
        vf_coef=0.5,             # 기본값
        max_grad_norm=0.5,
        target_kl=0.03,          # ✅ 정책 안정성
        verbose=1,
        device="cuda" if torch.cuda.is_available() else "cpu",
        tensorboard_log=str(log_dir / "tensorboard"),
    )
    print(f"  Device: {model.device}")
    print(f"  Policy: MlpPolicy")
    print(f"  🔧 Learning Rate: 3e-4")
    print(f"  🔧 n_epochs: 10")
    print(f"  🔧 vf_coef: 0.5")
    print(f"  ✅ clip_range_vf: 1.0")
    print(f"  ✅ ent_coef: 0.01 (탐색 강화)")
    print(f"  ✅ target_kl: 0.03")
    print(f"  📚 Curriculum: Phase 0 (Easy Mode)")
    print(f"  🎯 Shaped-Sparse: 근접(+5) 그립(+10) 리프트(+15) 목표(+20) Success(+100)")
    print()
    
    # 학습 시작
    print("🎓 학습 시작...\n")
    start_time = time.time()
    
    try:
        model.learn(
            total_timesteps=timesteps,
            callback=[checkpoint_callback, early_warning_callback],  # ✅ 조기 경보 추가
            progress_bar=False,
        )
    except KeyboardInterrupt:
        print("\n\n⚠️ 학습 중단 (Ctrl+C)")
    
    # 학습 완료
    elapsed_time = time.time() - start_time
    print("\n" + "=" * 60)
    print("✅ 학습 완료!")
    print("=" * 60)
    print(f"  총 시간: {elapsed_time / 60:.1f}분")
    print(f"  종료 시각: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    # 최종 모델 저장
    final_model_dir = log_dir / "final_model"
    final_model_dir.mkdir(parents=True, exist_ok=True)
    model_path = final_model_dir / "roarm_ppo_dense_final.zip"
    model.save(str(model_path))
    print(f"\n💾 최종 모델 저장: {model_path}")
    
    return model


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="RoArm-M3 Dense Reward RL Training")
    parser.add_argument(
        "--timesteps",
        type=int,
        default=100000,
        help="총 학습 timesteps (기본: 100,000)"
    )
    
    args = parser.parse_args()
    
    try:
        train(args.timesteps)
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n🔚 Isaac Sim 종료 중...")
        simulation_app.close()


if __name__ == "__main__":
    main()
