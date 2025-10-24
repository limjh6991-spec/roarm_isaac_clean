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
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))
print(f"📁 Project root: {project_root}")

from envs.roarm_pick_place_env import RoArmPickPlaceEnv, RoArmPickPlaceEnvCfg

# Early warning callback (옵션)
try:
    from scripts.rl.early_warning_callback import EarlyWarningCallback
    EARLY_WARNING_AVAILABLE = True
except ImportError:
    print("⚠️ EarlyWarningCallback not found, skipping")
    EarlyWarningCallback = None
    EARLY_WARNING_AVAILABLE = False

# Training progress callback
try:
    from scripts.rl.training_callbacks import TrainingProgressCallback, CurriculumCallback
    TRAINING_CALLBACKS_AVAILABLE = True
except ImportError:
    print("⚠️ Training callbacks not found, using basic logging")
    TrainingProgressCallback = None
    CurriculumCallback = None
    TRAINING_CALLBACKS_AVAILABLE = False


class GymWrapper(gym.Env):
    """Isaac Sim 환경을 Gymnasium 형식으로 래핑
    
    ✅ TimeLimit 명시적 설정 (max_episode_steps)
    """
    
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
        
        # ═══════════════════════════════════════════════════════════
        # ✅ TimeLimit 명시적 설정 (에피소드 길이 보장!)
        # ═══════════════════════════════════════════════════════════
        # Isaac Sim 환경의 max_steps를 Gymnasium TimeLimit으로 전달
        self._max_episode_steps = self.env.max_steps
        print(f"  ✅ Max episode steps: {self._max_episode_steps} (from env.max_steps)")
        
        # Observation space (28 dim now!)
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
        
        # 에피소드 추적
        self.episode_count = 0
        self.total_steps = 0
        self._elapsed_steps = 0  # 현재 에피소드 스텝 수
        print("✅ Shaped-Sparse + Curriculum 환경 생성 완료\n")
    
    def reset(self, seed=None, options=None):
        if seed is not None:
            # ═══════════════════════════════════════════════════════════
            # ✅ 시드 전파 (재현성 강화!)
            # ═══════════════════════════════════════════════════════════
            np.random.seed(seed)
            torch.manual_seed(seed)
            if torch.cuda.is_available():
                torch.cuda.manual_seed(seed)
        
        obs = self.env.reset()
        self.episode_count += 1
        self._elapsed_steps = 0  # 에피소드 스텝 카운터 리셋
        
        # 주기적으로 진행 상황 출력 (10 → 50 에피소드마다, 스팸 줄이기!)
        if self.episode_count % 50 == 0:
            print(f"📊 Episode {self.episode_count} | Total steps: {self.total_steps}")
        
        return obs, {}
    
    def step(self, action):
        obs, reward, done, info = self.env.step(action)
        self.total_steps += 1
        self._elapsed_steps += 1
        
        # ═══════════════════════════════════════════════════════════
        # ✅ TimeLimit 명시적 체크 (truncation 분리)
        # ═══════════════════════════════════════════════════════════
        # 환경 내부 done: SUCCESS, 안전 위반 등 → terminated
        # TimeLimit 도달: max_episode_steps → truncated
        terminated = done and self._elapsed_steps < self._max_episode_steps
        truncated = self._elapsed_steps >= self._max_episode_steps
        
        # ✅ TimeLimit truncation을 info에 명시 (SB3/Monitor가 인식)
        if truncated:
            info["TimeLimit.truncated"] = True
            print(f"  ⏱️ TimeLimit reached: {self._elapsed_steps}/{self._max_episode_steps} steps")
        
        return obs, reward, terminated, truncated, info


def train(timesteps: int = 200000):
    """RL 학습 실행 (Curriculum + Shaped-Sparse)"""
    print("=" * 60)
    print("🚀 RoArm-M3 Curriculum + Shaped-Sparse RL 학습 시작")
    print("=" * 60)
    print(f"  목표 timesteps: {timesteps:,}")
    print(f"  학습 시작: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    
    # 로그 디렉토리 생성
    log_dir = Path("logs/rl_training_curriculum")  # Curriculum 로그
    log_dir.mkdir(parents=True, exist_ok=True)
    
    # ═══════════════════════════════════════════════════════════
    # ✅ SB3 권장 패턴: DummyVecEnv([lambda: Monitor(env)])
    # ═══════════════════════════════════════════════════════════
    def make_env():
        """환경 생성 팩토리 (Monitor 포함)"""
        env = GymWrapper()
        env = Monitor(env, str(log_dir / "monitor.monitor.csv"))
        return env
    
    print("🔧 환경 생성 중...")
    env = DummyVecEnv([make_env])
    
    # ✅ VecNormalize 적용
    print("🔧 VecNormalize 적용 중...")
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
    callbacks = []
    
    # 1. 학습 진행 상황 로깅 (NEW!)
    if TRAINING_CALLBACKS_AVAILABLE:
        progress_callback = TrainingProgressCallback(
            verbose=1,
            log_freq=10,  # 10 에피소드마다 출력
        )
        callbacks.append(progress_callback)
        print("✅ 학습 진행 상황 로깅 활성화 (10 에피소드마다)")
    
    # 2. Curriculum 자동 승급 (NEW!)
    if TRAINING_CALLBACKS_AVAILABLE:
        # env_wrapper는 DummyVecEnv의 첫 번째 환경
        env_wrapper = env.envs[0]
        curriculum_callback = CurriculumCallback(
            env_wrapper=env_wrapper,
            success_window=100,
            phase_0_threshold=0.30,  # 30% 성공률로 Phase 1
            phase_1_threshold=0.60,  # 60% 성공률로 Phase 2
            verbose=1,
        )
        callbacks.append(curriculum_callback)
        print("✅ Curriculum 자동 승급 활성화")
        print("   - Phase 0→1: 성공률 30% (100 에피소드)")
        print("   - Phase 1→2: 성공률 60% (100 에피소드)")
    
    # 3. 체크포인트 콜백
    checkpoint_callback = CheckpointCallback(
        save_freq=5000,
        save_path=str(log_dir / "checkpoints"),
        name_prefix="roarm_ppo_curriculum",
        save_replay_buffer=False,
        save_vecnormalize=True,  # ✅ VecNormalize 통계 저장
    )
    callbacks.append(checkpoint_callback)
    
    # 4. 조기 경보 시스템 (사용 가능한 경우만)
    if EARLY_WARNING_AVAILABLE:
        early_warning_callback = EarlyWarningCallback(
            ev_threshold=0.05,       # EV < 0.05
            ev_consecutive=5,        # 5회 연속
            vl_consecutive=5,        # VL 증가 5회 연속
            verbose=1,
        )
        callbacks.append(early_warning_callback)
        print("✅ 조기 경보 시스템 활성화")
        print("   - EV < 0.05 연속 5회 → 자동 중단")
        print("   - Value Loss 증가 연속 5회 → 자동 중단")
    else:
        print("⚠️ 조기 경보 시스템 비활성화 (모듈 없음)")
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
            callback=callbacks,  # ✅ 조건부 콜백 리스트
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
    
    # ═══════════════════════════════════════════════════════════
    # ✅ 최종 모델 + VecNormalize 통계 저장
    # ═══════════════════════════════════════════════════════════
    final_model_dir = log_dir / "final_model"
    final_model_dir.mkdir(parents=True, exist_ok=True)
    
    # 1. PPO 모델 저장
    model_path = final_model_dir / "roarm_ppo_dense_final.zip"
    model.save(str(model_path))
    print(f"\n💾 최종 모델 저장: {model_path}")
    
    # 2. VecNormalize 통계 저장 (재현 필수!)
    vecnorm_path = final_model_dir / "vecnormalize.pkl"
    env.save(str(vecnorm_path))
    print(f"💾 VecNormalize 통계 저장: {vecnorm_path}")
    print("   ⚠️ 테스트 시 반드시 함께 로드하세요!")
    
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
