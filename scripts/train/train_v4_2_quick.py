#!/usr/bin/env python3
"""
🚀 V4.2 Quick Training Script
- 6 DOF STL Mesh URDF
- 100K steps (빠른 테스트)
- Curriculum: Easy only
"""

import sys
import argparse
# 🔧 FIX: stdout 리다이렉트 제거 (Isaac Sim 로깅 충돌 방지)
sys.stdout = sys.__stdout__
sys.stderr = sys.__stderr__

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

print("="*80, flush=True)
print("🚀 V4.2 Quick Training - SimulationApp 초기화 완료!", flush=True)
print("="*80, flush=True)

import os
import numpy as np
from datetime import datetime

from envs.roarm_pick_place_env import RoArmPickPlaceEnv
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, CallbackList, BaseCallback

# 간단한 Milestone 콜백
class SimpleMilestoneCallback(BaseCallback):
    def __init__(self, verbose=0):
        super().__init__(verbose)
        self.episode_count = 0
        
    def _on_step(self) -> bool:
        if self.locals.get('dones', [False])[0]:
            self.episode_count += 1
            info = self.locals.get('infos', [{}])[0]
            
            if self.episode_count % 10 == 0:
                print(f"\n📊 Episode {self.episode_count}")
                if 'milestone_counts' in info:
                    counts = info['milestone_counts']
                    print(f"  REACH: {counts.get('reach', 0)}")
                    print(f"  ATTACH: {counts.get('grip', 0)}")
                    print(f"  LIFT: {counts.get('lift', 0)}")
        return True

def make_env():
    """환경 생성 함수 - Monitor 없이"""
    env = RoArmPickPlaceEnv(cfg=None)
    return env

def main(total_timesteps: int = 100_000, log_root: str = "logs", progress_bar: bool = True):
    print("\n" + "="*70, flush=True)
    print(f"🚀 V4.2 Quick Training ({total_timesteps:,} steps)", flush=True)
    print("="*70, flush=True)
    
    try:
        # 1. 환경 생성 (벡터 환경 없이 단일 환경)
        print("\n📦 환경 초기화...", flush=True)
        env = make_env()
        print("✅ 환경 생성 완료!", flush=True)
    
        # 2. 로그 디렉토리 설정
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = os.path.join(log_root, f"ppo_v4.2_quick_{timestamp}")
        os.makedirs(log_dir, exist_ok=True)
        print(f"📁 로그 디렉토리: {log_dir}", flush=True)
    
        # 3. PPO 모델 생성
        print("\n🤖 PPO 모델 생성...", flush=True)
        model = PPO(
            "MlpPolicy",
            env,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.01,
            verbose=1,
            tensorboard_log=log_dir,
            device="cpu",
        )
        print("✅ PPO 모델 생성 완료!", flush=True)
        
        # 4. 콜백 설정
        print("\n⚙️ 콜백 설정...", flush=True)
        
        # 체크포인트 (매 25K steps)
        checkpoint_callback = CheckpointCallback(
            save_freq=25000,
            save_path=log_dir,
            name_prefix="v4_2_quick",
        )
        
        # 마일스톤 추적
        milestone_callback = SimpleMilestoneCallback(verbose=1)
        
        callbacks = CallbackList([
            checkpoint_callback,
            milestone_callback,
        ])
        print("✅ 콜백 설정 완료!", flush=True)
    
        # 5. 학습 시작
        print("\n" + "="*70, flush=True)
        print("🎯 학습 시작!", flush=True)
        print("="*70, flush=True)
        print(f"  - 총 스텝: {total_timesteps:,}", flush=True)
        print(f"  - Curriculum: Easy only", flush=True)
        print(f"  - 체크포인트: 매 25K", flush=True)
        print(f"  - 로그: {log_dir}", flush=True)
        print("="*70 + "\n", flush=True)
        
        try:
            model.learn(
                total_timesteps=total_timesteps,
                callback=callbacks,
                progress_bar=progress_bar,
            )
            
            # 최종 모델 저장
            final_model_path = os.path.join(log_dir, "v4_2_quick_final.zip")
            model.save(final_model_path)
            print(f"\n✅ 최종 모델 저장: {final_model_path}", flush=True)
            
        except KeyboardInterrupt:
            print("\n⚠️ 학습 중단됨 (Ctrl+C)", flush=True)
            interrupted_model_path = os.path.join(log_dir, "v4_2_quick_interrupted.zip")
            model.save(interrupted_model_path)
            print(f"💾 중단 시점 모델 저장: {interrupted_model_path}", flush=True)
    
    except Exception as e:
        print(f"\n❌ main() 함수에서 에러: {e}", flush=True)
        import traceback
        traceback.print_exc()
        raise
    
    finally:
        print("\n🧹 정리 작업 시작...", flush=True)
        try:
            env.close()
            print("  ✅ env.close() 완료", flush=True)
        except:
            pass
        try:
            simulation_app.close()
            print("  ✅ simulation_app.close() 완료", flush=True)
        except:
            pass
        print("🏁 학습 종료", flush=True)

if __name__ == "__main__":
    print("\n🎬 __main__ 블록 진입! __name__ =", __name__, flush=True)
    try:
        parser = argparse.ArgumentParser(description="Train RoArm-M3 PPO quick run")
        parser.add_argument("--total-steps", type=int, default=100_000,
                            help="Total timesteps for PPO.learn (default: 100000)")
        parser.add_argument("--log-root", type=str, default="logs",
                            help="Directory where log subfolders will be created (default: logs)")
        parser.add_argument("--progress-bar", dest="progress_bar", action="store_true",
                            help="Enable training progress bar (default: enabled)")
        parser.add_argument("--no-progress-bar", dest="progress_bar", action="store_false",
                            help="Disable training progress bar")
        parser.set_defaults(progress_bar=True)

        args = parser.parse_args()
        print(f"  ⚙️ 파싱된 인자: total_steps={args.total_steps}, progress_bar={args.progress_bar}", flush=True)
        print(f"  🔜 main() 함수 호출...", flush=True)
        main(total_timesteps=args.total_steps, log_root=args.log_root, progress_bar=args.progress_bar)
        print(f"  ✅ main() 함수 완료!", flush=True)
    except Exception as e:
        print(f"\n❌ __main__ 블록에서 에러: {e}", flush=True)
        import traceback
        traceback.print_exc()
else:
    print(f"\n❌ __main__ 블록 건너뜀! __name__ = {__name__}", flush=True)
