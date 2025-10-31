#!/usr/bin/env python3
"""
🎮 학습된 모델 GUI 테스트 스크립트
- 학습된 PPO 모델 로드
- GUI로 에이전트 행동 관찰
- 여러 에피소드 실행
"""

import sys
import argparse
from pathlib import Path

# 프로젝트 루트 추가
project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

# stdout 복원
sys.stdout = sys.__stdout__
sys.stderr = sys.__stderr__

from isaacsim import SimulationApp
# GUI 모드로 SimulationApp 초기화!
simulation_app = SimulationApp({"headless": False})

print("="*80, flush=True)
print("🎮 학습된 모델 GUI 테스트 - SimulationApp 초기화 완료!", flush=True)
print("="*80, flush=True)

import os
import numpy as np
import time

from envs.roarm_pick_place_env import RoArmPickPlaceEnv
from stable_baselines3 import PPO


def test_model(model_path: str, num_episodes: int = 10, render_delay: float = 0.01):
    """
    학습된 모델 테스트
    
    Args:
        model_path: 학습된 모델 경로 (.zip)
        num_episodes: 테스트할 에피소드 수
        render_delay: 각 스텝 후 대기 시간 (초, GUI 관찰용)
    """
    print("\n" + "="*70, flush=True)
    print(f"🤖 학습된 모델 테스트", flush=True)
    print("="*70, flush=True)
    print(f"  - 모델: {model_path}", flush=True)
    print(f"  - 에피소드: {num_episodes}", flush=True)
    print(f"  - Render Delay: {render_delay}s", flush=True)
    print("="*70, flush=True)
    
    try:
        # 1. 환경 생성
        print("\n📦 환경 초기화...", flush=True)
        env = RoArmPickPlaceEnv()
        print("✅ 환경 생성 완료!", flush=True)
        
        # 2. 모델 로드
        print(f"\n🔧 모델 로드 중: {model_path}", flush=True)
        if not os.path.exists(model_path):
            print(f"❌ 모델 파일이 존재하지 않습니다: {model_path}", flush=True)
            return
        
        model = PPO.load(model_path, env=env)
        print("✅ 모델 로드 완료!", flush=True)
        
        # 3. 에피소드 테스트
        print("\n" + "="*70, flush=True)
        print("🎯 에피소드 테스트 시작!", flush=True)
        print("="*70, flush=True)
        
        episode_rewards = []
        episode_lengths = []
        success_count = 0
        
        for ep in range(num_episodes):
            print(f"\n{'='*70}", flush=True)
            print(f"📺 Episode {ep+1}/{num_episodes}", flush=True)
            print(f"{'='*70}", flush=True)
            
            obs, info = env.reset()
            done = False
            episode_reward = 0
            step_count = 0
            
            while not done:
                # 모델로 액션 예측
                action, _states = model.predict(obs, deterministic=True)
                
                # 🔍 디버깅: 액션 출력 (처음 5스텝만)
                if step_count < 5:
                    print(f"  [Step {step_count}] Action: {action}", flush=True)
                
                # 환경 스텝
                obs, reward, terminated, truncated, info = env.step(action)
                done = terminated or truncated
                
                episode_reward += reward
                step_count += 1
                
                # GUI 관찰을 위한 딜레이
                if render_delay > 0:
                    time.sleep(render_delay)
                
                # 진행 상황 출력 (매 100 스텝)
                if step_count % 100 == 0:
                    print(f"  Step {step_count}: reward={reward:.2f}, total={episode_reward:.1f}", flush=True)
            
            # 에피소드 종료
            episode_rewards.append(episode_reward)
            episode_lengths.append(step_count)
            
            # SUCCESS 체크 (info에서)
            is_success = info.get('is_success', False)
            if is_success:
                success_count += 1
            
            print(f"\n  ✅ Episode {ep+1} 완료!", flush=True)
            print(f"     Total Reward: {episode_reward:.1f}", flush=True)
            print(f"     Steps: {step_count}", flush=True)
            print(f"     Success: {'🎉 YES' if is_success else '❌ NO'}", flush=True)
        
        # 4. 통계 출력
        print("\n" + "="*70, flush=True)
        print("📊 테스트 결과 통계", flush=True)
        print("="*70, flush=True)
        print(f"  총 에피소드: {num_episodes}", flush=True)
        print(f"  평균 Reward: {np.mean(episode_rewards):.1f} ± {np.std(episode_rewards):.1f}", flush=True)
        print(f"  평균 Steps: {np.mean(episode_lengths):.1f} ± {np.std(episode_lengths):.1f}", flush=True)
        print(f"  Success Rate: {success_count}/{num_episodes} ({100*success_count/num_episodes:.1f}%)", flush=True)
        print(f"  Best Reward: {max(episode_rewards):.1f}", flush=True)
        print(f"  Worst Reward: {min(episode_rewards):.1f}", flush=True)
        print("="*70, flush=True)
        
    except Exception as e:
        print(f"\n❌ 테스트 중 에러: {e}", flush=True)
        import traceback
        traceback.print_exc()
    
    finally:
        print("\n🧹 정리 작업...", flush=True)
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
        print("🏁 테스트 종료!", flush=True)


if __name__ == "__main__":
    print("\n🎬 __main__ 블록 진입!", flush=True)
    
    parser = argparse.ArgumentParser(description="Test trained PPO model with GUI")
    parser.add_argument("--model", type=str, required=True,
                        help="Path to trained model (.zip)")
    parser.add_argument("--episodes", type=int, default=10,
                        help="Number of test episodes (default: 10)")
    parser.add_argument("--delay", type=float, default=0.01,
                        help="Delay between steps for visualization (default: 0.01s)")
    
    args = parser.parse_args()
    print(f"  ⚙️ 파싱된 인자: model={args.model}, episodes={args.episodes}, delay={args.delay}", flush=True)
    
    test_model(
        model_path=args.model,
        num_episodes=args.episodes,
        render_delay=args.delay
    )
