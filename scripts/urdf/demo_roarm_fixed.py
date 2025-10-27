#!/usr/bin/env python3
"""
RoArm-M3 Pick and Place 환경 간단한 데모 (수정 버전)
랜덤 액션으로 환경 테스트
"""

import sys
from pathlib import Path

# ✅ 핵심: stdout buffering 해결
sys.stdout = sys.stderr

from isaacsim import SimulationApp

# Isaac Sim 초기화
simulation_app = SimulationApp({
    "headless": False,
    "width": 1920,
    "height": 1080,
})

import numpy as np

# 환경 임포트
sys.path.append(str(Path(__file__).parent.parent))
from envs.roarm_pick_place_env import RoArmPickPlaceEnv, RoArmPickPlaceEnvCfg


def run_demo(num_episodes: int = 3, max_steps: int = 200):
    """랜덤 액션으로 환경 데모"""
    print("="*60)
    print("🎮 RoArm-M3 Pick and Place 데모")
    print("="*60)
    print(f"  Episodes: {num_episodes}")
    print(f"  Max steps per episode: {max_steps}")
    print(f"\n💡 랜덤 액션으로 로봇을 움직입니다.")
    print(f"   (학습 전 환경 검증용)\n")
    
    # 환경 생성
    try:
        print("📦 환경 Config 생성 중...")
        cfg = RoArmPickPlaceEnvCfg()
        cfg.episode_length_s = 10.0
        
        print("🌍 환경 초기화 중...")
        env = RoArmPickPlaceEnv(cfg)
        print("✅ 환경 초기화 완료!")
    except Exception as e:
        print(f"❌ 환경 초기화 실패: {e}")
        import traceback
        traceback.print_exc()
        raise
    
    # 데모 실행
    for episode in range(num_episodes):
        print(f"\n{'='*60}")
        print(f"📺 Episode {episode + 1}/{num_episodes}")
        print(f"{'='*60}")
        
        obs = env.reset()
        print(f"Initial observation:")
        print(f"  - Joint positions: {obs[:6]}")
        print(f"  - Gripper state: {obs[6:8]}")
        print(f"  - End-effector position: {obs[8:11]}")
        print(f"  - Object position: {obs[11:14]}")
        print(f"  - Distance to target: {obs[14]:.3f}m")
        
        done = False
        total_reward = 0
        step = 0
        
        while not done and step < max_steps:
            # 랜덤 액션 생성
            joint_actions = np.random.uniform(-0.3, 0.3, size=6)
            
            # Gripper: 간헐적으로 열고 닫기
            if step % 50 < 25:
                gripper_actions = np.array([0.5, 0.5])  # 열기
            else:
                gripper_actions = np.array([-0.5, -0.5])  # 닫기
            
            action = np.concatenate([joint_actions, gripper_actions])
            
            # 환경 스텝
            obs, reward, done, info = env.step(action)
            total_reward += reward
            step += 1
            
            # 주기적으로 정보 출력
            if step % 50 == 0:
                print(f"\n  📊 Step {step}:")
                print(f"     - Cube position: {info['cube_position']}")
                print(f"     - Distance to target: {info['distance_to_target']:.3f}m")
                print(f"     - Reward: {reward:.2f}")
                print(f"     - Total reward: {total_reward:.2f}")
        
        # 에피소드 결과
        print(f"\n  {'='*56}")
        print(f"  📊 Episode {episode + 1} 결과:")
        print(f"     - Total steps: {step}")
        print(f"     - Total reward: {total_reward:.2f}")
        print(f"     - Final distance: {info['distance_to_target']:.3f}m")
        
        if info['distance_to_target'] < 0.05:
            print(f"     - 상태: ✅ SUCCESS!")
        else:
            print(f"     - 상태: ❌ Failed")
        print(f"  {'='*56}")
    
    # 환경 종료
    env.close()
    
    print(f"\n{'='*60}")
    print(f"✅ 데모 완료!")
    print(f"{'='*60}")
    print(f"\n💡 다음 단계:")
    print(f"  1. 환경이 정상적으로 동작하는지 확인했습니다.")
    print(f"  2. 학습을 시작하려면:")
    print(f"     ~/isaacsim/python.sh scripts/train_roarm_rl.py --mode train --timesteps 50000")
    print(f"  3. GUI에서 로봇 동작을 관찰하세요.")


if __name__ == "__main__":
    import argparse
    import traceback
    
    # 디버그 로깅
    with open("/tmp/demo_debug.log", "w") as f:
        f.write("SCRIPT STARTED\n")
    
    parser = argparse.ArgumentParser(description="RoArm-M3 환경 데모")
    parser.add_argument("--episodes", type=int, default=3,
                        help="에피소드 수")
    parser.add_argument("--steps", type=int, default=200,
                        help="에피소드당 최대 스텝")
    
    args = parser.parse_args()
    
    try:
        with open("/tmp/demo_debug.log", "a") as f:
            f.write("CALLING run_demo()\n")
        print("🚀 데모 시작!", flush=True)
        
        run_demo(num_episodes=args.episodes, max_steps=args.steps)
        
        with open("/tmp/demo_debug.log", "a") as f:
            f.write("run_demo() COMPLETED\n")
    except KeyboardInterrupt:
        print("\n⚠️ 사용자 중단", flush=True)
        with open("/tmp/demo_debug.log", "a") as f:
            f.write("KEYBOARD INTERRUPT\n")
    except Exception as e:
        print(f"\n❌ 에러 발생: {e}", flush=True)
        traceback.print_exc()
        with open("/tmp/demo_debug.log", "a") as f:
            f.write(f"EXCEPTION: {e}\n")
            f.write(traceback.format_exc())
    finally:
        with open("/tmp/demo_debug.log", "a") as f:
            f.write("CLOSING simulation_app\n")
        simulation_app.close()
        print("\n👋 Simulation 종료", flush=True)
        with open("/tmp/demo_debug.log", "a") as f:
            f.write("SCRIPT ENDED\n")
