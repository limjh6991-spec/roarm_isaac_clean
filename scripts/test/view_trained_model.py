#!/usr/bin/env python3
"""
학습된 모델 GUI 시각화 스크립트
- 100K 학습 결과를 GUI에서 확인
- 실시간 동작 관찰
"""

import os
import sys

# Isaac Sim 경로 추가
isaac_sim_path = os.path.expanduser("~/isaacsim")
sys.path.insert(0, isaac_sim_path)

from isaacsim import SimulationApp

# GUI 모드로 시작
simulation_app = SimulationApp({"headless": False})

import numpy as np
from stable_baselines3 import PPO
import time

# 환경 import (SimulationApp 초기화 후)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from envs.roarm_pick_place_env import RoArmPickPlaceEnv

def main():
    print("\n" + "="*60)
    print("🎮 학습된 모델 GUI 시각화")
    print("="*60)
    
    # 환경 생성
    print("\n📦 환경 초기화 중...")
    env = RoArmPickPlaceEnv()
    
    # 최신 모델 찾기
    print("\n🔍 학습된 모델 탐색 중...")
    log_dir = "logs"
    model_dirs = [d for d in os.listdir(log_dir) if d.startswith("ppo_v4.2_quick_")]
    if not model_dirs:
        print("❌ 모델을 찾을 수 없습니다!")
        return
    
    latest_dir = sorted(model_dirs)[-1]
    model_path = os.path.join(log_dir, latest_dir, "v4_2_quick_final.zip")
    
    if not os.path.exists(model_path):
        print(f"❌ 모델 파일이 없습니다: {model_path}")
        return
    
    print(f"✅ 모델 로드: {model_path}")
    model = PPO.load(model_path)
    
    # 에피소드 실행
    print("\n🎬 에피소드 시작!")
    print("  - 총 5개 에피소드 실행")
    print("  - 각 에피소드 최대 600 스텝")
    print("  - ESC를 눌러 종료")
    print("="*60 + "\n")
    
    for episode in range(5):
        print(f"\n📺 Episode {episode + 1}/5")
        print("-" * 60)
        
        obs, _ = env.reset()
        episode_reward = 0
        step_count = 0
        reach_achieved = False
        
        for step in range(600):
            # 학습된 정책으로 action 선택
            action, _states = model.predict(obs, deterministic=True)
            
            # Action 실행
            obs, reward, done, truncated, info = env.step(action)
            episode_reward += reward
            step_count += 1
            
            # 🎨 렌더링 강제 업데이트 (화면에 로봇 움직임 표시)
            # - env.step()에서도 렌더링하지만, 확실히 보장하기 위해 추가 호출
            # - GUI 모드에서 뷰포트가 업데이트되도록 강제
            if hasattr(env.world, 'render'):
                env.world.render()
            
            # REACH 달성 체크
            if not reach_achieved:
                ee_pos = env._get_ee_position()
                cube_pos = env.cube.get_world_pose()[0]
                dist = float(np.linalg.norm(cube_pos - ee_pos))
                if dist < 0.03:
                    reach_achieved = True
                    print(f"  🎯 REACH! (step={step}, dist={dist:.3f}m)")
            
            # 100 스텝마다 진행상황 출력
            if step % 100 == 0 and step > 0:
                ee_pos = env._get_ee_position()
                cube_pos = env.cube.get_world_pose()[0]
                dist = float(np.linalg.norm(cube_pos - ee_pos))
                gripper_pos = env.robot.get_joint_positions()[5]
                print(f"  Step {step:3d}: dist={dist:.3f}m, gripper={gripper_pos:.3f}, reward={episode_reward:.1f}")
            
            if done or truncated:
                break
            
            # 시뮬레이션 속도 조절 (너무 빠르면 관찰 어려움)
            time.sleep(0.01)
        
        print(f"\n  ✅ Episode {episode + 1} 완료!")
        print(f"     Total Steps: {step_count}")
        print(f"     Total Reward: {episode_reward:.1f}")
        print(f"     REACH: {'✅' if reach_achieved else '❌'}")
        print("-" * 60)
        
        # 에피소드 간 3초 대기
        if episode < 4:
            print("  ⏳ 다음 에피소드까지 3초 대기...")
            time.sleep(3)
    
    print("\n" + "="*60)
    print("🎉 모든 에피소드 완료!")
    print("="*60)
    
    # 종료
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
