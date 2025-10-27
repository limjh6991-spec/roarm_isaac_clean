#!/usr/bin/env python3
"""
초기 자세만 확인 (학습된 모델 로딩 없이)
"""
import sys
import os

# Isaac Sim 경로
sys.path.insert(0, os.path.expanduser("~/isaacsim/exts/omni.isaac.kit/"))

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import time

# 환경 import
from envs.roarm_pick_place_env import RoArmPickPlaceEnv

print("\n" + "="*60)
print("🔍 로봇 초기 자세 확인 (모델 없음)")
print("="*60)

# 환경 생성
env = RoArmPickPlaceEnv()
obs = env.reset()

print("\n✅ 환경 리셋 완료!")
print("📋 초기 자세를 10초간 확인하세요...")
print("   (아무 동작도 하지 않습니다)")

# 15초 동안 초기 자세 유지 (스크린샷 찍기 시간)
print("📸 15초간 대기 - 스크린샷을 찍으세요!")
for i in range(900):  # 60 FPS * 15초
    env.world.step(render=True)
    simulation_app.update()
    if i % 60 == 0:
        print(f"  {i//60 + 1}초...")
    time.sleep(1/60)  # 60 FPS

print("\n✅ 확인 완료!")
simulation_app.close()
