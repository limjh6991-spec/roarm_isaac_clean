#!/usr/bin/env python3
"""
간단한 URDF GUI 확인 스크립트
Isaac Sim의 Python으로 실행하세요.

실행 방법:
  ~/.local/share/ov/pkg/isaac-sim-*/python.sh test_urdf_simple.py
"""

import sys
import os

# 프로젝트 경로 추가
project_root = os.path.expanduser("~/roarm_isaac_clean")
sys.path.insert(0, project_root)

print("=" * 70)
print("    🔍 URDF 빠른 확인")
print("=" * 70)

from envs.roarm_pick_place_env import RoArmPickPlaceEnv
import numpy as np
import time

# GUI 모드로 환경 생성
print("\n✓ Isaac Sim 시작 중... (30초~1분 소요)")
env = RoArmPickPlaceEnv(render=True, headless=False)

print("✓ 환경 생성 완료!")
print("\n👀 확인 사항:")
print("  1. 그리퍼 형상 (박스형 팁 보이는가?)")
print("  2. 그리퍼 동작 (열리고 닫히는가?)")
print()

# 초기화
obs, info = env.reset()
time.sleep(2)

# 그리퍼 테스트 (5회)
print("🔧 그리퍼 동작 테스트 (5회)...")
for i in range(5):
    # 닫기
    action = np.array([0, 0, 0, 0, 0, 0, 1, 1])
    for _ in range(5):
        obs, reward, terminated, truncated, info = env.step(action)
        time.sleep(0.1)
    
    # 열기
    action = np.array([0, 0, 0, 0, 0, 0, -1, -1])
    for _ in range(5):
        obs, reward, terminated, truncated, info = env.step(action)
        time.sleep(0.1)
    
    print(f"  반복 {i+1}/5")

print("\n✅ 테스트 완료!")
print("창을 닫으려면 Ctrl+C를 누르세요.")

try:
    time.sleep(60)
except KeyboardInterrupt:
    pass

env.close()
print("✓ 종료")
