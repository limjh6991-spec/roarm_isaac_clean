#!/usr/bin/env python3
"""
v3.7.2 환경 직접 테스트 (module cache 회피)
"""

import sys
from pathlib import Path

# 프로젝트 루트를 Python path에 추가
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

# 🔥 강제 reload
if 'envs.roarm_pick_place_env' in sys.modules:
    del sys.modules['envs.roarm_pick_place_env']

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import numpy as np

# 환경 임포트
from envs.roarm_pick_place_env import RoArmPickPlaceEnv
from envs.config import EnvConfig

print("\n" + "=" * 60)
print("🧪 v3.7.2 환경 직접 테스트")
print("=" * 60)

# 환경 생성
cfg = EnvConfig()
env = RoArmPickPlaceEnv(cfg)

# Reset
obs = env.reset()
print(f"\n✅ Reset 완료, observation shape: {obs.shape}")
print(f"   obs[23] (gripper_width): {obs[23]:.4f}")

# 10 steps 실행
print("\n🔄 10 steps 실행:")
for i in range(10):
    # 랜덤 액션
    action = np.random.uniform(-0.5, 0.5, size=7)
    action[6] = 1.0  # Gripper 완전 열기
    
    obs, reward, done, info = env.step(action)
    
    print(f"  Step {i+1}: obs[23]={obs[23]:.4f}, reward={reward:.2f}")

print("\n✅ 테스트 완료!")

simulation_app.close()
