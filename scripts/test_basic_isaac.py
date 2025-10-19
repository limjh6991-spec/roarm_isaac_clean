#!/usr/bin/env python3
"""
가장 기본적인 Isaac Sim 테스트
"""

from isaacsim import SimulationApp

# SimulationApp은 Import 후 즉시 생성해야 함
simulation_app = SimulationApp({"headless": False})

# 이제 다른 Isaac Sim 모듈 Import 가능
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

print("="*60)
print("🎮 Isaac Sim 기본 테스트")
print("="*60)

print("\n1. Creating World...")
world = World()

print("2. Adding ground plane...")
world.scene.add_default_ground_plane()

print("3. Adding a cube...")
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="test_cube",
        position=np.array([0.0, 0.0, 0.5]),
        size=0.05,
        color=np.array([1.0, 0.0, 0.0]),
    )
)

print("4. Resetting world...")
world.reset()

print("\n5. Running simulation for 300 steps...")
for i in range(300):
    world.step(render=True)
    if i % 60 == 0:
        print(f"   ⏱️  Step {i}/300")

print("\n6. Closing simulation...")
simulation_app.close()

print("\n="*60)
print("✅ 테스트 완료!")
print("="*60)
