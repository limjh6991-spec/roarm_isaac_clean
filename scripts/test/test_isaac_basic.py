#!/usr/bin/env python3
"""
Basic Isaac Sim Test - 최소한의 초기화만 테스트
"""

print("=" * 80)
print("Step 1: Import AppLauncher...")
print("=" * 80)

# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

print("\n" + "=" * 80)
print("Step 2: Create AppLauncher...")
print("=" * 80)
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

print("\n" + "=" * 80)
print("Step 3: Import Isaac modules...")
print("=" * 80)

import torch
# TODO_5.1: Isaac Lab not available in 5.1, consider using native API
# from omni.isaac.lab.sim import SimulationCfg, SimulationContext

print("\n" + "=" * 80)
print("Step 4: Create simulation...")
print("=" * 80)

sim_cfg = SimulationCfg(dt=1/60.0, device="cuda:0")
sim = SimulationContext(sim_cfg)

print("\n" + "=" * 80)
print("Step 5: Reset simulation...")
print("=" * 80)

sim.reset()

print("\n" + "=" * 80)
print("Step 6: Run 10 steps...")
print("=" * 80)

for i in range(10):
    sim.step()
    if i % 2 == 0:
        print(f"  Step {i+1}/10 completed")

print("\n" + "=" * 80)
print("✅ SUCCESS! Isaac Sim is working properly")
print("=" * 80)

simulation_app.close()
