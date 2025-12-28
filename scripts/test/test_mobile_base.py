#!/usr/bin/env python3
"""
Mobile Base Test
Jetbot 모바일 베이스 로드 및 기본 제어 테스트

Usage:
    /home/roarm_m3/isaacsim/python.sh scripts/test/test_mobile_base.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import sys
sys.path.insert(0, "/home/roarm_m3/roarm_isaac_clean")

# Isaac Sim imports
from isaacsim.core.api import World
from pxr import UsdLux
import omni.usd

# Project imports
from envs.mobile_manipulator.mobile_base import MobileBase


def main():
    print("\n" + "="*60)
    print("🤖 Mobile Base Test - Jetbot")
    print("="*60)
    
    # Create world
    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    
    # Ground plane
    world.scene.add_default_ground_plane()
    
    # Light
    light_prim = stage.DefinePrim("/World/DomeLight", "DomeLight")
    light = UsdLux.DomeLight(light_prim)
    light.GetIntensityAttr().Set(1500.0)
    
    world.reset()
    
    # Create mobile base
    print("\n📦 Loading Jetbot...")
    mobile_base = MobileBase(world, prim_path="/World/Jetbot")
    
    if not mobile_base.load():
        print("❌ Jetbot 로드 실패!")
        print("\n   Isaac Sim에서 Jetbot 경로를 확인하세요:")
        print("   - /Isaac/Robots/Jetbot/jetbot.usd")
        print("   - Nucleus Server 연결 필요")
        simulation_app.close()
        return
    
    print("\n✅ Jetbot 로드 성공!")
    
    # Stabilize
    for _ in range(100):
        world.step(render=True)
    
    # Test movement
    print("\n🎮 이동 테스트 시작")
    print("   1. 전진 (2초)")
    print("   2. 회전 (2초)")
    print("   3. 정지")
    print("\n   Ctrl+C로 종료")
    
    step = 0
    phase = 0
    
    try:
        while True:
            # Phase control
            if step < 200:
                # Forward
                mobile_base.set_velocity(linear=0.3, angular=0.0)
                if step == 0:
                    print("\n▶️ 전진 중...")
            elif step < 400:
                # Turn
                mobile_base.set_velocity(linear=0.0, angular=1.0)
                if step == 200:
                    print("🔄 회전 중...")
            else:
                # Stop
                mobile_base.stop()
                if step == 400:
                    print("⏹️ 정지")
            
            world.step(render=True)
            step += 1
            
            # Print position every 50 steps
            if step % 50 == 0:
                pos = mobile_base.get_position()
                print(f"   Step {step}: Position = {np.round(pos, 3)}")
            
            # Loop back
            if step > 600:
                step = 0
                print("\n🔁 반복...")
                
    except KeyboardInterrupt:
        print("\n\n⏹️ 사용자 중단")
    
    world.stop()
    simulation_app.close()
    print("✅ 테스트 완료")


if __name__ == "__main__":
    main()
