#!/usr/bin/env python3
"""
Isaac Sim GUI 실행 스크립트

사용법:
    python scripts/launch_gui.py
"""

from isaacsim import SimulationApp

# GUI 모드로 Isaac Sim 시작
simulation_app = SimulationApp({
    "headless": False,  # GUI 활성화
    "renderer": "RayTracedLighting",
})

print("=" * 70)
print("Isaac Sim GUI 시작됨!")
print("=" * 70)
print()
print("URDF Import 방법:")
print("  1. 메뉴: Isaac Utils → URDF Importer")
print("  2. Input File: 찾아보기 클릭")
print("  3. URDF 파일 선택: /home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3.urdf")
print("  4. Import Config:")
print("     - Fix Base: ✅ 체크")
print("     - Merge Fixed Joints: ❌ 체크 해제")
print("  5. Import 버튼 클릭")
print()
print("GUI를 종료하려면 창을 닫으세요.")
print("=" * 70)

# GUI가 열려있는 동안 대기
import carb
while simulation_app.is_running():
    simulation_app.update()

# 종료
simulation_app.close()
print("Isaac Sim GUI 종료됨")
