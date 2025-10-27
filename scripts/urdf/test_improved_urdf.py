#!/usr/bin/env python3
"""
개선된 URDF 시각적 테스트 스크립트
- 그리퍼 형상 확인
- 관절 범위 테스트
- 물체 파지 시도
"""

import sys
import os
import time
import numpy as np

# 프로젝트 루트를 path에 추가
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, project_root)

print("=" * 70)
print("    🖥️ Isaac Sim GUI 테스트")
print("=" * 70)
print()

# 환경 import
print("🔍 환경 Import 중...")
from envs.roarm_pick_place_env import RoArmPickPlaceEnv

print("✓ Import 성공!")
print()
print("🚀 Isaac Sim 실행 중... (30초~1분 소요)")
print("   창이 열릴 때까지 기다려주세요.")
print()

# GUI 모드로 환경 생성
env = RoArmPickPlaceEnv(
    render=True,
    headless=False
)

print("✓ 환경 초기화 완료!")
print()
print("=" * 70)
print("📊 테스트 시나리오")
print("=" * 70)
print("1️⃣  초기 자세 확인 (3초 대기)")
print("2️⃣  그리퍼 열기/닫기 반복 (10회)")
print("3️⃣  각 관절 개별 테스트")
print("4️⃣  물체 파지 시도")
print("=" * 70)
print()

try:
    # 초기화
    obs, info = env.reset()
    print("✓ 환경 리셋 완료")
    print("   👀 초기 자세를 확인하세요...")
    time.sleep(3)

    # 1. 그리퍼 테스트
    print("\n" + "-" * 70)
    print("1️⃣  그리퍼 동작 테스트")
    print("-" * 70)
    print("   그리퍼가 열리고 닫히는 모습을 확인하세요.")
    print("   👀 그리퍼 형상: 박스형 팁(20x10x10mm)이 보여야 합니다.")
    print()
    
    for i in range(10):
        # 그리퍼 닫기
        action = np.array([0, 0, 0, 0, 0, 0, 1, 1])
        obs, reward, terminated, truncated, info = env.step(action)
        time.sleep(0.3)
        
        # 그리퍼 열기
        action = np.array([0, 0, 0, 0, 0, 0, -1, -1])
        obs, reward, terminated, truncated, info = env.step(action)
        time.sleep(0.3)
        
        if (i + 1) % 3 == 0:
            print(f"   진행: {i+1}/10 완료")
    
    print("✓ 그리퍼 테스트 완료")

    # 2. 관절 테스트
    print("\n" + "-" * 70)
    print("2️⃣  관절 범위 테스트")
    print("-" * 70)
    print("   각 관절이 순서대로 동작합니다.")
    print("   👀 관절이 반대로 꺾이는지 확인하세요.")
    print()
    
    joint_names = [
        'joint_1 (Base - 360° 회전)',
        'joint_2 (Shoulder - 180°)',
        'joint_3 (Elbow - 180°)',
        'joint_4 (Wrist1 - 180°)',
        'joint_5 (Roll - 360°)',
        'joint_6 (EE Rotation - 180°)'
    ]
    
    for joint_idx in range(6):
        print(f"   테스트: {joint_names[joint_idx]}")
        
        # 중립 위치로 리셋
        obs, info = env.reset()
        time.sleep(0.5)
        
        # 양방향 천천히 이동
        for direction in [0.5, -0.5, 0]:
            action = np.zeros(8)
            action[joint_idx] = direction
            
            # 천천히 이동 (10 steps)
            for _ in range(10):
                obs, reward, terminated, truncated, info = env.step(action)
                time.sleep(0.05)
        
        time.sleep(0.5)
    
    print("✓ 관절 테스트 완료")

    # 3. 물체 파지 시도
    print("\n" + "-" * 70)
    print("3️⃣  물체 파지 시도")
    print("-" * 70)
    print("   로봇팔이 큐브를 잡으려고 시도합니다.")
    print("   👀 그리퍼가 큐브를 잡을 수 있는지 확인하세요.")
    print()
    
    obs, info = env.reset()
    time.sleep(1)
    
    # 큐브로 접근 (간단한 휴리스틱)
    print("   큐브로 접근 중...")
    for step in range(50):
        # EE를 아래로 + 앞으로 + 그리퍼 준비
        action = np.array([0, 0.3, -0.2, 0.2, 0, 0, -0.3, -0.3])
        obs, reward, terminated, truncated, info = env.step(action)
        time.sleep(0.05)
        
        if step == 25:
            print("   그리퍼 닫기...")
            # 그리퍼 닫기 시작
            action = np.array([0, 0.3, -0.2, 0.2, 0, 0, 1, 1])
        
        if terminated or truncated:
            print("   에피소드 종료")
            break
    
    print("✓ 파지 시도 완료")

    # 결과 확인
    print("\n" + "=" * 70)
    print("✅ GUI 테스트 완료!")
    print("=" * 70)
    print()
    print("👀 확인 체크리스트:")
    print("   [ ] 그리퍼 형상이 박스형(20x10mm 팁)으로 보이는가?")
    print("   [ ] 그리퍼가 부드럽게 열리고 닫히는가?")
    print("   [ ] 그리퍼 개폐 범위가 0~25mm인가?")
    print("   [ ] 모든 관절이 정상 범위 내에서 동작하는가?")
    print("   [ ] 관절이 반대로 꺾이는 현상이 있는가? (있으면 문제)")
    print("   [ ] 그리퍼가 큐브에 접촉할 수 있는가?")
    print("   [ ] 큐브를 잡을 수 있는가?")
    print()
    print("창을 계속 열어두려면 아무 키나 누르세요...")
    print("종료하려면 Ctrl+C를 누르세요.")
    
    # 사용자 입력 대기
    try:
        input()
        print("\n추가 30초 대기 중... (자유롭게 확인하세요)")
        time.sleep(30)
    except KeyboardInterrupt:
        print("\n✓ 사용자 중단")

except KeyboardInterrupt:
    print("\n\n✓ 사용자가 테스트를 중단했습니다.")
except Exception as e:
    print(f"\n\n❌ 오류 발생: {e}")
    import traceback
    traceback.print_exc()
finally:
    print("\n환경 종료 중...")
    env.close()
    print("✓ 환경 종료 완료")
    print()
    print("테스트 결과를 확인하셨나요?")
    print("문제가 있으면 알려주세요!")
